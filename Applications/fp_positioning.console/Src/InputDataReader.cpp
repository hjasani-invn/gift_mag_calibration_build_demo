#define _USE_MATH_DEFINES
#include <iomanip>
#include <iostream>
#include "InputDataReader.hpp"
#include "stringMac_to_intMac.h"
#include <fstream>
#include <cassert>
#include <algorithm>

// Altitude sources
#define TPN_PARSING_USE_REGULAR_ALTITUDE       0
#define TPN_PARSING_USE_FILTERED_ALTITUDE      1
#define TPN_PARSING_USE_BAROMETRIC_ALTITUDE    2
#define TPN_PARSING_NO_BAROMETER               3

#define TPN_PARSING_ALTITUDE_SOURCE TPN_PARSING_USE_BAROMETRIC_ALTITUDE

namespace FPPositionConsole
{
    InputDataReader::InputDataReader()
    {
        wifiMeasurementNumberPrev = 1;
        wifiMeasurement.timestamp = 0;
        bleMeasurementNumberPrev = 1;
        bleMeasurement.timestamp = 0;
    }

    InputDataReader::~InputDataReader()
    {
        fileslist.clear();

        if ( inputPosMagDataStream.is_open() )
            inputPosMagDataStream.close();

        if ( inputWiFiDataStream.is_open() )
            inputWiFiDataStream.close();

        if ( inputBleDataStream.is_open() )
            inputBleDataStream.close();
    }



    Fppe::ReturnStatus InputDataReader::setInputDataFolder( const std::string &input_data_folder )
    {
        DIR *dir_pointer;
        struct dirent *entry;

        this->input_data_folder = std::string( input_data_folder );

#ifdef _WIN32

        if ( this->input_data_folder.back() != '\\' )
        {
            this->input_data_folder = this->input_data_folder + "\\";
        }

#else

        if ( this->input_data_folder.back() != '/' )
        {
            this->input_data_folder = this->input_data_folder + "/";
        }

#endif

        dir_pointer = opendir( this->input_data_folder.c_str() );

        if ( dir_pointer == NULL )
        {
            return Fppe::ReturnStatus::STATUS_UNKNOWN_ERROR;
        }

        std::string fullfilename;

        while ( ( entry = readdir( dir_pointer ) ) )
        {
            if ( entry->d_type != DT_DIR )
            {
                fileslist.push_back( entry->d_name );
            }
        }

        //    std::sort(fileslist.begin(), fileslist.end());
        return Fppe::ReturnStatus::STATUS_SUCCESS;
    }

    Fppe::ReturnStatus InputDataReader::setInputIncMagDataFile( const std::string &file_mask )
    {
        return setInputDataFile( inputPosMagDataStream, file_mask );
    }


    Fppe::ReturnStatus InputDataReader::setInputWiFiDataFile( const std::string &file_mask )
    {
        return setInputDataFile( inputWiFiDataStream, file_mask );
    }

    Fppe::ReturnStatus InputDataReader::setInputBleDataFile( const std::string &file_mask )
    {
        return setInputDataFile( inputBleDataStream, file_mask );
    }

    Fppe::ReturnStatus InputDataReader::setInputFrameworkDataFile(const std::string &file_mask)
    {
        return setInputDataFile(inputFrameworkDataStream, file_mask);
    }

    Fppe::ReturnStatus InputDataReader::setInputCollaborationDataFile(const std::string &file_mask)
    {
        return setInputDataFile(inputFrameworkDataStream, file_mask);
    }

    Fppe::ReturnStatus InputDataReader::setInputDataFile( std::ifstream &inputDataStream, const std::string &file_mask, std::ios_base::openmode mode )
    {
        std::string input_data_file= "";

        for ( auto it = fileslist.begin(); it != fileslist.end(); ++it )
        {
            if ( std::regex_match( *it, std::regex( file_mask ) ) )
            {
                //std::cout << *it << std::endl;
                input_data_file = *it;
                break;
            }
        }

        if(input_data_file == "")
            return Fppe::ReturnStatus::STATUS_UNKNOWN_ERROR;
        
        std::string fullfilename = input_data_folder + input_data_file;

        inputDataStream.open( fullfilename, mode );

        if ( inputDataStream.fail() )
        {
            inputDataStream.close();
            return Fppe::ReturnStatus::STATUS_UNKNOWN_ERROR;
        }
        else
            return Fppe::ReturnStatus::STATUS_SUCCESS;
    }

    Fppe::ReturnStatus InputDataReader::getInputWiFiData( Fppe::WiFiScanResult &wifi_scan_result )
    {
        wifi_scan_result.scanWiFi.clear();
        wifi_scan_result.timestamp = 0;

        if ( wifiMeasurement.timestamp != 0 )
        {
            wifi_scan_result.scanWiFi.push_back( wifiMeasurement );
            wifiMeasurement.timestamp = 0;
        }

        std::string line;
        const char delim = ',';

        uint64_t timestamp;
        int16_t  rssi;


        std::string macStr;

        uint64_t wifiMeasurementNumber;

        while ( std::getline( inputWiFiDataStream, line ) )
        {
            std::stringstream buf( line );
            std::string token;
            std::vector<std::string> result;

            while ( std::getline( buf, token, delim ) )
            {
                result.push_back( token );
            }

            std::stringstream ss;
            ss << result[0];
            ss >> timestamp;
            wifiMeasurement.timestamp = timestamp;
            ss.str( "" );
            ss.clear();

            // NOTE: value number [1] is skipped, unused
            ss << result[1];
            ss >> wifiMeasurementNumber;
            ss.str( "" );
            ss.clear();

            ss << result[2];
            ss >> macStr;
            wifiMeasurement.mac = stringMac_to_intMac( macStr );
            ss.str( "" );
            ss.clear();

            ss << result[4];
            ss >> wifiMeasurement.frequency;
            ss.str( "" );
            ss.clear();

            ss << result[6];
            ss >> rssi;
            wifiMeasurement.rssi = rssi;
            ss.str( "" );
            ss.clear();
            // wifi data parsing done

            if ( wifiMeasurementNumber != wifiMeasurementNumberPrev )
            {
                wifi_scan_result.timestamp = wifi_scan_result.scanWiFi[0].timestamp;

                wifiMeasurementNumberPrev = wifiMeasurementNumber;

                return Fppe::ReturnStatus::STATUS_SUCCESS;

            }

            wifi_scan_result.scanWiFi.push_back( wifiMeasurement );

        }


        return Fppe::ReturnStatus::STATUS_SUCCESS;
    }

    Fppe::ReturnStatus InputDataReader::getInputBleData( Fppe::BleScanResult &ble_scan_result )
    {
        ble_scan_result.scanBle.clear();

        if ( bleMeasurement.timestamp != 0 )
        {
            ble_scan_result.scanBle.push_back( bleMeasurement );
            bleMeasurement.timestamp = 0;
        }

        std::string line;
        const char delim = ',';

        uint64_t timestamp;
        int16_t  rssi;
        uint16_t  major, minor;
        int16_t txPower;

        std::string macStr;

        uint64_t bleMeasurementNumber = 0;

        while ( std::getline( inputBleDataStream, line ) )
        {
            std::stringstream buf( line );
            std::string token;
            std::vector<std::string> result;

            while ( std::getline( buf, token, delim ) )
            {
                result.push_back( token );
            }

            std::stringstream ss;
            ss << result[0];
            ss >> timestamp;
            bleMeasurement.timestamp = timestamp;
            ss.str( "" );
            ss.clear();

            // NOTE: value number [1] is skipped, unused
            ss << result[1];
            ss >> bleMeasurementNumber;
            ss.str( "" );
            ss.clear();

            ss << result[2];
            ss >> macStr;
            bleMeasurement.hasMAC = false;

            if ( &macStr == NULL || macStr == "" )
            {
                bleMeasurement.mac = 0;
            }
            else
            {
                bleMeasurement.mac = stringMac_to_intMac( macStr );

                if ( bleMeasurement.mac != 0 )
                    bleMeasurement.hasMAC = true;
            }

            ss.str( "" );
            ss.clear();

            ss << result[4];
            ss >> bleMeasurement.frequency;
            ss.str( "" );
            ss.clear();

            ss << result[6];
            ss >> rssi;
            bleMeasurement.rssi = rssi;
            ss.str( "" );
            ss.clear();

            ss << result[8];
            ss >> major;
            bleMeasurement.major = major;
            ss.str( "" );
            ss.clear();

            ss << result[9];
            ss >> minor;
            bleMeasurement.minor = minor;
            ss.str( "" );
            ss.clear();

            if (bleMeasurement.hasMAC == false)
            {
                bleMeasurement.mac = ((uint64_t)major << 16) + minor;
            }

            ss << result[10];
            ss >> txPower;
            bleMeasurement.txPower = txPower;
            ss.str( "" );
            ss.clear();
            // wifi data parsing done

            if ( bleMeasurementNumber != bleMeasurementNumberPrev )
            {
                ble_scan_result.timestamp = ble_scan_result.scanBle[0].timestamp;

                bleMeasurementNumberPrev = bleMeasurementNumber;

                return Fppe::ReturnStatus::STATUS_SUCCESS;
            }

            ble_scan_result.scanBle.push_back( bleMeasurement );

        }

        return Fppe::ReturnStatus::STATUS_SUCCESS;
    }

    Fppe::ReturnStatus InputDataReader::getInputFrameworkData(Fppe::Position &frame_work_position)
    {
        Fppe::ReturnStatus result = Fppe::ReturnStatus::STATUS_UNKNOWN_ERROR;

        std::string line;
        if (std::getline(inputFrameworkDataStream, line))
        {
            std::stringstream buf(line);
            std::string token;
            std::vector<std::string> fields;

            const char delim = ',';
            while (std::getline(buf, token, delim))
            {
                fields.push_back(token);
            }

            const std::size_t k_framework_log_data_field_number = 13;
            if (fields.size() == k_framework_log_data_field_number)
            {
                std::stringstream ss;

                uint64_t timestamp;
                ss << fields[0];     ss >> timestamp;                       ss.str("");     ss.clear();
                frame_work_position.timestamp = timestamp;

                ss << fields[1];    ss >> frame_work_position.lattitude;    ss.str("");     ss.clear();

                ss << fields[2];    ss >> frame_work_position.longitude;    ss.str("");     ss.clear();

                ss << fields[3];    ss >> frame_work_position.altitude;     ss.str("");     ss.clear();

                ss << fields[4];    ss >> frame_work_position.floor_number; ss.str("");     ss.clear();

                ss << fields[5];    ss >> frame_work_position.azimuth;      ss.str("");     ss.clear();

                ss << fields[6];    ss >> frame_work_position.covariance_lat_lon[0][0];      ss.str("");     ss.clear();
                ss << fields[7];    ss >> frame_work_position.covariance_lat_lon[1][0];      ss.str("");     ss.clear();
                ss << fields[8];    ss >> frame_work_position.covariance_lat_lon[0][1];      ss.str("");     ss.clear();
                ss << fields[9];    ss >> frame_work_position.covariance_lat_lon[1][1];      ss.str("");     ss.clear();

                ss << fields[10];   ss >> frame_work_position.floor_std;   ss.str("");     ss.clear();

                ss << fields[11];   ss >> frame_work_position.azimuth_std; ss.str("");     ss.clear();

                int is_valid;
                ss << fields[12];   ss >> is_valid;                        ss.str("");     ss.clear();
                frame_work_position.is_valid = (is_valid != 0) ? true : false;

                result = Fppe::ReturnStatus::STATUS_SUCCESS;
                // framework data parsing done
            }
            else
            {
                frame_work_position.is_valid = false;
                result = Fppe::ReturnStatus::STATUS_UNKNOWN_ERROR;
            }
        }
        return result;
    }

    Fppe::ReturnStatus InputDataReader::getInputCollaborationData(std::vector <Fppe::CollaborationData> &collaboration_position)
    {
        Fppe::ReturnStatus result = Fppe::ReturnStatus::STATUS_UNKNOWN_ERROR;
#if 0
        Fppe::CollaborationData position;

        collaboration_position.clear();

        position.timestamp = 30000;     /**< unix time [ms] */
        position.lattitude = 2;     /**< lattitude [deg] [-90..+90]*/
        position.longitude = 59;     /**< lattitude [deg] [-90..+90]*/
        position.cov_ne[0][0] = 1;
        position.cov_ne[0][1] = 0;
        position.cov_ne[1][0] = 0;
        position.cov_ne[1][1] = 1;
        position.floor_number = 5;  /**< discrete floor number [0..32767], must be positive or zero */
        position.distance = 2;
        position.distance_uncertainty = 1;

        collaboration_position.push_back(position);

        position.timestamp = 31000;     /**< unix time [ms] */
        position.lattitude = 2;     /**< lattitude [deg] [-90..+90]*/
        position.longitude = 49;     /**< lattitude [deg] [-90..+90]*/
        position.cov_ne[0][0] = 1;
        position.cov_ne[0][1] = 0;
        position.cov_ne[1][0] = 0;
        position.cov_ne[1][1] = 1;
        position.floor_number = 5;  /**< discrete floor number [0..32767], must be positive or zero */
        position.distance = 2;
        position.distance_uncertainty = 1;

        collaboration_position.push_back(position);

        position.timestamp = 32000;     /**< unix time [ms] */
        position.lattitude = 2;     /**< lattitude [deg] [-90..+90]*/
        position.longitude = 39;     /**< lattitude [deg] [-90..+90]*/
        position.cov_ne[0][0] = 1;
        position.cov_ne[0][1] = 0;
        position.cov_ne[1][0] = 0;
        position.cov_ne[1][1] = 1;
        position.floor_number = 5;  /**< discrete floor number [0..32767], must be positive or zero */
        position.distance = 2;
        position.distance_uncertainty = 1;

        collaboration_position.push_back(position);

        result = Fppe::ReturnStatus::STATUS_SUCCESS;
#endif
        return result;
    }

    Fppe::ReturnStatus TpnInputDataReader::setInputIncMagDataFile( const std::string &file_mask )
    {
        Fppe::ReturnStatus result = setInputDataFile( inputPosMagDataStream, file_mask, std::ios_base::in | std::ios_base::binary );

        if ( result == Fppe::ReturnStatus::STATUS_SUCCESS )
        {
            tpn_reader.setStream( &inputPosMagDataStream );

            if ( tpn_reader.read_file_header() == false )
            {
                result = Fppe::ReturnStatus::STATUS_UNKNOWN_ERROR;
                return result;
            }

            tpn_packet_header_data_t header_data;

            if ( tpn_parser.get_packet_header_data( header_data ) )
            {
                //std::cout << "header: ";
                //std::cout << header_data.packet_id_ << "  " << header_data.epoch_number_ << "  " << header_data.number_of_entities_;
            }
        }

        return result;
    }
    
    Fppe::ReturnStatus TpnInputDataReader::getInputIncMagData(TpnOutput &tpnOutput, Fppe::WiFiScanResult &wifi_scan_result, Fppe::BleScanResult &ble_scan_result)
    {
        Fppe::ReturnStatus status;

        tpnOutput = {};
        tpnOutput.position.is_valid = false;
        tpnOutput.mag_meas.is_valid = false;
        tpnOutput.attitude.is_valid = false;
        tpnOutput.pdr.is_valid = false;
        tpnOutput.position.navigation_phase = -127; // no data

        wifi_scan_result.scanWiFi.clear();
        wifi_scan_result.timestamp = 0;

        ble_scan_result.scanBle.clear();
        ble_scan_result.timestamp = 0;

        std::fstream gnss_log;
        gnss_log.open("gnss.log", std::ofstream::out | std::ofstream::app);
        gnss_log.precision(12);

        if ( tpn_reader.is_state_correct() )
        {
            //while ( tpn_reader.read_next_packet( tpn_parser ) )
            if (tpn_reader.read_next_packet(tpn_parser))
            {
                packet_count++;
                // parse paket
                tpn_packet_header_data_t header_data;

                if ( tpn_parser.get_packet_header_data( header_data ) )
                {
                    //std::cout << "packet " << packet_count << ": " << std::endl;
                    //std::cout << header_data.packet_id_ << "  " << header_data.epoch_number_ << "  " << header_data.number_of_entities_ << std::endl;
                }

                tpn_entity_time_t e_time;
                tpn_entity_position_t e_pos;
                tpn_entity_attitude_t e_att;
                tpn_entity_magnetometer_data_t e_mag;
                tpn_entity_grp_07_t e_grp_07;
                tpn_entity_internal_dbg_t e_dbg;
                tpn_entity_floor_number_t e_floor;
                tpn_entity_position_std_t e_pos_std = {};
                tpn_entity_velocity_t e_vel;
                tpn_entity_velocity_std_t e_vel_std;
                tpn_entity_stride_information_t e_stride_inf;
                tpn_entity_heading_misalignment_t e_head_mis = {};
                tpn_entity_mode_of_transit e_transit_mode;

                // special logig is required for e_baro_data processing due to 1Hz samplerate
                tpn_entity_barometer_data e_baro_data;

                bool f_time = tpn_parser.get_entity_time( e_time );

                // parse wifi data
                if ( f_time )
                {
                    tpn_entity_wifi_scan_t e_wifi;
                    if (tpn_parser.get_entity(e_wifi, 0))
                    {
                        wifi_scan_result.scanWiFi.clear();
                        wifi_scan_result.timestamp = (int64_t)(e_time.timetag_ * 1.e3);
                    }
                    //wifi_scan_result.timestamp = 0;

                    uint16_t wifi_number = 0;
                    bool new_wifi_entity = true;

                    while ( new_wifi_entity )
                    {
                        new_wifi_entity = tpn_parser.get_entity( e_wifi, wifi_number );

                        if ( new_wifi_entity )
                        {
                            // fill one WiFiMeasurement element
                            Fppe::WiFiMeasurement wifiMeasurement;
                            wifiMeasurement.timestamp = e_wifi.timestamp;
                            wifiMeasurement.frequency = e_wifi.frequency;
                            wifiMeasurement.mac = e_wifi.mac;
                            wifiMeasurement.rssi = e_wifi.rssi;

                            wifi_scan_result.scanWiFi.push_back( wifiMeasurement );

                            //std::cout << "number = " << wifi_number << " timestamp = " << wifiMeasurement.timestamp;
                            //std::cout << " bssid = " << wifiMeasurement.mac << " rssi = " << wifiMeasurement.rssi << std::endl;

                            wifi_number++;
                        }
                    }

                }

                // parse ble data
                if (f_time)
                {
                    tpn_entity_ble_scan_t e_ble;
                    if (tpn_parser.get_entity(e_ble, 0))
                    {
                        ble_scan_result.scanBle.clear();
                        ble_scan_result.timestamp = (int64_t)(e_time.timetag_ * 1.e3);
                    }

                    uint16_t ble_number = 0;
                    bool new_ble_entity = true;

                    while (new_ble_entity)
                    {
                        new_ble_entity = tpn_parser.get_entity(e_ble, ble_number);
                        if (new_ble_entity)
                        {
                            // fill one element
                            Fppe::BleMeasurement bleMeasurement;
                            bleMeasurement.timestamp = e_ble.timestamp;
                            bleMeasurement.frequency = e_ble.frequency;
                            bleMeasurement.rssi = e_ble.rssi;
                            bleMeasurement.major = e_ble.major;
                            bleMeasurement.minor = e_ble.minor;
                            bleMeasurement.txPower = e_ble.tx_power;
                            
                            bleMeasurement.hasMAC = (e_ble.has_MAC != 0) ? true : false;
                            if (bleMeasurement.hasMAC == false)
                            {
                                // WARNING: TO DO this conversion by using hash function
                                bleMeasurement.mac = ((uint64_t)e_ble.major << 16) + e_ble.minor;
                            }
                            else
                            {
                                bleMeasurement.mac = e_ble.mac;
                            }

                            // WARNING: UUID curently is 128-bit number (16-byte arry) in big endiam format
                            memcpy(bleMeasurement.uuid, &(e_ble.uuid_l), sizeof(e_ble.uuid_l));
                            memcpy(bleMeasurement.uuid + sizeof(e_ble.uuid_l), &(e_ble.uuid_h), sizeof(e_ble.uuid_h));

                            ble_scan_result.scanBle.push_back(bleMeasurement);
                            ble_number++;
                        }
                    }
                }

                // TODO: add GNSS entity parsing
                if (f_time)
                {
                    tpn_entity_gnss_pvt_t e_gnss;
                    bool f_gnss = tpn_parser.get_entity(e_gnss);
                    if (f_gnss)
                    {
                        // output GNSS position data to file
                        gnss_log << e_time.timetag_ << " " << e_gnss.timestamp << " " << e_gnss.latitude << " " << e_gnss.longitude << std::endl;
                    }

                }

                //parse tpn
                bool f_pos = tpn_parser.get_entity_position(e_pos);
                bool f_att = tpn_parser.get_entity_attitude(e_att);
                bool f_mag = tpn_parser.get_entity_magnetic_data(e_mag);
                bool f_grp_7 = tpn_parser.get_entity_grp_07(e_grp_07);
                bool f_dbg = tpn_parser.get_entity_internal_dbg(e_dbg);
                bool f_floor = tpn_parser.get_entity_floor_number(e_floor);
                bool has_pos_std = tpn_parser.get_entity_position_std(e_pos_std);
                bool f_stride_inf = tpn_parser.get_entity_stride_information(e_stride_inf);
                bool f_mis = tpn_parser.get_entity_heading_misalignment(e_head_mis);
                bool f_baro = tpn_parser.get_entity(e_baro_data);
                bool f_mode_of_transit = tpn_parser.get_entity_mode_of_transit(e_transit_mode);

                // baro data propagation for case when baro meass is missed
                if (f_baro)
                {
                    this->stored_e_baro_data.height_ = e_baro_data.height_;
                    this->stored_e_baro_data.height_standard_deviation_ = 0; // this->stored_e_baro_data.height_standard_deviation_; - this field is not felt by IRPL
                }
                else if (this->stored_e_baro_data.height_standard_deviation_ < -1) // < 0 means baro data is not initialized
                {
                    this->stored_e_baro_data.height_ = 0;
                    this->stored_e_baro_data.height_standard_deviation_ = -1; // this->stored_e_baro_data.height_standard_deviation_; - this field is not felt by IRPL
                }
                //else: do not update 
                

#if TPN_PARSING_ALTITUDE_SOURCE != TPN_PARSING_USE_BAROMETRIC_ALTITUDE
                this->stored_e_baro_data.height_standard_deviation_ = (std::max)(0.0, (double)(this->stored_e_baro_data.height_standard_deviation_));
#endif

#if TPN_PARSING_ALTITUDE_SOURCE == TPN_PARSING_NO_BAROMETER
                    this->stored_e_baro_data.height_ = 0;
                    this->stored_e_baro_data.height_standard_deviation_ = 0; 
#endif

                if (f_time)
                {
                    tpnOutput.timestamp = e_time.timetag_;
                }

                if (f_time && f_pos && f_grp_7 && f_dbg && f_floor && has_pos_std && (this->stored_e_baro_data.height_standard_deviation_ >= -1))
                    //this->stored_e_baro_data.height_standard_deviation_ >= 0 means that e_baro entity have been already processed at least once
                {
                    tpnOutput.position.lattitude = e_pos.latitude_;
                    tpnOutput.position.longitude = e_pos.longitude_;
                    tpnOutput.position.floor = e_floor.floor_number_;
                    tpnOutput.position.navigation_phase = e_grp_07.m_nav_phase; /// alignment status

                    /// -1 : Pre - alignment
                    ///  0 : Alignment
                    ///+ 1 : Navigation / Available
                    ///+ 2 : Navigation / Reliable
                    ///+ 3 : Navigation / Vertical Alignment
                    ///+ 4 : Navigation / Drive to Walk

                    tpnOutput.position.sigma_north = e_pos_std.position_north_standard_deviation_;
                    tpnOutput.position.sigma_east = e_pos_std.position_east_standard_deviation_;

                    tpnOutput.position.fidgeting_flag = 0; // e_dbg.walking_fidgeting_flag;

                    tpnOutput.position.is_valid = true;
                    
                    tpnOutput.position.mode_of_transit = (f_mode_of_transit) ? (e_transit_mode.mode) : (1);

#if TPN_PARSING_ALTITUDE_SOURCE == TPN_PARSING_USE_FILTERED_ALTITUDE
                    tpnOutput.position.altitude = e_dbg.barometer_filter_height_;
                    tpnOutput.position.sigma_altitude = e_dbg.barometer_filter_height_standard_deviation_;
#elif TPN_PARSING_ALTITUDE_SOURCE == TPN_PARSING_USE_BAROMETRIC_ALTITUDE
                     tpnOutput.position.altitude = this->stored_e_baro_data.height_;
                     tpnOutput.position.sigma_altitude = e_dbg.barometer_filter_height_standard_deviation_; // this->stored_e_baro_data.height_standard_deviation_; - this field is not felt by IRPL
#elif TPN_PARSING_ALTITUDE_SOURCE == TPN_PARSING_USE_REGULAR_ALTITUDE
                    tpnOutput.position.altitude = e_pos.height_;
                    tpnOutput.position.sigma_altitude = e_pos_std.height_standard_deviation_;
#else 
                    tpnOutput.position.altitude = 0;
                    tpnOutput.position.sigma_altitude = 0;
#endif
                }

                if (f_time && f_stride_inf)
                {
                    tpnOutput.pdr.stride_length = (f_stride_inf) ? (e_stride_inf.stride_distance_) : (0);
                    tpnOutput.pdr.is_valid = true;
                    
                    tpnOutput.pdr.misalignment = (f_mis) ? (e_head_mis.heading_misalignment_) : (0);
                    tpnOutput.pdr.misalignment_p1 = 0;
                    tpnOutput.pdr.misalignment_p2 = (f_mis && f_dbg) ? (e_dbg.misalignment_walking_pca_p2_) : (0);
                }
                
                if (f_time && /*f_att &&*/ f_dbg)
                {
                    tpnOutput.attitude.roll = e_dbg.m_att_filter_roll;;
                    tpnOutput.attitude.pitch = e_dbg.m_att_filter_pitch;
                    tpnOutput.attitude.heading = e_dbg.m_att_filter_heading;
                    //tpnOutput.attitude.alignment_status = e_grp_07.m_nav_phase; // WARNING: it was removed, since it is UNUSED and not included in TpnData.hpp

                    tpnOutput.attitude.sigma_roll = e_dbg.m_att_filter_roll_std;
                    tpnOutput.attitude.sigma_pitch = e_dbg.m_att_filter_pitch_std;
                    tpnOutput.attitude.sigma_heading = e_dbg.m_att_filter_heading_std;
                    tpnOutput.attitude.orientation_id = e_dbg.m_orientation_on_pitch;
                    tpnOutput.attitude.is_valid = true;
                    //tpnOutput.attitude.is_valid &= e_mag.raw_data_available_; //ACHTUNG
                }

                if (f_time && f_mag)
                {
                    tpnOutput.mag_meas.mX = e_mag.raw_data_x_;
                    tpnOutput.mag_meas.mY = e_mag.raw_data_y_;
                    tpnOutput.mag_meas.mZ = e_mag.raw_data_z_;
                    tpnOutput.mag_meas.is_valid = (e_mag.raw_data_available_ != 0) ? true : false;
                }
                        //break; //exit while
            }

            status = Fppe::ReturnStatus::STATUS_SUCCESS;
        }
        else
        {
            //std::cout << "Can't open TPN PDR data file";
            std::cout << "No more TPN data";
            status = Fppe::ReturnStatus::STATUS_UNKNOWN_ERROR;
        }

        gnss_log.close();

        return status;
    }

} // namespace FPBuilderConsole
