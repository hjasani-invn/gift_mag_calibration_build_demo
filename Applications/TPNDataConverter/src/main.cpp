
#define _USE_MATH_DEFINES
#include <fstream>
#include <iostream>

#include "tpn_data_reader.hpp"
#include "tpn_packet_parser.hpp"
#include "tpn_data_adapter.hpp"
#include "CoordinateConverter.h"
#include "CmdReader.hpp"

#include <cmath>

static void testFrameComverter(const Fppe::TpnPosition &tpn_pos0, const Fppe::TpnPosition &tpn_pos1);

class FpblAdapter
{
    public:
        FpblAdapter( std::ofstream &trk, std::ofstream &mag )
        {
            os_trk = &trk;
            os_mag = &mag;
        }

        void setVenue( const Fppe::Venue &_venue )
        {
            venue = _venue;
            converter.SetFrameParams( venue.origin_lattitude, venue.origin_longitude, venue.alfa, venue.beta, venue.origin_azimuth );
            origin = {};
            origin.lattitude = venue.origin_lattitude;
            origin.longitude = venue.origin_longitude;
            origin.is_valid = true;
        }

        void process_data( const Fppe::TpnOutput &tpn_data )
        {
            double X( 0 ), Y( 0 ), H( 0 );
            Fppe::TpnPosition tpn_pos = tpn_data.position;
            Fppe::CoordinatesIncrement local_pos = {};

            bool result = converter.Geo2Local( tpn_data.position.lattitude, tpn_data.position.longitude, &X, &Y );
            result &= converter.Geo2Local_Heading( tpn_data.attitude.heading, &H );
            Fppe::TpnAttitude tpn_att = tpn_data.attitude;
            //tpn_att.heading = H;
            Fppe::Attitude fp_att;
            TpnDataConverter::convert_attitude_data( tpn_att, &fp_att );
            Fppe::MagneticVector mag = {};
            TpnDataConverter::convert_magnetic_data( tpn_data.timestamp, tpn_data.mag_data, &mag );


            TpnDataConverter::convert_position_data(origin, tpn_pos, &local_pos);

            double a = venue.origin_azimuth * M_PI / 180;
            double sin_a = sin( a );
            double cos_a = cos( a );

            if ( result && tpn_data.position.is_valid && fp_att.is_valid )
            {
                double s2_n = tpn_data.position.sigma_lattitude * tpn_data.position.sigma_lattitude;
                double s2_e = tpn_data.position.sigma_longitude * tpn_data.position.sigma_longitude;

                *os_trk << mag.timestamp << ", "
                        << true << ", " //!TODO
                        << X << ", "
                        << Y << ", "
                        << tpn_data.position.height << ", "
                        << tpn_data.position.floor << ", "
                        << s2_n * cos_a * cos_a + s2_e * sin_a * sin_a << ", " /// sig2_lat*cos^2 + sig2_lon*sin^2
                        << ( s2_e - s2_n ) * cos_a * sin_a << ", "
                        << ( s2_e - s2_n ) * cos_a * sin_a << ", "
                        << s2_n * sin_a * sin_a + s2_e * cos_a * cos_a << ", " /// sig2_lat*sin^2 + sig2_lon*cos^2
                        << tpn_data.position.sigma_height << ", "
                        << 0 << ", " //floor std
                        << fp_att.is_valid << ", "
                        << fp_att.quaternion[0] << ", "
                        << fp_att.quaternion[1] << ", "
                        << fp_att.quaternion[2] << ", "
                        << fp_att.quaternion[3] << ", "
                        << fp_att.covariance_quaternion[0][0] << ", "
                        << fp_att.covariance_quaternion[0][1] << ", "
                        << fp_att.covariance_quaternion[0][2] << ", "
                        << fp_att.covariance_quaternion[0][3] << ", "
                        << fp_att.covariance_quaternion[1][0] << ", "
                        << fp_att.covariance_quaternion[1][1] << ", "
                        << fp_att.covariance_quaternion[1][2] << ", "
                        << fp_att.covariance_quaternion[1][3] << ", "
                        << fp_att.covariance_quaternion[2][0] << ", "
                        << fp_att.covariance_quaternion[2][1] << ", "
                        << fp_att.covariance_quaternion[2][2] << ", "
                        << fp_att.covariance_quaternion[2][3] << ", "
                        << fp_att.covariance_quaternion[3][0] << ", "
                        << fp_att.covariance_quaternion[3][1] << ", "
                        << fp_att.covariance_quaternion[3][2] << ", "
                        << fp_att.covariance_quaternion[3][3] << std::endl;


                *os_mag << mag.timestamp << ", "
                        << "0, "
                        << mag.mX << ", "
                        << mag.mY << ", "
                        << mag.mZ << std::endl;
            }

        }
    private:
        GeoLocConverter2D converter;
        std::ostream *os_trk, *os_mag;
        Fppe::Venue venue;
        Fppe::TpnPosition origin;
};

int main( int argc, const char *argv[], char *envp[] )
{
    tpn_data_reader data_reader;
    TpnDataConverter data_adapter;
    std::ifstream ifs( argv[1], std::ios::in | std::ios::binary );

    std::ofstream fos( "rtfppl_data.txt", std::ios_base::out );
    std::ofstream fos_tpn( "tpn_data.txt", std::ios_base::out );
    std::ofstream fos_pos ( "fpbl_pos.txt", std::ios_base::out );
    std::ofstream fos_mag ( "fpbl_mag.txt", std::ios_base::out );

    FpblAdapter fpbl_adapter( fos_pos, fos_mag );
    Fppe::Venue venue = {};

    setOptionFromCmd( argc, argv, "--venue_lat", &venue.origin_lattitude );
    setOptionFromCmd( argc, argv, "--venue_lon", &venue.origin_longitude );
    setOptionFromCmd( argc, argv, "--venue_azim", &venue.origin_azimuth );

    fpbl_adapter.setVenue( venue );

    data_reader.setStream( &ifs );
    data_adapter.set_tpn_converter_out_stream( fos );

    if ( data_reader.is_state_correct() )
    {
        tpn_packet_parser parser;
        int packet_count = 0;
		Fppe::TpnPosition tpos0 = { 0 };

        while ( data_reader.read_next_packet( parser ) )
        {
            packet_count++;
            // parse paket
            tpn_packet_header_data_t header_data;

            if ( parser.get_packet_header_data( header_data ) )
            {
                std::cout << "packet " << packet_count << ": ";
                std::cout << header_data.packet_id_ << "  " << header_data.epoch_number_ << "  " << header_data.number_of_entities_;
            }

            tpn_entity_time_t e_time;
            tpn_entity_position_t e_pos;
            tpn_entity_attitude_t e_att;
            tpn_entity_attitude_standard_deviation_t e_att_std;
            tpn_entity_magnetometer_data_t e_mag;
            tpn_entity_grp_07_t e_grp_07;
            tpn_entity_internal_dbg_t e_dbg;
            tpn_entity_position_std_t e_pos_std;
            tpn_entity_velocity_t e_vel;
            tpn_entity_velocity_std_t e_vel_std;
            tpn_device_heading e_dev_head;
            tpn_heading_misalignment e_head_ma;

            bool f_time = parser.get_entity_time( e_time );
            bool f_pos = parser.get_entity_position( e_pos );
            bool f_att = parser.get_entity_attitude( e_att );
            bool f_att_std = parser.get_entity_attitude_standard_deviation(e_att_std);
            bool f_mag = parser.get_entity_magnetic_data( e_mag );
            bool f_grp_7 = parser.get_entity_grp_07( e_grp_07 );
            bool f_dbg = parser.get_entity_internal_dbg( e_dbg );
            bool f_pos_std = parser.get_entity_position_std( e_pos_std );
            bool f_dev_head = parser.get_entity_device_heading(e_dev_head);
            bool f_head_ma = parser.get_entity_heading_misalignment(e_head_ma);
            if (!f_dev_head) e_dev_head = {};
            if (!f_head_ma) e_head_ma = {};
            if (!f_att_std) e_att_std = {};

            if ( f_time && f_pos && f_att && f_mag && f_grp_7 && f_dbg && f_pos_std)
                if ( e_grp_07.m_nav_phase > 0 )
                {
                    Fppe::TpnOutput  tpn_pdr_data = {};
                    bool valid;

                    tpn_pdr_data.timestamp = e_time.timetag_;
                    std::cout << "    " << e_time.timetag_;
                    fos_tpn << e_time.timetag_;

                    std::cout << "    " << ( int )e_grp_07.m_nav_phase;
                    fos_tpn << "  ,  " << ( int )e_grp_07.m_nav_phase;

                    tpn_pdr_data.position.lattitude = e_pos.latitude_;
                    tpn_pdr_data.position.longitude = e_pos.longitude_;
                    tpn_pdr_data.position.height = e_pos.height_;
                    tpn_pdr_data.position.sigma_lattitude = e_pos_std.position_north_standard_deviation_;
                    tpn_pdr_data.position.sigma_longitude = e_pos_std.position_east_standard_deviation_;
                    tpn_pdr_data.position.sigma_height = e_pos_std.height_standard_deviation_;
                    tpn_pdr_data.position.is_valid = true;
                    fos_tpn.precision( 15 );
                    fos_tpn << "  ,  " << e_pos.latitude_ << " , " << e_pos.longitude_ << " , " << e_pos.height_;

					std::cout << "   | " << e_att.roll_ << "  " << e_att.pitch_ << "  " << e_att.heading_;
                    fos_tpn.precision( 7 );
                    fos_tpn << "   ,  " << e_att.roll_ << " , " << e_att.pitch_ << " , " << e_att.heading_;

                    tpn_pdr_data.attitude.roll = e_dbg.m_att_filter_roll;
                    tpn_pdr_data.attitude.pitch = e_dbg.m_att_filter_pitch;
                    tpn_pdr_data.attitude.heading = e_dbg.m_att_filter_heading;
                    tpn_pdr_data.attitude.sigma_roll = e_dbg.m_att_filter_roll_std;
                    tpn_pdr_data.attitude.sigma_pitch = e_dbg.m_att_filter_pitch_std;
                    tpn_pdr_data.attitude.sigma_heading = e_dbg.m_att_filter_heading_std;
                    tpn_pdr_data.attitude.orientation_id = e_dbg.m_orientation_on_pitch;
                    tpn_pdr_data.attitude.is_valid = true;
                    std::cout << "   | " << tpn_pdr_data.attitude.roll << "  " << tpn_pdr_data.attitude.pitch << "  " << tpn_pdr_data.attitude.heading << " " << ( int )e_dbg.m_orientation_on_pitch << "   | ";
                    fos_tpn.precision( 15 );
                    fos_tpn << "  ,  " << tpn_pdr_data.attitude.roll << " , " << tpn_pdr_data.attitude.pitch << " , " << tpn_pdr_data.attitude.heading << " , " << ( int )e_dbg.m_orientation_on_pitch;


                    tpn_pdr_data.mag_data.mX = e_mag.raw_data_x_;   // +146.5; //+140.84;
                    tpn_pdr_data.mag_data.mY = e_mag.raw_data_y_;   // +500.0; //+500.30;
                    tpn_pdr_data.mag_data.mZ = e_mag.raw_data_z_;   // -333.8; //-330.76;

                    //tpn_pdr_data.mag_data.mX = e_mag.calibrated_data_x_;
                    //tpn_pdr_data.mag_data.mY = e_mag.calibrated_data_y_;
                    //tpn_pdr_data.mag_data.mZ = e_mag.calibrated_data_z_;

                    std::cout << "    " << e_mag.raw_data_x_ << "  " << e_mag.raw_data_y_ << "  " << e_mag.raw_data_z_;
                    fos_tpn << "  ,  " << e_mag.raw_data_x_ << " , " << e_mag.raw_data_y_ << " , " << e_mag.raw_data_z_;
                    valid = e_mag.raw_data_available_;

                    fos_tpn << "  ,  " << (int)e_dev_head.device_heading_available_ << " , " << e_dev_head.device_heading_ << " , " << e_dev_head.misalignment_angle_;
                    fos_tpn << "  ,  " << e_head_ma.heading_misalignment_;
                    fos_tpn << "  ,  " << e_att_std.roll_standard_deviation_ << " , " << e_att_std.pitch_standard_deviation_ << " , " << e_att_std.heading_standard_deviation_;
                    
                    data_adapter.process_pdr_data( tpn_pdr_data );
                    fpbl_adapter.process_data( tpn_pdr_data );

                    fos_tpn << "\n";

                    // Uncoment code below for Coordinate converter testing
                    //testFrameComverter(tpos0, tpn_pdr_data.position);
                    //tpos0 = tpn_pdr_data.position;
                }
            std::cout << "\n";
        }
    }
    else
        std::cout << "Can't open TPN PDR data file";

    fos.close();
    fos_tpn.close();
    ifs.close();
    return 0;
}


static void GEO2NED_local(double lat0, double h, double d_lat, double d_lon, double &d_n, double &d_e)
{
	static const double R_EARTH_A = 6378137; // Earth semi-major axis, m
	static const double R_EARTH_B = 6356752; // Earth semi-minor axis, m
	d_n = d_lat * (R_EARTH_B + h);
	d_e = d_lon * (R_EARTH_A + h) * cos(lat0);
}

const double rad_per_degree = 0.017453292519943;
static void testFrameComverter(const Fppe::TpnPosition &tpn_pos0, const Fppe::TpnPosition &tpn_pos1)
{
	double d_north, d_east, d_down, d_up;
	double d_lat = (tpn_pos1.lattitude - tpn_pos0.lattitude) * rad_per_degree;
	double d_lon = (tpn_pos1.longitude - tpn_pos0.longitude) * rad_per_degree;
	GEO2NED_local(tpn_pos0.lattitude * rad_per_degree, tpn_pos0.height, d_lat, d_lon, d_north, d_east);

	GeoLocConverter2D glc;
	glc.SetFrameParams(tpn_pos0.lattitude, tpn_pos0.longitude, 0, 0, 30);
	double x0, y0, x1, y1;
	glc.Geo2Local(tpn_pos0.lattitude, tpn_pos0.longitude, &x0, &y0);
	glc.Geo2Local(tpn_pos1.lattitude, tpn_pos1.longitude, &x1, &y1);

	double dpos_geo[2], dpos_geo_ref[2] = { d_lat , d_lon };
	double dpos_loc[2], dpos_loc_ref[2] = { x1 - x0, y1 - y0 };
	
	glc.Geo2Local_vector(dpos_geo_ref, dpos_loc);
	glc.Local2Geo_vector(dpos_loc, dpos_geo);

	std::cout << "\n" << "loc_vec_err:" << dpos_loc_ref[0] - dpos_loc[0] << " , " << dpos_loc_ref[1] - dpos_loc[1];
	std::cout << "\n" << "geo_vec_err:" << dpos_geo_ref[0] - dpos_geo[0] << " , " << dpos_geo_ref[1] - dpos_geo[1];

	double cov_geo[2][2], cov_geo_ref[2][2] = { { dpos_geo_ref[0] * dpos_geo_ref[0], dpos_geo_ref[0] * dpos_geo_ref[1] },
												{ dpos_geo_ref[1] * dpos_geo_ref[0], dpos_geo_ref[1] * dpos_geo_ref[1] } };
	double cov_loc[2][2], cov_loc_ref[2][2] = { { dpos_loc_ref[0] * dpos_loc_ref[0], dpos_loc_ref[0] * dpos_loc_ref[1] },
											    { dpos_loc_ref[1] * dpos_loc_ref[0], dpos_loc_ref[1] * dpos_loc_ref[1] } };

	glc.Geo2Local_cov_matrix(cov_geo_ref, cov_loc);
	glc.Local2Geo_cov_matrix(cov_loc, cov_geo);

	std::cout << "\n" << "loc_matr_err:" << cov_loc_ref[0][0] - cov_loc[0][0] << " , " << cov_loc_ref[0][1] - cov_loc[0][1] 
		                       << " , " << cov_loc_ref[1][0] - cov_loc[1][0] << " , " << cov_loc_ref[1][1] - cov_loc[1][1];
	std::cout << "\n" << "geo_matr_err:" << cov_geo_ref[0][0] - cov_geo[0][0] << " , " << cov_geo_ref[0][1] - cov_geo[0][1]
		                       << " , " << cov_geo_ref[1][0] - cov_geo[1][0] << " , " << cov_geo_ref[1][1] - cov_geo[1][1];
};

