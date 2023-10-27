/*****************************************************************************/
/**
* @copyright   Copyright © InvenSense, Inc. 2016
* @project     RTFPPL
* @file        tpn_packet_parser.hpp
* @author      D Churikov
* @date        14 Apr 2016
* @brief       TPN data packet parser interface
*/
/*****************************************************************************/
#ifndef TPN_PACKET_PARSER_HPP
#define TPN_PACKET_PARSER_HPP

#include <stdint.h>

//----------------------------------------------------------------------------
struct tpn_packet_header_data_t
{
    tpn_packet_header_data_t() : payload_length_(0), packet_id_(0), stream_id_(0), epoch_number_(0), number_of_entities_(0) {}
    uint16_t payload_length_;    ///< Length of the payload(i.e.packet body) attached to the packet header excluding the packet checksum.
    uint16_t packet_id_;    ///< packet_id; 0x0003
    uint16_t stream_id_;    ///< Identifies the stream identification number of the current epoch if more than one output stream exists
    uint32_t epoch_number_;    ///< Number of the current epoch
    uint16_t number_of_entities_;    ///< Number of entities(E) within the current epoch.The maximum number of entities is 65, 535. Each entity in the packet has two fields : entity_id_ and entity_start_address_

};

//----------------------------------------------------------------------------
#define         ID_TIME                         0x00EA
struct tpn_entity_time_t
{
    double timetag_;    ///< time tag, sec
};

//----------------------------------------------------------------------------
#define         ID_POSITION                     0x0016
struct tpn_entity_position_t
{
    double latitude_;   ///< Lattitude, deg
    double longitude_;  ///< Longitude, deg
    float height_;      ///< Heeght, m

};

//----------------------------------------------------------------------------
// Position Standard Deviation
#define         ID_POSITION_STD                 0x00BD
struct tpn_entity_position_std_t
{
    float position_north_standard_deviation_;   ///> north position standard deviation, m
    float position_east_standard_deviation_;    ///> east position standard deviation, m
    float height_standard_deviation_;           ///> heightstandard deviation, m
};

//----------------------------------------------------------------------------
// 9.4.Velocity
#define         ID_VELOCITY                     0x00A2
struct tpn_entity_velocity_t
{
    float velocity_north_;      ///> north velocity, m/sec
    float velocity_east_;       ///> east velocity, m/sec
    float velocity_down_;       ///> down velocity, m/sec
};

//----------------------------------------------------------------------------
// 9.5.Velocity Standard Deviation
#define         ID_VELOCITY_STD                 0x005F
struct tpn_entity_velocity_std_t
{
    float velocity_north_standard_deviation_;    ///> north velocity standard deviation, m/sec
    float velocity_east_standard_deviation_;     ///> east velocity standard deviation, m/sec
};

//----------------------------------------------------------------------------
#define         ID_ATTITUDE                     0x00E9
struct tpn_entity_attitude_t
{
    float roll_;      ///< Device roll, deg
    float pitch_;     ///< Device pitch, deg
    float heading_;   ///< Device heading, deg
};

//----------------------------------------------------------------------------
#define         ID_ATTITUDE_STANDARD_DEVIATION  0x0060
struct tpn_entity_attitude_standard_deviation_t
{
    float roll_standard_deviation_;      ///< Device roll, deg
    float pitch_standard_deviation_;     ///< Device pitch, deg
    float heading_standard_deviation_;   ///< Device heading, deg
};

//----------------------------------------------------------------------------
#define         ID_FLOOR_NUMBER                 0x00DD
struct tpn_entity_floor_number_t
{
    uint16_t floor_number_;             ///< Floor number
};

//----------------------------------------------------------------------------
#define         ID_MAGNETOMETER_DATA            0x0062
struct tpn_entity_magnetometer_data_t
{
    uint8_t raw_data_available_;     ///< 0x00: raw data is not available, 0x01 : raw data is available
    float raw_data_x_;      ///< raw magnetic sensor x data, mG
    float raw_data_y_;      ///< raw magnetic sensor y data, mG
    float raw_data_z_;      ///< raw magnetic sensor z data, mG
    uint8_t raw_data_accuracy_flag_;    ///< 0x00 : Flag not available, 0x01 : Invalid data, 0x02 : Valid data

    uint8_t calibrated_data_available_;    ///<    0x00 : calibrated data is not available, 0x01 : calibrated data is available
    float calibrated_data_x_;      ///< raw magnetic sensor x data, mG
    float calibrated_data_y_;      ///< raw magnetic sensor x data, mG
    float calibrated_data_z_;      ///< raw magnetic sensor x data, mG
    uint8_t calibrated_data_accuracy_flag_;    ///< 0x00 : Flag not available, 0x01 : Unreliable, 0x02 : Low Accuracy, 0x03 : Medium Accuracy, 0x04 : High Accuracy
};

//----------------------------------------------------------------------------
#define         ID_ORIENTATION_PITCH_D_T        0x00fd
struct tpn_entity_orientation_pitch_d_t
{
    int8_t m_orientation;       ///< it orientation based on pitch flag
};

//----------------------------------------------------------------------------
#define         ID_HEADING_MISSALIGNMENT                0x004D
struct tpn_entity_heading_misalignment_t
{
    float heading_misalignment_; //deg
};

//----------------------------------------------------------------------------
#define          ID_DEVICE_HEADING                      0x00BB
struct tpn_entity_device_heading_t
{
    int8_t device_heading_available_; //0x00: Device heading value is not available directly and will be computed from the platform_heading_ and misalignment_angle_ fields.
    //0x01: Device heading value is available directly from the device_heading_ field.
    float device_heading_;
    float platform_heading_;
    float misalignment_angle_;
};

//----------------------------------------------------------------------------
/// grp 7 data
#define         ID_GRP_07_T                     0x00e7
struct tpn_entity_grp_07_t
{
    int8_t m_nav_phase;    ///< navigation phase flag
};

//-----------------------------------------------------------------------------
/// stride information
#define         ID_STRIDE_INFORMATION           0x005E
struct tpn_entity_stride_information_t
{
    float stride_distance_;    ///< m
    float stride_velocity_;    ///< m/s
};

//------------------------------------------------------------------------------
/// internal debug info
#define         ID_INTERNAL_DBG_T               0x008c
struct tpn_entity_internal_dbg_t
{
    float m_imu_smp_proc_time;            //1
    // ...
    int8_t m_orientation_on_pitch;         //6  ///< it orientation based on pitch flag
    // ...
    int8_t walking_fidgeting_flag;         //9  ///< fidjeting flag
    // ...
    float misalignment_walking_pca_p2_;    //22

    double m_internal_roll;                //29
    double m_internal_pitch;               //30
    // ...
    float m_att_filter_roll;              //49
    float m_att_filter_pitch;             //50
    float m_att_filter_heading;           //51
    float m_att_filter_roll_std;          //52
    float m_att_filter_pitch_std;         //53
    float m_att_filter_heading_std;       //54
    
    float barometer_filter_height_;//65
    float barometer_filter_height_standard_deviation_; //66
    
    // ...
};

//-----------------------------------------------------------------------------
///  ble scan result
#define         ID_BLE_SCAN_T                   0x0206
struct tpn_entity_ble_scan_t
{
    uint_fast64_t   timestamp;      /// unix time [us]
    uint_fast64_t   mac;            /// MAC addres in decimal form 
    uint8_t         has_MAC;        /// mac address avaliability flag
    uint16_t        frequency;      /// central channel frequency [MHz]
    uint16_t        major;          /// iBeacon major number
    uint16_t        minor;          /// iBeacon minor number
    uint64_t        uuid_h;         /// iBeacon uuid 64 hight bytes
    uint64_t        uuid_l;         /// iBeacon uuid 64 low bytes
    int8_t          tx_power;       /// iBeacon tx power level [dbm] on 1m distance
    int8_t          rssi;           /// RSSI value [dbm]
};

//-----------------------------------------------------------------------------
/// wifi scan result
#define         ID_WIFI_SCAN_T                  0x0207
struct tpn_entity_wifi_scan_t
{
    uint_fast64_t  timestamp;       /// unix time [us]
    uint_fast64_t mac;              /// MAC address in decimal form
    int8_t   rssi;                  /// RSSI value [dbm]
    uint16_t frequency;             /// central channel frequency [MHz]
};

//-----------------------------------------------------------------------------
/// GNSS PVT data
#define         ID_GNSS_PVT_T                  0x001C
struct tpn_entity_gnss_pvt_t
{
    double timestamp;            /// GNSS time [sec]
    double latitude;            /// GNSS latitude [deg]
    double longitude;            /// GNSS longitude [deg]
    float height;                /// GNSS height [m]
    float velocity_north;        /// GNSS velocity north [m/sec]
    float velocity_east;        /// GNSS velocity east [m/sec]
    float velocity_down;        /// GNSS velocity down [m/sec]
    float pos_north_std;        /// GNSS position north standard deviation [m]
    float pos_east_std;            /// GNSS position east standard deviation [m]
    float height_std;            /// GNSS height standard deviation [m]
    float vel_north_std;        /// GNSS velocity north standard deviation [m/sec]
    float vel_east_std;            /// GNSS velocity east standard deviation [m/sec]
    float vel_down_std;            /// GNSS velocity down standard deviation [m/sec]
    uint8_t dop_data_available; /// GNSS doppler data availability flag (0 - not available, 1 -available)
    float horizontal_dop;        /// GNSS horizontal doppler
    float vertical_dop;            /// GNSS vertical doppler
};

//-----------------------------------------------------------------------------
/// 9.17. Mode of Transit
#define         ID_MODE_OF_TRANSIT                  0x00C8
struct tpn_entity_mode_of_transit
{
    uint8_t mode; // Current mode of transit:
                    //0x00: Driving
                    //0x01: Walking
                    //0x02: Elevator
                    //0x03: Stairs
                    //0x04: Escalator walking
                    //0x05: Escalator standing
                    //0x06: Fidgeting
                    //0x07: Conveyer walking
                    //0x08: Conveyer standing
                    //0x09: Running
                    //0x0A: Cycling
};

//-----------------------------------------------------------------------------
/// 9.28. Barometer Data*
#define         ID_BAROMETER_DATA                  0x0083
struct tpn_entity_barometer_data
{
    float height_;  // barometer height, m
    float height_standard_deviation_; // barometer height standard deviation, m
};

//-----------------------------------------------------------------------------
/// 9.28. Barometer Height*
#define         ID_BAROMETER_HEIGHT                  0x00E0
struct tpn_entity_barometer_height
{
    float barometer_filter_height_;  // barometer height, m
    float barometer_filter_height_standard_deviation_; // barometer height standard deviation, m
};

//------------------------------------------------------------------------------
class tpn_packet_parser
{
    public:
        tpn_packet_parser();
        //tpn_packet_parser( uint8_t* tpn_packet, uint16_t tpn_packet_size );
        bool set_packet_data( uint8_t* tpn_packet, uint16_t tpn_packet_size );
        ~tpn_packet_parser();
        bool is_state_correct();

        bool get_packet_header_data( tpn_packet_header_data_t &packet_header_data );

        bool get_entity_time( tpn_entity_time_t &ent );
        bool get_entity_position( tpn_entity_position_t &ent );
        bool get_entity_position_std( tpn_entity_position_std_t &ent );
        bool get_entity_velocity( tpn_entity_velocity_t &ent );
        bool get_entity_velocity_std( tpn_entity_velocity_std_t &ent );
        bool get_entity_attitude( tpn_entity_attitude_t &ent );
        bool get_entity_floor_number(tpn_entity_floor_number_t &ent);
        bool get_entity_magnetic_data( tpn_entity_magnetometer_data_t &ent );
        bool get_entity_orientation_pitch_d( tpn_entity_orientation_pitch_d_t &ent );
        bool get_entity_grp_07( tpn_entity_grp_07_t &ent );
        bool get_entity_internal_dbg( tpn_entity_internal_dbg_t &ent );
        bool get_entity_device_heading(tpn_entity_device_heading_t &ent);
        bool get_entity_heading_misalignment(tpn_entity_heading_misalignment_t &ent);
        bool get_entity_attitude_standard_deviation(tpn_entity_attitude_standard_deviation_t &ent);
        bool get_entity_stride_information(tpn_entity_stride_information_t &ent);
        bool get_entity_mode_of_transit(tpn_entity_mode_of_transit &ent);

        // new stile interface
        bool get_entity(tpn_entity_ble_scan_t &ent, uint16_t entity_index);
        bool get_entity(tpn_entity_wifi_scan_t &ent, uint16_t entity_index);
        bool get_entity(tpn_entity_gnss_pvt_t &ent);
        bool get_entity(tpn_entity_barometer_data &ent);
        bool get_entity(tpn_entity_barometer_height &ent);


    private:
        uint8_t* get_entity_address(uint16_t id, uint16_t idx = 0);

    private:
        uint16_t sz;
        uint8_t* data;
        tpn_packet_header_data_t header_data;
};


#endif