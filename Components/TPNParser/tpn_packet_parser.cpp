/*****************************************************************************/
/**
* @copyright   Copyright © InvenSense, Inc. 2016
* @project     RTFPPL
* @file        tpn_packet_parser.cpp
* @author      D Churikov
* @date        14 Apr 2016
* @brief       TPN data packet parser
*/
/*****************************************************************************/

#include"tpn_packet_parser.hpp"

#include <memory.h>

//---------------------------------------------------------------------------
tpn_packet_parser::tpn_packet_parser()
{
    sz = 0;        data = 0;
}
/*
//---------------------------------------------------------------------------
tpn_packet_parser::tpn_packet_parser( uint8_t* tpn_packet, uint16_t tpn_packet_size )
{
    set_packet_data( tpn_packet, tpn_packet_size );
}
*/
//---------------------------------------------------------------------------
bool tpn_packet_parser::set_packet_data( uint8_t* tpn_packet, uint16_t tpn_packet_size )
{
    sz = tpn_packet_size;

    if ( data ) 
        delete data;

    data = new uint8_t[tpn_packet_size];

    if ( data )
    {
        memcpy( data, tpn_packet, tpn_packet_size * sizeof( uint8_t ) );
        this->get_packet_header_data( this->header_data );
    }

    return is_state_correct();
}

//---------------------------------------------------------------------------
tpn_packet_parser::~tpn_packet_parser()
{
    if ( data ) 
        delete data;
}

//---------------------------------------------------------------------------
bool tpn_packet_parser::is_state_correct()
{
    return data;
}

//#define GET_VALUE(TYPE, ADDRESS) (*(((TYPE)*)(ADDRESS)))
#define GET_VALUE(TYPE, ADDRESS) (*(TYPE*)(ADDRESS))
//---------------------------------------------------------------------------
bool tpn_packet_parser::get_packet_header_data( tpn_packet_header_data_t &packet_header_data )
{
    bool result = false;

    if ( this->is_state_correct() )
    {
        packet_header_data.payload_length_ = this->sz - ( 15 - 3 - 2 );
        //packet_header_data.packet_id_ = *((uint16_t*)(this->data + 0));
        //packet_header_data.stream_id_ = *((uint16_t*)(this->data + 2));
        //packet_header_data.epoch_number_ = *((uint32_t*)(this->data + 4));
        //packet_header_data.number_of_entities_ = *((uint16_t*)(this->data + 8));
        packet_header_data.packet_id_           = GET_VALUE( uint16_t, this->data + 0 );
        packet_header_data.stream_id_           = GET_VALUE( uint16_t, this->data + 2 );
        packet_header_data.epoch_number_        = GET_VALUE( uint32_t, this->data + 4 );
        packet_header_data.number_of_entities_  = GET_VALUE( uint16_t, this->data + 8 );

        result = true;
    }

    return result;
}

//---------------------------------------------------------------------------
uint8_t* tpn_packet_parser::get_entity_address( uint16_t id , uint16_t idx)
{
    uint8_t* e_address = 0;
    uint8_t* e_start_address = data + ( 15 - 3 - 2 ) + 2 * sizeof( uint16_t ) * this->header_data.number_of_entities_;
    uint16_t* e_table = ( uint16_t* )( this->data + ( 15 - 3 - 2 ) );

    for (uint16_t i = 0; i < this->header_data.number_of_entities_ * 2; i += 2)
    {
        if (e_table[i] == id)
            if (idx-- == 0)
            {
                e_address = e_start_address + e_table[i + 1];
                break;
            }
    }
    return e_address;
}

//---------------------------------------------------------------------------
bool tpn_packet_parser::get_entity_time( tpn_entity_time_t &ent )
{
    bool result = false;

    if (uint8_t* e_address = this->get_entity_address(ID_TIME))
    {
        ent.timetag_ = GET_VALUE( double, e_address + 0 );
        result = true;
    }

    return result;
}

//---------------------------------------------------------------------------
bool tpn_packet_parser::get_entity_position( tpn_entity_position_t &ent )
{
    bool result = false;
    uint8_t* e_address;

    if (e_address = this->get_entity_address(ID_POSITION))
    {
        ent.latitude_   = GET_VALUE( double, e_address + 0 );
        ent.longitude_  = GET_VALUE( double, e_address + 8 );
        ent.height_     = GET_VALUE( float,  e_address + 16 );
        result = true;
    }

    return result;
}

//---------------------------------------------------------------------------
bool tpn_packet_parser::get_entity_position_std( tpn_entity_position_std_t &ent )
{
    bool result = false;

    if (uint8_t* e_address = this->get_entity_address(ID_POSITION_STD))
    {
        ent.position_north_standard_deviation_ = GET_VALUE( float, e_address );   e_address += sizeof( float );
        ent.position_east_standard_deviation_ = GET_VALUE( float, e_address );    e_address += sizeof( float );
        ent.height_standard_deviation_ = GET_VALUE( float, e_address );
        result = true;
    }

    return result;
}

//---------------------------------------------------------------------------
bool tpn_packet_parser::get_entity_velocity( tpn_entity_velocity_t&ent )
{
    bool result = false;

    if (uint8_t* e_address = this->get_entity_address(ID_VELOCITY))
    {
        ent.velocity_north_ = GET_VALUE( float, e_address );      e_address += sizeof( float );
        ent.velocity_east_ = GET_VALUE( float, e_address );       e_address += sizeof( float );
        ent.velocity_down_ = GET_VALUE( float, e_address );
        result = true;
    }

    return result;
}

//---------------------------------------------------------------------------
bool tpn_packet_parser::get_entity_velocity_std( tpn_entity_velocity_std_t&ent )
{
    bool result = false;

    if (uint8_t* e_address = this->get_entity_address(ID_VELOCITY_STD))
    {
        ent.velocity_north_standard_deviation_ = GET_VALUE( float, e_address );   e_address += sizeof( float );
        ent.velocity_east_standard_deviation_ = GET_VALUE( float, e_address );
        result = true;
    }

    return result;
}

//---------------------------------------------------------------------------
bool tpn_packet_parser::get_entity_attitude( tpn_entity_attitude_t &ent )
{
    bool result = false;

    if (uint8_t* e_address = this->get_entity_address(ID_ATTITUDE))
    {
        ent.roll_       = GET_VALUE( float, e_address + 0 );
        ent.pitch_      = GET_VALUE( float, e_address + 4 );
        ent.heading_    = GET_VALUE( float, e_address + 8 );
        result = true;
    }
    return result;
}

//---------------------------------------------------------------------------
bool tpn_packet_parser::get_entity_floor_number(tpn_entity_floor_number_t &ent)
{
    bool result = false;

    if (uint8_t* e_address = this->get_entity_address(ID_FLOOR_NUMBER))
    {
        ent.floor_number_ = GET_VALUE(uint16_t, e_address + 0);
        result = true;
    }
    return result;
}

//---------------------------------------------------------------------------
bool tpn_packet_parser::get_entity_attitude_standard_deviation(tpn_entity_attitude_standard_deviation_t &ent)
{
    bool result = false;

    if (uint8_t* e_address = this->get_entity_address(ID_ATTITUDE_STANDARD_DEVIATION))
    {
        ent.roll_standard_deviation_ = GET_VALUE(float, e_address + 0);
        ent.pitch_standard_deviation_ = GET_VALUE(float, e_address + 4);
        ent.heading_standard_deviation_ = GET_VALUE(float, e_address + 8);
        result = true;
    }
    return result;
}

bool tpn_packet_parser::get_entity_magnetic_data( tpn_entity_magnetometer_data_t &ent )
{
    bool result = false;

    if (uint8_t* e_address = this->get_entity_address(ID_MAGNETOMETER_DATA))
    {
        ent.raw_data_available_ = GET_VALUE( uint8_t, e_address + 0 );
        ent.raw_data_x_ = GET_VALUE( float, e_address + 1 );
        ent.raw_data_y_ = GET_VALUE( float, e_address + 5 );
        ent.raw_data_z_ = GET_VALUE( float, e_address + 9 );
        ent.raw_data_accuracy_flag_ = GET_VALUE( uint8_t, e_address + 13 );

        ent.calibrated_data_available_ = GET_VALUE(uint8_t, e_address + 14);
        ent.calibrated_data_x_ = GET_VALUE( float, e_address + 15 );
        ent.calibrated_data_y_ = GET_VALUE( float, e_address + 19 );
        ent.calibrated_data_z_ = GET_VALUE( float, e_address + 23 );
        ent.calibrated_data_accuracy_flag_ = GET_VALUE(uint8_t, e_address + 27 );
        result = true;
    }

    return result;
}

bool tpn_packet_parser::get_entity_stride_information(tpn_entity_stride_information_t &ent)
{
    bool result = false;
    if (uint8_t* e_address = this->get_entity_address(ID_STRIDE_INFORMATION))
    {
        ent.stride_distance_ = GET_VALUE(float, e_address + 0);
        ent.stride_velocity_ = GET_VALUE(float, e_address + 4);
        result = true;
    }

    return result;
}

bool tpn_packet_parser::get_entity_orientation_pitch_d( tpn_entity_orientation_pitch_d_t &ent )
{
    bool result = false;

    if (uint8_t* e_address = this->get_entity_address(ID_ORIENTATION_PITCH_D_T))
    {
        ent.m_orientation = GET_VALUE( int8_t, e_address + 0 );
        result = true;
    }

    return result;
}

bool tpn_packet_parser::get_entity_device_heading(tpn_entity_device_heading_t &ent)
{
    bool result = false;

    if (uint8_t* e_address = this->get_entity_address(ID_DEVICE_HEADING))
    {
        ent.device_heading_available_ = GET_VALUE(int8_t, e_address);       e_address += 1;
        ent.device_heading_ = GET_VALUE(float, e_address);                  e_address += 4;
        ent.platform_heading_ = GET_VALUE(float, e_address);                e_address += 4;
        ent.misalignment_angle_ = GET_VALUE(float, e_address);
        result = true;
    }

    return result;
}

bool tpn_packet_parser::get_entity_heading_misalignment(tpn_entity_heading_misalignment_t &ent)
{
    bool result = false;

    if (uint8_t* e_address = this->get_entity_address(ID_HEADING_MISSALIGNMENT))
    {
        ent.heading_misalignment_ = GET_VALUE(float, e_address);
        result = true;
    }

    return result;
}

bool tpn_packet_parser::get_entity_grp_07( tpn_entity_grp_07_t &ent )
{
    bool result = false;

    if (uint8_t* e_address = this->get_entity_address(ID_GRP_07_T))
    {
        ent.m_nav_phase = GET_VALUE( int8_t, e_address + 0 );
        result = true;
    }

    return result;
}

bool tpn_packet_parser::get_entity_internal_dbg( tpn_entity_internal_dbg_t &ent )
{
    bool result = false;

    if (uint8_t* e_address = this->get_entity_address(ID_INTERNAL_DBG_T))
    {
        uint8_t* e_address0 = e_address;

        ent.m_imu_smp_proc_time = GET_VALUE( float, e_address + 0 ); e_address += sizeof( float );
        double radius_of_rotation_ = GET_VALUE( double, e_address ); e_address += sizeof( double ); // 2, DOUBLE64    m
        double radius_of_rotation_x_ = GET_VALUE( double, e_address ); e_address += sizeof( double ); // DOUBLE64    m
        double radius_of_rotation_y_ = GET_VALUE( double, e_address ); e_address += sizeof( double ); // DOUBLE64    m
        double radius_of_rotation_z_ = GET_VALUE( double, e_address ); e_address += sizeof( double ); // DOUBLE64    m

        ent.m_orientation_on_pitch = GET_VALUE( int8_t, e_address ); e_address += sizeof( int8_t ); // 6, INT8,   N/A

        uint8_t automatic_ear_flag_ = GET_VALUE( uint8_t, e_address ); e_address += sizeof( uint8_t ); //     UINT8    N / A
        uint8_t sensors_ear_flag_ = GET_VALUE( uint8_t, e_address ); e_address += sizeof( uint8_t ); //     UINT8    N / A
        ent.walking_fidgeting_flag = GET_VALUE(uint8_t, e_address); e_address += sizeof(uint8_t); //     UINT8    N / A
        int16_t height_changing_modes_ = GET_VALUE( int16_t, e_address ); e_address += sizeof( int16_t ); //     INT16    N / A
        uint8_t children_info_ = GET_VALUE( uint8_t, e_address ); e_address += sizeof( uint8_t ); // UINT8    N / A
        float misalignement_driving_gps_ = GET_VALUE( float, e_address ); e_address += sizeof( float ); // FLOAT32    deg
        float misalignement_driving_radius_ = GET_VALUE( float, e_address ); e_address += sizeof( float ); // FLOAT32    deg
        float misalignement_driving_kalman_filter_ = GET_VALUE( float, e_address ); e_address += sizeof( float ); // FLOAT32    deg
        float misalignment_driving_sensors_ = GET_VALUE( float, e_address ); e_address += sizeof( float ); // FLOAT32    deg
        float misalignment_driving_sensors_combined_ = GET_VALUE( float, e_address ); e_address += sizeof( float ); // FLOAT32    deg
        float misalignment_driving_children_ = GET_VALUE( float, e_address ); e_address += sizeof( float ); // FLOAT32    deg
        float misalignment_driving_sensors_align_average_ = GET_VALUE( float, e_address ); e_address += sizeof( float ); // FLOAT32    deg
        float misalignment_driving_sensors_ratio_ = GET_VALUE( float, e_address ); e_address += sizeof( float ); // FLOAT32    N / A
        int16_t    misalignment_driving_sensors_count_ = GET_VALUE( int16_t, e_address ); e_address += sizeof( int16_t ); // INT16    N / A
        float misalignment_walking_pca_p1_ = GET_VALUE( float, e_address ); e_address += sizeof( float ); // FLOAT32    deg
        ent.misalignment_walking_pca_p2_ = GET_VALUE(float, e_address); e_address += sizeof(float); // FLOAT32    deg
        //float misalignment_walking_pca_p2_ = GET_VALUE( float, e_address ); e_address += sizeof( float ); // FLOAT32    deg
        float misalginment_walking_outdoors_ = GET_VALUE( float, e_address ); e_address += sizeof( float ); // FLOAT32    deg
        float misalginment_walking_averaged_ = GET_VALUE( float, e_address ); e_address += sizeof( float ); // FLOAT32    deg
        float misalginment_walking_snap_ = GET_VALUE( float, e_address ); e_address += sizeof( float ); // FLOAT32    deg
        float primary_device_heading_ = GET_VALUE( float, e_address ); e_address += sizeof( float ); // FLOAT32    deg
        float roll_misalignment_ = GET_VALUE( float, e_address ); e_address += sizeof( float ); // FLOAT32    deg
        float pitch_misalignment_ = GET_VALUE( float, e_address ); e_address += sizeof( float ); //     FLOAT32    deg

        ent.m_internal_roll = GET_VALUE( double, e_address0 + 109 ); //29
        ent.m_internal_pitch = GET_VALUE( double, e_address0 + 117 ); //30
        // ...
        ent.m_att_filter_roll = GET_VALUE( float, e_address0 + 249 ); //49
        ent.m_att_filter_pitch = GET_VALUE( float, e_address0 + 253 ); //50
        ent.m_att_filter_heading = GET_VALUE( float, e_address0 + 257 ); //51
        ent.m_att_filter_roll_std = GET_VALUE( float, e_address0 + 261 ); //52
        ent.m_att_filter_pitch_std = GET_VALUE( float, e_address0 + 265 ); //53
        ent.m_att_filter_heading_std = GET_VALUE( float, e_address0 + 269 ); //54

        ent.barometer_filter_height_ = GET_VALUE(float, e_address0 + 310); //65
        ent.barometer_filter_height_standard_deviation_ = GET_VALUE(float, e_address0 + 314); //66

        result = true;
    }

    return result;
}


bool tpn_packet_parser::get_entity( tpn_entity_ble_scan_t &ent, uint16_t entity_index )
{
    bool result = false;

    if (uint8_t* e_address = this->get_entity_address(ID_BLE_SCAN_T, entity_index))
    {
        ent.timestamp = GET_VALUE(uint_fast64_t, e_address); e_address += sizeof(uint_fast64_t);
        ent.mac = GET_VALUE(uint_fast64_t, e_address); e_address += sizeof(uint_fast64_t);
        ent.has_MAC = GET_VALUE(uint8_t, e_address); e_address += sizeof(uint8_t);
        ent.frequency = GET_VALUE(uint16_t, e_address); e_address += sizeof(uint16_t);
        ent.major = GET_VALUE(uint16_t, e_address); e_address += sizeof(uint16_t);
        ent.minor = GET_VALUE(uint16_t, e_address); e_address += sizeof(uint16_t);
        ent.uuid_h = GET_VALUE(uint64_t, e_address); e_address += sizeof(uint64_t);
        ent.uuid_l = GET_VALUE(uint64_t, e_address); e_address += sizeof(uint64_t);
        ent.tx_power = GET_VALUE(int8_t, e_address); e_address += sizeof(int8_t);
        ent.rssi = GET_VALUE(int8_t, e_address); e_address += sizeof(int8_t);

        result = true;
    }
    return result;
}

bool tpn_packet_parser::get_entity( tpn_entity_wifi_scan_t &ent, uint16_t entity_index )
{
    bool result = false;

    if (uint8_t* e_address = this->get_entity_address(ID_WIFI_SCAN_T, entity_index))
    {
        ent.timestamp = GET_VALUE(uint_fast64_t, e_address); e_address += sizeof(uint_fast64_t);
        ent.mac = GET_VALUE(uint_fast64_t, e_address); e_address += sizeof(uint_fast64_t);
        ent.rssi = GET_VALUE(int8_t, e_address); e_address += sizeof(int8_t);
        ent.frequency = GET_VALUE(uint16_t, e_address); e_address += sizeof(uint16_t);

        result = true;
    }
    return result;
}

bool tpn_packet_parser::get_entity(tpn_entity_gnss_pvt_t &ent)
{
    bool result = false;

    if (uint8_t* e_address = this->get_entity_address(ID_GNSS_PVT_T))
    {
        ent.timestamp = GET_VALUE(double, e_address + 0); e_address += sizeof(double);
        ent.latitude = GET_VALUE(double, e_address); e_address += sizeof(double); 
        ent.longitude = GET_VALUE(double, e_address); e_address += sizeof(double);
        ent.height = GET_VALUE(float, e_address); e_address += sizeof(float);
        ent.velocity_north = GET_VALUE(float, e_address); e_address += sizeof(float);
        ent.velocity_east = GET_VALUE(float, e_address); e_address += sizeof(float);
        ent.velocity_down = GET_VALUE(float, e_address); e_address += sizeof(float);
        ent.pos_north_std = GET_VALUE(float, e_address); e_address += sizeof(float);
        ent.pos_east_std = GET_VALUE(float, e_address); e_address += sizeof(float);
        ent.height_std = GET_VALUE(float, e_address); e_address += sizeof(float);
        ent.vel_north_std = GET_VALUE(float, e_address); e_address += sizeof(float);
        ent.vel_east_std = GET_VALUE(float, e_address); e_address += sizeof(float);
        ent.vel_down_std = GET_VALUE(float, e_address); e_address += sizeof(float);
        ent.dop_data_available = GET_VALUE(uint8_t, e_address); e_address += sizeof(uint8_t);
        ent.horizontal_dop = GET_VALUE(float, e_address); e_address += sizeof(float);
        ent.vertical_dop = GET_VALUE(float, e_address); e_address += sizeof(float);

        result = true;
    }
    
    return result;
}

bool tpn_packet_parser::get_entity_mode_of_transit(tpn_entity_mode_of_transit &ent)
{
    bool result = false;
    
    if (uint8_t* e_address = this->get_entity_address(ID_MODE_OF_TRANSIT))
    {
        ent.mode = GET_VALUE(uint8_t, e_address);
        result = true;
    }

    return result;
}

bool tpn_packet_parser::get_entity(tpn_entity_barometer_data &ent)
{
    bool result = false;

    if (uint8_t* e_address = this->get_entity_address(ID_BAROMETER_DATA))
    {
        ent.height_ = GET_VALUE(float, e_address + 0); e_address += sizeof(double);
        ent.height_standard_deviation_ = GET_VALUE(float, e_address); e_address += sizeof(double);
        result = true;
    }

    return result;
}

bool tpn_packet_parser::get_entity(tpn_entity_barometer_height &ent)
{
    bool result = false;

    if (uint8_t* e_address = this->get_entity_address(ID_BAROMETER_DATA))
    {
        ent.barometer_filter_height_ = GET_VALUE(float, e_address + 0); e_address += sizeof(double);
        ent.barometer_filter_height_standard_deviation_ = GET_VALUE(float, e_address); e_address += sizeof(double);
        result = true;
    }

    return result;
}