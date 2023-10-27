
#include <cstddef>
#include <iostream>

#include"tpn_entity_creator.hpp"

tpn_entity_creator::tpn_entity_creator()
{
}

tpn_entity_creator::~tpn_entity_creator()
{

}
bool tpn_entity_creator::is_state_correct()
{
	return true;
}

bool tpn_entity_creator::create_entity_time(const tpn_entity_time_t &ent, entity_id_size_data &entity)
{	
	entity.id = ID_TIME;
	entity.size = sizeof(ent.timetag_);
	entity.data = new uint8_t[entity.size];
	uint8_t* data = entity.data;
	uint8_t* src = (uint8_t*)&ent.timetag_;
	memcpy(data, src, entity.size);

	return true;
}

bool tpn_entity_creator::create_entity_position(const tpn_entity_position_t &ent, entity_id_size_data &entity)
{
	entity.id = ID_POSITION;
	uint16_t size1 = sizeof(ent.latitude_);
	uint16_t size2 = sizeof(ent.longitude_);
	uint16_t size3 = sizeof(ent.height_);
	entity.size = size1 + size2 + size3;
	entity.data = new uint8_t[entity.size];
	uint8_t* data = entity.data;
	uint8_t* src = (uint8_t*)&ent.latitude_;
	memcpy(data, src, size1);
	data += size1;
	src = (uint8_t*)&ent.longitude_;
	memcpy(data, src, size2);
	data += size2;
	src = (uint8_t*)&ent.height_;
	memcpy(data, src, size3);
	data += size3;

	return true;
}
bool tpn_entity_creator::create_entity_position_std(const tpn_entity_position_std_t &ent, entity_id_size_data &entity)
{
	entity.id = ID_POSITION_STD;
	uint16_t size1 = sizeof(ent.position_north_standard_deviation_);
	uint16_t size2 = sizeof(ent.position_east_standard_deviation_);
	uint16_t size3 = sizeof(ent.height_standard_deviation_);
	entity.size = size1 + size2 + size3;
	entity.data = new uint8_t[entity.size];
	uint8_t* data = entity.data;
	uint8_t* src = (uint8_t*)&ent.position_north_standard_deviation_;
	memcpy(data, src, size1);
	data += size1;
	src = (uint8_t*)&ent.position_east_standard_deviation_;
	memcpy(data, src, size2);
	data += size2;
	src = (uint8_t*)&ent.height_standard_deviation_;
	memcpy(data, src, size3);
	data += size3;

	return true;
}
/*
bool tpn_entity_creator::create_entity_velocity(const tpn_entity_velocity_t &ent, entity_id_size_data &entity)
{
	return true;
}
bool tpn_entity_creator::create_entity_velocity_std(const tpn_entity_velocity_std_t &ent, entity_id_size_data &entity)
{
	return true;
}
*/
bool tpn_entity_creator::create_entity_attitude(const tpn_entity_attitude_t &ent, entity_id_size_data &entity)
{  
	entity.id = ID_ATTITUDE;
	uint16_t size1 = sizeof(ent.roll_);
	uint16_t size2 = sizeof(ent.pitch_);
	uint16_t size3 = sizeof(ent.heading_);
	entity.size = size1 + size2 + size3;
	entity.data = new uint8_t[entity.size];
	uint8_t* data = entity.data;
	uint8_t* src = (uint8_t*)&ent.roll_;
	memcpy(data, src, size1);
	data += size1;
	src = (uint8_t*)&ent.pitch_;
	memcpy(data, src, size2);
	data += size2;
	src = (uint8_t*)&ent.heading_;
	memcpy(data, src, size3);
	data += size3;

	return true;
}

bool tpn_entity_creator::create_entity_attitude_standard_deviation(const tpn_entity_attitude_standard_deviation_t &ent,
	                                                               entity_id_size_data &entity)
{
	entity.id = ID_ATTITUDE_STANDARD_DEVIATION;
	uint16_t size1 = sizeof(ent.roll_standard_deviation_);
	uint16_t size2 = sizeof(ent.pitch_standard_deviation_);
	uint16_t size3 = sizeof(ent.heading_standard_deviation_);
	entity.size = size1 + size2 + size3;
	entity.data = new uint8_t[entity.size];
	uint8_t* data = entity.data;
	uint8_t* src = (uint8_t*)&ent.roll_standard_deviation_;
	memcpy(data, src, size1);
	data += size1;
	src = (uint8_t*)&ent.pitch_standard_deviation_;
	memcpy(data, src, size2);
	data += size2;
	src = (uint8_t*)&ent.heading_standard_deviation_;
	memcpy(data, src, size3);
	data += size3;

	return true;
}

bool tpn_entity_creator::create_entity_magnetic_data(const tpn_entity_magnetometer_data_t &ent, entity_id_size_data &entity)
{
	entity.id = ID_MAGNETOMETER_DATA;
	uint16_t size1 = sizeof(ent.raw_data_available_);
	uint16_t size2 = sizeof(ent.raw_data_x_);
	uint16_t size3 = sizeof(ent.raw_data_y_);
	uint16_t size4 = sizeof(ent.raw_data_z_);
	uint16_t size5 = sizeof(ent.raw_data_accuracy_flag_);

	uint16_t size6 = sizeof(ent.calibrated_data_available_);
	uint16_t size7 = sizeof(ent.calibrated_data_x_);
	uint16_t size8 = sizeof(ent.calibrated_data_y_);
	uint16_t size9 = sizeof(ent.calibrated_data_z_);
	uint16_t size10 = sizeof(ent.calibrated_data_accuracy_flag_);

	entity.size = size1 + size2 + size3 + size4 + size5 +
		size6 + size7 + size8 + size9 + size10;
	entity.data = new uint8_t[entity.size];
	uint8_t* data = entity.data;
	*data++ = ent.raw_data_available_; // 1 byte
	uint8_t* src = (uint8_t*)&ent.raw_data_x_;
	memcpy(data, src, size2);
	data += size2;
	src = (uint8_t*)&ent.raw_data_y_;
	memcpy(data, src, size3);
	data += size3;
	src = (uint8_t*)&ent.raw_data_z_;
	memcpy(data, src, size4);
	data += size4;
	*data++ = ent.raw_data_accuracy_flag_; // 1 byte
	*data++ = ent.calibrated_data_available_; // byte

	src = (uint8_t*)&ent.calibrated_data_x_;
	memcpy(data, src, size7);
	data += size7;
	src = (uint8_t*)&ent.calibrated_data_y_;
	memcpy(data, src, size8);
	data += size8;
	src = (uint8_t*)&ent.calibrated_data_z_;
	memcpy(data, src, size9);
	data += size9;
	*data++ = ent.calibrated_data_accuracy_flag_; // 1 byte

	return true;
}

bool tpn_entity_creator::create_entity_orientation_pitch_d(const tpn_entity_orientation_pitch_d_t &ent, entity_id_size_data &entity)
{
	entity.id = ID_ORIENTATION_PITCH_D_T;
	entity.size = sizeof(ent.m_orientation);
	entity.data = new uint8_t[entity.size];
	uint8_t* data = entity.data;
	uint8_t* src = (uint8_t*)&ent.m_orientation;
	memcpy(data, src, entity.size);

	return true;
}
bool tpn_entity_creator::create_entity_grp_07(const tpn_entity_grp_07_t &ent, entity_id_size_data &entity)
{
	entity.id = ID_GRP_07_T;
	entity.size = sizeof(ent.m_nav_phase);
	entity.data = new uint8_t[entity.size];
	uint8_t* data = entity.data;
	uint8_t* src = (uint8_t*)&ent.m_nav_phase;
	memcpy(data, src, entity.size);

	return true;
}

bool tpn_entity_creator::create_entity_device_heading(const tpn_entity_device_heading_t &ent, entity_id_size_data &entity)
{	
	entity.id = ID_DEVICE_HEADING;
	uint16_t size1 = sizeof(ent.device_heading_available_);
	uint16_t size2 = sizeof(ent.device_heading_);
	uint16_t size3 = sizeof(ent.platform_heading_);
	uint16_t size4 = sizeof(ent.misalignment_angle_);

	entity.size = size1 + size2 + size3 + size4;
	entity.data = new uint8_t[entity.size];
	uint8_t* data = entity.data;
	*data++ = ent.device_heading_available_; // 1 byte
	uint8_t* src = (uint8_t*)&ent.device_heading_;
	memcpy(data, src, size2);
	data += size2;
	src = (uint8_t*)&ent.platform_heading_;
	memcpy(data, src, size3);
	data += size3;
	src = (uint8_t*)&ent.misalignment_angle_;
	memcpy(data, src, size4);
	data += size4;
	
	return true;
}


bool tpn_entity_creator::create_entity_internal_dbg(const tpn_entity_internal_dbg_t &ent, entity_id_size_data &entity)
{	
	entity.id = ID_INTERNAL_DBG_T;
	uint16_t size1 = sizeof(ent.m_imu_smp_proc_time);
	uint16_t size2 = sizeof(ent.m_orientation_on_pitch);
  uint16_t size2_1 = sizeof(ent.walking_fidgeting_flag);
	uint16_t size3 = sizeof(ent.m_internal_roll);
	uint16_t size4 = sizeof(ent.m_internal_pitch);
	uint16_t size5 = sizeof(ent.m_att_filter_roll);
	uint16_t size6 = sizeof(ent.m_att_filter_pitch);
	uint16_t size7 = sizeof(ent.m_att_filter_heading);
	uint16_t size8 = sizeof(ent.m_att_filter_roll_std);
	uint16_t size9 = sizeof(ent.m_att_filter_pitch_std);
	uint16_t size10 = sizeof(ent.m_att_filter_heading_std);

	//ent.size = size1 + size2 + size3 + size4 + size5 +
	//	       size6 + size7 + size8 + size9 + size10;
	entity.size = 378; // 1000;
	entity.data = new uint8_t[entity.size];
	uint8_t* data = entity.data;
	memset(data, 0, entity.size);
	uint8_t* src = (uint8_t*)&ent.m_imu_smp_proc_time;
	memcpy(data, src, size1);
	data += size1;
	
	data += 32; // radiusis of rotation are skiped
	src = (uint8_t*)&ent.m_orientation_on_pitch;
	memcpy(data, src, size2);
	data += size2;

  data += 2; // radiusis of rotation are skiped
  src = (uint8_t*)&ent.walking_fidgeting_flag;
  memcpy(data, src, size2_1);
  data += size2_1;
  
	data += 69;	  // fields are skiped
	src = (uint8_t*)&ent.m_internal_roll;
	memcpy(data, src, size3);
	data += size3;
	
	src = (uint8_t*)&ent.m_internal_pitch;
	memcpy(data, src, size4);
	data += size4;
	
	data += 124;	   // fields are skiped
	src = (uint8_t*)&ent.m_att_filter_roll;
	memcpy(data, src, size5);
	data += size5;
	
	src = (uint8_t*)&ent.m_att_filter_pitch;
	memcpy(data, src, size6);
	data += size6;
	
	src = (uint8_t*)&ent.m_att_filter_heading;
	memcpy(data, src, size7);
	data += size7;
	
	src = (uint8_t*)&ent.m_att_filter_roll_std;
	memcpy(data, src, size8);
	data += size8;
	
	src = (uint8_t*)&ent.m_att_filter_pitch_std;
	memcpy(data, src, size9);
	data += size9;
	src = (uint8_t*)&ent.m_att_filter_heading_std;
	memcpy(data, src, size10);
	data += size10;
	data += 105; // fields are skiped
	//*data = 1; // control writing to know array size

	return true;
}


bool tpn_entity_creator::create_entity_heading_misalignment(const tpn_entity_heading_misalignment_t &ent, entity_id_size_data &entity)
{
	entity.id = ID_HEADING_MISSALIGNMENT;
	entity.size = sizeof(ent.heading_misalignment_);
	entity.data = new uint8_t[entity.size];
	uint8_t* data = entity.data;
	uint8_t* src = (uint8_t*)&ent.heading_misalignment_;
	memcpy(data, src, entity.size);

	return true;
}

bool tpn_entity_creator::create_entity_floor_number(const tpn_entity_floor_number_t &ent, entity_id_size_data &entity)
{
	entity.id = ID_FLOOR_NUMBER;
	entity.size = sizeof(ent.floor_number_);
	entity.data = new uint8_t[entity.size];
	uint8_t* data = entity.data;
	uint8_t* src = (uint8_t*)&ent.floor_number_;
	memcpy(data, src, entity.size);

	return true;
}

bool tpn_entity_creator::create_entity_stride_information(const tpn_entity_stride_information_t &ent, entity_id_size_data &entity)
{
	entity.id = ID_STRIDE_INFORMATION;
	uint16_t size1 = sizeof(ent.stride_distance_);
	uint16_t size2 = sizeof(ent.stride_velocity_);
	entity.size = size1 + size2;
	entity.data = new uint8_t[entity.size];
	uint8_t* data = entity.data;
	uint8_t* src = (uint8_t*)&ent.stride_distance_;
	memcpy(data, src, size1);
	data += size1;
	src = (uint8_t*)&ent.stride_velocity_;
	memcpy(data, src, size2);
	data += size2;

	return true;
}

bool tpn_entity_creator::create_entity_wifi_scan(const tpn_entity_wifi_scan_t &ent, entity_id_size_data &entity)
{
    entity.id = ID_WIFI_SCAN_T;
    uint16_t size1 = sizeof(ent.timestamp);
    uint16_t size2 = sizeof(ent.mac);
    uint16_t size3 = sizeof(ent.rssi);
    uint16_t size4 = sizeof(ent.frequency);

    entity.size = size1 + size2 + size3 + size4;
    entity.data = new uint8_t[entity.size];
    uint8_t* data = entity.data;

    uint8_t* src = (uint8_t*)&ent.timestamp;
    memcpy(data, src, size1);
    data += size1;

    src = (uint8_t*)&ent.mac;
    memcpy(data, src, size2);
    data += size2;

    src = (uint8_t*)&ent.rssi;
    memcpy(data, src, size3);
    data += size3;

    src = (uint8_t*)&ent.frequency;
    memcpy(data, src, size4);
    data += size4;

    return true;
}

bool tpn_entity_creator::create_entity_ble_scan(const tpn_entity_ble_scan_t &ent, entity_id_size_data &entity)
{
    entity.id = ID_BLE_SCAN_T;
    uint16_t size1 = sizeof(ent.timestamp);
    uint16_t size2 = sizeof(ent.mac);
    uint16_t size3 = sizeof(ent.has_MAC);
    uint16_t size4 = sizeof(ent.frequency);
    uint16_t size5 = sizeof(ent.major);
    uint16_t size6 = sizeof(ent.minor);
    uint16_t size7 = sizeof(ent.uuid_h);
    uint16_t size8 = sizeof(ent.uuid_l);
    uint16_t size9 = sizeof(ent.tx_power);
    uint16_t size10 = sizeof(ent.rssi);

    entity.size = size1 + size2 + size3 + size4 + size5 + size6 + size7 + size8 + size9 + size10;
    entity.data = new uint8_t[entity.size];
    uint8_t* data = entity.data;

    uint8_t* src = (uint8_t*)&ent.timestamp;
    memcpy(data, src, size1);
    data += size1;

    src = (uint8_t*)&ent.mac;
    memcpy(data, src, size2);
    data += size2;

    src = (uint8_t*)&ent.has_MAC;
    memcpy(data, src, size3);
    data += size3;

    src = (uint8_t*)&ent.frequency;
    memcpy(data, src, size4);
    data += size4;

    src = (uint8_t*)&ent.major;
    memcpy(data, src, size5);
    data += size5;

    src = (uint8_t*)&ent.minor;
    memcpy(data, src, size6);
    data += size6;

    src = (uint8_t*)&ent.uuid_h;
    memcpy(data, src, size7);
    data += size7;

    src = (uint8_t*)&ent.uuid_l;
    memcpy(data, src, size8);
    data += size8;

    src = (uint8_t*)&ent.tx_power;
    memcpy(data, src, size9);
    data += size9;

    src = (uint8_t*)&ent.rssi;
    memcpy(data, src, size10);
    data += size10;

    return true;
}
