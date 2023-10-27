
#include"model_data_parser.hpp"


model_data_parser::model_data_parser()
{

}

model_data_parser::model_data_parser(uint8_t* tpn_packet, uint16_t tpn_packet_size)
{

}


bool model_data_parser::set_packet_data(uint8_t* tpn_packet, uint16_t tpn_packet_size)
{
	return true;
}

model_data_parser::~model_data_parser()
{

}
bool model_data_parser::is_state_correct()
{
	return true;
}
/*
bool model_data_parser::set_packet_header_data(tpn_packet_header_data_t &packet_header_data)
{
	return true;
}

bool model_data_parser::set_entity_time(tpn_entity_time_t &ent)
{
	return true;
}

bool model_data_parser::set_entity_position(tpn_entity_position_t &ent)
{
	return true;
}
bool model_data_parser::set_entity_position_std(tpn_entity_position_std_t &ent)
{
	return true;
}
bool model_data_parser::set_entity_velocity(tpn_entity_velocity_t &ent)
{
	return true;
}
bool model_data_parser::set_entity_velocity_std(tpn_entity_velocity_std_t &ent)
{
	return true;
}
bool model_data_parser::set_entity_attitude(tpn_entity_attitude_t &ent)
{
	return true;
}
bool model_data_parser::set_entity_magnetic_data(tpn_entity_magnetometer_data_t &ent)
{
	return true;
}
bool model_data_parser::set_entity_orientation_pitch_d(tpn_entity_orientation_pitch_d_t &ent)
{
	return true;
}
bool model_data_parser::set_entity_grp_07(tpn_entity_grp_07_t &ent)
{
	return true;
}
bool model_data_parser::set_entity_internal_dbg(tpn_entity_internal_dbg_t &ent)
{
	return true;
}
bool model_data_parser::set_entity_device_heading(tpn_device_heading &ent)
{
	return true;
}
bool model_data_parser::set_entity_heading_misalignment(tpn_heading_misalignment &ent)
{
	return true;
}
bool model_data_parser::set_entity_attitude_standard_deviation(tpn_entity_attitude_standard_deviation_t &ent)
{
	return true;
}
*/
