#ifndef TPN_ENTITY_CREATOR_HPP
#define TPN_ENTITY_CREATOR_HPP

#include <vector>

#include "entity_id_size_data.hpp"
#include "tpn_packet_parser.hpp"

class tpn_entity_creator
{
public:

	tpn_entity_creator();
	~tpn_entity_creator();
	bool is_state_correct();

	bool create_entity_time(const tpn_entity_time_t &ent, entity_id_size_data &entity);

	bool create_entity_position(const tpn_entity_position_t &ent, entity_id_size_data &entity);
	bool create_entity_position_std(const tpn_entity_position_std_t &ent, entity_id_size_data &entity);
	//bool create_entity_velocity(const tpn_entity_velocity_t &ent, entity_id_size_data &entity);
	//bool create_entity_velocity_std(const tpn_entity_velocity_std_t &ent, entity_id_size_data &entity);
	bool create_entity_attitude(const tpn_entity_attitude_t &ent, entity_id_size_data &entity);
	bool create_entity_attitude_standard_deviation(const tpn_entity_attitude_standard_deviation_t &ent, entity_id_size_data &entity);

	bool create_entity_magnetic_data(const tpn_entity_magnetometer_data_t &ent, entity_id_size_data &entity);
	bool create_entity_orientation_pitch_d(const tpn_entity_orientation_pitch_d_t &ent, entity_id_size_data &entity);
	bool create_entity_grp_07(const tpn_entity_grp_07_t &ent, entity_id_size_data &entity);
	bool create_entity_device_heading(const tpn_entity_device_heading_t &ent, entity_id_size_data &entity);
	bool create_entity_internal_dbg(const tpn_entity_internal_dbg_t &ent, entity_id_size_data &entity);
	bool create_entity_heading_misalignment(const tpn_entity_heading_misalignment_t &ent, entity_id_size_data &entity);
	bool create_entity_floor_number(const tpn_entity_floor_number_t &ent, entity_id_size_data &entity);
	bool create_entity_stride_information(const tpn_entity_stride_information_t &ent, entity_id_size_data &entity);
  bool create_entity_wifi_scan(const tpn_entity_wifi_scan_t &ent, entity_id_size_data &entity);
  bool create_entity_ble_scan(const tpn_entity_ble_scan_t &ent, entity_id_size_data &entity);

private:


	uint16_t sz;
	uint8_t* data;

	uint32_t epoch_number;
	
	static const uint16_t sync_size = 3; // bytes
	uint8_t sync_array[sync_size];

	tpn_packet_header_data_t header_data;
	static const uint16_t header_array_size = 12; // bytes
	uint16_t  header_array[header_array_size/2];

	std::vector <uint16_t> entity_ids;
	std::vector <uint16_t> entity_sizes;

	std::vector <entity_id_size_data> entities;

	uint16_t payload_length;
	uint8_t  *body_data;

	static const uint16_t checksum_size = 2; // bytes
	uint8_t  checksum[checksum_size];

};


#endif // TPN_PACKET_CREATOR_HPPP
