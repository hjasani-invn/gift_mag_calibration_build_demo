#ifndef TPN_PACKET_CREATOR_HPP
#define TPN_PACKET_CREATOR_HPP
#include <vector>
#include "entity_id_size_data.hpp"
#include "tpn_packet_parser.hpp"
class tpn_packet_creator
{
public:
	tpn_packet_creator();
//	tpn_packet_creator(uint8_t* tpn_packet, uint16_t tpn_packet_size);
//	bool set_packet_data(uint8_t* tpn_packet, uint16_t tpn_packet_size);
	~tpn_packet_creator();
	bool is_state_correct();
	bool set_epoch_number(uint32_t ent);
	bool create_packet(const std::vector <entity_id_size_data> &entities);
	bool get_packet_data(uint8_t** tpn_packet, uint16_t &tpn_packet_size);
private:
	//uint8_t* get_entity(uint16_t id, uint8_t* entity, uint16_t &size);
	void create_packet_synchronization();
	void create_packet_header(const std::vector <entity_id_size_data> &entities);
	void create_packet_body(const std::vector <entity_id_size_data> &entities);
	void create_packet_checksum();
private:
	uint16_t sz;
	uint8_t* data;
 	uint32_t epoch_number;
	uint16_t payload_length;

	static const uint16_t sync_size = 3; // bytes
	uint8_t sync_array[sync_size];
	//tpn_packet_header_data_t header_data;
	static const uint16_t header_array_size = 12; // bytes
	uint16_t  header_array[header_array_size/2];
	std::vector <uint16_t> entity_ids;
	std::vector <uint16_t> entity_sizes;
	std::vector <entity_id_size_data> entities;
	uint8_t  *body_data;
	//uint16_t body_data_size;
	static const uint16_t checksum_size = 2; // bytes
	uint8_t  checksum[checksum_size];
};
#endif // TPN_PACKET_CREATOR_HPPP
