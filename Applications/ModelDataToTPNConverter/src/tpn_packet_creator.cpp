#include <cstddef>

#include"tpn_packet_creator.hpp"

tpn_packet_creator::tpn_packet_creator()
{	
}

tpn_packet_creator::~tpn_packet_creator()
{

}

bool tpn_packet_creator::is_state_correct()
{
	return true;
}

bool tpn_packet_creator::set_epoch_number(uint32_t ep)
{
	epoch_number = ep;
	return true;
}

bool tpn_packet_creator::create_packet(const std::vector <entity_id_size_data> &entities)
{
	create_packet_synchronization();
	create_packet_header(entities);
	create_packet_body(entities);
	create_packet_checksum();

	return true;
}

// private methods
void tpn_packet_creator::create_packet_synchronization()
{
	sync_array[0] = 0x54;
	sync_array[1] = 0x50;
	sync_array[2] = 0x49;
}

/*
struct tpn_packet_header_data_t
{
uint16_t payload_length_;	///< Length of the payload(i.e.packet body) attached to the packet header excluding the packet checksum.
uint16_t packet_id_;	///< packet_id; 0x0003
uint16_t stream_id_;	///< Identifies the stream identification number of the current epoch if more than one output stream exists
uint32_t epoch_number_;	///< Number of the current epoch
uint16_t number_of_entities_;	///< Number of entities(E) within the current epoch.The maximum number of entities is 65, 535. Each entity in the packet has two fields : entity_id_ and entity_start_address_

};
*/
void tpn_packet_creator::create_packet_header(const std::vector <entity_id_size_data> &entities)
{

	payload_length = 0;
	for (auto it = entities.begin();
		it != entities.end(); ++it)
	{
		payload_length += it->size;
	}

	uint16_t number = (uint16_t)entities.size();//number_of_entities_;
	payload_length += 2 * 2 * number;

	header_array[0] = (uint16_t)payload_length;
	header_array[1] = (uint16_t)0x3;
	header_array[2] = (uint16_t)0x0;
	header_array[3] = (uint16_t)(epoch_number & 0xFFFF);
	header_array[4] = (uint16_t)((epoch_number >> 16) & 0xFFFF);
	header_array[5] = (uint16_t)entities.size();//number_of_entities_;
}

/*
entity_id (1)
entity_start_address (1)
.
.
.
entity_id (n)
entity_start_address (n)
entity (1)
.
.
.
entity (n)
*/
void tpn_packet_creator::create_packet_body(const std::vector <entity_id_size_data> &entities)
{
	payload_length = 0;
	for (auto it = entities.begin();
		it != entities.end(); ++it)
	{
		payload_length += it->size;
	}

	uint16_t number = (uint16_t)entities.size();//number_of_entities_;
 	payload_length += 2 * 2 * number;

	body_data = new uint8_t[payload_length];

	uint16_t  *p16_data = (uint16_t *)body_data;

	uint16_t i = 0;
	uint16_t start_adrress = 0;
	for (auto it = entities.begin();
		it != entities.end(); ++it)
	{

		*p16_data++ =  it->id;
		*p16_data++ = start_adrress;
		start_adrress += it->size;
	}

	uint8_t  *p8_data = (uint8_t *)p16_data;
	for (auto it = entities.begin();
		it != entities.end(); ++it)
	{
		memcpy(p8_data, it->data, it->size);
		p8_data += it->size;
	}
}

void tpn_packet_creator::create_packet_checksum()
{
	checksum[0] = 0;
	checksum[1] = 0;
}

bool tpn_packet_creator::get_packet_data(uint8_t** tpn_packet, uint16_t &tpn_packet_size)
{

	tpn_packet_size = sync_size + header_array_size + payload_length + checksum_size;

	*tpn_packet = new  uint8_t[tpn_packet_size];

	uint8_t* p_tpn_packet = *tpn_packet;

	memcpy(p_tpn_packet, sync_array, sizeof(sync_array));
	p_tpn_packet += sizeof(sync_array);

	memcpy(p_tpn_packet, header_array, sizeof(header_array));
	p_tpn_packet += sizeof(header_array);

	memcpy(p_tpn_packet, body_data, payload_length);
	p_tpn_packet += payload_length;

	memcpy(p_tpn_packet, checksum, sizeof(checksum));
	p_tpn_packet += sizeof(checksum);

	return true;
}
