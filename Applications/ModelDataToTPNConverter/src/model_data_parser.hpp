#ifndef MODEL_DATA_PARSER_HPP
#define MODEL_DATA_PARSER_HPP

#include <stdint.h>

class model_data_parser
{
public:
	model_data_parser();
	model_data_parser(uint8_t* tpn_packet, uint16_t tpn_packet_size);
	bool set_packet_data(uint8_t* tpn_packet, uint16_t tpn_packet_size);
	~model_data_parser();
	bool is_state_correct();


private:
	uint8_t* get_entity(uint16_t id);

private:
	uint16_t sz;
	uint8_t* data;
	//tpn_packet_header_data_t header_data;

};

#endif // MODEL_DATA_PARSER_HPP
