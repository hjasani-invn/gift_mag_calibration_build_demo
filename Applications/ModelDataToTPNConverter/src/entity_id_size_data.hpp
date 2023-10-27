#ifndef ENTITY_ID_SIZE_HPP
#define ENTITY_ID_SIZE_HPP

#include <stdint.h>
#include <string.h>

class entity_id_size_data
{
public:
	entity_id_size_data()
	{
		id = 0;
		size = 0;
		data = NULL;
	}
	entity_id_size_data (const entity_id_size_data &en) {
		this->id = en.id;
		this->size = en.size;
		this->data = new uint8_t[en.size];
		memcpy(this->data, en.data, en.size);
	}
	~entity_id_size_data()
	{
	    if (data != NULL)
		delete[] data;
	}

	uint16_t id;
	uint16_t size;
	uint8_t *data;
};
#endif
