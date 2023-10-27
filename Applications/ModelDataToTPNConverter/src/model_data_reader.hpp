/*****************************************************************************/
/**
* @copyright   Copyright © InvenSense, Inc. 2016
* @project     RTFPPL
* @file        tpn_data_reader.hpp
* @author      D Churikov
* @date        13 Apr 2016
* @brief       TPN data reader interface
*/
/*****************************************************************************/
#ifndef MODEL_DATA_READER_HPP
#define MODEL_DATA_READER_HPP

#include <stdint.h>
#include <string>
#include <fstream>

#include"model_output_data_struct.hpp"

class model_data_reader
{
    public:
        model_data_reader()
        {
			is = NULL;
        }

        ~model_data_reader()
        {
			if (is != NULL)
				is->close();
		}
        void setStream( std::ifstream* ios )
        {
			if (is != NULL)
				is->close();
            is = ios;
        }

        bool is_state_correct()
        {
            return is->is_open() && !is->bad() && !is->eof();
        }

		bool read_next_line(model_output_data &output);
        //bool read_file_header( /*TODO: tpn_file_header &file_header*/ );

    private:
        //bool find_preambule();

    private:
        std::ifstream *is;
        //    uint8_t* packet_buffer;
        //    uint16_t packet_size;
};


#endif