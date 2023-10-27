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
#ifndef TPN_DATA_READER_HPP
#define TPN_DATA_READER_HPP

#include <stdint.h>
#include <string>
#include <fstream>

#include"tpn_packet_parser.hpp"

class tpn_data_reader
{
    public:
        tpn_data_reader() : is(0) {};
        tpn_data_reader(std::ifstream* ios) : is(ios) {};

        ~tpn_data_reader()
        {
        }
        void setStream( std::ifstream* ios )
        {
            is = ios;
        }

        bool is_state_correct()
        {
            bool b1 = is->is_open();
            bool b2 = is->bad();
            bool b3 = is->eof();

            return (is != 0) ?  (b1 && !b2 && !b3) : false;
        }

        bool read_next_packet( tpn_packet_parser &tpn_parser );
        bool read_file_header( /*TODO: tpn_file_header &file_header*/ );

    private:
        bool find_preambule();

    private:
        std::ifstream *is;
        //    uint8_t* packet_buffer;
        //    uint16_t packet_size;
};


#endif