/*****************************************************************************/
/**
* @copyright   Copyright © InvenSense, Inc. 2016
* @project     RTFPPL
* @file        tpn_data_reader.cpp
* @author      D Churikov
* @date        13 Apr 2016
* @brief       TPN data reader
*/
/*****************************************************************************/

#include"tpn_data_reader.hpp"

//---------------------------------------------------------------------------
bool tpn_data_reader::read_file_header( /*TODO: tpn_file_header &file_header*/ )
{
    // TO DO: read file header
    uint8_t buffer[515];
    is->read( ( char* )buffer, 515 ); // read and skip file header packet

    return true;
}

//---------------------------------------------------------------------------
bool tpn_data_reader::read_next_packet( tpn_packet_parser &tpn_parser )
{

    bool result = false;

    if ( this->is_state_correct() )
    {
        if ( this->find_preambule() )
        {
            uint16_t packet_data_size;

            if ( !is->read( ( char* )&packet_data_size, sizeof( packet_data_size ) ).eof() ) // read packet data size
            {
                uint16_t packet_size = packet_data_size + 15 - 3 - 2;

                if ( uint8_t *buffer = new uint8_t[packet_size] ) // create packet buffer
                {
                    if ( !is->read( ( char* )buffer, packet_size * sizeof( uint8_t ) ).eof() ) // read packet
                    {
                        if ( tpn_parser.set_packet_data( buffer, packet_size ) ) // set packet data
                        {
                            uint16_t check_sum;

                            if ( !is->read( ( char* )&check_sum, sizeof( check_sum ) ).eof() ) // read check sum
                            {
                                // TODO: calck & check checksum
                                result = true;
                            }
                        }
                    }

                    delete [] buffer;
                }
            }
        }
    }

    return result;
}

//---------------------------------------------------------------------------
bool tpn_data_reader::find_preambule()
{
    uint8_t buffer[3];
    buffer[0] = buffer[1] = buffer[2] = 0;

    while ( !is->eof() )
    {
        is->read( ( char* )&buffer[2], 1 );

        if ( ( buffer[0] == 0x54 ) && ( buffer[1] == 0x50 ) && ( buffer[2] == 0x49 ) )
            return true;

        buffer[0] = buffer[1];
        buffer[1] = buffer[2];
    }

    return false;
}