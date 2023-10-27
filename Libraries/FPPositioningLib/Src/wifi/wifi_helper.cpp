/**
* \copyright       Copyright (C) SPIRIT Navigation, LLC., 2014
* \brief           Reading WiFi logs, buffering and filtering wifi data
* \file            wifi_helper.cpp
* \ingroup         WiFi
* \author          M. Frolov
* \date            28.11.2014
*/

#include "wifi_helper.hpp"
#include <iostream>
#include <fstream>

#ifdef _MSC_VER
#   pragma warning( push )
#   pragma warning( pop )
#   pragma warning( disable : 4201 )
#   pragma warning( disable : 4505 )
#endif
#   include "dirent.h"

#define FIFO_PUSH_SIZE_MAX 20
#define SOLUTION_FIFO_PUSH_SIZE_MAX 5

WiFi_Helper::WiFi_Helper(const std::string &wifi_log )
{
    open( wifi_log );
}

bool WiFi_Helper::open(const std::string &wifi_log )
{
    fname = wifi_log;
    tmp_meas.clear();

    if( is.is_open() )
    {
        is.close();
        is.clear();
    }

    is.open( fname.c_str(), std::ios::in );
    current_n = -1;
    current_timestamp = 0;

    return is.good();
}

#ifdef _MSC_VER
#pragma warning( push )
#pragma warning( disable : 4706 )
#endif
/*
bool WiFi_Helper::openDIR( std::string dir_log, const std::string &prefix )
{
    DIR *d;
    struct dirent *de = NULL;
    bool success = false;
    std::vector<std::string> files;

    if( ( dir_log.find_last_of( "/" ) != ( dir_log.size() - 1 ) ) &&
            ( dir_log.find_last_of( "\\" ) != ( dir_log.size() - 1 ) ) )
    {
        dir_log.append( "/" );
    }

    d = opendir( dir_log.c_str() );

    if( d != NULL )
    {
        while( ( de = readdir( d ) ) != NULL )
        {
            if( DT_REG == de->d_type )
            {
                files.push_back( de->d_name );
            }
        }

        for( std::vector<std::string>::iterator it = files.begin(); it != files.end(); ++it )
        {
            size_t pos = ( *it ).find( prefix );

            if( pos == 0 )
            {
                success = open( dir_log + ( *it ) );
                break;
            }
        }

        closedir( d );
    }

    return success;
}
*/
#ifdef _MSC_VER
#pragma warning( pop )
#endif
bool WiFi_Helper::readmeas( int64_t &timestamp, int &n, WiFi_Measurement &meas )
{
    M m;
    bool  last = false;
    bool ok = false;
    n = 0;

    while( fifo.size() < 2 && last == false )
    {
        last = readNext( m.t, m.n , m.data );

        if( m.data.size() > 0 )
        {
            fifo.push( m );
        }
    }

    if( fifo.size() > 0 )
    {
        m = fifo.front();
        timestamp = m.t;
        n = m.n;
        meas = m.data;
        fifo.pop();
        ok = true;
    }

    return ok;
}
/*
void WiFi_Helper::readall()
{
    M m;
    bool  last = false;

    while( last == false )
    {
        last = readNext( m.t, m.n , m.data );

        if ( m.data.size() > 0 ) fifo.push( m );
    }
}
*/
void WiFi_Helper::pushMeas( int64_t timestamp, int n, const WiFi_Measurement &meas )
{
    M m;
    m.n = n;
    m.t = timestamp;
    m.data = meas;

    fifo.push( m );

    if ( fifo.size() > FIFO_PUSH_SIZE_MAX )
    {
        fifo.pop();
    }
}

void WiFi_Helper::pushWiFiSolution(int64_t timestamp, const WiFi_Location &wifi_pos)
{
    Location_and_time loc_time;
    loc_time.location = wifi_pos;
    loc_time.t = timestamp;
    wifi_pos_queue.push(loc_time);

    if (wifi_pos_queue.size() > SOLUTION_FIFO_PUSH_SIZE_MAX)
    {
        wifi_pos_queue.pop();
    }
}

void WiFi_Helper::pushWiFiSolution(int64_t timestamp, const std::vector<WiFi_Location> &loc_list)
{
    Multi_location_and_time m_loc_time;
    m_loc_time.location_list = loc_list;
    m_loc_time.t = timestamp;
    wifi_multi_pos_queue.push(m_loc_time);

    if (wifi_multi_pos_queue.size() > SOLUTION_FIFO_PUSH_SIZE_MAX)
    {
        wifi_multi_pos_queue.pop();
    }
}

bool WiFi_Helper::readWiFiSolution(int64_t &timestamp, WiFi_Location &wifi_pos)
{
    Location_and_time loc_time;

    bool ok = false;
    if( wifi_pos_queue.size() > 0 )
    {
        loc_time = wifi_pos_queue.front();
        wifi_pos = loc_time.location;
        timestamp = loc_time.t;
        wifi_pos_queue.pop();
        ok = true;
    }

    return ok;
}

bool WiFi_Helper::readWiFiSolution(int64_t &timestamp, std::vector<WiFi_Location> &loc_list)
{
    Multi_location_and_time m_loc_time;

    bool ok = false;
    if (wifi_multi_pos_queue.size() > 0)
    {
        m_loc_time = wifi_multi_pos_queue.front();
        loc_list = m_loc_time.location_list;
        timestamp = m_loc_time.t;
        wifi_multi_pos_queue.pop();
        ok = true;
    }

    return ok;
}

bool WiFi_Helper::getNextTimestamp( int64_t &timestamp )
{
    M m;
    bool ok = false;

    if( fifo.size() > 0 )
    {
        m = fifo.front();
        timestamp = m.t;
        ok = true;
    }

    return ok;
}


bool WiFi_Helper::readNext( int64_t &timestamp, int &n, WiFi_Measurement &meas )
{
    int linenumber = current_n;
    std::string line;
    std::vector<std::string> tokens;
    std::stringstream ss;
    int64_t t;
    int level1, level2;
    std::string mac, ssid;
    bool is_first_line = ( tmp_meas.size() == 0 );
    meas.clear();

    while( linenumber == current_n && !is.eof() && is.is_open() )
    {
        std::getline( is, line );
        tokens = parse( line, ',' );

        if( tokens.size() < 6 )
        {
            continue;
        }

        ss.clear();
        ss.str( tokens[0] );
        ss >> t;

        ss.clear();
        ss.str( tokens[1] );
        ss >> linenumber;

        ss.clear();
        ss.str( tokens[2] );
        ss >> mac;

        //ss.clear();
        ssid = tokens[3];
        int i = tokens[3].find_first_not_of( " " );

        if( i >= 0 )
        {
            ssid = tokens[3].substr( i, tokens[3].length() );
        }

        ss.clear();
        ss.str( tokens[5] );
        ss >> level1;
        level2 = level1;

        if( tokens.size() > 6 )
        {
            ss.clear();
            ss.str( tokens[6] );
            ss >> level2;
        }

        //new meas
        if( linenumber != current_n && is_first_line == false )
        {
            meas = tmp_meas;
            tmp_meas.clear();
            n = current_n;
            timestamp = current_timestamp;
            current_n = linenumber;

            WiFi_ApInfo ap;
            ap.bssid = WiFi_ApInfo::string_to_mac( mac );
            ap.rssi = level2;
            tmp_meas.push_back( ap );
            current_timestamp = t;

            //std::cout << std::endl;
            //std::cout << t << " "
            //          << linenumber << " "
            //          << mac << " "
            //          << ssid << " "
            //          << level1 << " "
            //          << level2 << "\n";

            break;
        }

        WiFi_ApInfo ap;
        ap.bssid = WiFi_ApInfo::string_to_mac( mac );
        ap.rssi = level2;
        tmp_meas.push_back( ap );
        current_timestamp = t;
        current_n = linenumber;
        is_first_line = false;

        //std::cout << t << " "
        //          << linenumber << " "
        //          << mac << " "
        //          << ssid << " "
        //          << level1 << " "
        //          << level2 << "\n";


        if( is.eof() )
        {
            meas = tmp_meas;
            tmp_meas.clear();
            n = current_n;
            timestamp = current_timestamp;
        }


    }

    if( is.eof() && tmp_meas.size() > 0 )
    {
        meas = tmp_meas;
        tmp_meas.clear();
        n = current_n;
        timestamp = current_timestamp;
    }

    return is.eof() || ( !is.is_open() );

}


std::vector<std::string> WiFi_Helper::parse( const std::string &s, const char &delim ) const
{
    std::stringstream buf( s );
    std::string token;
    std::vector<std::string> result;


    while( std::getline( buf, token, delim ) )
    {
        result.push_back( token );
    }

    return result;
}