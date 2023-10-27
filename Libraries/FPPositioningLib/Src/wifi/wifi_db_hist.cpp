/**
* \copyright       Copyright (C) SPIRIT Navigation, LLC., 2014
* \brief           Fingerprint storage class. Implements histogram and GM models
* \file            wifi_db_hist.cpp
* \ingroup         WiFi
* \author          M. Frolov
* \date            28.11.2014
*/

#define _CRT_SECURE_NO_WARNINGS
#include <fstream>
#include <sstream>
#include <string>
#include <cstring>  // for gcc
#include "wifi_db_hist.hpp"
#include "wifi_loc.hpp"
#include "median.h"
#include <streambuf>


WiFi_DB_HIST::WiFi_DB_HIST(std::string db_name) : bssid(0)
{
    const WiFi_DB db_t( db_name );
    readFormatDB( &db_t );
}

WiFi_DB_HIST::WiFi_DB_HIST(const WiFi_DB *db) : bssid(0)
{
    readFormatDB( db );
}


WiFi_DB_HIST::const_iterator WiFi_DB_HIST::begin() const
{
    return db.begin();
}

WiFi_DB_HIST::const_iterator WiFi_DB_HIST::end() const
{
    return db.end();
}

size_t WiFi_DB_HIST::size() const
{
    return db.size();
}

size_t WiFi_DB_HIST::getAPpCount()
{
    size_t sz = 0;
    if (db.size() > 0)
    {
        sz = db.crbegin()->fingerprintGM.size();
    }
    return sz;
}

/*
static void hist2hist( const WiFi_DB::APData &fp, WiFi_DB::APHist &ap_hist, double &u )
{
    ap_hist =  WiFi_DB::APData2Hist( fp );
    double med = median( ap_hist.begin(), ap_hist.end() );
    std::vector<double> tmpV;
    for( WiFi_DB::APHist::iterator h_it = ap_hist.begin(); h_it != ap_hist.end(); ++h_it )
    {
        tmpV.push_back( fabs( ( *h_it ) - med ) );
    }
    double sig = median( tmpV.begin(), tmpV.end() ) / 0.6745;
    u = 0;

    if( sig <= 0 )
    {
        std::sort( ap_hist.begin(), ap_hist.end() );
        WiFi_DB::APHist::iterator first = ap_hist.begin();
        WiFi_DB::APHist::iterator last  = ap_hist.begin() + ap_hist.size() - 1;
        sig = ( *last ) - ( *first );
    }
    if( sig > 0 )
    {
        u = sig * pow( ( 4. / ( 3 * tmpV.size() ) ) , ( 1. / 5 ) );
    }
    else
    {
        u = 1.0;
    }
}

*/

template<class T> static T normpdf( T x, T sig )
{
    return 1 / ( sqrt( 2 * M_PI * sig * sig ) ) * exp( -( x * x ) / ( 2 * sig * sig ) );
}

void WiFi_DB_HIST::readFormatDB( const WiFi_DB *tmp_db )
{
    for( WiFi_DB::const_iterator t_it = tmp_db->begin(); t_it != tmp_db->end(); ++t_it )
    {
        //DB loop
        WiFi_DB_HIST::DBRecord db_rec;
        db_rec.location = ( *t_it ).location;

        for( WiFi_DB::APList::const_iterator ap_it = ( *t_it ).fingerprint.begin(); ap_it != ( *t_it ).fingerprint.end(); ++ap_it )
        {
            //BSSID loop
            BSSID  bssid = ( *ap_it ).first;
            WiFi_DB_HIST::APHist ap_hist;

            const WiFi_DB::APData  &fp = ( *ap_it ).second;
            ap_hist.assign( 101, 0. );


            //WiFi_DB::APHist tmp_hist_1, tmp_hist_2;
            //double u;
            //hist2hist( fp, tmp_hist_1, u );
            //tmp_hist_2.resize( tmp_hist_1.size() );

            for( int i_rssi = 0; i_rssi < 101; ++i_rssi )
            {
                //WiFi_DB::APHist tmp_hist_2 = tmp_hist_1;
                //for( WiFi_DB::APHist::iterator h_it = tmp_hist_2.begin(); h_it != tmp_hist_2.end(); ++h_it )
                //{
                //    ( *h_it ) = ( -i_rssi - ( *h_it ) ) / u;
                //}
                //double sum = 0;
                //for( WiFi_DB::APHist::iterator h_it = tmp_hist_2.begin(); h_it != tmp_hist_2.end(); ++h_it )
                //{
                //    double w = 1. / tmp_hist_2.size();
                //    sum += w * normpdf( *h_it, 2.0 );
                //}
                //sum /= u;
                //tProb p = sum;

                tProb p = WiFi_Locator::hist2prob( fp, -i_rssi, 2.0 );
                ap_hist[i_rssi] = p;
            }

            db_rec.fingerprint.insert( std::pair<BSSID, WiFi_DB_HIST::APHist>( bssid, ap_hist ) );
        }

        db.push_back( db_rec );
    }
}

// todo: place this class in separate file (see realisation of this clas in files: FppeImp.hpp, ble_proximity_db.cpp, wifi_db_hist.cpp)
template <typename char_type>
class iostreambuf : public std::basic_streambuf<char_type, std::char_traits<char_type> >
{
    public :
    iostreambuf( char_type* buffer, std::streamsize bufferLength )
    {
        // Sets the values of the pointers defining the put area. Specifically, after the call pbase() == pbeg, pptr() == pbeg, epptr() == pend
        // pbeg - pointer to the new beginning of the put area
        // pend - pointer to the new end of the put area
        std::basic_streambuf<char_type, std::char_traits<char_type> >::setp( buffer, buffer + bufferLength );

        //Sets the values of the pointers defining the get area.Specifically, after the call eback() == gbeg, gptr() == gcurr, egptr() == gend
        //  gbeg - pointer to the new beginning of the get area
        //	gcurr - pointer to the new current character(get pointer) in the get area
        //	gend - pointer to the new end of the get area
        std::basic_streambuf<char_type, std::char_traits<char_type> >::setg( buffer, buffer, buffer + bufferLength );
    }
};

bool WiFi_DB_HIST::readFormat2( const char* const pWiFiMap, const size_t wifiFileSizeInBytes )
{
    int status = 0;

    iostreambuf<char> iostreamBuffer( ( char * )pWiFiMap, wifiFileSizeInBytes );
    std::iostream stringstr( &iostreamBuffer );

    std::string line;
    const int line_length = 1000;
    char line_buf[line_length];
    stringstr.get( line_buf, line_length - 1, '\n' );
    stringstr.ignore( 7777, '\n' );

    line = std::string( line_buf );
    line = line + "\n";

    if ( strcmp( line.c_str(), "WIFI_DB VER 2\r\n" ) != 0 )
    {
        bool success = readFormat3( pWiFiMap, wifiFileSizeInBytes );

        if ( !success ) printf( "wrong db format\n" );

        return success;
    }

    bool pos, ap;

    while ( !stringstr.eof() )
    {
        stringstr.get( line_buf, line_length - 1, '\n' );
        stringstr.ignore( 7777, '\n' );
        line = std::string(line_buf);
        line = line + "\n";

        pos = ( strstr( line.c_str(), "POINT" ) != 0 );
        ap = ( strstr( line.c_str(), "AP_MAC" ) != 0 );

        if( pos )
        {
            onNewPos( line.c_str() );
            status |= 1;
        }
        else if( ap )
        {
            onNewAP( line.c_str() );
            status |= 2;
        }
        else
        {
            onNewMeas( line.c_str() );
            status |= 4;
        }
    }

    onNewPos( 0 );

    return ( status == 7 );
}


bool WiFi_DB_HIST::readFormat3( const char* const pWiFiMap, const size_t wifiFileSizeInBytes )
{
    int status = 0;
    char c;
    std::string line;
    const int line_length = 1000;
    char line_buf[line_length];

    iostreambuf<char> iostreamBuffer( ( char * )pWiFiMap, wifiFileSizeInBytes );
    std::iostream stringstr( &iostreamBuffer );

    stringstr.get( line_buf, line_length - 1, '\n' );
    //stringstr.getline(line_buf, line_length - 1);
    stringstr.ignore( 7777, '\n' );

    line = std::string( line_buf );
    line = line + "\n";

    if ( strcmp( line.c_str(), "WIFI_DB VER 3\r\n" ) != 0 )
    {
        //printf( "wrong db format\n" );
        //assert( 0 );
        return false;
    }

    //TODO fix file reading with incorrect format
    while ( !stringstr.eof() )
    {
        stringstr >> c;

        //if ( !stringstr.eof() )
        // return true;

        if( c != ' ' && c != '\n' && c != '\r' )
        {
            stringstr.unget();
            break;
        }
    }

    //if( stringstr.eof() )
    //  return true;

    bool pos, ap;

    while ( !stringstr.eof() )
    {
        stringstr.get( line_buf, line_length - 1, '\n' );
        //stringstr.getline(line_buf, line_length - 1);
        stringstr.ignore( 7777, '\n' );
        line = std::string( line_buf );
        line = line + "\n";

        pos = ( strstr( line.c_str(), "POINT" ) != 0 );
        ap = ( strstr( line.c_str(), "AP_MAC" ) != 0 );

        if( pos )
        {
            onNewPos( line.c_str() );
            status |= 1;
        }
        else if( ap )
        {
            status = onNewAP3(line.c_str()) ? status | 2 : 0; // read untill first incorrect line
            if (!status)   break;
        }
    }

    onNewPos( 0 );
    return ( status == 3 );
}


bool WiFi_DB_HIST::readFormat3_body(const char* const pWiFiMap, const size_t wifiFileSizeInBytes)
{
    int status = 0;
    char c;
    std::string line;
    const int line_length = 1000;
    char line_buf[line_length];

    iostreambuf<char> iostreamBuffer((char *)pWiFiMap, wifiFileSizeInBytes);
    std::iostream stringstr(&iostreamBuffer);

    //stringstr.get(line_buf, line_length - 1, '\n');
    //stringstr.getline(line_buf, line_length - 1);
    //stringstr.ignore(7777, '\n');

    /*line = std::string(line_buf);
    line = line + "\n";

    if (strcmp(line.c_str(), "WIFI_DB VER 3\r\n") != 0)
    {
        //printf( "wrong db format\n" );
        //assert( 0 );
        return false;
    }
    */
    //TODO fix file reading with incorrect format
    /*while (!stringstr.eof())
    {
        stringstr >> c;

        //if ( !stringstr.eof() )
        // return true;

        if (c != ' ' && c != '\n' && c != '\r')
        {
            stringstr.unget();
            break;
        }
    }*/

    //if( stringstr.eof() )
    //  return true;

    bool pos, ap;

    while (!stringstr.eof())
    {
        stringstr.ignore(7777, '\n');
        stringstr.get(line_buf, line_length - 1, '\n');
        //stringstr.getline(line_buf, line_length - 1);
        //stringstr.ignore(7777, '\n');
        line = std::string(line_buf);
        line = line + "\n";

        pos = (strstr(line.c_str(), "POINT") != 0);
        ap = (strstr(line.c_str(), "AP_MAC") != 0);

        if (pos)
        {
            onNewPos(line.c_str());
            status |= 1;
        }
        else if (ap)
        {
            status = onNewAP3(line.c_str()) ? status | 2 : 0; // read untill first incorrect line
            if (!status )   break;
        }
    }

    onNewPos(0);
    return (status == 3);
}


void WiFi_DB_HIST::onNewPos( const char *str )
{
    onNewAP( 0 );

    if( ap_list.size() > 0 || ap_list_gm.size() > 0 )
    {
        db_rec.fingerprint = ap_list;
        db_rec.fingerprintGM = ap_list_gm;
        db.push_back( db_rec );
    }

    db_rec.fingerprint.clear();
    ap_list.clear();
    ap_list_gm.clear();
    ap_hist.clear();

    if( str != 0 )
    {

        double pos[] = {0., 0., 0.};
        const char *pch = strtok( ( char * )str, " " );

        if( pch != 0 )
        {
            pch = strtok( NULL, " " );
        }

        int cnt = 0 ;

        while( pch && cnt < 3 )
        {
            int k = sscanf( pch, "%lf", &pos[cnt] );
            pch = strtok( NULL, " " );

            if ( k > 0 ) ++cnt;
            else assert( 0 );
        }

        ( db_rec ).location.valid = ( cnt >= 2 );
        ( db_rec ).location.x = pos[0];
        ( db_rec ).location.y = pos[1];
        ( db_rec ).location.z = pos[2];
        ( db_rec ).location.p = 1;
    }

}

void WiFi_DB_HIST::onNewAP( const char *str )
{
    char tmp_str[1000];
    BSSID new_bssid( 0 );

    if( ap_hist.size() > 0 )
    {
        ap_list.insert( std::pair<BSSID, APHist>( bssid, ap_hist ) );
    }

    ap_hist.clear();

    if( str != 0 )
    {
        int k = sscanf( str, "%s %llu", tmp_str, &new_bssid );

        if ( k <= 0 ) assert( 0 );

        bssid = new_bssid;
    }
}


bool WiFi_DB_HIST::onNewAP3( const char *str )
{
    bool status = false;
    if( str != 0 )
    {
        BSSID new_bssid(0);
        GMixture gm = {};
        std::stringstream ss(str);
        std::string s;

        //sscanf( str, "%s %lld %f %f %f %f %f %f", tmp_str, &new_bssid, &gm.w1, &gm.mu1, &gm.sig1, &gm.w2, &gm.mu2, &gm.sig2  );
        if ((ss >> s >> new_bssid >> gm.w1 >> gm.mu1 >> gm.sig1 >> gm.w2 >> gm.mu2 >> gm.sig2))
        {
            bssid = new_bssid;
            ap_list_gm.insert(std::pair<BSSID, GMixture>(bssid, gm));
            status = true;
        }
    }
    return status;
}



void WiFi_DB_HIST::onNewMeas( const char *str )
{
    int idx;
    double prob( 0 );

    if( ap_hist.size() == 0 )
    {
        ap_hist.assign( 101, 0. );
    }

    if( str != 0 )
    {
        int k = sscanf( str, "%d %le", &idx, &prob );

        if ( k <= 0 ) assert( 0 );

        ap_hist[idx] = prob;
    }
}
