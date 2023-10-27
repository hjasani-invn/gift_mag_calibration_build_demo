/**
* \copyright       Copyright (C) SPIRIT Navigation, LLC., 2014
* \brief           Fingerprint storage class. Stores raw rssi measurements.
* Esimates additional metrics for AP's selection
* \file            wifi_db.cpp
* \ingroup         WiFi
* \author          M. Frolov
* \date            28.11.2014
*/

#define _USE_MATH_DEFINES
#include <fstream>
#include <sstream>
#include <algorithm>
#include "ble_db.hpp"
#include <cmath>
#include "median.h"


Ble_DB::Ble_DB( const Fpbl::BleGrid &bleGrid )
{
    for ( auto pos_it = bleGrid.cbegin(); pos_it != bleGrid.cend(); ++pos_it )
    {
        DBRecord rec;
        Fpbl::CoordinatesInGrid pos = pos_it->coordinates;
        rec.location.set( pos.x, pos.y, pos.floor, true, 1. );
        for ( auto scan_it = pos_it->bleData.cbegin(); scan_it != pos_it->bleData.cend(); ++scan_it )
        {
            if ( scan_it->scanBle.size() == 0 ) continue;

            APMeas rawScan;

            for ( auto ap_it = scan_it->scanBle.begin(); ap_it != scan_it->scanBle.end(); ++ap_it )
            {

                BSSID prefix = 0;
                int freq = ap_it->frequency;

                if ( freq >= 4915 )
                {
                    prefix = BSSID( 2 ) << 48;
                }
                else if ( freq >= 3657 )
                {
                    prefix = BSSID( 1 ) << 48;
                }
                else
                {
                    prefix = 0;
                }

                BSSID mac = ap_it->mac | prefix;
                RSSI rssi = ap_it->rssi;

                auto mac_it = rec.fingerprint.find( mac );

                if ( mac_it != rec.fingerprint.end() )
                {
                    auto rssi_it = mac_it->second.find( rssi );

                    if ( rssi_it != mac_it->second.end() )
                    {
                        rssi_it->second++;
                    }
                    else
                    {
                        mac_it->second.insert( std::pair<RSSI, int>( rssi, 1 ) );
                    }
                }
                else
                {
                    APData rssi_rec;
                    rssi_rec.insert( std::pair<RSSI, int>( rssi, 1 ) );
                    rec.fingerprint.insert( std::pair<BSSID, APData>( mac, rssi_rec ) );
                }

                rawScan.insert( std::pair<BSSID, RSSI>( mac, rssi ) );
            }

            rec.measVector.push_back( rawScan );
        }

        if ( rec.fingerprint.empty() == false )
        {
            normalizeRecord( rec );
            db.push_back( rec );
        }
    }

    normalizeDB();
}


Ble_DB::Ble_DB( std::string db_name )
{
    readFormat1( db_name );
}


Ble_DB::const_iterator Ble_DB::begin() const
{
    return db.begin();
}

Ble_DB::const_iterator Ble_DB::end() const
{
    return db.end();
}

size_t Ble_DB::size() const
{
    return db.size();
}



void Ble_DB::readFormat1( const std::string &fname )
{
    std::ifstream db_f( fname.c_str(), std::ios::in );
    std::string line;
    std::stringstream convertor;
    DBRecord db_p;
    bool headerOK = true;

    if( db_f.fail() )
    {
        std::string error = "\nError:can't open WiFi DB file!";
        std::cout << error << std::endl;
        throw( error ); // !!!
    }

    if( db_f.eof() != true )
    {
        std::string sHdr, sVer;
        std::getline( db_f, line );
        int nVer( 0 );

        convertor.clear();
        convertor.str( line );
        convertor >> sHdr >> sVer >> nVer;

        if( sHdr.compare( std::string( "WIFI_DB" ) ) ||
                sVer.compare( std::string( "VER" ) ) ||
                nVer != 1 )
        {
            headerOK = false;
        }
    }


    while( db_f && headerOK == true )
    {
        float x( 0 ), y( 0 ), z( 0 );
        RSSI rssi( 0 );
        std::string token;
        BSSID bssid( 0 );

        std::getline( db_f, line );
        convertor.clear();
        convertor.str( line );
        convertor >> token;

        if( token.compare( std::string( "POINT" ) ) == 0 )
        {
            convertor >> x >> y >> z;
            //std::cout << "new point " << x << " " <<
            //          y << " " << z << " " << std::endl;

            if( db_p.fingerprint.empty() != true )
            {
                db.push_back( db_p );
            }

            db_p.fingerprint.clear();
            db_p.location.set( x, y, z, 1., true );

        }
        else if( token.size() > 0 )
        {
            //std::cout << "new AP" << std::endl;
            bssid =  Ble_ApInfo::string_to_mac( token );
            convertor >> rssi;
            //std::cout << "BSSID = " << std::hex << bssid << " RSSI = " << std::dec << rssi << std::endl;

            APList::iterator ap_it = db_p.fingerprint.find( bssid );

            if( ap_it != db_p.fingerprint.end() )
            {
                APData &ap_hist = ( *ap_it ).second;
                APData::iterator r_it = ap_hist.find( rssi );

                if( r_it != ap_hist.end() )
                {
                    ap_hist[rssi] += 1;
                }
                else
                {
                    ap_hist[rssi] = 1;
                }

            }
            else
            {
                APData ap_d;
                ap_d[rssi] = 1;
                db_p.fingerprint[bssid] = ap_d;
            }


        }
    }

    if( db_p.fingerprint.empty() != true )
    {
        db.push_back( db_p );
    }

    db_f.close();
}

Ble_DB::APHist Ble_DB::APData2Hist(const Ble_DB::APData &Ap)
{
	Ble_DB::APData::const_iterator it;
	Ble_DB::APHist hist;

    for( it = Ap.begin(); it != Ap.end(); ++it )
    {
        if( ( *it ).second > 0 )
        {
            RSSI rssi = ( *it ).first;
            int N = ( *it ).second;

            for( int i = 0; i < N; ++i )
            {
                hist.push_back( rssi );
            }
        }
    }

    return hist;
}


void Ble_DB::readFormatLogs( const std::string &fname, double x_min, double x_max, double y_min, double y_max, double z_ )
{
    std::ifstream db_f( fname.c_str(), std::ios::in );
    std::string line;
    std::stringstream convertor;
    DBRecord db_p;
	Ble_DB::APMeas m_vec;


    int point_num = -1;
    //int meas_num = -1;
    //int ap_num = -1;
    int p_n = -1;
    int m_n = -1;
    double x = 0, y = 0, z = z_;

    if( db_f.fail() )
    {
        std::string error = "\nError:can't open WiFi DB file!";
        std::cout << error << std::endl;
        throw( error ); // !!!
    }

    while( !db_f.eof() )
    {
        std::getline( db_f, line );
        std::vector<std::string> tokens = tokenize( line, ',' );
        int idx_offset = 0;
        int mac_idx = 2;
        int freq_idx = 4;
        int rssi_idx = 5;

        if( line.length() == 0 )
        {
            continue;
        }

        std::size_t pos_p = tokens[0].find( "Map point number:" );

        if( pos_p == 0 )
        {
            idx_offset += 3;

            std::vector<std::string> tokens_2 = tokenize( tokens[0], ':' );
            std::stringstream ss( tokens_2[1] );
            ss >> p_n;

            tokens_2 = tokenize( tokens[2], ':' );
            ss.clear();
            ss.str( tokens_2[1] );
            ss >> m_n;


            tokens_2 = tokenize( tokens[1], '=' );
            ss.clear();
            ss.str( tokens_2[1] );
            ss >> x;
            ss.clear();
            ss.str( tokens_2[2] );
            ss >> y;


            if( point_num < 0 ) //first line
            {
                point_num = p_n;
                db_p.location.set( x, y, z, 1., true );
            }
            else
            {
                db_p.measVector.push_back( m_vec );
                m_vec.clear();
            }
        }

        if( p_n != point_num && point_num > 0 )
        {

            bool skip = ( db_p.location.x < x_min || db_p.location.x > x_max || db_p.location.y < y_min || db_p.location.y > y_max );

            if( db_p.fingerprint.empty() != true  && !skip )
            {
                normalizeRecord( db_p );
                db.push_back( db_p );
            }

            db_p.fingerprint.clear();
            db_p.measVector.clear();
            db_p.location.set( x, y, z, 1., true );
            point_num = p_n;
        }

        std::stringstream ss( tokens[mac_idx + idx_offset] );
        std::string mac;
        ss >> mac;

        int freq;
        ss.clear();
        ss.str( tokens[freq_idx + idx_offset] );
        ss >> freq;

        if( mac.length() == 17 )
        {
            std::string mac_prefix;

            if( freq >= 4915 )
            {
                mac_prefix = "02:";
            }
            else if( freq >= 3657 )
            {
                mac_prefix = "01:";
            }
            else
            {
                mac_prefix = "00:";
            }

            mac = mac_prefix + mac;

        }

        int rssi;
        ss.clear();
        ss.str( tokens[rssi_idx + idx_offset] );
        ss >> rssi;

        BSSID bssid =  Ble_ApInfo::string_to_mac( mac );


        APList::iterator ap_it = db_p.fingerprint.find( bssid );

        if( ap_it != db_p.fingerprint.end() )
        {
            APData &ap_hist = ( *ap_it ).second;
            APData::iterator r_it = ap_hist.find( rssi );

            if( r_it != ap_hist.end() )
            {
                ap_hist[rssi] += 1;
            }
            else
            {
                ap_hist[rssi] = 1;
            }

        }
        else
        {
            APData ap_d;
            ap_d[rssi] = 1;
            db_p.fingerprint[bssid] = ap_d;
        }

        m_vec[bssid] = rssi;



    }

    bool skip = ( db_p.location.x < x_min || db_p.location.x > x_max || db_p.location.y < y_min || db_p.location.y > y_max );

    if( db_p.fingerprint.empty() != true && !skip )
    {
        normalizeRecord( db_p );
        db_p.measVector.push_back( m_vec );
        db.push_back( db_p );
    }

    db_f.close();

    normalizeDB();

}


std::vector<std::string> Ble_DB::tokenize( const std::string &s, const char &delim ) const
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



void Ble_DB::normalizeRecord( DBRecord &rec )
{
    int meas = 0;

	for (Ble_DB::APList::const_iterator it = rec.fingerprint.begin(); it != rec.fingerprint.end(); ++it)
    {
        int m = 0;

		for (Ble_DB::APData::const_iterator ap_it = (*it).second.begin(); ap_it != (*it).second.end(); ++ap_it)
        {
            m += ( *ap_it ).second;
        }

        meas =  std::max( meas, m );
    }

	for (Ble_DB::APList::iterator it = rec.fingerprint.begin(); it != rec.fingerprint.end(); ++it)
    {
        int m = 0;

		for (Ble_DB::APData::const_iterator ap_it = (*it).second.begin(); ap_it != (*it).second.end(); ++ap_it)
        {
            if ( ( *ap_it ).first > -100 )
            {
                m += ( *ap_it ).second;
            }
        }

        m = meas - m;

        if( m > 0 )
        {
            ( *it ).second[-100] = m;
        }
    }
}


double Ble_DB::entropy_map()
{
    int size = db.size();
    double e = 0;

	for (Ble_DB::LocalDB::const_iterator it = db.begin(); it != db.end(); ++it)
    {
        double p = ( *it ).location.p / size;

        if( p > 0 )
        {
            e -= ( p * log( p ) / log( 2. ) );
        }
    }

    return e;
}


double Ble_DB::entropy_bssid( BSSID bssid )
{
    double e = 0;
	Ble_DB::APData global_ap_hist;
    int meas = 0;

	for (Ble_DB::LocalDB::const_iterator it = db.begin(); it != db.end(); ++it)
    {
		Ble_DB::APList::const_iterator ap_it = (*it).fingerprint.find(bssid);

        if( ap_it != ( *it ).fingerprint.end() )
        {
			for (Ble_DB::APData::const_iterator hist_it = (*ap_it).second.begin();
                    hist_it != ( *ap_it ).second.end();
                    ++hist_it )
            {
                RSSI rssi = ( *hist_it ).first;

                if( global_ap_hist.find( rssi ) != global_ap_hist.end() )
                {
                    global_ap_hist[rssi] += ( *hist_it ).second;
                }
                else
                {
                    global_ap_hist[rssi]  = ( *hist_it ).second;
                }

                meas += ( *hist_it ).second;

                //std::cout << bssid <<" " << rssi << " " << ( *hist_it ).second <<" " << meas << std::endl;


            }

        }
        else
        {
            //std::cout << bssid << std::endl;
        }

    }

    for( RSSI rssi = -100; rssi <= 0; ++rssi )
    {
        if( global_ap_hist.find( rssi ) != global_ap_hist.end() )
        {
            double p_rssi = static_cast<double>( global_ap_hist[rssi] ) / meas;

            if( p_rssi > 0 )
            {
                double h_pos_rssi = entropy_rssi( bssid, rssi );
                e += p_rssi * h_pos_rssi;
                //std::cout << bssid << " " << p_rssi << " " << h_pos_rssi << " " << rssi << " " << meas  << std::endl;
            }
        }
    }

    return e;
}



double Ble_DB::entropy_rssi( BSSID bssid, RSSI rssi )
{
    double e = 0;
    std::vector<double> p;
    double sumP = 0;

	for (Ble_DB::LocalDB::const_iterator it = db.begin(); it != db.end(); ++it)
    {

		Ble_DB::APList::const_iterator ap_it = (*it).fingerprint.find(bssid);

        if( ap_it != ( *it ).fingerprint.end() )
        {
            int meas = 0;
            double p_pos_rssi = 0;

			for (Ble_DB::APData::const_iterator hist_it = (*ap_it).second.begin();
                    hist_it != ( *ap_it ).second.end();
                    ++hist_it )
            {
                meas += ( *hist_it ).second;
            }


			Ble_DB::APData::const_iterator rssi_it = (*ap_it).second.find(rssi);

            if( rssi_it != ( *ap_it ).second.end() )
            {
                p_pos_rssi = static_cast<double>( ( *rssi_it ).second ) / meas;
            }

            if( p_pos_rssi > 0 )
            {
                p.push_back( p_pos_rssi );
                sumP += p_pos_rssi;
            }
        }
    }

    for ( std::vector<double>::const_iterator it = p.begin(); it != p.end(); ++it )
    {
        double p_norm = ( *it ) / sumP;
        e -= ( p_norm * log( p_norm ) / log( 2. ) );

    }

    return e;

}

std::vector<BSSID> Ble_DB::getApList()
{
    std::vector<BSSID> list;

	for (Ble_DB::LocalDB::const_iterator it = db.begin(); it != db.end(); ++it)
    {
		for (Ble_DB::APList::const_iterator ap_it = (*it).fingerprint.begin();
                ap_it != ( *it ).fingerprint.end();
                ++ap_it )
        {
            BSSID bssid = ( *ap_it ).first;

            if( std::find( list.begin(), list.end(), bssid ) == list.end() )
            {
                list.push_back( bssid );
            }
        }
    }

    return list;
}


void Ble_DB::normalizeDB()
{
    std::vector<BSSID>  apList  = getApList();

	for (Ble_DB::LocalDB::iterator it = db.begin(); it != db.end(); ++it)
    {

        int meas = 0;

		for (Ble_DB::APList::const_iterator ap_list_it = (*it).fingerprint.begin(); ap_list_it != (*it).fingerprint.end(); ++ap_list_it)
        {
            int m = 0;

            for( Ble_DB::APData::const_iterator hist_it = ( *ap_list_it ).second.begin(); hist_it != ( *ap_list_it ).second.end(); ++hist_it )
            {
                m += ( *hist_it ).second;
            }

            meas =  std::max( meas, m );
        }

        for( std::vector<BSSID>::const_iterator bssid_it = apList.begin();
                bssid_it != apList.end();
                ++bssid_it )
        {
            if( ( *it ).fingerprint.find( *bssid_it )  == ( *it ).fingerprint.end() )
            {
                APData ap_d;

                ap_d[-100] = meas;
                ( *it ).fingerprint[*bssid_it] = ap_d;
            }
        }
    }
}

template<class T> static T normpdf( T x, T sig )
{
    return 1 / ( sqrt( 2 * M_PI * sig * sig ) ) * exp( -( x * x ) / ( 2 * sig * sig ) );
}

tProb Ble_DB::hist2prob( const Ble_DB::APData &fp, int rssi, tProb kernel_sig )
{
    tProb p_rssi = 0;
	Ble_DB::APHist ap_hist = Ble_DB::APData2Hist(fp);
    double med = median( ap_hist.begin(), ap_hist.end() );
    std::vector<double> tmpV;

    for ( Ble_DB::APHist::iterator h_it = ap_hist.begin(); h_it != ap_hist.end(); ++h_it )
    {
        tmpV.push_back( fabs( ( *h_it ) - med ) );
    }

    double sig = median( tmpV.begin(), tmpV.end() ) / 0.6745;
    double u = 0;

    if ( sig <= 0 )
    {
        std::sort( ap_hist.begin(), ap_hist.end() );
		Ble_DB::APHist::iterator first = ap_hist.begin();
		Ble_DB::APHist::iterator last = ap_hist.begin() + ap_hist.size() - 1;
        sig = ( *last ) - ( *first );
    }

    if ( sig > 0 )
    {
        u = sig * pow( ( 4. / ( 3 * tmpV.size() ) ), ( 1. / 5 ) );
    }
    else
    {
        u = 1.0;
    }


    for ( Ble_DB::APHist::iterator h_it = ap_hist.begin(); h_it != ap_hist.end(); ++h_it )
    {
        ( *h_it ) = ( rssi - ( *h_it ) ) / u;
    }


    double sum = 0;

    for ( Ble_DB::APHist::iterator h_it = ap_hist.begin(); h_it != ap_hist.end(); ++h_it )
    {
        double w = 1. / ap_hist.size();
        sum += w * normpdf( *h_it, kernel_sig );
    }

    sum /= u;
    p_rssi = sum;
    return p_rssi;

}
