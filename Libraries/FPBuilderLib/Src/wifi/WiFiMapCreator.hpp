#ifndef WIFI_MAP_CREATOR_HPP
#define WIFI_MAP_CREATOR_HPP
#include "IWiFiBuilder.hpp"
#include "wifi_db.hpp"
#include <armadillo>
#include <iomanip>

#define WIFI_DEFAULT_BSSID_COUNT_MAX    150
#define WIFI_DEFAULT_SCAN_COUNT_MIN     5
#define WIFI_DEFAULT_INFORMATION_GAIN_MIN     0

class WiFiMapCreator : public Fpbl::IWiFiBuilder
{
    public:
        WiFiMapCreator()
        {
            scanCountMin = WIFI_DEFAULT_SCAN_COUNT_MIN;
            bssidsCountMax = WIFI_DEFAULT_BSSID_COUNT_MAX;
            igMinimumValue = WIFI_DEFAULT_INFORMATION_GAIN_MIN;
        }
        WiFiMapCreator(Fpbl::FPGrid wifi_fp)
        {
            scanCountMin = wifi_fp.min_data_count;
            bssidsCountMax = wifi_fp.max_bssid_count;
            igMinimumValue = wifi_fp.min_ig;
        }
        ~WiFiMapCreator()
        {
            ;
        }

        Fpbl::ReturnStatus buildFingerprint(const Fpbl::WiFiGrid &wifiGrid, Fpbl::WiFiBuilder::LocalDB *wifiFp)
        {
            Fpbl::ReturnStatus result = Fpbl::ReturnStatus::STATUS_UNKNOWN_ERROR;
            WiFi_DB db( wifiGrid );
            IGList igList = CalcIG( &db, std::vector<BSSID>() );

            auto bssids = selectByIg( igList, igMinimumValue );
            auto N = std::min( bssidsCountMax, bssids.size() );

            bssids.resize( N );
            saveFormatGMix( db, bssids, scanCountMin, wifiFp );

            if ( wifiFp->size() > 0 ) 
				result = Fpbl::ReturnStatus::STATUS_SUCCESS;
			/*
			saveOldLogs(wifiGrid, "wifi_old.log");
			WiFi_DB db2;
			db2.readFormatLogs("wifi_old.log", -100000, 10000, -10000, 10000);
			IGList igList2 = CalcIG(&db2, std::vector<BSSID>());
			*/
            return result;
        }
    private:
        size_t scanCountMin;
        size_t bssidsCountMax;
        double igMinimumValue;

        struct IGInfo
        {
            double ig;
            BSSID  bssid;
            static bool compare( const IGInfo &i1, const IGInfo &i2 )
            {
                return ( i1.ig > i2.ig );
            }
        };

        typedef std::vector<IGInfo> IGList;

        double entropy_map( WiFi_DB *db )
        {
            double res = db->entropy_map();
            return res;
        }

        double entropy_bssid( WiFi_DB *db, BSSID bssid )
        {
            double res = db->entropy_bssid( bssid );
            return res;
        }


        IGList CalcIG( WiFi_DB *db, std::vector<BSSID> aps )
        {
            std::vector<BSSID> ap_list;
            ap_list = db->getApList();
            IGList result;

            double e_map = entropy_map( db );

            for ( std::vector<BSSID>::const_iterator it = ap_list.begin(); it != ap_list.end(); ++it )
            {
                if ( aps.size() > 0 )
                {
                    if ( std::find( aps.begin(), aps.end(), ( *it ) ) == aps.end() )
                    {
                        continue;
                    }
                }

                BSSID bssid = ( *it );
                double e_bssid = entropy_bssid( db, bssid );
                double ig = e_map - e_bssid;

                IGInfo ap_ig;
                ap_ig.ig = ig;
                ap_ig.bssid = bssid;
                result.push_back( ap_ig );
            }

            std::sort( result.begin(), result.end(), IGInfo::compare );
            return result;
        }


        void saveFormatGMix( const WiFi_DB &db, const std::vector<BSSID> &bssids, const unsigned int c_min, Fpbl::WiFiBuilder::LocalDB *wifiFp )
        {
            int pos_idx = 0;
            int measCntMin = c_min;
            int measCnt;

            for ( WiFi_DB::const_iterator db_it = db.begin(); db_it != db.end(); ++db_it )
            {
                measCnt = ( *db_it ).measVector.size();
		            //measCnt = ((*db_it).fingerprint).size();

                if ( measCnt < measCntMin ) continue;

                Fpbl::WiFiBuilder::DBRecord rec;
                rec.location.x = ( *db_it ).location.x;
                rec.location.y = ( *db_it ).location.y;
                rec.location.floor = (int16_t)(( *db_it ).location.z);


                for ( std::vector<BSSID>::const_iterator bssid_it = bssids.begin(); bssid_it != bssids.end(); ++bssid_it )
                {
                    std::vector<double> v;
					uint16_t true_ap_scan_count_in_cell = 0;

                    for ( WiFi_DB::MeasVector::const_iterator meas_it = ( *db_it ).measVector.begin(); meas_it != ( *db_it ).measVector.end(); ++meas_it )
                    {
                        WiFi_DB::APMeas::const_iterator ap_it = ( *meas_it ).find( *bssid_it );

                        if ( ap_it != ( *meas_it ).end() )
                        {
                            v.push_back( ( *ap_it ).second );
							true_ap_scan_count_in_cell++;
                        }
                        else
                        {
                            v.push_back( -100 );
                        }
                    }

                    std::sort(v.begin(), v.end());

                    arma::gmm_diag model;
                    arma::rowvec data( v );
                    //data.print();
                    model.learn( data, 1, arma::eucl_dist, arma::static_spread, 0, 10, 1e-10, false );

                    //model.dcovs.print();
                    if ( model.dcovs( 0.0 ) > 1. && data.size() > 10 )
                    {
                        model.learn( data, 2, arma::eucl_dist, arma::static_spread, 0, 10, 1e-10, false );
                    }

                    arma::colvec m( 2 ), d( 2 ), h( 2 );
                    m.zeros(); d.fill( 5.0 ); h.zeros();

                    for ( arma::uword i = 0; i < model.hefts.size(); ++i )
                    {
                        double w = model.hefts( i );

                        if ( w > 0.1 ) h( i ) = w;

                        m( i ) = model.means( 0, i );
                    }

                    h = h / arma::sum( h );

                    Fpbl::WiFiBuilder::GMixture gaussianMixture;
                    gaussianMixture.mu1 = m( 0 );
                    gaussianMixture.sig1 = d( 0 );
                    gaussianMixture.w1 = h( 0 );

                    gaussianMixture.mu2 = m( 1 );
                    gaussianMixture.sig2 = d( 1 );
                    gaussianMixture.w2 = h( 1 );
					gaussianMixture.scan_count = true_ap_scan_count_in_cell;

                    rec.fingerprintGM[*bssid_it] = gaussianMixture;
                }

                wifiFp->push_back( rec );
                pos_idx++;
            }
        }
        std::vector<BSSID>  selectFirstN( const IGList &igList, int N )
        {
            std::vector<BSSID> bssids;

            int i = 0;

            for ( IGList::const_iterator it = igList.begin(); it != igList.end() && i < N; ++it, ++i )
            {
                bssids.push_back( it->bssid );
            }

            return bssids;
        }
        std::vector<BSSID>  selectByIg( const IGList &igList, double ig )
        {
            std::vector<BSSID> bssids;

            for ( IGList::const_iterator it = igList.begin(); it != igList.end(); ++it )
            {
                if ( it->ig >= ig )
                {
                    bssids.push_back( it->bssid );
                }
            }

            return bssids;
        }

		void saveOldLogs(const Fpbl::WiFiGrid &wifiGrid, const std::string fname)
		{
			std::ofstream fs(fname.c_str(), std::ios::out);
			int point = 0;
			if (fs.is_open())
			{
				for (auto grid_it = wifiGrid.cbegin(); grid_it != wifiGrid.cend(); ++grid_it)
				{
					int measNumber = 0;
                    if (grid_it->wifiData.size() > 0) point++;					
					for (auto meas_it = grid_it->wifiData.cbegin(); meas_it != grid_it->wifiData.cend(); ++meas_it)
					{
						measNumber++;
						fs << "Map point number: " << point << ", "
							<< "Position: " << "x= " << (int)(grid_it->coordinates.x*100) << " y= " << (int)(grid_it->coordinates.y*100) << ", "
							<< "Measure number: " << measNumber << ", ";

						for (auto ap_it = meas_it->scanWiFi.cbegin(); ap_it != meas_it->scanWiFi.cend(); ++ap_it)
						{
							//ap_it->mac
							fs.unsetf(std::ios::showbase);

                            fs << "2013_01_01_00_00_00, 0, " << std::hex << /*std::uppercase <<*/
                                std::fixed << std::setw(2) << std::setfill('0') <<
                                ((ap_it->mac >> 48) & 0xFF) << ":" <<
                                std::fixed << std::setw(2) << std::setfill('0') <<
                                ((ap_it->mac >> 40) & 0xFF) << ":" <<
                                std::fixed << std::setw(2) << std::setfill('0') <<
                                ((ap_it->mac >> 32) & 0xFF) << ":" <<
                                std::fixed << std::setw(2) << std::setfill('0') <<
                                ((ap_it->mac >> 24) & 0xFF) << ":" <<
                                std::fixed << std::setw(2) << std::setfill('0') <<
                                ((ap_it->mac >> 16) & 0xFF) << ":" <<
                                std::fixed << std::setw(2) << std::setfill('0') <<
                                ((ap_it->mac >>  8) & 0xFF) << ":" <<
                                std::fixed << std::setw(2) << std::setfill('0') <<
                                ((ap_it->mac >> 0) & 0xFF) << std::dec << ", , "
                                << ap_it->frequency << ", " << static_cast<int>(ap_it->rssi) << ", 0, 0, 0" << std::endl;
						}
					}
				}
				fs.close();
			}
		}
};

#endif //WIFI_MAP_CREATOR_HPP
