#ifndef BLE_MAP_CREATOR_HPP
#define BLE_MAP_CREATOR_HPP
#include "IBleBuilder.hpp"
#include "ble_db.hpp"
#include <armadillo>
#include <iomanip>

#define BLE_DEFAULT_BSSID_COUNT_MAX    150
#define BLE_DEFAULT_SCAN_COUNT_MIN     5
#define BLE_DEFAULT_INFORMATION_GAIN_MIN     0

class BleMapCreator : public Fpbl::IBleBuilder
{
    public:
		BleMapCreator()
        {
            scanCountMin = BLE_DEFAULT_SCAN_COUNT_MIN;
            bssidsCountMax = BLE_DEFAULT_BSSID_COUNT_MAX;
            igMinimumValue = BLE_DEFAULT_INFORMATION_GAIN_MIN;
        }
		BleMapCreator(Fpbl::FPGrid ble_fp)
        {
            scanCountMin = ble_fp.min_data_count;
            bssidsCountMax = ble_fp.max_bssid_count;
            igMinimumValue = ble_fp.min_ig;
        }
		~BleMapCreator()
        {
            ;
        }

    Fpbl::ReturnStatus buildFingerprint(const Fpbl::BleGrid &bleGrid, Fpbl::BleBuilder::LocalDB *bleFp)
    {
        Fpbl::ReturnStatus result = Fpbl::ReturnStatus::STATUS_UNKNOWN_ERROR;
        Ble_DB db(bleGrid);
        IGList igList = CalcIG(&db, std::vector<BSSID>());

        auto bssids = selectByIg(igList, igMinimumValue);
        auto N = std::min(bssidsCountMax, bssids.size());

        bssids.resize(N);
        saveFormatGMix(db, bssids, scanCountMin, bleFp);

        if (bleFp->size() > 0)
            result = Fpbl::ReturnStatus::STATUS_SUCCESS;
        /*
        saveOldLogs(bleGrid, "ble_old.log");
        Ble_DB db2;
        db2.readFormatLogs("ble_old.log", -100000, 10000, -10000, 10000);
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

        double entropy_map( Ble_DB *db )
        {
            double res = db->entropy_map();
            return res;
        }

        double entropy_bssid( Ble_DB *db, BSSID bssid )
        {
            double res = db->entropy_bssid( bssid );
            return res;
        }


        IGList CalcIG( Ble_DB *db, std::vector<BSSID> aps )
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

	void saveFormatGMix(const Ble_DB &db, const std::vector<BSSID> &bssids, const unsigned int c_min, Fpbl::BleBuilder::LocalDB *bleFp)
  {
      int pos_idx = 0;
      int measCntMin = c_min;

      for (Ble_DB::const_iterator db_it = db.begin(); db_it != db.end(); ++db_it)
      {
          int measCnt = ( *db_it ).measVector.size();
          //int measCnt = (*db_it).fingerprint.size();

          if ( measCnt < measCntMin ) 
              continue;

          Fpbl::BleBuilder::DBRecord rec;
          rec.location.x = (*db_it).location.x;
          rec.location.y = (*db_it).location.y;
          rec.location.floor = (*db_it).location.z;

          for (std::vector<BSSID>::const_iterator bssid_it = bssids.begin(); bssid_it != bssids.end(); ++bssid_it)
          {
              std::vector<double> v;
			  uint16_t true_beacon_scan_count_in_cell = 0;

              for (Ble_DB::MeasVector::const_iterator meas_it = (*db_it).measVector.begin(); meas_it != (*db_it).measVector.end(); ++meas_it)
              {
                  Ble_DB::APMeas::const_iterator ap_it = (*meas_it).find(*bssid_it);

                  if (ap_it != (*meas_it).end())
                  {
                      v.push_back((*ap_it).second);
					  true_beacon_scan_count_in_cell++;
                  }
                  else
                  {
                      v.push_back(-100);
                  }
              }

              std::sort(v.begin(), v.end());

              arma::gmm_diag model;
              arma::rowvec data(v);
              //data.print();

              model.learn(data, 1, arma::eucl_dist, arma::static_spread, 0, 10, 1e-10, false);

              //model.dcovs.print();
              if (model.dcovs(0.0) > 1. && data.size() > 10)
              {
                  model.learn(data, 2, arma::eucl_dist, arma::static_spread, 0, 10, 1e-10, false);
              }

              arma::colvec m(2), d(2), h(2);
              m.zeros(); d.fill(5.0); h.zeros();

              for (arma::uword i = 0; i < model.hefts.size(); ++i)
              {
                  double w = model.hefts(i);

                  if (w > 0.1) h(i) = w;

                  m(i) = model.means(0, i);
              }

              h = h / arma::sum(h);

              Fpbl::BleBuilder::GMixture gaussianMixture;
              gaussianMixture.mu1 = m(0);
              gaussianMixture.sig1 = d(0);
              gaussianMixture.w1 = h(0);

              gaussianMixture.mu2 = m(1);
              gaussianMixture.sig2 = d(1);
              gaussianMixture.w2 = h(1);
			  gaussianMixture.scan_count = true_beacon_scan_count_in_cell;

              rec.fingerprintGM[*bssid_it] = gaussianMixture;
          }

          bleFp->push_back(rec);
          pos_idx++;
      }
  }

  std::vector<BSSID>  selectFirstN(const IGList &igList, int N)
  {
      std::vector<BSSID> bssids;

      int i = 0;

      for (IGList::const_iterator it = igList.begin(); it != igList.end() && i < N; ++it, ++i)
      {
          bssids.push_back(it->bssid);
      }

      return bssids;
  }
  std::vector<BSSID>  selectByIg(const IGList &igList, double ig)
  {
      std::vector<BSSID> bssids;

      for (IGList::const_iterator it = igList.begin(); it != igList.end(); ++it)
      {
          if (it->ig >= ig)
          {
              bssids.push_back(it->bssid);
          }
      }

      return bssids;
  }

		void saveOldLogs(const Fpbl::BleGrid &bleGrid, const std::string fname)
		{
			std::ofstream fs(fname.c_str(), std::ios::out);
			int point = 0;
			if (fs.is_open())
			{
				for (auto grid_it = bleGrid.cbegin(); grid_it != bleGrid.cend(); ++grid_it)
				{
					int measNumber = 0;
                    if (grid_it->bleData.size() > 0) point++;					
					for (auto meas_it = grid_it->bleData.cbegin(); meas_it != grid_it->bleData.cend(); ++meas_it)
					{
						measNumber++;
						fs << "Map point number: " << point << ", "
							<< "Position: " << "x= " << (int)(grid_it->coordinates.x*100) << " y= " << (int)(grid_it->coordinates.y*100) << ", "
							<< "Measure number: " << measNumber << ", ";

						for (auto ap_it = meas_it->scanBle.cbegin(); ap_it != meas_it->scanBle.cend(); ++ap_it)
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

#endif //BLE_MAP_CREATOR_HPP
