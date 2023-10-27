/**
* \copyright       Copyright (C) InvenSense 2017
* \brief           Position update
* \ingroup         PF
* \file            UpdateExternalPos.hpp
* \author          M.Frolov
* \date            25.05.2017
*/

#ifndef _UPDATE_BLE_PROXIMITY_H
#define _UPDATE_BLE_PROXIMITY_H

#include <cmath>
#include <iomanip>
#include <eigen/Core>

#include "IPF.hpp"
#include "IFFSettings.hpp"
#include "../ble_proximity/ble_proximity_if.hpp"

namespace PF
{
    static const Float default_ble_proximity_sigma = 5.;

    class UpdateBLEProximity : public UpdateSource
    {
        public:
            UpdateBLEProximity()
            {
                initialize( 0 , 0);
            }
            
            UpdateBLEProximity(IBLEProximity **pIBLEProximity, IFFData<> *pIFFData)
            {
                initialize(pIBLEProximity, pIFFData);
            }
            
            ~UpdateBLEProximity() {};

            void initialize(IBLEProximity **pIBLEProximity, IFFData<> *pIFFData)
            {
                num_beacons_in_update = counter = 0;
                operatingSigBle = sigBLE = default_ble_proximity_sigma; // m
                posValid = false;
                isEnabled = false;
                this->pIBLEProximity = pIBLEProximity;
                pIFF = pIFFData;
                sig_level = 1.5;
                locationList.clear();
                update_was_called = false;
                name = "UpdateBLEProximity";
                pulling_type = pt_nonePulling;
                max_processing_distance_in_stop = 0.75;
                pulling_sigma = 0.5;
                isPullingEnabled = (pulling_type == pt_nonePulling) ? false : true;
                is_near = is_step = false;
                distance_to_beacon = 0.;
            }

            void bleProximityPos(WiFi_Location ble_loc)
            {
                posBLE.x = ble_loc.x;
                posBLE.y = ble_loc.y;
                posBLE.level = ble_loc.z;
                posValid = ble_loc.valid;
            }

            void bleProximityPositions(std::vector<WiFi_Location> LocationList)
            {
                locationList = LocationList;
                posValid = true;
            }

            void setSig(Float sigmaBLE, Float sigLevel)
            {
                operatingSigBle = sigBLE = sigmaBLE;
                sig_level = sigLevel;
            }

            void getSig(Float &sigmaBLE, Float &sigLevel)
            {
                sigmaBLE = sigBLE;
                sigLevel = sig_level;
            }

            void setBlpPulling(ePullingType type, double pulling_distance, double pulling_sigma = 0.5)
            {
                pulling_type = type;
                max_processing_distance_in_stop = (pulling_distance > 0) ? pulling_distance : 0;
                this->pulling_sigma = (pulling_sigma > 0) ? pulling_sigma : 0.5;
                isPullingEnabled = (pulling_type == pt_nonePulling) ? false : true;
            }

            void setMaxProcessingDistanceInStop(Float max_processing_distance_in_stop)
            {
                this->max_processing_distance_in_stop = max_processing_distance_in_stop;
            }

            virtual bool enabled()
            {
                return isEnabled;
            }
            void  setUpdate( bool flag )
            {
                isEnabled = flag;
            }

            void setName( const std::string &objName )
            {
                name = objName;
            }

            void print(std::ostream  &os) const
            {
                os << name << "<UpdBLEProximity>" << ": " << (isEnabled ? 1 : 0) << " " << (update_was_called ? 1 : 0) << " " << (posValid ? 1 : 0)
                    << " " << posBLE.x << " " << posBLE.y << " " << posBLE.level << " " << sigBLE << " " << sig_level << " " << distance_to_beacon
                    << " " << (int)pulling_type << " " << max_processing_distance_in_stop << " " << operatingSigBle
                    << " " << (is_step ? 1 : 0) << " " << (is_near ? 1 : 0) << " " << num_beacons_in_update << std::endl;
            }

        private:
            
            virtual void update(IFilter *pFilter)
            {
                // reset state variables
                update_was_called = false;
                num_beacons_in_update = 0;

                if (enabled() && posValid)
                {
                    if (locationList.size() > 0)
                    {
                        is_step = (pIFF != 0) ? pIFF->getPrediction().is_motion : true; // WARNING: currently is_step = is_motion; it should be revised and is_step and is_motion using should be seperated for pulling and BLP updater
                        bool isMultyUpdateAlowed  = !((isPullingEnabled == true) && (is_step == false));

                        size_t n = locationList.size();

                        //for (auto it = locationList.begin(); it != (locationList.begin() + num_beacons_in_update); ++it)
                        for (int i = n ; i > 0; i--) // we need bacward cycle to store nearest beacon in distance_to_beacon
                        {
                            auto it = locationList.begin() + i - 1;
                            update_was_called = true;
                            posBLE.x = it->x;
                            posBLE.y = it->y;
                            posBLE.level = it->z;
                            posValid = it->valid;

                            double current_sigBLE = std::max(it->rms_xy, sigBLE); // sigma BPL adjustment disabled 
                            
                            distance_to_beacon = it->metric;
                            if ( (true == isMultyUpdateAlowed) && (it->p != -2) )
                            {
                                update_for_one_beacon(pFilter, posBLE, distance_to_beacon, current_sigBLE);
                            }
                            if ( (false == isMultyUpdateAlowed ) && (it->p == -2) )
                            {
                                update_for_one_beacon(pFilter, posBLE, distance_to_beacon, current_sigBLE);
                            }
                        }
                        posValid = false;
                        locationList.clear();
                    }
                }
            }
            
            void update_for_one_beacon(IFilter *pFilter, Position<Float> posBLE, double expected_distance, double sigBLE)
            {
                if (enabled() && posValid)
                {
                    counter++;
                    posValid = false;

                    int N = pFilter->getParcticlesCount();
                    Particle *state_est = pFilter->getState();
                    Particle *state_prop = pFilter->getPrediction();

                    int floor = (pIFF != 0) ? round(pIFF->getPrediction().floor) : int(posBLE.level); /// !!! bug - to use floor not from prediction but from estimation or from particle
                    is_near = isPullingEnabled && (expected_distance < max_processing_distance_in_stop) /*&& (floor == int(posBLE.level))*/; // patch: disable floor checking for proximity pooling

                    pFilter->inPulling = (!is_step) && is_near;

                    if (pulling_type == pt_softPulling)
                    {
                        operatingSigBle = ((!is_step) && is_near) ? (pulling_sigma + operatingSigBle) / 2 : sigBLE; // soft pulling
                    }
                    else if (pulling_type == pt_instantPulling)
                    {
                        operatingSigBle = ((!is_step) && is_near) ? pulling_sigma : sigBLE; // instant pulling
                    }
                    else
                    {
                        operatingSigBle = sigBLE; // no pulling 
                    }

                    if (is_step || is_near)
                    {
						// updating both height and XY
                        num_beacons_in_update++;

                        Float sigBLE2 = operatingSigBle * operatingSigBle;
                        Float sig_level2 = sig_level * sig_level;

                        for (int i = 0; i < N; ++i)
                        {
                            Float dx = norm2D(state_prop[i].pos, posBLE);
                            //expected_distance = 0;
                            dx = std::abs(dx - expected_distance);
                            Float dx2 = dx * dx;

                            Float w = 1 / (operatingSigBle * sqrt(2. * M_PI)) * exp(-dx2 / (2.* sigBLE2));
                            state_est[i].w *= w;
                            
                            Float dz = state_prop[i].pos.level - posBLE.level;
                            Float dz2 = dz * dz;

                            w = 1 / (sqrt(2 * M_PI) * sig_level) * exp(-dz2 / (2. * sig_level2));
                            state_est[i].w *= w;
                        }
                    }

					else
					{
						// only the height update
						num_beacons_in_update++;

						Float sig_level2 = sig_level * sig_level;

						for (int i = 0; i < N; ++i)
						{
							Float dz = state_prop[i].pos.level - posBLE.level;
							Float dz2 = dz * dz;

							Float w = 1 / (sqrt(2 * M_PI) * sig_level) * exp(-dz2 / (2. * sig_level2));
							state_est[i].w *= w;
						}
					}					
                }
            }

            IBLEProximity **pIBLEProximity;
            IFFData<> *pIFF;
            bool  isEnabled;
            bool isPullingEnabled;
            std::string name;
            Position<Float> posBLE;
            bool  posValid;
            Float sigBLE, operatingSigBle;
            Float sig_level;
            int counter;
            std::vector<WiFi_Location> locationList;
            bool update_was_called;
            bool is_near;
            bool is_step;
            ePullingType pulling_type;
            double max_processing_distance_in_stop;
            double distance_to_beacon;
            double pulling_sigma;
            int num_beacons_in_update;
    };
}

#endif  // _UPDATE_BLE_PROXIMITY_H
