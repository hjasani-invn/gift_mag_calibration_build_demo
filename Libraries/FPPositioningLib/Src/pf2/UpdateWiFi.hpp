/**
* \copyright       Copyright (C) SPIRIT Navigation, LLC., 2014
* \brief           WiFi updater
* \ingroup         PF
* \file            UpdateWiFi.hpp
* \author          M.Frolov
* \date            28.11.2014
*/

#ifndef UPDATE_MODEL_WIFI_H
#define UPDATE_MODEL_WIFI_H
#define _USE_MATH_DEFINES

#include "IPF.hpp"
#include "IFFSettings.hpp"
#include "../wifi/wifi_if.hpp"
#include <cmath>
#include <ostream>


namespace PF
{

    static const Float default_wifi_sigma = 10.;
    /**
    * WiFi/BLE updater, assumes normal distribution
    */
    class UpdateWiFi : public UpdateSource
    {
        public:
            UpdateWiFi()
            {
                initialize( 0, 0 );
            }
            UpdateWiFi(WiFi **pWifi, IFFData<> *pIFFData)
            {
                initialize(pWifi, pIFFData);
            }

            virtual ~UpdateWiFi() {};

            void initialize(WiFi **pWifi, IFFData<> *pIFFData)
            {
                sigWiFi =  default_wifi_sigma; // m
                posValid = false;
                isEnabled = false;
                pWiFi = pWifi;
                sig_level = 1.5;
                name = "UpdateWiFi";
                pIFF = pIFFData;
            }

            //void wifiMeas( WiFi_Measurement wifi_meas )
            //{
            //    WiFi_Location wifi_loc = (*pWiFi)->GetLocation( wifi_meas, 5 );
            //    posWiFi.x = wifi_loc.x;
            //    posWiFi.y = wifi_loc.y;
            //    posWiFi.z = wifi_loc.z;
            //    posValid  = wifi_loc.valid;
            //}

            void wifiPos( WiFi_Location wifi_loc )
            {
                posWiFi.x = wifi_loc.x;
                posWiFi.y = wifi_loc.y;
                posWiFi.level = wifi_loc.z;
                posValid  = wifi_loc.valid;
            }

            void setSig( Float sigmaWiFi, Float sigLevel )
            {
                sigWiFi = sigmaWiFi;
                sig_level = sigLevel;
            }

            void getSig( Float &sigmaWiFi, Float &sigLevel )
            {
                sigmaWiFi = sigWiFi;
                sigLevel = sig_level;
            }

            void setName(const std::string &objName)
            {
                name = objName;
            }

            virtual bool enabled()
            {
                return isEnabled;
            }

            void  setUpdate( bool flag )
            {
                isEnabled = flag;
            }

            void print( std::ostream  &os ) const
            {
                os << name << "<UpdWiFi>" << ": " << (isEnabled ? 1 : 0) << " " << (posValid ? 1 : 0) << " "
                   << posWiFi.x << " " << posWiFi.y << " " << posWiFi.level << " "
                   << sigWiFi   << " " << sig_level << std::endl;
            }

        private:
            virtual void update( IFilter *pFilter )
            {
                if( enabled() && posValid )
                {
                    //double time = pIFF->getPrediction().t;

                    int N = pFilter->getParcticlesCount();
                    Particle *state_est = pFilter->getState();
                    Particle *state_prop = pFilter->getPrediction();

                    bool is_motion = (pIFF != 0) ? pIFF->getPrediction().is_motion : true;
                    Float sigWiFi2 = (is_motion) ? (sigWiFi* sigWiFi) : (4. * sigWiFi* sigWiFi);
                    Float sig_level2 = (is_motion) ? (sig_level * sig_level) : (4. * sig_level * sig_level);

                    for( int i = 0; i < N; ++i )
                    {
                        Float dx = norm2D( state_prop[i].pos, posWiFi );
                        Float dx2 = dx * dx;
                        
                        Float w = 1 / (2 * M_PI * sigWiFi2) * exp(-dx2 / (2 * sigWiFi2));
                        state_est[i].w *= w;

                        Float dz = state_prop[i].pos.level - posWiFi.level;
                        Float dz2 = dz * dz;
                        
                        w = 1 / sqrt(2 * M_PI * sig_level2) * exp(-dz2 / (2 * sig_level2));
                        state_est[i].w *= w;

                    }
                }
            }

            WiFi **pWiFi;
            IFFData<> *pIFF;
            Float sigWiFi;
            Float sig_level;
            Position<Float> posWiFi;
            bool  isEnabled;
            bool  posValid;
            std::string name;
    };
}

#endif
