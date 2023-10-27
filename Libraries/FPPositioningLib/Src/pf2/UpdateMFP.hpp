/**
* \copyright       Copyright (C) SPIRIT Navigation, LLC., 2014
* \brief           Magnitude MFP updater implementation
* \ingroup         PF
* \file            UpdateMFP.hpp
* \author          M.Frolov
* \date            28.11.2014
*/

#ifndef UPDATE_MODEL_MFP_H
#define UPDATE_MODEL_MFP_H
#define _USE_MATH_DEFINES

#include "IPF.hpp"
#include "../magnetic/MFP.h"
#include <cmath>


namespace PF
{
    static const Float default_mfp_threshold = 0;

    /**
    * magnitude MFP updater (obsolete)
    */
    class UpdateMFP : public UpdateSource
    {
        public:
            UpdateMFP()
            {
                initialize( 0 );
            }
            UpdateMFP( tMFP **pMfp )
            {
                initialize( pMfp );
            }

            virtual ~UpdateMFP()
            {
                delete [] pMfpParticles;
                delete [] pMfpWeights;
            };

            void initialize( tMFP **pMfp )
            {

                isEnabled = false;
                validity = false;
                pMFP = pMfp;
                pMfpParticles = 0;
                pMfpWeights = 0;
                mfpSize = 0;
                weightThreshold = default_mfp_threshold; //NOT USED
                sigMag = 0;
                name = "UpdateMFP";
            }

            void setMagnitude( Float _magnitude, bool valid )
            {
                magnitude = _magnitude;
                validity = valid;
            }

            void setSig( Float sigmaMag )
            {
                sigMag = sigmaMag;
            }

            void getSig( Float &sigmaMag )
            {
                sigmaMag = sigMag;
            }

            void setWeightThreshold( Float value )
            {
                weightThreshold = value;
            }

            void getWeightThreshold( Float &value )
            {
                value = weightThreshold;
            }

            virtual bool enabled()
            {
                return isEnabled;
            }
            void  setUpdate( bool flag )
            {
                isEnabled = flag;
            }

            void setName(const std::string &objName)
            {
                name = objName;
            }

            void print( std::ostream  &os ) const
            {
                void * ptr = ( pMFP == 0 ) ? 0 : ( *(pMFP) );
                os << name << "<UpdMfp>" << ": " << (isEnabled ? 1 : 0) << " " << ptr << " " << mfpSize << " "
                   << validity  << " " <<   magnitude << " "
                   << sigMag << std::endl;
            }

        private:
            virtual void update( IFilter *pFilter )
            {
                bool has_map = ( pMFP == 0 ) ? false : ( *pMFP != 0 );

                if( has_map && enabled() && validity )
                {
                    int N = pFilter->getParcticlesCount();
                    resizeMfpArrays( N );
                    Particle *state_est = pFilter->getState();
                    Particle *state_prop = pFilter->getPrediction();


                    for( int i = 0; i < N; ++i )
                    {
                        pMfpParticles[i].X  = state_prop[i].pos.x * 100;
                        pMfpParticles[i].Y  = state_prop[i].pos.y * 100;
                        pMfpParticles[i].Floor  = ( int )floor( state_prop[i].pos.level + 0.5 );
                    }

                     // weightThreshold is NOT USED!
                    ( *pMFP )->CheckParticles( pMfpParticles, pMfpWeights, N, &magnitude, 1, true, 0. );


                    for( int i = 0; i < N; ++i )
                    {
                        state_est[i].w *= pMfpWeights[i];
                    }

                }
            }

            void resizeMfpArrays( int new_size )
            {
                if( new_size != mfpSize )
                {
                    delete [] pMfpParticles;
                    delete [] pMfpWeights;

                    pMfpParticles = new tMFP_Particle[new_size];
                    pMfpWeights   = new double[new_size];
                    mfpSize = new_size;
                }
            }


            tMFP   **pMFP;
            Float  sigMag;
            double magnitude;
            bool   isEnabled;
            bool   validity;

            tMFP_Particle *pMfpParticles;
            double        *pMfpWeights;
            int           mfpSize;
            double        weightThreshold;
            std::string   name;
    };
}

#endif
