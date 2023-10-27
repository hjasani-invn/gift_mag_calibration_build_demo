/**
* \copyright       Copyright (C) InvenSense 2017
* \brief           Position update
* \ingroup         PF
* \file            UpdateExternalPos.hpp
* \author          M.Frolov
* \date            25.05.2017
*/

#ifndef _UPDATE_EXTERNAL_POSITION_H
#define _UPDATE_EXTERNAL_POSITION_H

#include "IPF.hpp"
#include "IFFSettings.hpp"
#include <cmath>
#include "IFFSettings.hpp"
#include <eigen/Core>


namespace PF
{

    static const Float default_pos_sigma = 1.;

    /**
    * location service(or GNSS) updater
    */
    class UpdateExternalPosition : public UpdateSource
    {
        public:
            UpdateExternalPosition()
            {
                initialize( 0 );
            }
            UpdateExternalPosition( IFFData<> *pIFFData )
            {
                initialize( pIFFData );
            }

            ~UpdateExternalPosition() {};

            void initialize( IFFData<> *pIFFData )
            {
                pIFF = pIFFData;
                isEnabled = false;
                name = "UpdateExternalPos";
            }

            void setIFF( IFFData<> *pIFFData )
            {
                pIFF = pIFFData;
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

            void print( std::ostream  &os ) const
            {
                PositionWithUncertainties pos = {};
                void *ptr = pIFF;

                if ( pIFF != 0 )
                {
                    pos = pIFF->getExternalPosition();
                }


                os << name << "<UpdEP>" << ": " << (isEnabled ? 1 : 0) << " " << ptr << " " << (pos.valid ? 1 : 0) << " "
                   << pos.timestamp << " " << pos.x << " " << pos.y << " " << pos.z << " " << pos.heading << " "
                   << pos.cov_xy[0][0] << " " << pos.cov_xy[0][1] << " " << pos.cov_xy[1][1] << " " << pos.std_z << " " << pos.std_h << " "
                   << std::endl;
            }

        private:
            virtual void update( IFilter *pFilter )
            {
                bool pos_valid = ( pIFF != 0 ) ? pIFF->getExternalPosition().valid : false;


                if ( enabled() && pos_valid )
                {
                    int N = pFilter->getParcticlesCount();
                    Particle *state_est = pFilter->getState();
                    Particle *state_pred = pFilter->getPrediction();
                    PositionWithUncertainties pos = pIFF->getExternalPosition();
                    Eigen::Matrix<Float, 2, 1> ref, pred, d_pos;
                    Eigen::Matrix<Float, 2, 2> invCov;
                    Float d_z2, sig2_z;

                    ref( 0, 0 ) = pos.x;
                    ref( 1, 0 ) = pos.y;

                    invCov( 0, 0 ) = pos.cov_xy[0][0];
                    invCov( 0, 1 ) = pos.cov_xy[0][1];
                    invCov( 1, 0 ) = pos.cov_xy[1][0];
                    invCov( 1, 1 ) = pos.cov_xy[1][1];


                    invCov = invCov.inverse().eval();
                    sig2_z = 2 * pow( pos.std_z, 2 );

                    for ( int i = 0; i < N; ++i )
                    {
                        pred( 0, 0 ) = state_pred[i].pos.x;
                        pred( 1, 0 ) = state_pred[i].pos.y;
                        d_pos = pred - ref;
                        d_z2 = ( state_pred[i].pos.level - pos.z ) * ( state_pred[i].pos.level - pos.z );

                        double e = ( d_pos ).transpose() * invCov * ( d_pos );
                        Float w = exp( -e / 2 );
                        w *= exp( -d_z2 / sig2_z );
                        state_est[i].w *= w;
                    }
                }
            }

            IFFData<> *pIFF;
            bool  isEnabled;
            std::string name;
    };
}

#endif
