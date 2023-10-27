/**
* \copyright       Copyright (C) InvenSense 2017
* \brief           Position update
* \ingroup         PF
* \file            UpdateCollaboration.hpp
* \author          M.Pentyukhov
* \date            03.09.2020
*/

#ifndef _UPDATE_COLLABORATION_H
#define _UPDATE_COLLABORATION_H

#include "IPF.hpp"
#include "IFFSettings.hpp"
#include <cmath>
#include "IFFSettings.hpp"
#include <eigen/Core>

namespace PF
{
    /**
    * ble service updater
    */
    class UpdateBLECollaboration : public UpdateSource
    {
        public:
            UpdateBLECollaboration()
            {
                initialize( 0 );
            }
            UpdateBLECollaboration( IFFData<> *pIFFData )
            {
                initialize( pIFFData );
            }

            ~UpdateBLECollaboration() {};

            void initialize( IFFData<> *pIFFData )
            {
                pIFF = pIFFData;
                isEnabled = false;
                name = "UpdateBLECollaboration";
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


                os << name << "<UpdateBLECollaboration>" << ": " << (isEnabled ? 1 : 0) << " " << ptr << " " << (pos.valid ? 1 : 0) << " "
                   << pos.timestamp << " " << pos.x << " " << pos.y << " " << pos.z << " " << pos.heading << " "
                   << pos.cov_xy[0][0] << " " << pos.cov_xy[0][1] << " " << pos.cov_xy[1][1] << " " << pos.std_z << " " << pos.std_h << " "
                   << std::endl;
            }

        private:
            virtual void update( IFilter *pFilter )
            {
            }

            IFFData<> *pIFF;
            bool  isEnabled;
            std::string name;
    };
}

#endif
