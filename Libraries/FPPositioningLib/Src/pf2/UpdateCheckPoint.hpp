/**
* \copyright       Copyright (C) SPIRIT Navigation, LLC., 2014
* \brief           Checkpoint updater implementation [obsolete]
* \ingroup         PF
* \file            UpdateCheckPoint.hpp
* \author          M.Frolov
* \date            28.11.2014
*/

#ifndef _UPDATE_MODEL_CHECKPOINT_H
#define _UPDATE_MODEL_CHECKPOINT_H
#define _UPDATE_MODEL_CHECKPOINT_H

#include "IPF.hpp"
#include <vector>
#include <cmath>


namespace PF
{

    static const Float default_pos_sigma = 1.;

    class UpdateCheckPoint : public UpdateSource
    {
        public:
            UpdateCheckPoint()
            {
                initialize( 0 );
            }
            UpdateCheckPoint( std::vector<PF::Particle>  **checkPoints )
            {
                initialize( checkPoints );
            }

            virtual ~UpdateCheckPoint() {};

            void initialize( std::vector<PF::Particle>  **checkPoints )
            {
                sigPos =  default_pos_sigma; // m
                posValid = false;
                isEnabled = false;
                pCheckPoints = checkPoints;
                time = 0;
                time_upd = 0;
                name = "UpdateCheckPoint";
            }


            void setTime( int64_t timestamp_ms )
            {
                time = timestamp_ms;
            }

            void setCheckPointId( unsigned int id, int64_t timestamp_ms )
            {
                posValid = false;

                if( pCheckPoints != 0 )
                {
                    if( id >= 0 && id < ( *pCheckPoints )->size() )
                    {
                        checkPointState = ( **pCheckPoints ) [id];
                        posValid = true;
                        time_upd = timestamp_ms;
                    }
                }
            }


            void setSig( Float sigmaPos )
            {
                sigPos = sigmaPos;
            }

            void getSig( Float &sigmaPos )
            {
                sigmaPos = sigPos;
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
                void * ptr = ( pCheckPoints == 0 ) ? 0 : ( *(pCheckPoints) );
                uint64_t dt = ( ( time - time_upd ) > 0 ) ? ( time - time_upd ) : -( time - time_upd );

                os << name << "<UpdCP>" << ": " << (isEnabled ? 1 : 0) << " " << ptr << " " << (posValid ? 1 : 0 )<< " "
                   << checkPointState.pos.x    << " " << checkPointState.pos.y << " "
                   << sigPos   << " " << time << " "
                   << time_upd << " " << std::endl;
            }

        private:
            virtual void update( IFilter *pFilter )
            {
                uint64_t dt = ( ( time - time_upd ) > 0 ) ? ( time - time_upd ) : -( time - time_upd );

                if( enabled() && posValid && dt < 500 )
                {
                    int N = pFilter->getParcticlesCount();
                    Particle *state_est = pFilter->getState();
                    Particle *state_prop = pFilter->getPrediction();

                    for( int i = 0; i < N; ++i )
                    {
                        Float dx = norm2D( state_prop[i].pos, checkPointState.pos );
                        Float dx2 = pow( dx, 2 );
                        Float sigPos2 = 2 * pow( sigPos, 2 );

                        Float w = 1 / ( sigPos2 * M_PI ) * exp( -dx2 / sigPos2 );
                        state_est[i].w *= w;

                        //state_prop[i].pos.x =  checkPointState.pos.x + sigPos * pFilter->randn();
                        //state_prop[i].pos.y =  checkPointState.pos.y + sigPos * pFilter->randn();
                        state_prop[i].pos.level =  checkPointState.pos.level;

                    }

                    posValid = false;
                }
            }

            std::vector<PF::Particle>  **pCheckPoints;
            Float sigPos;
            PF::Particle checkPointState;
            int64_t time;
            int64_t time_upd;
            bool  isEnabled;
            bool  posValid;
            std::string name;
    };
}

#endif
