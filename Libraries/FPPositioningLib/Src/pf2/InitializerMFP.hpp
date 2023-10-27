/**
* \copyright       Copyright (C) SPIRIT Navigation, LLC., 2015
* \brief           Separate PF that provides magnetic start
* \ingroup         PF
* \file            InitializerMFP.hpp
* \author          M.Frolov
* \date            21.05.2015
*/

#ifndef MFP_PF_INITIALIZER_H
#define MFP_PF_INITIALIZER_H

#include "IPF.hpp"
#include <iostream>

#define PF_MFP_PARTICLE_COUNT 4000
#define PF_MFP_COUNTER_MAX 20
#define PF_MFP_COUNTER_INC 2
#define PF_MFP_COUNTER_DEC 1
#define PF_MFP_SUCCESS_SIG 3.0

namespace PF
{
    /**
    * This class is a wrapper for mag-start PF
    * implements automated start/stop of the PF
    * implements initializer interface
    */
    class InitializerMfpPF: public InitializerPF
    {
        public:
            InitializerMfpPF( std::string objName = "InitializerMfpPF" ): InitializerPF( objName )
            {
                initialize( 0 );
            }
            InitializerMfpPF( FilterInitializer *next, std::string objName = "InitializerMfpPF" ): InitializerPF( next, objName )
            {
                initialize( 0 );
            }

            ~InitializerMfpPF()
            {
                delete pFilter;
            }


            void setInitializer( FilterInitializer *pInit )
            {
                pFilter->setInitializer( pInit );
                pInitializer = pInit;
            }

            void setPredictionModel( PredictionModel *model )
            {
                pFilter->setPredictionModel( model );
            }

            void setUpdater( UpdateMFP3D *pUpdaterMFP, UpdateRBFBias *pRBF )
            {
                pFilter->clearUpdateModels();
                pFilter->clearRBFs();
                pFilter->addUpdateModel( pUpdaterMFP );
                pFilter->addRBFModel( pRBF );
            }

            void addUpdater( PF::UpdateSource *pUpd )
            {
                pFilter->addUpdateModel( pUpd );
            }

            void print( std::ostream  &os ) const
            {
                void * ptr = ( ppMFP == 0 ) ? 0 : ( *( ppMFP ) );
                os << name << ": " << cnt << " " << ptr << " " << step << " "
                   << std << std::endl;

                pFilter->print( os );

                if ( pFilter->initialized() )
                {
                    PF::Particle pos_mfp, pos_mfp_std;
                    bool inTrackingMfp = pFilter->estimate( pos_mfp, pos_mfp_std );

                    os << name << ": " << pos_mfp.pos.x << " " << pos_mfp.pos.y << " " << pos_mfp.pos.level << " " << pos_mfp.h << " " <<
                       pos_mfp_std.pos.x << " " << pos_mfp_std.pos.y << " " << pos_mfp_std.pos.level << " " << pos_mfp_std.h << " "
                       << pos_mfp.rbf.b( 0 ) << " "
                       << pos_mfp.rbf.b( 1 ) << " "
                       << pos_mfp.rbf.b( 2 ) << " "
                       << sqrt( pos_mfp.rbf.P( 0, 0 ) ) << " "
                       << sqrt( pos_mfp.rbf.P( 1, 1 ) ) << " "
                       << sqrt( pos_mfp.rbf.P( 2, 2 ) ) << " "
                       //<< m.second[0] - pos_mfp.rbf.z(0) << " "
                       //<< m.second[1] - pos_mfp.rbf.z(1) << " "
                       //<< m.second[2] - pos_mfp.rbf.z(2) << " "
                       //<< m.second[0] - pos_mfp.rbf.b(0) << " "
                       //<< m.second[1] - pos_mfp.rbf.b(1) << " "
                       //<< m.second[2] - pos_mfp.rbf.b(2) << " "
                       << std::endl;
                    os << pos_mfp.rbf.C << std::endl;
                    os << pos_mfp.rbf.S << std::endl;
                    FilterInitializer *pInit = pInitializer;

                    while ( pInit != NULL )
                    {
                        pInit->print( os );
                        pInit = pInit->getNext();
                    }
                }
            }

            void process()
            {
                if( cnt > 1 )
                {
                    if( ( ++step % 1000 ) == 0 )
                    {
                        //                        pInitializer->init(pFilter, 0.50);
                    }
                    else if( ( step % 40 ) == 0 )
                    {
                        //                        pInitializer->init(pFilter, 0.50);
                    }
                    else if( ( step % 10 ) == 0 )
                    {
                        //                        pInitializer->init(pFilter, 0.15);
                        //pInitializer->init( pFilter, 0.10 );
                    }

                    pFilter->predict();
                    pFilter->update();
                    Particle est, est_std;
                    pFilter->estimate( est, est_std );
                    std = ( est_std.pos.x + est_std.pos.y ) / 2;

                    //std::cout << std << std::endl;
                }
                else if( cnt == 1 )
                {
                    //TODO!!! Unclear logic! this is similar to restart but without reseting rng seed and some internal counters
                    pFilter->setEnabled( false ); //TODO?! set different rng seed
                    pFilter->setEnabled( true ); //TODO?! set different rng seed
                    pFilter->resize(PF_MFP_PARTICLE_COUNT);
                }

                if( cnt > 0 ) cnt -= PF_MFP_COUNTER_DEC;
            }

            void restart()
            {
                pFilter->restart();
                pFilter->resize(PF_MFP_PARTICLE_COUNT);
                cnt = 0;
                step = 0;
            }

            IFilter* getPF()
            {
                return pFilter;
            }

        private:

            void initialize( tMFP **pMfp )
            {
                pFilter = new ParticleFilter( PF_MFP_PARTICLE_COUNT );
                dynamic_cast<ParticleFilter*>( pFilter )->setName( "ParticleFilter_MagStart" );
                pFilter->setEnabled( true );
                pFilter->setInjection( false );

                ppMFP = pMfp;
                cnt = 0;
                step = 0;
                std = 0;
            }

            bool postInit( Particle &state )
            {
                return true;
            }

            bool tryInit( IFilter *pF, Float p )
            {
                if (p >= 1.)
                { // enable filter when initialisation is required
                    if (cnt < PF_MFP_COUNTER_MAX)
                    {
                        cnt += PF_MFP_COUNTER_INC;
                    }
                }
                //else: do not enable filter for injection

                bool result = success();

                if( result )
                {
                    result = InitializerPF::tryInit( pF, p );
                }

                return result;
            }

            bool success()
            {

                bool result = false;
                bool valid = ( pFilter != 0 ) ? pFilter->initialized() : false;

                if( valid )
                {
                    if( std < PF_MFP_SUCCESS_SIG ) result = true;
                }

                return result;
            }

            int cnt;
            tMFP **ppMFP;
            Float std;
            FilterInitializer *pInitializer;
            unsigned int step;
    };



}

#endif //MFP_INITIALIZER_H