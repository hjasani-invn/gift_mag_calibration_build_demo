/**
* \copyright       Copyright (C) SPIRIT Navigation, LLC., 2014
* \brief           PF initialzers implementation.
* \ingroup         PF
* \file            Initializer.hpp
* \author          M.Frolov D.Churikov
* \date            28.11.2014
*/

#ifndef PF_INITIALIZER_H
#define PF_INITIALIZER_H

#include <ostream>
#include <string>
#include "IPF.hpp"
#include "RBFInitializer.hpp"
#include "ff_config.h"

#include "../wifi/wifi_if.hpp"
#include "../magnetic/MFP.h"


namespace PF
{


    /** Particle filter initializer based on state of another PF,
    * implements FilterInitializer interface
    */
    class InitializerPF: public FilterInitializer
    {
        public:
            /**
            * Creates terminal initialzer object
            * \param[in] objName initializer name, used for debug purposes
            */
            InitializerPF( std::string objName = "InitializerPF" ): FilterInitializer()
            {
                pFilter = 0;
                name = objName;
				current_position_uncertainty = 0.0;
            };
            /**
            * Creates initialzer object and link it with next initializer
            * \param[in] next next initializer in the chain
            * \param[in] objName initializer name, used for debug purposes
            */
            InitializerPF( FilterInitializer *next, std::string objName = "InitializerPF" ): FilterInitializer( next )
            {
                pFilter = 0;
                name = objName;
				current_position_uncertainty = 0.0;
            };
            /** Sets base particle-filter, wich state is used for initialization
            * \param[in] pF pointer to base-PF
            */
            void setPF( IFilter *pF )
            {
                pFilter = pF;
            };
            IFilter * getPF(IFilter *pF)
            {
                return pF;
            };
            ~InitializerPF() {};
            /// debug output
            virtual void print( std::ostream &os ) const
            {
                bool valid = ( pFilter != 0 ) ? pFilter->initialized() : false;
                os << name << ": " << this->next << " " << pFilter << " "
                   << valid << std::endl;
            }

        protected:
            virtual bool success()
            {
                PF::Particle state;
                bool result = ( pFilter != 0 ) ? pFilter->estimate( state ) : false;
                return result;
            }
            bool postInit( Particle &state )
            {
                return true;
            }

			void tryGetBestInit(FilterInitializer** current_best_initializer, double& current_best_uncertainty)
			{
				// this initializer can't be chosen over any other
			}

            virtual bool tryInit( IFilter *pF, Float p )
            {
                PF::Particle state;
                //bool success = ( pFilter != 0 ) ? pFilter->estimate( state ) : false;
                bool result = success();

                if( result == true )
                {
                    PF::Particle *state_est = pF->getState();
                    PF::Particle *state_prop = pF->getPrediction();
                    const int N = pF->getParcticlesCount();
                    const int K = pFilter->getParcticlesCount();

                    for( int i = 0; i < N; ++i )
                    {
                        Float r = pF->rand();

                        if( r < p )
                        {
                            //maybe resample is needed
                            state = pFilter->getState()[i % K];
                            //state.w = 1. / N;
                            state.w *= Float( K ) / N;
                            postInit( state );

                            state_est[i] = state_prop[i] = state;
                        }
                    }
                }

                return result;
            }

            //        private:
            IFilter *pFilter;
            //std::string name;

    };

    /** Rao-blackwellized particle filter initializer based on state of another PF,
    * implements FilterInitializer interface
    */
    class InitializerPF_RBF: public InitializerPF
    {
        public:
            /**
            * Creates terminal RBF initialzer object
            * \param[in] rbf_init RBF initializer
            * \param[in] objName initializer name, used for debug purposes
            */
            InitializerPF_RBF( RBFInitializer *rbf_init, std::string objName = "InitializerPF_RBF" )
                : InitializerPF( objName ), rbf_init( rbf_init ) {};
            /**
            * Creates RBF initialzer object
            * \param[in] rbf_init RBF initializer
            * \param[in] next next initializer in the chain
            * \param[in] objName initializer name, used for debug purposes
            */
            InitializerPF_RBF( RBFInitializer *rbf_init, FilterInitializer *next, std::string objName = "InitializerPF_RBF" )
                : InitializerPF( next, objName ), rbf_init( rbf_init ) {};

            void print( std::ostream &os ) const
            {
                InitializerPF::print( os );

                if ( rbf_init )
                {
                    rbf_init->print( os );
                }
            }

        private:
            bool tryInit( IFilter *pF, Float p )
            {
                n = 0;
                bool result = InitializerPF::tryInit( pF, p );

                if( result )
                {
                    rbf_init->incrementSig( 10 );
                }

                return result;
            }
            bool postInit( Particle &state )
            {
                bool result = ( rbf_init != NULL );

                if( result )
                {
                    state.rbf = rbf_init->getRBF();

                    if( ( ++n & 1 ) == 0 )
                    {
                        state.rbf.P = rbf_init->getDefault().P;
                    }
                }

                return result;
            }
            RBFInitializer *rbf_init;
            unsigned int n;

    };

    /** Particle filter initializer based on WiFi position,
    * implements FilterInitializer interface
    */
    class InitializerWiFi: public FilterInitializer
    {
        public:
            /**
            * Creates terminal PF initialzer object
            * \param[in] objName initializer name, used for debug purposes
            */
            InitializerWiFi( std::string objName = "InitializerWiFi" ): FilterInitializer()
            {
                last_pos = 0;
                name = objName;
            };
            /**
            * Creates PF initialzer object
            * \param[in] next next initializer in the chain
            * \param[in] objName initializer name, used for debug purposes
            */
            InitializerWiFi( FilterInitializer *next,  std::string objName = "InitializerWiFi" ): FilterInitializer( next )
            {
                last_pos = 0;
                name = objName;
            };
            ~InitializerWiFi() {};


            /**
            * sets data source
            * \param[in] wifi_loc pointer to the wifi position
            */
            void setWiFiPos( const WiFi_Location *wifi_loc )
            {
                last_pos = wifi_loc;
            }

            /// debug output
            virtual void print( std::ostream &os ) const
            {
                WiFi_Location l;

                if( last_pos != 0 )
                {
                    l = *last_pos;
                }

                os << name << ": " << this->next << " " << l.valid << " "
                   << l.x << " " << l.y << " " << l.z << " " <<  l.p << " " << std::endl;
            }

        protected:
            bool postInit( Particle &state )
            {
                return true;
            }

            virtual bool tryInit( IFilter *pF, Float p )
            {
                bool success = ( last_pos != 0 ) ? last_pos->valid : false;

                if( success == true )
                {
                    PF::Particle *state_est = pF->getState();
                    PF::Particle *state_prop = pF->getPrediction();
                    int N = pF->getParcticlesCount();
                    Particle state;

                    for( int i = 0; i < N; ++i )
                    {
                        Float r = pF->rand();

                        if( r < p )
                        {
                            state.w = 1. / N;
                            state.h = ( 2 * M_PI ) * pF->rand();
                            state.pos.x = last_pos->x + 5.0 * pF->randn() / sqrt( 2. );
                            state.pos.y = last_pos->y + 5.0 * pF->randn() / sqrt( 2. );
                            state.pos.level = floor( last_pos->z + 1 * pF->randn() + 0.5 );
                            postInit( state );

                            state_est[i] = state_prop[i] = state;
                        }
                    }
                }

                return success;

            }
            const WiFi_Location *last_pos;
            //std::string name;
    };

    /** RBPF filter initializer based on WiFi position,
    * implements FilterInitializer interface
    */
    class InitializerWiFi_RBF: public InitializerWiFi
    {
        public:
            /**
            * Creates terminal RBF initialzer object
            * \param[in] rbf_init RBF initializer
            * \param[in] objName initializer name, used for debug purposes
            */
            InitializerWiFi_RBF( RBFInitializer *rbf_init, std::string objName = "InitializerWiFi_RBF" )
                : InitializerWiFi( objName ), rbf_init( rbf_init ) {};

            /**
            * Creates RBF initialzer object
            * \param[in] rbf_init RBF initializer
            * \param[in] next next initializer in the chain
            * \param[in] objName initializer name, used for debug purposes
            */
            InitializerWiFi_RBF( RBFInitializer *rbf_init, FilterInitializer *next,  std::string objName = "InitializerWiFi_RBF" )
                : InitializerWiFi( next, objName ), rbf_init( rbf_init ) {};

            void print( std::ostream &os ) const
            {
                InitializerWiFi::print( os );

                if ( rbf_init )
                {
                    rbf_init->print( os );
                }
            }

        private:
            bool tryInit( IFilter *pF, Float p )
            {
                n = 0;
                bool result = InitializerWiFi::tryInit( pF, p );

                if( result )
                {
                    rbf_init->incrementSig( 10 );
                }

                return result;
            }

            bool postInit( Particle &state )
            {
                bool result = ( rbf_init != NULL );
                static unsigned int kk = 0;

                if( result )
                {
                    state.rbf = rbf_init->getRBF();

                    if( ( ++n & 1 ) == 0 )
                    {
                        state.rbf.P = rbf_init->getDefault().P;
                    }
                }

                return result;
            }
            RBFInitializer *rbf_init;
            unsigned int n;
    };

    /** Initialize PF based on fine known position
    */
    class InitializerFine: public FilterInitializer
    {
        public:
            /** Creates terminal initialzer object
            * \param[in] objName initializer name, used for debug purposes
            */
            InitializerFine( std::string objName = "InitializerFine" ): FilterInitializer()
            {
                f_start = false;
                name = objName;
            };
            /** Creates linked initialzer object
            * \param[in] next next initializer in the chain
            * \param[in] objName initializer name, used for debug purposes
            */
            InitializerFine( FilterInitializer *next, std::string objName = "InitializerFine" ): FilterInitializer( next )
            {
                f_start = false;
                name = objName;
            };
            ~InitializerFine() {};
            void setFinePos( Float _x, Float _y, Float _level, Float _heading )
            {
                x = _x;
                y = _y;
                level = _level;
                heading = _heading;
                f_start = true;
            }

            /// for debug purposes
            virtual void print( std::ostream &os ) const
            {
                os << name << ": " << f_start << " "
                   << x << " " << y << " " << level << " " << heading << std::endl;
            }

        protected:
            bool postInit( Particle &state )
            {
                return true;
            }
            virtual bool tryInit( IFilter *pF, Float p )
            {
                bool success = f_start;

                if( f_start == true )
                {
                    f_start = false;
                    PF::Particle *state_est = pF->getState();
                    PF::Particle *state_prop = pF->getPrediction();
                    PF::Particle state;
                    int N = pF->getParcticlesCount();

                    for( int i = 0; i < N; ++i )
                    {
                        Float r = pF->rand();

                        if( r < p )
                        {
                            state.w = 1. / N;
                            state.h = heading;
                            state.pos.x = x;
                            state.pos.y = y;
                            state.pos.level = level;
                            postInit( state );

                            state_est[i] = state_prop[i] = state;
                        }
                    }
                }

                return success;

            }
            Float x, y, level, heading;
            bool f_start;
            //std::string name;
    };

    /** Initialize RBPF based on fine known position
    */
    class InitializerFine_RBF: public InitializerFine
    {
        public:
            /** Creates terminal initialzer object
            * \param[in] rbf_init pointer to the  RB-state initializer
            * \param[in] objName initializer name, used for debug purposes
            */
            InitializerFine_RBF( RBFInitializer *rbf_init, std::string objName = "InitializerFine_RBF" )
                : InitializerFine( objName ), rbf_init( rbf_init ) {};
            /** Creates linked initialzer object
            * \param[in] rbf_init pointer to the  RB-state initializer
            * \param[in] next next initializer in the chain
            * \param[in] objName initializer name, used for debug purposes
            */
            InitializerFine_RBF( RBFInitializer *rbf_init, FilterInitializer *next, std::string objName = "InitializerFine_RBF" )
                : InitializerFine( next, objName ), rbf_init( rbf_init ) {};

            void print( std::ostream &os ) const
            {
                InitializerFine::print( os );

                if ( rbf_init )
                {
                    rbf_init->print( os );
                }
            }

        private:
            bool tryInit( IFilter *pF, Float p )
            {
                bool result = InitializerFine::tryInit( pF, p );

                if( result )
                {
                    rbf_init->incrementSig( 10 );
                }

                return result;
            }

            bool postInit( Particle &state )
            {
                bool result = ( rbf_init != NULL );

                if( result )
                {
                    state.rbf = rbf_init->getRBF();
                }

                return result;
            }
            RBFInitializer *rbf_init;
    };

    /** Initialize PF with unoform ditribution with MFP map restrictions
    */
    class InitializerMFP: public FilterInitializer
    {
        public:
            /** Creates terminal initialzer object
            * \param[in] mfp module instance
            * \param[in] objName initializer name, used for debug purposes
            */
            InitializerMFP( tMFP **mfp, std::string objName = "InitializerMFP" ): FilterInitializer()
            {
                name = objName;
                ppMFP = mfp;
				current_position_uncertainty = 0.0;
            };
            /** Creates linked initialzer object
            * \param[in] mfp module instance
            * \param[in] next next initializer in the chain
            * \param[in] objName initializer name, used for debug purposes
            */
            InitializerMFP( tMFP **mfp, FilterInitializer *next, std::string objName = "InitializerMFP" ): FilterInitializer( next )
            {
                name = objName;
                ppMFP = mfp;
				current_position_uncertainty = 0.0;
            };

            ~InitializerMFP() {};

            virtual void print( std::ostream &os ) const
            {
                void *ptr = ( ppMFP != 0 ) ? *ppMFP : 0;
                os << name << ": " << ptr << std::endl;
            }
        protected:
            //private:
            bool postInit( Particle &state )
            {
                return true;
            }

			void tryGetBestInit(FilterInitializer** current_best_initializer, double& current_best_uncertainty)
			{
				// this initializer can't be chosen over any other
			}
			
            virtual bool tryInit( IFilter *pF, Float p )
            {
                bool ok = ( ppMFP != 0 && *ppMFP != 0 && ( *ppMFP )->getRandomPosCount() > 0 );

                if( ok )
                {
                    //const int HEADING_DISCRETS = 8;
                    //const int PARTICLES_PER_CELL_MIN = 2 * HEADING_DISCRETS;
                    const int HEADING_DISCRETS = 16;
                    const int PARTICLES_PER_CELL_MIN = 1 * HEADING_DISCRETS;
                    const int PARTICLES_PER_CELL_MAX = 4 * HEADING_DISCRETS;
                    //const int MAX_PARTICLES_NUMBER = 50000;
                    const RBF<Float> rbf_default( 100 );


                    PF::Particle state;
                    PF::Particle *state_est;
                    PF::Particle *state_prop;
                    int N = pF->getParcticlesCount();
                    int cells = ( *ppMFP )->getRandomPosCount();
                    int ppc = N / cells;

                    double h[HEADING_DISCRETS];
                    bool random_heading = false;

                    for ( int i = 0; i < HEADING_DISCRETS; ++i )
                    {
                        h[i] = i * 2 * M_PI / HEADING_DISCRETS;
                    }

                    if( ppc < PARTICLES_PER_CELL_MIN )
                    {
                        N =  cells * PARTICLES_PER_CELL_MIN;
                    }
                    else if( ppc > PARTICLES_PER_CELL_MAX )
                    {
                        N =  cells * PARTICLES_PER_CELL_MAX;
                    }

                     //limiting maximum number of particles to 50k
                    if (N > PF_MAX_PARTICLES_NUMBER)
                    {
                        N = PF_MAX_PARTICLES_NUMBER;
                        ppc = N /cells;
                        random_heading = true;
                        if (ppc < 4)
                        {
                            return false;
                        }
                    }

                    pF->resize(N);

                    state_est = pF->getState();
                    state_prop = pF->getPrediction();

                    for( int i = 0; i < N; ++i )
                    {
                        Float r = pF->rand();

                        if( r < p )
                        {
                            // TODO: if p < 1, distribution should be uniform
                            // position and heading should be random for any "p" (uniform)

                            //unsigned int idx = pF->randi();
                            unsigned int idx = i;
                            tMFP_Particle3D cell = ( *ppMFP )->getRandomPos( idx );

                            state.w = 1. / N;
                            if (random_heading)
                            {
                                state.h = 2. * M_PI * pF->rand();
                            }
                            else
                            {
                                idx = (i / (cells)) % HEADING_DISCRETS;
                                state.h = h[idx];
                            }

                            state.pos.x = cell.X / 100.;// +1 * (pF->rand() - 0.5);
                            state.pos.y = cell.Y / 100.;// +1 * (pF->rand() - 0.5);
                            state.pos.level = cell.Floor;
                            postInit( state );

                            const int N_K = N - ( PARTICLES_PER_CELL_MIN * cells / 2 );
			    
                            state_est[i] = state_prop[i] = state;
                        }
                    }
                }

                return ok;//success;

            }

            tMFP **ppMFP;
    };

    /** Initialize RBPF based on collected MFP map
    */
    class   InitializerMFP_RBF : public InitializerMFP
    {
        public:
            /** Creates terminal initialzer object
            * \param[in] rbf_init pointer to the  RB-state initializer
            * \param[in] mfp module instance
            * \param[in] objName initializer name, used for debug purposes
            */
            InitializerMFP_RBF( RBFInitializer *rbf_init, tMFP **mfp, std::string objName = "InitializerMFP_RBF" )
                : InitializerMFP( mfp, objName ), rbf_init( rbf_init ) {};
            /** Creates linked initialzer object
            * \param[in] rbf_init pointer to the  RB-state initializer
            * \param[in] mfp module instance
            * \param[in] next next initializer in the chain
            * \param[in] objName initializer name, used for debug purposes
            */
            InitializerMFP_RBF( RBFInitializer *rbf_init, tMFP **mfp, FilterInitializer *next, std::string objName = "InitializerMFP_RBF" )
                : InitializerMFP( mfp, next, objName ), rbf_init( rbf_init ) {};

            void print( std::ostream &os ) const
            {
                InitializerMFP::print( os );

                if ( rbf_init )
                {
                    rbf_init->print( os );
                }
            }

        private:

            bool tryInit( IFilter *pF, Float p )
            {
                bool result = InitializerMFP::tryInit( pF, p );

                if( result )
                {
                    rbf_init->incrementSig( 10 );
                }

                return result;
            }
            bool postInit( Particle &state )
            {
                bool result = ( rbf_init != NULL );

                if( result )
                {
                    state.rbf = rbf_init->getRBF();
                }

                return result;
            }
            RBFInitializer *rbf_init;
    };

}


#endif
