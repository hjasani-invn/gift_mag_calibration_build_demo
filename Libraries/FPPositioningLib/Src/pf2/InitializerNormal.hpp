/**
* \copyright       Copyright (C) InvenSense
* \brief           PF initialzer with cnown normal distribution.
* \ingroup         PF
* \file            InitializerNormal.hpp
* \author          M.Frolov
* \date            23.03.2016
*/

#ifndef PF_INITIALIZER_NORMAL_H
#define PF_INITIALIZER_NORMAL_H

#include <ostream>
#include <string>
#include "IPF.hpp"
#include "RBFInitializer.hpp"

namespace PF
{

    /** Initialize PF based on normal ditribution
    */
    class InitializerNormal : public FilterInitializer
    {
        public:
            /** Creates terminal initialzer object
            * \param[in] objName initializer name, used for debug purposes
            */
            InitializerNormal( std::string objName = "InitializerNormal" ) : FilterInitializer()
            {
                f_start = false;
                name = objName;
				max_start_position_uncertainty = 25.0;
				current_position_uncertainty = 0.0;
            };
            /** Creates linked initialzer object
            * \param[in] next next initializer in the chain
            * \param[in] objName initializer name, used for debug purposes
            */
            InitializerNormal( FilterInitializer *next, std::string objName = "InitializerNormal" ) : FilterInitializer( next )
            {
                f_start = false;
                name = objName;
				max_start_position_uncertainty = 25.0;
				current_position_uncertainty = 0.0;
            };
            ~InitializerNormal() {};

            void setPos( const PositionWithUncertainties &_position )
            {
                position = _position;
                f_start = true;
            }

			void setRBFinheritance(bool enable)
			{
				inherit_rbf = enable;
			}

            /// for debug purposes
            virtual void print( std::ostream &os ) const
            {
                os << name << ": "
                   << " f_start=" << f_start
                   << " time=" << position.timestamp
                   << " x=" << position.x
                   << " y=" << position.y
                   << " z=" << position.z
                   << " h=" << position.heading
                   << " cov_xx=" << position.cov_xy[0][0]
                   << " cov_xy=" << position.cov_xy[0][1]
                   << " cov_yy=" << position.cov_xy[1][1]
                   << " std_z=" << position.std_z
                   << " std_h=" << position.std_h << std::endl;
            }

        protected:
            bool postInit( Particle &state )
            {
                return true;
            }

			virtual void tryGetBestInit(FilterInitializer** current_best_initializer, double& current_best_uncertainty)
			{
				double self_uncertainty = 10000.0;
				bool result = evaluateUncertainty(self_uncertainty);

				if (result)
				{
					if (self_uncertainty < current_best_uncertainty)
					{
						current_best_uncertainty = self_uncertainty;
						*current_best_initializer = this;
					}
				}
			}

            virtual bool tryInit( IFilter *pF, Float p )
            {
				bool result = updateUncertaintyForCurrentTime();

				if (!result)
					return false;

                bool success = f_start;

                if ( f_start == true )
                {
                    f_start = false;
                    PF::Particle *state_est = pF->getState();
                    PF::Particle *state_prop = pF->getPrediction();
                    int N = pF->getParcticlesCount();
                    PF::Particle state;

                    for ( int i = 0; i < N; ++i )
                    {
                        Float r = pF->rand();

                        if ( r < p )
                        {
                            Float h = ( position.std_h >=0 ) ? (position.heading + position.std_h * pF->randn()) : state_prop[i].h;
                            Float x = pF->randn();
                            Float y = pF->randn();
                            y = position.y + y * std::sqrt(position.cov_xy[1][1]) + x * std::sqrt(position.cov_xy[0][1]);
                            x = position.x + x * std::sqrt(position.cov_xy[0][0]);

                            state.pos.x = x;
                            state.pos.y = y;
                            state.pos.level = position.z + position.std_z * pF->randn();
                            state.h = h;
                            state.w = 1. / N;
                            postInit( state );

							if ((inherit_rbf) && (state_est[i].w > 0.0))
							{
								state.rbf = state_est[i].rbf;
							}

                            state_est[i] = state_prop[i] = state;
                        }
                    }
                    if (p > 0.5)
                    {
                        for (int i = 0; i < N; ++i)
                        {
                            state_est[i].w = 1. / N;
                        }
                    }
                    else
                    {
                        // normalization
                        double sum = 0;
                        for (int i = 0; i < N; ++i)
                        {
                            sum += state_est[i].w;
                        }
                        for (int i = 0; i < N; ++i)
                        {
                            state_est[i].w /= sum;
                        }
                    }
               }

                return success;

            }

            bool f_start;
			bool inherit_rbf;
            //std::string name;
    };

    /** Initialize RBPF based on normal distribution
    */
    class InitializerNormal_RBF : public InitializerNormal
    {
        public:
            /** Creates terminal initialzer object
            * \param[in] rbf_init pointer to the  RB-state initializer
            * \param[in] objName initializer name, used for debug purposes
            */
            InitializerNormal_RBF( RBFInitializer *rbf_init, std::string objName = "InitializerNormal_RBF" )
                : InitializerNormal( objName ), rbf_init( rbf_init ) {};
            /** Creates linked initialzer object
            * \param[in] rbf_init pointer to the  RB-state initializer
            * \param[in] next next initializer in the chain
            * \param[in] objName initializer name, used for debug purposes
            */
            InitializerNormal_RBF( RBFInitializer *rbf_init, FilterInitializer *next, std::string objName = "InitializerNormal_RBF" )
                : InitializerNormal( next, objName ), rbf_init( rbf_init ) {};

            void print( std::ostream &os ) const
            {
                InitializerNormal::print( os );

                if ( rbf_init )
                {
                    rbf_init->print( os );
                }
            }

        private:
            bool tryInit( IFilter *pF, Float p )
            {
                bool result = InitializerNormal::tryInit( pF, p );

                if ( result )
                {
                    rbf_init->incrementSig( 10 );
                }

                return result;
            }

            bool postInit( Particle &state )
            {
                bool result = ( rbf_init != NULL );

                if ( result )
                {
                    state.rbf = rbf_init->getRBF();
                }

                return result;
            }
            RBFInitializer *rbf_init;
    };

}


#endif


