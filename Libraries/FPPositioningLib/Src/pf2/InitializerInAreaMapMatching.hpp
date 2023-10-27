#ifndef PF_INITIALIZER_AREA_MM_H
#define PF_INITIALIZER_AREA_MM_H

#include <ostream>
#include <string>
#include "IPF.hpp"
#include "RBFInitializer.hpp"
#include "InitializerNormal.hpp"
#include "ff_config.h"
#include "../Map_Matching/MapMatching.hpp"
#include "../magnetic/MFP.h"

namespace PF
{
    /**
    * Initializer calss which uses external position for initial particle distribution
    * and venue map as constrains
    */
    class InitializerInAreaMapMatching : public FilterInitializer
    {
        public:

            /** Creates terminal initialzer object
            * \param[in] mm module instance
            * \param[in] objName initializer name, used for debug purposes
            */
			InitializerInAreaMapMatching( MapMatching **mm, tMFP **mfp, std::string objName = "InitializerInAreaMapMatching" ) : FilterInitializer()
            {
                name = objName;
                ppMM = mm;
				ppMFP = mfp;
                position = {};
				max_start_position_uncertainty = 25.0;
				current_position_uncertainty = 0.0;
            };
            /** Creates linked initialzer object
            * \param[in] mm module instance
            * \param[in] next next initializer in the chain
            * \param[in] objName initializer name, used for debug purposes
            */
			InitializerInAreaMapMatching(MapMatching **mm, tMFP **mfp, FilterInitializer *next, std::string objName = "InitializerInAreaMapMatching" ) : FilterInitializer( next )
            {
                name = objName;
				ppMM = mm;
				ppMFP = mfp;
                position = {};
				max_start_position_uncertainty = 25.0;
				current_position_uncertainty = 0.0;
            };

            ~InitializerInAreaMapMatching() {};

            virtual void setPos( const PositionWithUncertainties &_position )
            {
                position = _position;
            }

			void setRBFinheritance(bool enable)
			{
				inherit_rbf = enable;
			}

            virtual void print( std::ostream &os ) const
            {
                void *ptr = (ppMM != 0 ) ? *ppMM : 0;
                os << name << ": " << ptr << " " << enabled << " "
                   << position.valid << " "
                   << position.timestamp << " " << position.x << " " << position.y << " " << position.z << " " << position.heading << " "
                   << position.cov_xy[0][0] << " " << position.cov_xy[0][1] << " " << position.cov_xy[1][1] << " " << position.std_z << " " << position.std_h << " "
                   << std::endl;
            }
        protected:
            //private:
            bool postInit( Particle &state )
            {
                return true;
            }

			std::vector<tMFP_Particle3D> getMappedCells()
			{
				return getMappedCells(position, std::sqrt(3.));
			}

			virtual std::vector<tMFP_Particle3D> getMappedCells(PositionWithUncertainties &pos, double sigma_scale)
			{
				std::vector<tMFP_Particle3D> cells_in_area;

				int cell_count = (*ppMFP)->getRandomPosCount();

				for (int i = 0; i < cell_count; ++i)
				{
					tMFP_Particle3D cell = (*ppMFP)->getRandomPos(i);

					bool floor_is_correct = (cell.Floor == (int)round(pos.z)) ? 1 : 0;
					double cell_X_m = cell.X / 100;
					double cell_Y_m = cell.Y / 100;

					double hol_xx = (pos.cov_xy[0][0] > 0) ? (sigma_scale * std::sqrt(pos.cov_xy[0][0])) : 0;
					double hol_yy = (pos.cov_xy[1][1] > 0) ? (sigma_scale * std::sqrt(pos.cov_xy[1][1])) : 0;

					if (floor_is_correct && (std::abs(cell_X_m - pos.x) < hol_xx) && (std::abs(cell_Y_m - pos.y) < hol_yy))
					{
						cells_in_area.push_back(cell);
					}
				}
				return cells_in_area;
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
                // Experimental version with uniform distribution
				bool result = updateUncertaintyForCurrentTime();

				if (!result)
					return false;

				bool ok = (ppMFP != 0 && *ppMFP != 0 && (*ppMFP)->getRandomPosCount() > 0) && position.valid && (ppMM != 0 && *ppMM != 0 && (*ppMM)->get_validity());
				
				if ( !ok )
					return false;

				double area_square_meters = 0.0;
                
				if ( ok )
                {
					std::vector<tMFP_Particle3D> mfp_cells;
					mfp_cells = getMappedCells();
					const double mfp_cell_size = (*ppMFP)->GetMapCellSize();

					double sigma_scale = std::sqrt(3.);
					double hol_xx = (position.cov_xy[0][0] > 0) ? (sigma_scale * std::sqrt(position.cov_xy[0][0])) : 0;
					double hol_yy = (position.cov_xy[1][1] > 0) ? (sigma_scale * std::sqrt(position.cov_xy[1][1])) : 0;
										
					bool mm_set_pos_result = (*ppMM)->set_area_for_initializer(int(round(position.z)), mfp_cells, mfp_cell_size, area_square_meters);
                }

                bool success = ok && area_square_meters > 0.0;
				
                // we start only if map matching object is present and is valid and if starting position was set (with cov_matrix of uncertainties)
                // if failed, "next" initializer will be called
                if ( success )
                {
					// calculation of required number of particles based on area_square_meters
					const int HEADING_DISCRETS = 32;
					const int PARTICLES_MIN = 5000;
					int N = pF->getParcticlesCount();
					int N_new = round(area_square_meters * HEADING_DISCRETS);
					if (N_new < PARTICLES_MIN)
					{
						N_new = PARTICLES_MIN;
					}
					double h[HEADING_DISCRETS];

					if (position.std_h < M_PI)
					{
						for (int i = 0; i < HEADING_DISCRETS; ++i)
						{
							h[i] = position.heading + position.std_h * pF->randn();
						}
					}
					else
					{
						for (int i = 0; i < HEADING_DISCRETS; ++i)
						{
							h[i] = position.heading + 2 * M_PI * pF->rand();
							//h[i] = i * 2 * M_PI / HEADING_DISCRETS;
						}
					}
					if (p >= 1.0)
					{
						if (N < N_new)
						{
							pF->resize(N = N_new);
							N = N_new;
							if (pF->initialized())
							{
								static_cast<PF::ParticleFilter*>(pF)->do_resample();
							}
						}
					}

					//std::cout << "Initializer MM " << " square meters: " << area_square_meters << " particle count N: " << N << std::endl;

					PF::Particle state;

					PF::Particle *state_est = pF->getState();
					PF::Particle *state_prop = pF->getPrediction();
					int j = 0;

					for (int i = 0, j = 0; i < N; ++i)
					{
						Float r = pF->rand();

						if (r < p)
						{
							state.w = 1. / N;
							// get random XY from the area
							double x = 0.0, y = 0.0;
							int floor = int(round(position.z));
							double pos_random_number = pF->rand();

							(*ppMM)->get_random_particle_pos_from_area(x, y, floor, pos_random_number);

							state.w = 1. / N;
							state.pos.x = x;
							state.pos.y = y;
							state.pos.level = floor;

							if (p > 0.5)
							{ // update heading
								double heading_random_number = pF->rand();
								unsigned int heading_idx = std::floor(heading_random_number * HEADING_DISCRETS);
								state.h = h[heading_idx];
							}
							else
							{ // inherit heading
								state.h = state_est[i].h;
							}

							postInit(state);

							if ((inherit_rbf) && (state_est[i].w > 0.0))
							{
								state.rbf = state_est[i].rbf;
							}

							state_est[i] = state_prop[i] = state;
							++j;
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

				(*ppMM)->clear_area_for_initializer();

                return success;
            }

            //std::string name;
			bool inherit_rbf;
			MapMatching **ppMM;
			tMFP **ppMFP;
    };


    /**
    * Initializer calss which uses external position for initial particle distribution
    * and venue map as constrains and initialize RBF part using RBFInitializer
    */
    class   InitializerInAreaMapMatching_RBF : public InitializerInAreaMapMatching
    {
        public:
            /** Creates terminal initialzer object
            * \param[in] rbf_init pointer to the  RB-state initializer
            * \param[in] mm module instance
            * \param[in] objName initializer name, used for debug purposes
            */
			InitializerInAreaMapMatching_RBF( RBFInitializer *rbf_init, MapMatching **mm, tMFP **mfp, std::string objName = "MagInitializerInAreaMapMatching_RBF" )
                : InitializerInAreaMapMatching( mm, mfp, objName ), rbf_init( rbf_init ) {};
            /** Creates linked initialzer object
            * \param[in] rbf_init pointer to the  RB-state initializer
            * \param[in] mm module instance
            * \param[in] next next initializer in the chain
            * \param[in] objName initializer name, used for debug purposes
            */
			InitializerInAreaMapMatching_RBF( RBFInitializer *rbf_init, MapMatching **mm, tMFP **mfp, FilterInitializer *next, std::string objName = "MagInitializerInAreaMapMatching_RBF" )
                : InitializerInAreaMapMatching( mm, mfp, next, objName ), rbf_init( rbf_init ) {};

            void print( std::ostream &os ) const
            {
				InitializerInAreaMapMatching::print( os );

                if ( rbf_init )
                {
                    rbf_init->print( os );
                }
            }

        private:

            bool tryInit( IFilter *pF, Float p )
            {
                bool result = InitializerInAreaMapMatching::tryInit( pF, p );

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
