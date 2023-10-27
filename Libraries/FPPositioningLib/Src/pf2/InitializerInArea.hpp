#ifndef PF_INITIALIZER_AREA_H
#define PF_INITIALIZER_AREA_H

#include <ostream>
#include <string>
#include "IPF.hpp"
#include "RBFInitializer.hpp"
#include "InitializerNormal.hpp"
#include "ff_config.h"

#include "../magnetic/MFP.h"

namespace PF
{
    /**
    * Initializer calss which uses external position for initial particle distribution
    * and Mag map as constrains
    */
    class InitializerInArea : public FilterInitializer
    {
        public:

            /** Creates terminal initialzer object
            * \param[in] mfp module instance
            * \param[in] objName initializer name, used for debug purposes
            */
            InitializerInArea( tMFP **mfp, std::string objName = "InitializerInArea" ) : FilterInitializer()
            {
                name = objName;
                ppMFP = mfp;
                position = {};
				inherit_rbf = false;
				max_start_position_uncertainty = 25.0;
				current_position_uncertainty = 0.0;
            };
            /** Creates linked initialzer object
            * \param[in] mfp module instance
            * \param[in] next next initializer in the chain
            * \param[in] objName initializer name, used for debug purposes
            */
            InitializerInArea( tMFP **mfp, FilterInitializer *next, std::string objName = "InitializerInArea" ) : FilterInitializer( next )
            {
                name = objName;
                ppMFP = mfp;
                position = {};
				inherit_rbf = false;
				max_start_position_uncertainty = 25.0;
				current_position_uncertainty = 0.0;
            };

            ~InitializerInArea() {};

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
                void *ptr = ( ppMFP != 0 ) ? *ppMFP : 0;
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

            std::vector<tMFP_Particle3D> getMappedCells()
            {
                return getMappedCells(position, std::sqrt(3.));
            }

            virtual std::vector<tMFP_Particle3D> getMappedCells(PositionWithUncertainties &pos, double sigma_scale)
            {
                std::vector<tMFP_Particle3D> cells_in_area;

                int cell_count = ( *ppMFP )->getRandomPosCount();

                for ( int i = 0; i < cell_count; ++i )
                {
                    tMFP_Particle3D cell = ( *ppMFP )->getRandomPos( i );

                    bool floor_is_correct = ( cell.Floor == (int)round(pos.z) ) ? 1 : 0; // was comparing int to double, added conversion
                    double cell_X_m = cell.X / 100;
                    double cell_Y_m = cell.Y / 100;

                    double hol_xx = (pos.cov_xy[0][0] > 0) ? (sigma_scale * std::sqrt(pos.cov_xy[0][0])) : 0;
                    double hol_yy = (pos.cov_xy[1][1] > 0) ? (sigma_scale * std::sqrt(pos.cov_xy[1][1])) : 0;

                    if (floor_is_correct && (std::abs(cell_X_m - pos.x) < hol_xx) && (std::abs(cell_Y_m - pos.y) < hol_yy))
                    {
                        cells_in_area.push_back( cell );
                    }
                }
                return cells_in_area;
            }

            virtual bool tryInit( IFilter *pF, Float p )
            {
                // version with uniform distribution

				bool result = updateUncertaintyForCurrentTime();

				if (!result)
					return false;

				std::vector<tMFP_Particle3D> cells_in_area;
                bool ok = ( ppMFP != 0 && *ppMFP != 0 && ( *ppMFP )->getRandomPosCount() > 0 ) && position.valid;
                if ( ok )
                {
                    cells_in_area = getMappedCells();
                }

#if 0 // debug output
                if(cells_in_area.size() > 0)
                {
                    int idx(0);
                    for (auto cell : cells_in_area)
                    {
                        std::cout << idx++
                            << ", " << cell.X
                            << ", " << cell.Y
                            << ", " << cell.Floor
                            << ", " << 1
                            << std::endl;
                    }
                }
#endif

                bool success = ok && cells_in_area.size() > 0;

                // we start only if MFP map is present and non-empty and if starting position was set (with cov_matrix of uncertainties)
                // if failed, "next" initializer will be called
                if (success)
                {

                    //std::cout << "p= " << p << std::endl;

                    const int HEADING_DISCRETS = 32;
                    const int PARTICLES_PER_CELL_MIN = 1 * HEADING_DISCRETS;
                    const int PARTICLES_PER_CELL_MAX = 1 * HEADING_DISCRETS;
                    const RBF<Float> rbf_default(100);
                    double h[HEADING_DISCRETS];
                    int N = pF->getParcticlesCount();
                    PF::Particle state;
                    size_t cells_count = cells_in_area.size();

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
                        int N_new = cells_count * HEADING_DISCRETS;
                        if (N < N_new)
                        {
                            pF->resize(N = N_new);
                            N = N_new; // warning - we replace N with new particles number here, it seems correct, but may affect KPI
                            if (pF->initialized())
                            {
                                static_cast<PF::ParticleFilter*>(pF)->do_resample();
                            }
                        }
                    }

                    PF::Particle *state_est = pF->getState();
                    PF::Particle *state_prop = pF->getPrediction();
                    int j = 0;
                    int n = 0;
                    double sum1 = 0;
                    double sum2 = 0;
                    for (int i = 0; i < N; ++i)
                    {
                        Float r = pF->rand();

                        if (r < p)
                        {
                            unsigned int idx = j % cells_count;

                            tMFP_Particle3D& cell = cells_in_area[idx];

                            state.pos.x = cell.X / 100. + 1 * (pF->rand() - 0.5);
                            state.pos.y = cell.Y / 100. + 1 * (pF->rand() - 0.5);
                            state.pos.level = cell.Floor;
                            state.w = 1. / N;
                            sum1 += state.w;

                            if (p > 0.5)
                            { // update heading
                                unsigned int heading_idx = (j / (cells_count)) % HEADING_DISCRETS;
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
                        else
                        {
                            sum2 += state_est[i].w;
                            n++;
                        }
                    }  

                    //std::cout << "p= " << j  << "   " << n << std::endl;
                    //std::cout << "p= " << sum1  << "   " << sum2 << std::endl;

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

            bool inherit_rbf;
            //std::string name;
            tMFP **ppMFP;
    };


    /**
    * Initializer calss which uses external position for initial particle distribution
    * and Mag map as constrains and initialize RBF part using RBFInitializer
    */
    class   InitializerInArea_RBF : public InitializerInArea
    {
        public:
            /** Creates terminal initialzer object
            * \param[in] rbf_init pointer to the  RB-state initializer
            * \param[in] mfp module instance
            * \param[in] objName initializer name, used for debug purposes
            */
            InitializerInArea_RBF( RBFInitializer *rbf_init, tMFP **mfp, std::string objName = "MagInitializerInArea_RBF" )
                : InitializerInArea( mfp, objName ), rbf_init( rbf_init ) {};
            /** Creates linked initialzer object
            * \param[in] rbf_init pointer to the  RB-state initializer
            * \param[in] mfp module instance
            * \param[in] next next initializer in the chain
            * \param[in] objName initializer name, used for debug purposes
            */
            InitializerInArea_RBF( RBFInitializer *rbf_init, tMFP **mfp, FilterInitializer *next, std::string objName = "MagInitializerInArea_RBF" )
                : InitializerInArea( mfp, next, objName ), rbf_init( rbf_init ) {};

            void print( std::ostream &os ) const
            {
                InitializerInArea::print( os );

                if ( rbf_init )
                {
                    rbf_init->print( os );
                }
            }

        private:

            bool tryInit( IFilter *pF, Float p )
            {
                bool result = InitializerInArea::tryInit( pF, p );

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

    /**
    * Initializer calss which uses external position for initial particle distribution
    * and Mag map as constrains but runs only once after call of "setPos" method 
    */
    class MagInitializerInArea : public FilterInitializer
    {
        public:

            /** Creates terminal initialzer object
            * \param[in] mfp module instance
            * \param[in] objName initializer name, used for debug purposes
            */
            MagInitializerInArea( tMFP **mfp, std::string objName = "MagInitializerInArea" ) : FilterInitializer()
            {
				f_success = f_start = false;
                name = objName;
                ppMFP = mfp;
                position = {};
				max_start_position_uncertainty = 25.0;
				current_position_uncertainty = 0.0;
            };

            /** Creates linked initialzer object
            * \param[in] mfp module instance
            * \param[in] next next initializer in the chain
            * \param[in] objName initializer name, used for debug purposes
            */
            MagInitializerInArea( tMFP **mfp, FilterInitializer *next, std::string objName = "MagInitializerInArea" ) : FilterInitializer( next )
            {
				f_success = f_start = false;
                name = objName;
                ppMFP = mfp;
                position = {};
				max_start_position_uncertainty = 25.0;
				current_position_uncertainty = 0.0;
            };

            ~MagInitializerInArea() {};

            virtual void setPos(const PositionWithUncertainties &_position)
            {
                position = _position;
                f_start = true;
                //f_start = _position.is_valid; - TODO, change this and check that validity is set everywhere
				f_success = false;
            }
			
            virtual void print( std::ostream &os ) const
            {
                void *ptr = ( ppMFP != 0 ) ? *ppMFP : 0;
                os << name << ": " << ptr << " " << f_start << " " << f_success << " "
                   << position.timestamp << " "
                   << position.x << " " << position.y << " " << position.z << " " << position.heading << " "
                   << position.cov_xy[0][0] << " " << position.cov_xy[0][1] << " " << position.cov_xy[1][1] << " " << position.std_z << " " << position.std_h
                   <<  std::endl;
            }
        protected:
            //private:
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

                // Experimental version with uniform distribution

                bool ok = ( ppMFP != 0 && *ppMFP != 0 && ( *ppMFP )->getRandomPosCount() > 0 );
                bool success = ok && f_start;

                // TODO: add check of position validity (and make sure it is set to valid)
                // this is done to reset position if it is too old

                // we start only if MFP map is present and non-empty and if starting position was set (with cov_matrix of uncertainties)
                // if failed, "next" initializer will be called
                if ( success )
                {
                    const int HEADING_DISCRETS = 16;
                    const int PARTICLES_PER_CELL_MIN = 1 * HEADING_DISCRETS;
                    const int PARTICLES_PER_CELL_MAX = 2 * HEADING_DISCRETS;
                    const RBF<Float> rbf_default( 100 );

                    PF::Particle state;

                    int cell_count = ( *ppMFP )->getRandomPosCount();

                    // TMP changes, make this better
                    position.cov_xy[0][0] *= 3;
                    position.cov_xy[1][1] *= 3;


                    std::vector<tMFP_Particle3D> cells_in_area;

                    int cell_in_area_count = 0;

                    for ( int i = 0; i < cell_count; ++i )
                    {
                        tMFP_Particle3D cell = ( *ppMFP )->getRandomPos( i );

                        //bool floor_is_correct = ( cell.Floor == position.level ) ? 1 : 0;
						bool floor_is_correct = (std::abs(position.z - cell.Floor) <= position.std_z) ? 1 : 0;
                        double cell_X_m = cell.X / 100;
                        double cell_Y_m = cell.Y / 100;

                        if ( floor_is_correct && ( std::abs( cell_X_m - position.x ) < std::sqrt(position.cov_xy[0][0]) ) && ( std::abs( cell_Y_m - position.y ) < std::sqrt(position.cov_xy[1][1])) )
                        {
                            cells_in_area.push_back( cell );
                            cell_in_area_count++;
                        }
                    }

					// required partice number
					int N = std::min(cell_in_area_count * PARTICLES_PER_CELL_MAX, PF_MAX_PARTICLES_NUMBER);
					
					// particle per cell restriction
					int ppc = 0;
					if (cell_in_area_count > 0)
					{
						ppc = N / cell_in_area_count;
					}
					
					success = (ppc >= PARTICLES_PER_CELL_MIN);

					if (success)
					{
						pF->resize(N);
						// heading directions
						double h[PARTICLES_PER_CELL_MAX];
						for (int i = 0; i < ppc; ++i)
						{
							h[i] = i * 2 * M_PI / ppc;
						}

						PF::Particle *state_est = pF->getState();
						PF::Particle *state_prop = pF->getPrediction();

						for (int i = 0; i < N; ++i)
						{
							Float r = pF->rand();

							if (r < p)
							{
								unsigned int idx_max = cells_in_area.size();
								unsigned int idx = i % idx_max;

								tMFP_Particle3D cell = cells_in_area[idx];

								state.w = 1. / N;
								unsigned int heading_idx = (i / (cell_in_area_count)) % ppc;
								state.h = h[heading_idx];
								state.pos.x = cell.X / 100. + 1 * (pF->rand() - 0.5);
								state.pos.y = cell.Y / 100. + 1 * (pF->rand() - 0.5);
								state.pos.level = cell.Floor;
								postInit(state);

								state_est[i] = state_prop[i] = state;
							}
						}
					}

					f_start = false;
					f_success = success;
                }
                return success;
            }

			bool f_start;
			bool f_success;
            //std::string name;
            tMFP **ppMFP;
    };


    /** 
    * MagInitializerInArea initializer  with RBFInitializer for linear part of RBPF
    */
    class   MagInitializerInArea_RBF : public MagInitializerInArea
    {
        public:
            /** Creates terminal initialzer object
            * \param[in] rbf_init pointer to the  RB-state initializer
            * \param[in] mfp module instance
            * \param[in] objName initializer name, used for debug purposes
            */
            MagInitializerInArea_RBF( RBFInitializer *rbf_init, tMFP **mfp, std::string objName = "MagInitializerInArea_RBF" )
                : MagInitializerInArea( mfp, objName ), rbf_init( rbf_init ) {};
            /** Creates linked initialzer object
            * \param[in] rbf_init pointer to the  RB-state initializer
            * \param[in] mfp module instance
            * \param[in] next next initializer in the chain
            * \param[in] objName initializer name, used for debug purposes
            */
            MagInitializerInArea_RBF( RBFInitializer *rbf_init, tMFP **mfp, FilterInitializer *next, std::string objName = "MagInitializerInArea_RBF" )
                : MagInitializerInArea( mfp, next, objName ), rbf_init( rbf_init ) {};

            void print( std::ostream &os ) const
            {
                MagInitializerInArea::print( os );

                if ( rbf_init )
                {
                    rbf_init->print( os );
                }
            }

        private:

            bool tryInit( IFilter *pF, Float p )
            {
                bool result = MagInitializerInArea::tryInit( pF, p );

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

    class InitializerInSeveralAreas : public InitializerInArea
    {
    public:

        /** Creates terminal initialzer object
        * \param[in] mfp module instance
        * \param[in] objName initializer name, used for debug purposes
        */
        InitializerInSeveralAreas(tMFP **mfp = 0, std::string objName = "InitializerInSeveralAreas") : ppMFP(mfp), InitializerInArea(mfp, objName)
        {
            name = objName;
            position = {};
        };

        /** Creates linked initialzer object
        * \param[in] mfp module instance
        * \param[in] next next initializer in the chain
        * \param[in] objName initializer name, used for debug purposes
        */
        InitializerInSeveralAreas(tMFP **mfp, FilterInitializer *next, std::string objName = "InitializerInSeveralAreas") : 
            ppMFP(mfp), InitializerInArea(mfp, next, objName)
        {
            name = objName;
        };

        // destructor
        ~InitializerInSeveralAreas() {};

        virtual void setPos(const PositionWithUncertainties &_position)
        {
            clearPos();
            addPos(_position, 1.);
        }

        void addPos(const PositionWithUncertainties &_position, double weight)
        {
            InitializerInSeveralAreas::WeightedPosition wp;
            *static_cast<PositionWithUncertainties*> (&wp) = _position;
            wp.w = weight;
            positions.push_back(wp);
        }

        void clearPos()
        {
            positions.clear();
        }

        void clearExpiredPos(int64_t expiration_timestamp)
        {
            for (auto pos = positions.begin(); pos != positions.end();)
            {
                if (pos->timestamp < expiration_timestamp)
                {
                    pos = positions.erase(pos);
                }
                else
                {
                    pos++;
                }
            }
        }

        virtual void print(std::ostream &os) const
        {
            int i = 1;
            for (auto position : positions)
            {
                void *ptr = (ppMFP != 0) ? *ppMFP : 0;
                os << name << ": " << ptr << " " << i++ << " "
                    << position.valid << " "
                    << position.timestamp << " " << position.x << " " << position.y << " " << position.z << " " << position.heading << " "
                    << position.cov_xy[0][0] << " " << position.cov_xy[0][1] << " " << position.cov_xy[1][1] << " " << position.std_z << " " << position.std_h << " "
                    << std::endl;
            }
        }
    protected:
        //private:
        bool postInit(Particle &state)
        {
            return true;
        }

        std::vector<Particle> getMappedCellsEx()
        {
            std::vector<Particle> cells;

            for (auto pos : positions)
            {
                PositionWithUncertainties pos_with_unsc = static_cast<PositionWithUncertainties>(pos);
                std::vector<tMFP_Particle3D> mfp_cells_in_single_area = this->getMappedCells(pos_with_unsc, std::sqrt(3.));
                std::vector<PF::Particle> cells_in_single_area;
                for (auto mfp_cell : mfp_cells_in_single_area)
                {
                    //check if selected cell have been already selected
                    auto it_cell = std::find_if(cells.begin(), cells.end(),
                    [&mfp_cell](const PF::Particle& it_cell)
                    {
                    return (it_cell.pos.x == mfp_cell.X) && (it_cell.pos.y == mfp_cell.Y) && (it_cell.pos.level == mfp_cell.Floor);
                    }
                    );
                    if (it_cell == cells.end())
                    { // add new cell
                        PF::Particle cell;
                        cell.pos.x = mfp_cell.X;
                        cell.pos.y = mfp_cell.Y;
                        cell.pos.level = mfp_cell.Floor;
                        cell.w = pos.w;
                        cells.push_back(cell);
                    }
                    else
                    { // update cell weight
                    it_cell->w += pos.w;
                    }
                }
            }

            return cells;
        }

        bool IsPositionValid()
        {
            bool result = false;
            for (auto pos : positions)
            {
                result = result || pos.valid;
            }
            return result;
        }

        virtual bool tryInit(IFilter *pF, Float p)
        {

            std::cout << "yyyyyyyyy" << std::endl;
            // Experimental version with uniform distribution

            std::vector<Particle> cells;
            bool success = (ppMFP != 0 && *ppMFP != 0 && (*ppMFP)->getRandomPosCount() > 0) && IsPositionValid();
            if (success)
            {
                //NormaPositionWeights();
                cells = getMappedCellsEx();
            }

            success = success && (cells.size() > 0);

#if 1
            {
                int idx(0);
                for (auto cell : cells)
                {
                    std::cout << "cell:" << idx++
                        << ", " << cell.pos.x
                        << ", " << cell.pos.y
                        << ", " << cell.pos.level
                        << ", " << cell.w
                        << std::endl;
                }
            }
#endif

            // we start only if MFP map is present and non-empty and if starting position was set (with cov_matrix of uncertainties)
            // if failed, "next" initializer will be called
            if (success)
            {
                const int HEADING_DISCRETS = 32;
                const int PARTICLES_PER_CELL_MIN = 1 * HEADING_DISCRETS;
                const int PARTICLES_PER_CELL_MAX = 1 * HEADING_DISCRETS;
                const RBF<Float> rbf_default(100);
                int N = pF->getParcticlesCount();
                PF::Particle state;
                size_t cells_count = cells.size();

                int N_new = cells_count * HEADING_DISCRETS;
                if (N < N_new)
                {
                    pF->resize(N = N_new);
                    if (pF->initialized())
                    {
                        static_cast<PF::ParticleFilter*>(pF)->do_resample();
                    }
                }

                PF::Particle *state_est = pF->getState();
                PF::Particle *state_prop = pF->getPrediction();

                double w_sum(0.); 
                for (auto cell : cells)
                {
                    w_sum += cell.w;
                }
                for (auto cell = cells.begin(); cell != cells.end(); cell++)
                {
                    if (w_sum > 0)
                    {
                        cell->w *= 0.5*N / w_sum; // number of particles required for each cell
                        //cell->w *= N / w_sum; // number of particles required for each cell
                        //cell->w = 0;
                    }
                    else
                    {
                        cell->w = N / cells.size(); // number of particles required for each cell
                    }
                }

                int heading_idx(0), idx(0);
                double cell_size = (*ppMFP)->GetMapCellSize();
                for (int i = 0, j = 0; i < N; ++i)
                {
                    Float r = pF->rand();
                    if (r < p)
                    {
                        if ((idx < cells.size()) && (cells[idx].w <= 0))
                        {
                            idx++;
                        }

                        Particle cell;
                        if (idx < cells.size())
                        {
                            cell = cells[idx];
                            cells[idx].w -= 1;
                        }
                        else
                        {
                            cell = cells[static_cast<int>(cells.size() * pF->rand())]; // the rest of particles are selected randomly
                        }
                        
                        state.w = 1. / N;
                        state.h = 2 * M_PI * pF->rand();
                        state.pos.x = cell.pos.x / 100. + cell_size * (pF->rand() - 0.5);
                        state.pos.y = cell.pos.y / 100. + cell_size * (pF->rand() - 0.5);
                        state.pos.level = cell.pos.level;
                        postInit(state);

						if ( (inherit_rbf) && (state_est[i].w > 0.0) )
						{
							state.rbf = state_est[i].rbf;
						}
#if 1
                        {
                            std::cout << "particles:" << i << ", " << idx
                                << ", " << state.pos.x
                                << ", " << state.pos.y
                                << ", " << state.pos.level
                                << ", " << state.h
                                << ", " << state.w
                                << std::endl;
                        }

#endif
                        state_est[i] = state_prop[i] = state;
                    }
                }
            }
            return success;
        }

        struct WeightedPosition : PositionWithUncertainties
        {
            double w;
        };

        std::vector <WeightedPosition>  positions;
        //std::string name;
        tMFP **ppMFP;
    };

    /**
    * Initializer calss which uses external position for initial particle distribution
    * and Mag map as constrains and initialize RBF part using RBFInitializer
    */
    class   InitializerInSeveralAreas_RBF : public InitializerInSeveralAreas
    {
    public:
        /** Creates terminal initialzer object
        * \param[in] rbf_init pointer to the  RB-state initializer
        * \param[in] mfp module instance
        * \param[in] objName initializer name, used for debug purposes
        */
        InitializerInSeveralAreas_RBF(RBFInitializer *rbf_init, tMFP **mfp, std::string objName = "InitializerInSeveralAreas_RBF")
            : InitializerInSeveralAreas(mfp, objName), rbf_init(rbf_init) {};
        /** Creates linked initialzer object
        * \param[in] rbf_init pointer to the  RB-state initializer
        * \param[in] mfp module instance
        * \param[in] next next initializer in the chain
        * \param[in] objName initializer name, used for debug purposes
        */
        InitializerInSeveralAreas_RBF(RBFInitializer *rbf_init, tMFP **mfp, FilterInitializer *next, std::string objName = "InitializerInSeveralAreas_RBF")
            : InitializerInSeveralAreas(mfp, next, objName), rbf_init(rbf_init) {};

        void print(std::ostream &os) const
        {
            InitializerInSeveralAreas::print(os);

            if (rbf_init)
            {
                rbf_init->print(os);
            }
        }

    private:

        bool tryInit(IFilter *pF, Float p)
        {
            bool result = InitializerInSeveralAreas::tryInit(pF, p);

            if (result)
            {
                rbf_init->incrementSig(10);
            }

            return result;
        }
        bool postInit(Particle &state)
        {
            bool result = (rbf_init != NULL);

            if (result)
            {
                state.rbf = rbf_init->getRBF();
            }

            return result;
        }
        RBFInitializer *rbf_init;
    };

}

#endif
