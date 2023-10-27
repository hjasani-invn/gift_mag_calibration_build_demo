/**
* \copyright       Copyright (C) TDK, LLC., 2020
* \brief           Map Matching MM updater implementation
* \ingroup         PF
* \file            UpdateMM.hpp
* \author          V.Pentiukhov
* \date            24.04.2020
*/

#ifndef UPDATE_MODEL_MM_H
#define UPDATE_MODEL_MM_H
#define _USE_MATH_DEFINES

#include "IPF.hpp"
#include "../Map_Matching/MapMatching.hpp"
#include <cmath>

namespace PF
{
    /**
    *Map Matching MM updater
    */
    class UpdateMM : public UpdateSource
    {
        public:
            UpdateMM()
            {
                initialize( 0 );
            }

            UpdateMM(IFFData<> *pIFFData)
            {
                initialize(pIFFData);
            }

            virtual ~UpdateMM()
            {
            }

            void initialize(IFFData<> *pIFFData)
            {
                name = "UpdateMM";
                pIFF = pIFFData;
                isEnabled = false;
                last_processed_particles_number = 0;
                last_killed_particles_number = 0;
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
                MapMatching* mm_ptr(0);
                bool has_map = false;

                if (pIFF != 0)
                {
                    mm_ptr = pIFF->getMapMatching();
                    has_map = (mm_ptr != 0);
                    if (has_map)
                    has_map = mm_ptr->get_validity();
                }
                os << name << "<UpdMM>" << ": " << (isEnabled ? 1 : 0) << " " << mm_ptr << " " << has_map;
                os << " " << last_processed_particles_number << " " << last_killed_particles_number << std::endl;
            }

        private:
            virtual void update( IFilter *pFilter )
            {
            // getting map matching object
            MapMatching* mm_ptr(0);
            bool has_map = false;

            if (pIFF != 0)
            {
                mm_ptr = pIFF->getMapMatching();
                has_map = (mm_ptr != 0);
            }

            // checking that Map Matching successfully initialized map 
            if (has_map)
                has_map = mm_ptr->get_validity();

            if ( has_map && enabled() )
            {
                int N = pFilter->getParcticlesCount();

                double x1 = 0.0, y1 = 0.0, x2 = 0.0, y2 = 0.0;
                int floor1 = 0, floor2 = 0;

                Particle *state_est = pFilter->getState();
                Particle *state_prop = pFilter->getPrediction();

                last_killed_particles_number = 0;

                for (int i = 0; i < N; ++i)
                {
                    x1 = state_est[i].pos.x;
                    y1 = state_est[i].pos.y;
                    x2 = state_prop[i].pos.x;
                    y2 = state_prop[i].pos.y;
                    floor1 = (int)floor(state_est[i].pos.level + 0.5);
                    floor2 = (int)floor(state_prop[i].pos.level + 0.5);

                    double mm_weight = 1.0;
                    mm_weight = mm_ptr->get_particle_weight(x1, y1, x2, y2, floor1, floor2);
                    if (mm_weight == 0.0)
                        last_killed_particles_number++;

                    state_est[i].w *= mm_weight;
                }

                last_processed_particles_number = N;
            }
        }

        std::string  name;
        IFFData<> *pIFF;
        bool  isEnabled;
        int last_processed_particles_number;
        int last_killed_particles_number;
    };
}

#endif
