/**
* \copyright       Copyright (C) TDK, LLC., 2020
* \brief           Map Matching MM updater implementation
* \ingroup         PF
* \file            UpdateMM.hpp
* \author          V.Pentiukhov
* \date            24.04.2020
*/

#ifndef UPDATE_MODEL_PORTALS_H
#define UPDATE_MODEL_PORTALS_H
#define _USE_MATH_DEFINES

#include "IPF.hpp"
#include "../Map_Matching/MapMatching.hpp"
#include <cmath>

namespace PF
{
    /**
    *Portal updater
    */
    class UpdatePortals : public UpdateSource
    {
        public:
            UpdatePortals()
            {
                initialize( 0 );
            }

            UpdatePortals(IFFData<> *pIFFData)
            {
                initialize(pIFFData);
            }

            virtual ~UpdatePortals()
            {
            }

            void initialize(IFFData<> *pIFFData)
            {
                name = "UpdateMM";
                pIFF = pIFFData;
                isEnabled = false;
                max_d_level = 0.2; // parts of floor
                elevator_speed = 1.2; // m/sec // Standard practice in the elevator industry is 1.5 M/S2, we need a bit lower threshold for rejection in start of elevator motion
                escalator_speed = 0.4; // m/sec
                stairs_speed = 0.3; // m/sec
                d_time = 0.5; // sec
                floor_height = 4; // meters // typical floor height for office builbungs
                out_of_venue_particles_number = total_particles_number = rejected_particles_number = portal_particles_number = 0;
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

            void setFloorHeight(double floor_height)
            {
                this->floor_height = floor_height;
            }

            void set_time_interval(double time_interval)
            {
                this->d_time = time_interval;
            }

            void setVverticalSpeed(double vertical_speed_low, double vertical_speed_high, PortalType portal_type)
            {
                switch (portal_type)
                {
                case k_Elevator:    this->elevator_speed = vertical_speed_high;
                    break;
                case k_Escalator:    this->escalator_speed = vertical_speed_high;
                    break;
                case k_Stairs:    this->stairs_speed = vertical_speed_high;
                    break;
                }
            }

            void print( std::ostream  &os ) const
            {
                tMFP * mfp_prt = (pIFF != 0) ? pIFF->getMFP() : nullptr;
                os << name << "<UpdPortals>" << ": " << (isEnabled ? 1 : 0) << " " << ((nullptr != mfp_prt) ? mfp_prt : 0);
                os << " " << total_particles_number << " " << portal_particles_number << " " << rejected_particles_number;
                os << " " << out_of_venue_particles_number;
                os << std::endl;
            }

        protected:
            virtual void update(IFilter *pFilter) = 0;

            std::string  name;
            IFFData<> *pIFF;
            bool  isEnabled;
            double max_d_level;
            double elevator_speed;
            double escalator_speed;
            double stairs_speed;
            double d_time;
            double floor_height;
            int portal_particles_number;
            int rejected_particles_number;
            int total_particles_number;
            int out_of_venue_particles_number;
    };

    class UpdatePortals1 : public UpdatePortals
    {

    private:
        virtual void update(IFilter *pFilter)
        {
            // getting mfp object
            tMFP * mfp_prt = (pIFF != 0) ? pIFF->getMFP() : nullptr;

            int portal_cells_number = mfp_prt->getPortalCells().size();

            if (this->enabled() && (mfp_prt != nullptr) && (portal_cells_number > 0))
            {
                Particle *state_est = pFilter->getState();
                Particle *state_prop = pFilter->getPrediction();
                
                int N = total_particles_number = pFilter->getParcticlesCount();
                rejected_particles_number = portal_particles_number = 0;
                out_of_venue_particles_number = 0;

                for (int i = 0; i < N; ++i)
                {
                    double x1 = state_est[i].pos.x;
                    double y1 = state_est[i].pos.y;
                    int floor1 = (int)floor(state_est[i].pos.level + 0.5);
                    double level1 = state_est[i].pos.level;
                    bool pos1_valid = (mfp_prt->CheckPosition(x1 * 100, y1 * 100) && mfp_prt->CheckFloor(floor1));

                    double x2 = state_prop[i].pos.x;
                    double y2 = state_prop[i].pos.y;
                    int floor2 = (int)floor(state_prop[i].pos.level + 0.5);
                    double level2 = state_prop[i].pos.level;
                    bool pos2_valid = (mfp_prt->CheckPosition(x2 * 100, y2 * 100) && mfp_prt->CheckFloor(floor2));

                    if ((true == pos1_valid) && (true == pos2_valid) && (state_est[i].w > 0))
                    {
                        double d_level = level2 - level1;

                        PortalType portaltype = mfp_prt->getPortalType(x1, y1, floor1);
                        if (portaltype == k_Elevator ||
                            portaltype == k_Escalator ||
                            portaltype == k_Stairs
                            )
                        {
                            portal_particles_number++;
                        }
                        else
                        {

                            bool particle_must_dye(false);
#if 1
                            // velocity criterion
                            double speed = elevator_speed; // this criterion is intended for fast particle rejection when elevator transition
                            max_d_level = speed * d_time / floor_height; // 0.15; 
                            particle_must_dye = particle_must_dye || (fabs(d_level) > max_d_level);
#endif
#if 1 
                            // floor changing criterion
                            particle_must_dye = particle_must_dye || (floor1 != floor2);
#endif

                            if (true == particle_must_dye)
                            {
                                state_est[i].w = 0; // *= 0.1;
                                rejected_particles_number++;
                            }
                        }
                    }
                    else // zeroe out of venue particles weight
                    {
                        // ! this restriction must be reviewerd when tile-based fingerprints are implemented applied
                        state_est[i].w = 0;
                        out_of_venue_particles_number++;
                    }
                }
            }
        }
    };
}
#endif
