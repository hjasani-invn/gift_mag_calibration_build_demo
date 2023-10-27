/**
* \copyright       Copyright (C) TDK-Invensense, 2017
* \brief           Multi-floor PDR prediction models with likelyhood update.
* \ingroup         PF
* \file            PredictTPN3D.hpp
* \author          Y.Kotik, M.Frolov, D.Churikov
* \date            2016-2018
*/

#ifndef PREDICT_MODEL_PDR_H
#define PREDICT_MODEL_PDR_H
#define _USE_MATH_DEFINES

#include "IPF.hpp"
#include <cmath>
#include <set>
#include "ff_config.h"
#include "IFFSettings.hpp"
#include "LikelihoodPos.hpp" //TODO remove dependany from Fppe


namespace PF
{
    static const Float default_cov_xx = 0.001;
    static const Float default_cov_yy = 0.001;
    static const Float default_cov_xy = 0.000;
    static const Float default_cov_yx = 0.000;
    static const Float default_sigma_floor = 0.0;
    static const Float default_sigma_h = ( 2 * M_PI / 180 );
    static const Float default_sigma_additive_pos = 0.01;  // note: it is not used now
    static const Float k_default_lkh_update_ratio = 0.05;

    /**
    * Template class for TPN prediction models
    */
    class tPredictTPN3D : public PredictionModel
    {
    public:

        ~tPredictTPN3D() {};

        void initialize(const std::string &objName)
        {
            coordinates_increment.d_x = 0;
            coordinates_increment.d_y = 0;
            coordinates_increment.d_floor = 0;

            coordinates_increment.d_floor_std = default_sigma_floor;
            coordinates_increment.covariance_yx[0][0] = default_cov_xx;
            coordinates_increment.covariance_yx[0][1] = default_cov_yx;
            coordinates_increment.covariance_yx[1][0] = default_cov_xy;
            coordinates_increment.covariance_yx[1][1] = default_cov_yy;
            
            coordinates_increment.mis_std = 0;
            coordinates_increment.is_motion = false;
			coordinates_increment.is_transit = false;
            lkh_update_ratio = k_default_lkh_update_ratio;
            kl_counter = 0;

            sigH = default_sigma_h;
            sigAdditive = 0.0;  // note: it is not used now
            name = objName;
            dotH = 0;
            floor_height = 5;
            elevator_speed = 1; // m/sec
            escalator_speed = 0.4; // m/sec
            stairs_speed = 0.3; // m/sec
            time_interval = 0.5; // sec
            floor_height = 5; // meters
            floor_change_blocked = false;
            enable_floor_prediction = true;

            pIff = nullptr;
        }

        void set_sigma_additive(Float &new_sigma_additive_pos)
        {
            sigAdditive = new_sigma_additive_pos;
        }

        void set_sigma_H(const Float &new_sigmaH)
        {
            sigH = new_sigmaH;
        }

        void set_dot_H(const Float &new_dotH)
        {
            dotH = new_dotH;
        }

        void setIff(IFFData<> *pIFF)
        {
            pIff = pIFF;
        }

        void set_lkh_update_ratio(const Float &new_ratio)
        {
            lkh_update_ratio = new_ratio;
        }

        void set_increment(const Prediction<PF::Float> &pred)
        {
            coordinates_increment.d_x = pred.x;
            coordinates_increment.d_y = pred.y;
            coordinates_increment.d_floor = pred.floor;

            coordinates_increment.covariance_yx[0][0] = pred.pos_cov(0, 0); // xx
            coordinates_increment.covariance_yx[0][1] = pred.pos_cov(0, 1); // xy
            coordinates_increment.covariance_yx[1][0] = pred.pos_cov(1, 0); // yx
            coordinates_increment.covariance_yx[1][1] = pred.pos_cov(1, 1); // yy
            coordinates_increment.d_floor_std = pred.pos_cov(2, 2);

            coordinates_increment.mis_std = pred.mis_std;
            coordinates_increment.is_motion = pred.is_motion;
			coordinates_increment.is_transit = pred.is_transit;
        }

        bool set_increment()
        {
            bool is_pIff_initialized = (pIff != nullptr);
            if (is_pIff_initialized)
            {
                set_increment(pIff->getPrediction());
            }
            return is_pIff_initialized;
        }

        void set_floor_height(const Float floor_height)
        {
            this->floor_height = floor_height;
        }

        void set_time_interval(double time_interval)
        {
            this->time_interval = time_interval;
        }

        void set_floor_change_blocked(bool _floor_change_blocked)
        {
            this->floor_change_blocked = _floor_change_blocked;
        }

        void set_vertical_speed(double vertical_speed_low, double vertical_speed_high, PortalType portal_type)
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

        void set_enable_floor_prediction(bool enable)
        {
            enable_floor_prediction = enable;
        }

        virtual void print(std::ostream &os) const
        {
            os << name << ":"
                << " d_x=" << coordinates_increment.d_x
                << " d_y=" << coordinates_increment.d_y
                << " d_f=" << coordinates_increment.d_floor
                << " cov_xx=" << coordinates_increment.covariance_yx[0][0]
                << " cov_xy=" << coordinates_increment.covariance_yx[0][1]
                << " cov_yx=" << coordinates_increment.covariance_yx[1][0]
                << " cov_yy=" << coordinates_increment.covariance_yx[1][1]
                << " std_df=" << coordinates_increment.d_floor_std
                << " sigH=" << sigH
                << " sigA=" << sigAdditive
                << " is_motion=" << coordinates_increment.is_motion
                << " mis_std=" << coordinates_increment.mis_std
                << " lkh_upd=" << lkh_update_ratio
                << " kl_counter=" << kl_counter
                << std::endl;
        }

    protected:
        Float sigH;
        Float sigAdditive;
        std::string name;
        Float dotH;
        Float floor_height; // meters
        double time_interval;
        double elevator_speed;
        double escalator_speed;
        double stairs_speed;

        LikelihoodPositionEstimator lkh;

        Float lkh_update_ratio;
        long kl_counter;

        // floor prediction control
        bool enable_floor_prediction; /// the flag is supposed to be used to disable floor prediction during whole time of navigation 
                                      /// for exampe depends on device sensor configuratrion
        bool floor_change_blocked;    /// the flag is supposed to be used to block floor prediction for short periods

        struct CoordinatesIncrement
        {
            double d_x;                 /**< x-coordinate increment [m] */
            double d_y;                 /**< y-coordinate increment [m] */
            double d_floor;             /**< z-axis increment [floor] */
            double covariance_yx[2][2]; /**< increment covariance matrix in column order [m^2]\n
                                        *  cov(x,x), cov(y,x)\n
                                        *  cov(x,y), cov(y,y) */
            double d_floor_std;         /**< z-axis standard deviation [floor] */
            
            double mis_std;             /**< missalignment angle standard deviation */
            bool is_motion;               /**< step/stop flag */
			bool is_transit;			/**<escalator, elevator,stairs*/
        };
        CoordinatesIncrement coordinates_increment;

        IFFData<> *pIff;
    };


    /**
    * General TPN prediction model with multi-floor support
    */
    class PredictTPN3D : public tPredictTPN3D
    {
        public:
            PredictTPN3D()
            {
                initialize( "PredictTPN3D" );
            }

            PredictTPN3D(std::string objName)
            {
                initialize(objName);
            }

            ~PredictTPN3D() {};

        private:
            void propagate(IFilter *pFilter)
            {
                if (pFilter != nullptr)
                {
                    const int N = pFilter->getParcticlesCount();
                    Particle *state_est = pFilter->getState();
                    Particle *state_pred = pFilter->getPrediction();

                    Float dX = coordinates_increment.d_x;
                    Float dY = coordinates_increment.d_y;
                    Float dF = coordinates_increment.d_floor;
                    Float sig_f = coordinates_increment.d_floor_std;
                    //Float sig_additive = sigAdditive;

                    // calculating Cholesky decomposition for covariance matrix (used for calculating correlated X and Y noise)
                    // [a 0] * [a b] = covariance matrix
                    // [b c]   [0 c]
                    Float a, b, c;
                    a = sqrt(coordinates_increment.covariance_yx[0][0]);
                    b = coordinates_increment.covariance_yx[0][1] / a;
                    c = sqrt(coordinates_increment.covariance_yx[1][1] - b * b);

                    Float mis_std = coordinates_increment.mis_std;

                    for (int i = 0; i < N; ++i)
                    {
                        state_pred[i] = state_est[i];

                        Float d_F = 0;
                        if (sig_f > 0)
                        {
                            d_F = dF + sig_f * pFilter->randn();
                        }

                        // calculating d_X and d_Y noise values, correlation is taken into account
                        Float d_X = pFilter->randn();
                        Float d_Y = pFilter->randn();

                        d_Y = d_Y * c + d_X * b;
                        d_X *= a;

                        //pdr step scale adaptation (state_pred[i].kl) is not provided;
                        Float kl = state_pred[i].kl;
                        d_X += kl*dX;
                        d_Y += kl*dY;

                        state_pred[i].pos.level += d_F;

                        //convert position increment from local to pf system
                        Float sig_H = sigH;
                        
                        if (mis_std > 0)
                        {
                            sig_H += mis_std;
                        }
                        Float phi = state_pred[i].h + sig_H * pFilter->randn() + dotH;

                        Float cos_phi = cos(phi);
                        Float sin_phi = sin(phi);
                        Float dXpf = d_X * cos_phi - d_Y * sin_phi;
                        Float dYpf = d_X * sin_phi + d_Y * cos_phi;

                        // additive noise is disabled now
                        //dXpf += pFilter->randn() * sig_additive;
                        //dYpf += pFilter->randn() * sig_additive;

                        state_pred[i].pos.x += dXpf;
                        state_pred[i].pos.y += dYpf;

                        Float c = (phi > 0) ? (-2. * M_PI) : (2. * M_PI);
                        state_pred[i].h = (std::abs(phi) > M_PI) ? (phi + c) : phi;
                    }
                }
            }
    };

    /**
    * TPN prediction model with sampling from likelihood(from Mag map) and multi-floor support
    * uses IFFData interface for obtaining input data
    */
    class PredictTPN3DLkH_MutiPoint : public tPredictTPN3D
    {
    public:
        PredictTPN3DLkH_MutiPoint()
        {
            initialize("PredictTPN3DLkH_MutiPoint");
        }

        PredictTPN3DLkH_MutiPoint(std::string objName)
        {
            initialize(objName);
        }

    private:
        void propagate(IFilter *pFilter)
        {
            if (pFilter != nullptr)
            {
                const int N = pFilter->getParcticlesCount();
                Particle *state_est = pFilter->getState();
                Particle *state_pred = pFilter->getPrediction();
				VenueType venue_type = pFilter->getVenueType();
				double cell_size = (pIff->getMFP())->GetMapCellSize() / 100.;
				double covx(25.), covy(25.), std_walk(2.), std_stop(1.);
								//covx = covy = 6.25*cell_size*cell_size;
				Particle stateMean, stateStd;
				pFilter->estimate(stateMean, stateStd); // to get is_in_tracking
				Float mis_std = coordinates_increment.mis_std;
				Float dX = coordinates_increment.d_x;
				Float dY = coordinates_increment.d_y;
				Float dF = coordinates_increment.d_floor;

				if (venue_type == kAisleVenue)
				{
					if (dX*dX + dY * dY < 0.6)
						covx = covy = 6.25;
					else
						covx = covy = 12.25;
					std_walk = 0.75;
					std_stop = 0.4;
				}
				else if (venue_type == kOfficeVenue)
				{
					if (dX*dX + dY * dY < 0.6)
					{
						covx = covy = 6.25;
						std_walk = 0.8;
					}
					else
					{
						covx = covy = 25.;
						std_walk = 1.0;
					}
					std_stop = 0.5;
				}
				else
				{
					covx = covy = 25.;
					std_walk = 2.0;
					std_stop = 1.0;
				}

				if (coordinates_increment.is_transit)
				{
					std_walk = 4.0;
					std_stop = 2.0;
				}
               
                
				const Float k_freeze_factor = 0.1; // freezing of floor variation in tracking
				Float sig_f = pFilter->is_in_tracking() ? k_freeze_factor*coordinates_increment.d_floor_std : coordinates_increment.d_floor_std;

				if ( (!enable_floor_prediction) || floor_change_blocked)
				{
					sig_f = 0.0;
					dF = 0.0;
				}

                // calculating Cholesky decomposition for covariance matrix (used for calculating correlated X and Y noise)
                // [a 0] * [a b] = covariance matrix
                // [b c]   [0 c]
                Float a, b, c;
                a = sqrt(coordinates_increment.covariance_yx[0][0]);
                b = coordinates_increment.covariance_yx[0][1] / a;
                c = sqrt(coordinates_increment.covariance_yx[1][1] - b * b);

                Float sig_H = sigH;
				
				if ((mis_std > 0) && (coordinates_increment.is_motion == true) && (pFilter->is_in_tracking() == false))
				{
					sig_H += mis_std;					
				}

				if (coordinates_increment.is_motion == true)
					pFilter->SetMotionParams(std_walk, std_walk, sig_H, 0.1, lkh_update_ratio);
				else
					pFilter->SetMotionParams(std_stop, std_stop, sig_H, 0.05, lkh_update_ratio);

                for (int i = 0; i < N; ++i)
                {
                    state_pred[i] = state_est[i];
					state_pred[i].lkh = false;
                    Float d_F = 0;
                    if (sig_f > 0)
                    {
                        d_F = dF + sig_f * pFilter->randn();
						state_pred[i].pos.level += d_F;
                    }

                    // get portal type for particle
                    PortalType portaltype(k_NoPortal);
                    if (pIff != 0)
                    {
                        tMFP * pMfp(0);
                        pMfp = pIff->getMFP();
                        if (pMfp != 0)
                        {
                            int fl = (int)floor(state_est[i].pos.level + 0.5);
                            portaltype = pMfp->getPortalType(state_pred[i].pos.x, state_pred[i].pos.y, fl);
                        }
                    }

                    // calculating d_X and d_Y noise values, correlation is taken into account
                    Float d_X(0), d_Y(0);
                    
                    if((PortalType::k_Elevator == portaltype) && (coordinates_increment.is_transit))// special position spread freezeng for elevators
                    {
                        d_X = 0.005*pFilter->randn();
                        d_Y = 0.005*pFilter->randn();
                    }
                    else if (coordinates_increment.is_motion == true )
                    {
                        d_X = pFilter->randn();
                        d_Y = pFilter->randn();
                        d_Y = d_Y * c + d_X * b;
                        d_X *= a;
                    }
                    else // position spread freezeng in stops 
                    {
                        d_X = 0.005*pFilter->randn();
                        d_Y = 0.005*pFilter->randn();
                    }

                   	d_X += dX;
					d_Y += dY;
					
                    //convert position increment from local to pf system
                    
                    Float phi = state_pred[i].h + sig_H * pFilter->randn() + dotH;
                    
                    Float cos_phi = cos(phi);
                    Float sin_phi = sin(phi);
                    Float dXpf = d_X * cos_phi - d_Y * sin_phi;
                    Float dYpf = d_X * sin_phi + d_Y * cos_phi;

                    state_pred[i].pos.x += dXpf;
                    state_pred[i].pos.y += dYpf;
                    
                    Float c = (state_pred[i].h > 0) ? (-2. * M_PI) : (2. * M_PI);
                    state_pred[i].h = (std::abs(phi) > M_PI) ? (phi + c) : phi;
                }

                // Impus LKH injection on large instant particle rejection
                double w_slow, w_fast, p_inj, reject_percent_inst, reject_percent_aver;
                pFilter->getReinitParameters(w_slow, w_fast, p_inj, reject_percent_inst, reject_percent_aver);
                Float k_reject_percent_inst_threshold = 0.05;
				PF::Float lkh_ratio = (reject_percent_inst < k_reject_percent_inst_threshold) ? lkh_update_ratio : 0.05;
				//std::cout << "w_slow = " << w_slow << "   w_fast" << w_fast << "   lkh_ratio" << lkh_ratio << "   rej_inst" << reject_percent_inst << std::endl;
                // LKH resampling in motion
                for (int i = 0; i < N; ++i)
                {
                    double r = pFilter->rand();

                    if ((r < lkh_ratio) && (pIff != nullptr) && pFilter->is_in_tracking() && coordinates_increment.is_motion)
                    {
                        TrackPoint track_point;
                        
                        lkh.ClearTrackHistory();
                        
                        Prediction<PF::Float> pred = pIff->getPrediction();
                        track_point.timestamp = pred.t;
                        track_point.x = state_est[i].pos.x;
                        track_point.y = state_est[i].pos.y;
                        track_point.floor = state_est[i].pos.level;
                        track_point.h = state_est[i].h;

                        track_point.bias.m[0] = state_est[i].rbf.b[0];
                        track_point.bias.m[1] = state_est[i].rbf.b[1];
                        track_point.bias.m[2] = state_est[i].rbf.b[2];
                        track_point.bias.valid = true;

                        track_point.mag = pIff->getMagMeas();

                        track_point.qi2l[0] = pred.qi2l.w();
                        track_point.qi2l[1] = pred.qi2l.x();
                        track_point.qi2l[2] = pred.qi2l.y();
                        track_point.qi2l[3] = pred.qi2l.z();

                        track_point.valid = true;

                        track_point.cov_xy[0][0] = covx;
						track_point.cov_xy[1][1] = covy;
						track_point.cov_xy[1][0] = track_point.cov_xy[0][1] = 0;
							
						lkh.set_heading_estimation_enable(false); 
						std::size_t cell_number;
						lkh.ProcessTrackPoint(track_point, pIff->getMFP(), state_est[i], cell_number);

						double cum[100];
						cum[0] = lkh.lh_cell_list[0].w;
						for (std::size_t i = 1; i < cell_number; i++)
						{
							cum[i] = cum[i - 1] + lkh.lh_cell_list[i].w;
						}
						

						PositionWithUncertainties lpos = lkh.GetLikelihoodPosition();
						//std::cout << "lpos = " << lpos.x << "   " << lpos.y << "   " << lpos.z << "   " << lpos.heading << std::endl;
						if (lpos.valid && cum[cell_number - 1] > 0.)
						{
							for (std::size_t i = 0; i < cell_number; i++)
							{
								cum[i] /= cum[cell_number - 1];
							}
							double sampl;
							sampl = pFilter->rand();
							int j = 0;
							while (cum[j] < sampl && j < cell_number)
								j++;

							double xl = lkh.lh_cell_list[j].pos.x;
							double yl = lkh.lh_cell_list[j].pos.y;
							double xc, yc, wl, xav = 0, yav = 0, wsum = 0;
							for (int q = 0;q < cell_number;q++)
							{
								xc = lkh.lh_cell_list[q].pos.x;
								yc = lkh.lh_cell_list[q].pos.y;
								wl = lkh.lh_cell_list[q].w;
								if (abs(xc - xl) <= cell_size && abs(yc - yl) <= cell_size)
								{
									xav += xc * wl;
									yav += yc * wl;
									wsum += wl;
								}
							}
							if (wsum > 0)
							{
								xav /= wsum;
								yav /= wsum;
							}
							else
							{
								xav = xl;
								yav = yl;
							}

							state_pred[i].pos.x = xav;
							state_pred[i].pos.y = yav;
							//state_pred[i].pos.x = lkh.lh_cell_list[j].pos.x;
							//state_pred[i].pos.y = lkh.lh_cell_list[j].pos.y;
							state_pred[i].lkh = true;
							//std::cout << "lpos_x = " << state_pred[i].pos.x << "   lpos_y = " << state_pred[i].pos.y << std::endl;
						}							
							
                    }
                }

                // moving particles to portal 
                tMFP * mfp_prt(0);
                // getting mfp object
                if (pIff != 0)
                    mfp_prt = pIff->getMFP();
                else
                    return;
                std::vector<tMFP_Particle3D> portalCells = mfp_prt->getPortalCells();
                if (portalCells.size() == 0)  // no portal cells
                    return;

                double cel_size = 0.01 * mfp_prt->GetMapCellSize(); // cel size in meters

                int portal_counter = 0;
                int fast_counter = 0;
                double max_d_level = 0.2; // part of floor
				double koef_max_d_level = 0.76;
                double min_fast_part = 0.25;
                double moved_part = 0.20;
                double max_distance = 5; // meters
                std::set <int> not_moved_particles;

                for (int i = 0; i < N; ++i)
                {
                    double x1 = state_est[i].pos.x;
                    double y1 = state_est[i].pos.y;
                    double x2 = state_pred[i].pos.x;
                    double y2 = state_pred[i].pos.y;
                    double level1 = state_est[i].pos.level;
                    double level2 = state_pred[i].pos.level;
                    double d_level = level2 - level1;
                    int floor1 = (int)floor(state_est[i].pos.level + 0.5);
                    int floor2 = (int)floor(state_pred[i].pos.level + 0.5);
					d_level *= floor_height;

                    if (state_est[i].w == 0)
                    {
                        not_moved_particles.insert(i);
                        continue;
                    }

                    PortalType portaltype = mfp_prt->getPortalType(x1, y1, floor1);
                    if (portaltype == k_Elevator /*||
                        portaltype == k_Escalator ||   // this part is disabled because there is no code for controll of stairs and escalators below
                        portaltype == k_Stairs */      // 
                        )
                    {
                        not_moved_particles.insert(i);
                        state_pred[i].portal_type = true;
                        portal_counter++;
                        continue;
                    }
                    else
                        state_pred[i].portal_type = false;

                    double speed = elevator_speed;
                    max_d_level = speed * time_interval;
					if (fabs(d_level) > max_d_level * koef_max_d_level)
                    {
                        fast_counter++;
                    }
                }
                if (fast_counter < (int)((N - portal_counter) * min_fast_part))
                    return;
                int M = (int)((N - portal_counter) * moved_part);
                int m = 0;
                for (int i = 0; i < N; ++i)
                {
                    double sampl = pFilter->rand();
                    int j = (int)(N * sampl);

                    if (not_moved_particles.count(j) > 0)
                        continue;

                    int n;
                    int min_n = -1;
                    double min_distance = 1e6;
                    int floor2 = (int)floor(state_pred[j].pos.level + 0.5);

                    for (n = 0; n < portalCells.size(); n++)
                    {
                        if (portalCells[n].Floor != floor2)
                            continue;
                        double dx = portalCells[n].X / 100 - state_pred[j].pos.x;
                        double dy = portalCells[n].Y / 100 - state_pred[j].pos.y;
                        double distance = sqrt(dx*dx + dy * dy);
                        if (distance < min_distance)
                        {
                            min_distance = distance;
                            min_n = n;
                        }
                    }
                    if ( min_distance < max_distance) // move particle in to portal
                    {
                        state_pred[j].pos.x = portalCells[min_n].X / 100 + cel_size *(pFilter->rand() - 0.5);
                        state_pred[j].pos.y = portalCells[min_n].Y / 100 + cel_size *(pFilter->rand() - 0.5);
                        state_pred[j].portal_type = true;
                        m++;
                        if (m >= M)
                        {
                            break;
                        }
                    }
                }
            }
        }
    };
}

#endif
