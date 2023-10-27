/**
* \copyright       Copyright (C) TDK, 2020
* \brief           Multi-floor PDR prediction models with likelyhood update.
* \ingroup         PF
* \file            PredictTPN3D.hpp
* \author          D.Churikov
* \date            07.05.2020
*/

#ifndef PREDICT_VDR_3D_HPP
#define PREDICT_VDR_3D_HPP
#define _USE_MATH_DEFINES

#include "PredictTPN3D.hpp"
#include <cmath>

namespace PF
{


    /**
    * VDR prediction model based onPredictTPN3DLkH_Mixed with scale factor adaptation
    * uses IFFData interface for obtaining input data
    */
    class PredictVdr3D : public tPredictTPN3D
    {
    public:
        PredictVdr3D()
        {
            initialize("PredictVdr3D");
        }

        PredictVdr3D(std::string objName)
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

                double sig_kl;
                for (int i = 0; i < N; ++i)
                {
                    state_pred[i] = state_est[i];

                    Float d_F = 0;
                    if (sig_f > 0)
                    {
                        d_F = dF + sig_f * pFilter->randn();
                    }

                    // calculating d_X and d_Y noise values, correlation is taken into account
                    Float d_X(0), d_Y(0);
                    //if (coordinates_increment.is_motion == true)
                    if (true)
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

                    if (pFilter->is_in_tracking() && (coordinates_increment.is_motion == true) && (coordinates_increment.mis_std <= (30. / 180 * M_PI)))
                    {// in motion

                        if (i == 0)
                        {
                            if (kl_counter < 1000000)
                            {
                                kl_counter++;
                            }

                            if (kl_counter < 100) sig_kl = 0.01;
                            else if (kl_counter < 200) sig_kl = 0.005;
                            else sig_kl = 0.0005;
                            //std::cout << "sig_kl=" << sig_kl << "; kl_counter=" << kl_counter << std::endl;
                        }

                        Float kl = state_pred[i].kl + sig_kl * pFilter->randn();
                        kl = std::max(kl, 0.6);
                        kl = std::min(kl, 1.5);
                        state_pred[i].kl = kl;
                    }
                    else
                    {
                        // during stop do nothing
                    }

                    d_X += state_pred[i].kl*dX;
                    d_Y += state_pred[i].kl*dY;

                    state_pred[i].pos.level += d_F;

                    //convert position increment from local to pf system
                    Float sig_H = 0 + sigH;
                    if ((mis_std > 0) && (coordinates_increment.is_motion == true) && (pFilter->is_in_tracking() == false))
                    {
                        sig_H += mis_std;
                    }
                    Float phi = state_pred[i].h + sig_H * pFilter->randn() + dotH;

                    Float cos_phi = cos(phi);
                    Float sin_phi = sin(phi);
                    Float dXpf = d_X * cos_phi - d_Y * sin_phi;
                    Float dYpf = d_X * sin_phi + d_Y * cos_phi;

                    // additive noise is disabled now
                    /*if (coordinates_increment.is_motion)
                    {
                        dXpf += pFilter->randn() * sig_additive;
                        dYpf += pFilter->randn() * sig_additive;
                    }*/

                    state_pred[i].pos.x += dXpf;
                    state_pred[i].pos.y += dYpf;

                    Float c = (state_pred[i].h > 0) ? (-2. * M_PI) : (2. * M_PI);
                    state_pred[i].h = (std::abs(phi) > M_PI) ? (phi + c) : phi;
                }
#if 0 // enable disable LKH sampling
                if (coordinates_increment.is_motion == false)
                {
                    LikelihoodPositionEstimator lkh;
                    lkh.set_heading_estimation_enable(true);

                    // LKH resampling
                    for (int i = 0; i < N; ++i)
                    {
                        double r = pFilter->rand();
                        lkh.set_heading_estimation_enable(true);
                        lkh.ClearTrackHistory();
                        if ((pIff != nullptr) && (r < lkh_update_ratio))
                        {
                            TrackPoint track_point;
                            Prediction<PF::Float> pred = pIff->getPrediction();
                            track_point.timestamp = pred.t;
                            track_point.x = state_pred[i].pos.x;
                            track_point.y = state_pred[i].pos.y;
                            track_point.floor = state_pred[i].pos.level;
                            track_point.h = state_pred[i].h;

                            track_point.cov_xy[0][0] = track_point.cov_xy[1][1] = 0;  // 0 to avoid position resampling
                            track_point.cov_xy[1][0] = track_point.cov_xy[0][1] = 0;

                            track_point.bias.m[0] = state_pred[i].rbf.b[0];
                            track_point.bias.m[1] = state_pred[i].rbf.b[1];
                            track_point.bias.m[2] = state_pred[i].rbf.b[2];
                            track_point.bias.valid = true;

                            track_point.mag = pIff->getMagMeas();

                            track_point.qi2l[0] = pred.qi2l.w();
                            track_point.qi2l[1] = pred.qi2l.x();
                            track_point.qi2l[2] = pred.qi2l.y();
                            track_point.qi2l[3] = pred.qi2l.z();

                            track_point.valid = true;

                            std::size_t cell_number;
                            lkh.ProcessTrackPoint(track_point, pIff->getMFP(), state_pred[i], cell_number);

                            PositionWithUncertainties lpos = lkh.GetLikelihoodPosition();
                            if (lpos.valid)
                            {
                                state_pred[i].h = lpos.heading;
                            }
                        }
                    }
                }
                else
                {
                    // LKH resampling
                    if ((pIff != nullptr) && (lkh_update_ratio > 0) &&
                        (coordinates_increment.mis_std > (30. / 180 * M_PI)) && pFilter->initialized())
                    {
                        lkh.set_heading_estimation_enable(true);
                        lkh.ClearTrackHistory();
                        Particle stateMean, stateStd;

                        if (pFilter->estimate(stateMean, stateStd)) // this condition equal to is_in_tracking
                        {
                            TrackPoint track_point;
                            Prediction<PF::Float> pred = pIff->getPrediction();

                            track_point.timestamp = pIff->getPrediction().t;
                            track_point.x = stateMean.pos.x;
                            track_point.y = stateMean.pos.y;
                            track_point.floor = stateMean.pos.level;
                            track_point.h = stateMean.h;

                            track_point.cov_xy[0][0] = 1 * stateStd.pos.x*stateStd.pos.x;
                            track_point.cov_xy[1][1] = 1 * stateStd.pos.y*stateStd.pos.y;
                            track_point.cov_xy[1][0] = track_point.cov_xy[0][1] = 0;

                            track_point.bias.m[0] = stateMean.rbf.b[0];
                            track_point.bias.m[1] = stateMean.rbf.b[1];
                            track_point.bias.m[2] = stateMean.rbf.b[2];
                            track_point.bias.valid = true; // ???

                            track_point.mag = pIff->getMagMeas();

                            track_point.qi2l[0] = pred.qi2l.w();
                            track_point.qi2l[1] = pred.qi2l.x();
                            track_point.qi2l[2] = pred.qi2l.y();
                            track_point.qi2l[3] = pred.qi2l.z();

                            track_point.valid = true;

                            std::size_t cell_number;
                            lkh.ProcessTrackPoint(track_point, pIff->getMFP(), stateMean, cell_number);

                            // lh_cell_list weight norming
                            PF::Float w_sum(0);
                            int iter = 0;
                            for (std::size_t i = 0; i < cell_number; i++)
                            {
                                //w_sum += cell->w;
                                w_sum += lkh.lh_cell_list[i].w;
                            }
                            iter = 0;
                            for (std::size_t i = 0; i < cell_number; i++)
                            {
                                //cell->w = cell->w * N * lkh_update_ratio / w_sum;
                                lkh.lh_cell_list[i].w = lkh.lh_cell_list[i].w * N * lkh_update_ratio / w_sum;
                            }

                            //auto lh_cell = lkh.lh_cell_list.begin();
                            int j = 0;
                            for (int i = 0; (i < N) && j < cell_number; ++i)
                            {
                                double r = pFilter->rand();
                                if (r < lkh_update_ratio)
                                {
                                    state_pred[i].pos.x = lkh.lh_cell_list[j].pos.x + 0.25*pFilter->randn();
                                    state_pred[i].pos.y = lkh.lh_cell_list[j].pos.y + 0.25*pFilter->randn();
                                    state_pred[i].h = lkh.lh_cell_list[j].h + 0.1*pFilter->randn();
                                    state_pred[i].kl = stateMean.kl;

                                    lkh.lh_cell_list[j].w -= 1;
                                    if ((int)lkh.lh_cell_list[j].w <= 0)
                                        j++;
                                }

                                if (std::abs(state_pred[i].h) > M_PI)
                                {
                                    Float c = (state_pred[i].h > 0) ? (-2 * M_PI) : (2 * M_PI);
                                    state_pred[i].h += c;
                                }
                            }
                        }
                    }
                }
#endif
            }
        }
    };
}
#endif

