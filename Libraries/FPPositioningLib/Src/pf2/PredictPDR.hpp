/**
* \copyright       Copyright (C) SPIRIT Navigation, LLC., 2014
* \brief           Single floor PDR prediction model.
* \ingroup         PF
* \file            PredictPDR.hpp
* \author          M.Frolov
* \date            28.11.2014
*/

#ifndef PREDICT_MODEL_PDR_H
#define PREDICT_MODEL_PDR_H
#define _USE_MATH_DEFINES

#include "IPF.hpp"
#include <cmath>
#include "ff_config.h"

#define DEFAULT_SIGMA_PDR_H (6*M_PI/180)
#define DEFAULT_SIGMA_PDR_L  0.3
#define DEFAULT_SIGMA_PDR_P  0.1

namespace PF
{
    class PredictPDR : public PredictionModel
    {
        public:
            PredictPDR()
            {
                initialize();
            }

            ~PredictPDR() {};
            void initialize()
            {
                sigH = DEFAULT_SIGMA_PDR_H; // 6 deg
                sigL = DEFAULT_SIGMA_PDR_L; // m
                sigP = DEFAULT_SIGMA_PDR_P; // m
                dL = 0;
                dH = 0;
            }
            void setSig( Float _sigH, Float _sigL, Float _sigP )
            {
                sigH = _sigH;
                sigL = _sigL;
                sigP = _sigP;
            }
            void getSig( Float &_sigH, Float &_sigL, Float &_sigP )
            {
                _sigH = sigH;
                _sigL = sigL;
                _sigP = sigP;
            }
            void setStepParams( Float _dL, Float _dH )
            {
                dL = _dL;
                dH = _dH;
            }

        private:
            void propagate( IFilter *pFilter )
            {
                int N = pFilter->getParcticlesCount();
                Particle *state_est  = pFilter->getState();
                Particle *state_pred = pFilter->getPrediction();
                for( int i = 0; i < N; ++i )
                {
                    state_pred[i] = state_est[i];

                    Float H = dH + sigH * 1.*pFilter->randn();
                    Float L = dL + sigL * 1.*pFilter->randn();

                    state_pred[i].h += H;
                    if( state_pred[i].h > M_PI )
                    {
                        state_pred[i].h = state_pred[i].h - 2 * M_PI;
                    }

                    state_pred[i].pos.x += L * cos( state_pred[i].h );
                    state_pred[i].pos.y += L * sin( state_pred[i].h );
                    state_pred[i].pos.x += sigP * pFilter->randn() / sqrt( 2. );
                    state_pred[i].pos.y += sigP * pFilter->randn() / sqrt( 2. );
                }
            }

			virtual void print(std::ostream &os) const
			{

			}

            Float sigH, sigL, sigP;
            Float dL, dH;
    };
}

#endif
