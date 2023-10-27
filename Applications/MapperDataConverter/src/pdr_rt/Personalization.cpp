/*****************************************************************************
 *    Copyright (c) 2013 Spirit Corp.
******************************************************************************/
/**
 *   @project                HIOL
 *   @brief                  Personalization module
 *   @file                   Personalization.cpp
 *   @author                 Y. Kotik, D. Churikov
 *   @date                   26.11.2013
 *   @version                1.0
 */
/*****************************************************************************/

#include "Personalization.h"
#include "Report.h"

#include <math.h>
#include <assert.h>
#include <algorithm>

//===================================================================================
// default
const float default_height = 175;        // default user height
const double P0_asl_default = 0.01;    // default asl covariance (for default_height)
const double R_asl_default = 0.25;             // meas covariance
const double Q_asl_default = 0.000005;       // covariance of estimation // base

// initial
const double P0_asl_init = 0.0025;          // asl covariance for reinit case
const double R_asl_init = 0.1;             // meas covariance
const double Q_asl_init = 0.0000005;       // covariance of estimation // base


// main
const double P0_asl_fine = 0.00025;         // asl covariance for reinit case
const double R_asl_fine = 0.1;           // meas covariance
const double Q_asl_fine = 0.000000005;      // default asl covariance

//const double P0_asl_init_thresh = 10*P0_asl_fine;          // asl covariance for reinit case
const double P0_asl_fine_thresh = 1.1 * P0_asl_fine;        // asl covariance for reinit case


const float min_persone_height = 120;
const float max_persone_height = 220;

const double min_valid_step_length = 0.25; // minimal step length for estmation update
const double max_valid_step_length = 2;   // maximal step length for estmation update

const int16_t start_latency = 25;         // personalization update latency (after first tracking start)
const int16_t restart_latency = 7;        // personalization update latency (after tracking restart)

//===================================================================================
static double GetASLFactor( float Height )
{
    return 0.008 * Height - 0.89; // empirical formula
}

//===================================================================================
double PF::GetPersoneHeight( double ASLFactor )
{
    return ( float )( ( ASLFactor + 0.89 ) / 0.008 ); // empirical formula
}

//===================================================================================
double GetPositionDisplacement( double x, double y, double x1, double y1 )
{
    return sqrt( ( x - x1 ) * ( x - x1 ) + ( y - y1 ) * ( y - y1 ) );
}

//===================================================================================
// constructor
PF::Personalizer::Personalizer()
{
    initialize( default_height );
    p_asl_factor = P0_asl_default;

    setEnabled( false );
    setSleep( false );
}

//===================================================================================
// initialization
bool PF::Personalizer::initialize( float NewHeight )
{
    bool f_set_new_height = ( NewHeight >= min_persone_height ) && ( NewHeight <= max_persone_height );

    if( f_set_new_height )
        PersoneHeight = NewHeight;
    else
        PersoneHeight = default_height;

    asl_factor = GetASLFactor( PersoneHeight );
    reset_params();

    return f_set_new_height;
}

//===================================================================================
void PF::Personalizer::reset()
{
    initialize( default_height );
}

//===================================================================================
void PF::Personalizer::reset_params()
{
    p_asl_factor = P0_asl_init;

    p_event1 = p_event0 = -1;
    pos_x1 = pos_y1 = 0;
    pos_x0 = pos_y0 = 0;
    pf_step_length1 = 0;
    count_down = start_latency;
}

//===================================================================================
double PF::Personalizer::getStepLength( double acc_span, float p_event )
{
    //assert(acc_span > 0);
    if( ( p_event > 0 ) && ( acc_span > 0 ) )
        return sqrt( sqrt( acc_span ) ) * asl_factor;
    else if( ( p_event > 0 ) && ( acc_span < 0 ) )
    {
        ReportPDR::LogReport( "\nError: (Personalizer::getStepLength) negative acc amplitude (%f) of step ignored.", acc_span );
        assert( 0 );
        return 0;
    }
    else
        return 0;
}

//===================================================================================
void PF::Personalizer::getContext( double* pASLFactor, double* pP_ASLFactor, bool *pisEnabled )
{
    *pP_ASLFactor = p_asl_factor;
    *pASLFactor = asl_factor;
    *pisEnabled = isEnabled;
}

//===================================================================================
void PF::Personalizer::setContext( double* pASLFactor, double* pP_ASLFactor, bool *pisEnabled )
{
    double heght = GetPersoneHeight( *pASLFactor );
    PersoneHeight = Min( Max( heght, min_persone_height ), max_persone_height );
    asl_factor = GetASLFactor( PersoneHeight );

    //p_asl_factor = Min( Max(*pP_ASLFactor, P0_asl_fine), P0_asl_init);
    p_asl_factor = Max( *pP_ASLFactor, P0_asl_fine );

    isEnabled = *pisEnabled;

    // reset update params
    p_event1 = p_event0 = -1;
    pos_x1 = pos_y1 = 0;
    pos_x0 = pos_y0 = 0;
    pf_step_length1 = 0;
    count_down = start_latency;
}

PF::tPersoneParams PF::Personalizer::getPersoneParams()
{
    tPersoneParams pp;
    pp.asl_factor = asl_factor;
    pp.p_asl_factor = p_asl_factor;
    pp.PersoneHeight = PersoneHeight;
    return pp;
}

double PF::Personalizer::getPersoneHeight()
{
    return PersoneHeight / 100;
}

//===================================================================================
// adaptation of state vector covariance
double PF::Personalizer::getQ()
{
    if( this->p_asl_factor > P0_asl_init )
    {
        printf( "\ndefault %f\n", this->p_asl_factor );
        return Q_asl_default;
    }
    else if( this->p_asl_factor > P0_asl_fine )
    {
        printf( "\ninit %f\n", this->p_asl_factor );
        return Q_asl_init;
    }
    else
    {
        printf( "\nfine %f\n", this->p_asl_factor );
        return Q_asl_fine;
    }
}

//===================================================================================
// adaptation of measurement covariance
double PF::Personalizer::getR()
{
    if( this->p_asl_factor > P0_asl_init )
        return R_asl_default;
    else if( this->p_asl_factor > P0_asl_fine )
        return R_asl_init;
    else
        return R_asl_fine;
}

double CosHeadingIncrement( double x1, double y1,
                            double x2, double y2,
                            double x3, double y3 )
{
    double v1v2dot = ( x3 - x2 ) * ( x2 - x1 ) + ( y3 - y2 ) * ( y2 - y1 );
    double v1_module = sqrt( ( x2 - x1 ) * ( x2 - x1 ) + ( y2 - y1 ) * ( y2 - y1 ) );
    double v2_module = sqrt( ( x3 - x2 ) * ( x3 - x2 ) + ( y3 - y2 ) * ( y3 - y2 ) );
    return v1v2dot / ( v1_module * v2_module );
}


//===================================================================================
// update personal parameters
void PF::Personalizer::update( bool inTracking, double pos_x, double pos_y, float p_event, double acc_span )
{
    if( enabled() && ( is_sleep() == false ) )
    {
        if( inTracking )
        {
            if( count_down > 0 )        count_down--;

            if( ( p_event1 >= 0 ) && ( count_down <= 0 ) ) // previose position is avaliable
            {
                bool f_update = ( p_event1 > 0.5 ) && ( p_event0 > 0.5 ) && ( p_event > 0.5 ); // not boundary step
                f_update = f_update && ( pf_step_length1 >= min_valid_step_length );
                f_update = f_update && ( pf_step_length1 <= max_valid_step_length );

                //f_update = f_update && (CosHeadingIncrement(pos_x0,pos_y0,pos_x1,pos_y1,pos_x,pos_y) > 0.95);
                if( f_update )
                {
                    double x;

                    double Q = getQ();
                    double R = getR(); // to do: R must depends from current position accurasy estimate

                    // Prediction
                    double x_minus = asl_factor;
                    double P_minus = p_asl_factor + Q;

                    // EKF algorithm
                    //double H = sqrt( sqrt(acc_span) );
                    double H = sqrt( sqrt( acc_span1 ) );
                    double y = pf_step_length1 - H * x_minus;
                    double S = H * P_minus * H + R;
                    double K = P_minus * H * ( 1 / S );

                    // check EKF singularity & update
                    bool f_singular = false; // to do: EKF singularity checkig
                    float h;
                    x = x_minus;

                    if( !f_singular ) // hight controll
                    {
                        x = x_minus + K * y;
                        h = GetPersoneHeight( x );

                        //f_singular = (h < min_persone_height) || (h > max_persone_height);
                        if( !( ( h < min_persone_height ) || ( h > max_persone_height ) ) )
                        {
                            // new EKF state
                            asl_factor = x;
                            p_asl_factor = ( 1 - K * H ) * P_minus;
                            PersoneHeight = h;
                        }

                        // else: not change state
                    }
                } // if (f_update)

                // save step length
                pf_step_length1 = GetPositionDisplacement( pos_x, pos_y, pos_x1, pos_y1 );
                acc_span1 = acc_span;
            } // if (p_event_0 >= 0)

            // save state
            p_event0 = p_event1;  // two steps ago
            p_event1 = p_event;    // previouse step
            pos_x0 = pos_x1;         // two steps ago
            pos_y0 = pos_y1;         // two steps ago
            pos_x1 = pos_x;         // previouse step
            pos_y1 = pos_y;         // previouse step
        }// if (inTracking)
        else
        {
            // reset of step sequence
            p_event0 = p_event1 = -1;
            pos_x1 = pos_x1 = 0.;
            count_down = std::max( restart_latency, count_down );
        }
    } // if (isEnabled)
}
