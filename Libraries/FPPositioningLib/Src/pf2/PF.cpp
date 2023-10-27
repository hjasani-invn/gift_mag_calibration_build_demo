/**
* \copyright       Copyright (C) SPIRIT Navigation, LLC., 2014
* \brief           Particle filter implementation.
* \ingroup         PF
* \file            PF.cpp
* \author          M.Frolov
* \date            28.11.2014
*/

#define _USE_MATH_DEFINES
#include "PF.hpp"
#include <cmath>
#include <algorithm>
#include <iostream>
#include <ctime>
#include <list>

using namespace PF;

#include "ziggurat_random.hpp"
#include "lfsr113.hpp"
#include "rng32.hpp"

const double k_p_inj = 0.0;

ParticleFilter::ParticleFilter()
{
    initialize(1000);
}

ParticleFilter::ParticleFilter(int particleCount)
{
    initialize(particleCount);
}

void ParticleFilter::initialize(int N)
{
    pPredictionModel = 0;

    state_est = new Particle[N];
    state_prop = new Particle[N];
    cumSum = new Float[N];

    ParcticlesCount = N;
    upd_thresh = 1.e-6;

    rng = new pf_random::rng32;
    rng->seed(2, 8, 16, 128);

    rngn = new pf_random::ziggurat_random(100);

    inTracking = false;
    isInitialized = false;
    callResampler = false;
    callCounter = 0;
    floor_queue_counter = 0;
    previous_floor_float = -1.0;
    previous_floor_int = -1.0;
    floor_latched = false;

    reinit_threshold_inst = default_reinit_threshold_inst;
    reinit_threshold_aver = default_reinit_threshold_aver;
    tracking_sigH_threshold = default_tracking_sigH_threshold;

    w_slow = 0.;
    w_fast = 0.;
    p_inj = k_p_inj;
    w_eff = 0.5;
    inj_is_enable = false;
    injection_self_adjustment = false;

    reject_percent_inst = 0.;
    reject_percent_aver = 0.;

    name = "ParticleFilter";
}

void ParticleFilter::setRandomSeeds()
{
    rng->seed(std::time(nullptr));
    rngn->seed( static_cast<uint32_t>(rng->randf() * rng->get_max_rand()) );
}

void ParticleFilter::restart()
{

    inTracking = false;
    isInitialized = false;
    callResampler = false;
    //rng->seed( 2, 8, 16, 128 );
    //rng->seed(128);
    callCounter = 0;

    w_slow = 0.;
    w_fast = 0.;
    p_inj = k_p_inj;

    reject_percent_inst = 0.;
    reject_percent_aver = 0.;
}

void ParticleFilter::predict()
{
    if( isEnabled == false ) return;

	initializer->set_timestamp(timestamp);

    if( isInitialized == false ) // injection for initialized filter if it is necesarily
    {
		// trying to get initializer with the best uncertainty
		FilterInitializer* initializer_copy = initializer;
		FilterInitializer** best_initializer = &initializer_copy;
		double best_uncertainty = 1000.0;
		initializer->get_best_initializer(best_initializer, best_uncertainty);

        isInitialized = (*best_initializer)->init( this , 1.0 );
    }
    else // injection for initialized filter if it is necesarily
    {
        if (inj_is_enable)
        {
			injector->set_timestamp(timestamp);
            bool reset_tracking(false);
            
            PF::Float __p_inj (0);

            if (p_inj > 0.) //particles injection
            {
                __p_inj = std::max(__p_inj, p_inj);
                w_fast *= 1.2; //empiric! update injection threshold
            }

            // ! code below never works in practice, because thresholds are never reached now 

            // on huge particle degradation
            if (reject_percent_inst > reinit_threshold_inst) // more than reinit_threshold_inst particles has zero weights
            {
                __p_inj = std::max(__p_inj, 0.9); //0.5
                reset_tracking = true;
            }
            
            if (reject_percent_aver > reinit_threshold_aver) // more than reinit_threshold_aver particles has zero weights
            {
                __p_inj = std::max(__p_inj, 0.5); // 0.1
                reset_tracking = true;
            }
            
            if (__p_inj > 0)
            {
                resample();  // resample is recquired for zero weight particle rejection
                injector->init(this, __p_inj);  // injectors are used for parpicles reinitialisation
                inTracking = reset_tracking ? false : inTracking;
            }
        }
    }

    if( isInitialized == true )
    {
        callCounter++;
        resample();

        if( inTracking )
        {

            for( std::set<UpdateSource *>::const_iterator it = Mappers.begin(); it != Mappers.end(); ++it )
            {
                ( *it )->update( this );
            }
        }

        if (pPredictionModel != 0)
        {
            pPredictionModel->propagate(this);
        }

        for( std::set<RBFModel *>::const_iterator it = rbf.begin(); it != rbf.end(); ++it )
        {
            ( *it )->propagate( this );
        }

    }
}

void ParticleFilter::update()
{
    if( isEnabled == false ) return;

    if( isInitialized == false )
    {
        isInitialized = initializer->init( this , 1.0 );
    }

    if( isInitialized == true )
    {
        if (UpdateSources.size() > 1)
            isInitialized = true;
        for( std::set<UpdateSource *>::const_iterator it = UpdateSources.begin(); it != UpdateSources.end(); ++it )
        {
            if( ( *it )->enabled() )
            {
                ( *it )->update( this );

                if( check_est() == false )
                {
                    isInitialized = false;
                    isInitialized = initializer->init( this, 1.0 );
                }
            }
        }

        for( std::set<RBFModel *>::const_iterator it = rbf.begin(); it != rbf.end(); ++it )
        {
            ( *it )->update( this );
        }

        check_rejected(); //check huge particle degradation

        swap_states();

        //check particle degradation
        Float w_avg = 0;

        for( int i = 0; i < getParcticlesCount(); ++i )
        {
            if( state_est[i].w > 0 )
            {
                w_avg += ( state_prop[i].w / state_est[i].w ) / getParcticlesCount();
            }

            state_est[i].w = state_prop[i].w;
        }

        Float a_slow = 0.005;
        Float a_fast = 0.02;

        w_slow += a_slow * ( w_avg - w_slow );
        w_fast += a_fast * ( w_avg - w_fast );
        
        if (injection_self_adjustment)
        {
            p_inj = std::max(PF::Float(0.0), 1 - w_fast / w_slow);
        }
    }
}

bool ParticleFilter::check_rejected()
{
    int N = getParcticlesCount();
    int k = 0;

    for( int i = 0; i < N; ++i )
    {
        if( state_est[i].w > 0 )
        {
            ++k;
        }
    }

    Float a_aver = 0.1;

    reject_percent_inst = 1. - ( Float ) k / N;
    reject_percent_aver += a_aver * ( reject_percent_inst - reject_percent_aver );
    return false;
}

void ParticleFilter::getReinitParameters( double &w_slow, double &w_fast, double &p_inj, double &reject_percent_inst, double &reject_percent_aver )
{
    w_slow = this->w_slow;
    w_fast = this->w_fast;
    p_inj = this->p_inj;
    reject_percent_inst = this->reject_percent_inst;
    reject_percent_aver = this->reject_percent_aver;
}

bool ParticleFilter::check_est()
{
    Float sum = 0;
    int N = getParcticlesCount();
    int k = 0;
    bool result;

    for( int i = 0; i < N; ++i )
    {
        sum += state_est[i].w;

        if( state_est[i].w > 0 )
        {
            ++k;
        }
    }

    result = ((static_cast<Float>(k) / static_cast<Float>(N)) > 0.05) ? true : false;
    return result;
}

void  ParticleFilter::swap_states()
{
    Particle *tmp = state_prop;
    state_prop    = state_est;
    state_est     = tmp;
}

void ParticleFilter::calcCumSum( Float *C, const Particle *state )
{
    int N = getParcticlesCount();
    C[0] = state[0].w;

    for( int i = 1; i < N; ++i )
    {
        C[i] = C[i - 1] + state[i].w;
    }

    for( int i = 0;  i < N; ++i )
    {
        C[i] /= C[N - 1];
    }
}

bool ParticleFilter::skipResample()
{
    bool result = false;
    Float Neff = 0;
    int N = getParcticlesCount();

    for( int i = 1; i < N; ++i )
    {
        Neff += state_est[i].w * state_est[i].w;
    }

    Neff = 1 / Neff;

    if( Neff > w_eff * N )
    {
        result = true;
    }

    return result;
}

void ParticleFilter::do_resample()
{
    const int N = getParcticlesCount();
    callResampler = true;

    normalize();
    calcCumSum( cumSum, state_est );
    assert( cumSum[N - 1] > 0. );
    swap_states();

    for ( int i = 0; i < N; ++i )
    {
        //Float r = rng.randf<Float>();
        Float r = rng->randf();

        auto *itr = std::lower_bound( cumSum, cumSum + N, r );
        assert( ( itr != cumSum + N ) && !( r > *itr ) );
        size_t k = std::distance( cumSum, itr );
        state_est[i] = state_prop[k];
        state_est[i].w = 1. / N;
    }
}

void ParticleFilter::resample()
{
    int N = getParcticlesCount();
    normalize();
    callResampler = false;
    calcCumSum( cumSum, state_est );
    assert( cumSum[N - 1] > 0. );
    swap_states();
    {
        for( int i = 0; i < N; ++i )
        {
            //Float r = rng.randf<Float>();
            Float r = rng->randf();

            state_est[i] = state_prop[i];

            if (state_est[i].w > 0. ) continue;

            auto *itr = std::lower_bound( cumSum, cumSum + N, r );
            assert( ( itr != cumSum + N ) && !( r > *itr ) );
            size_t k = std::distance( cumSum, itr );
			state_est[i] = state_prop[k];
        }

        normalize();
    }

    if( skipResample() == false )
    {
        do_resample();
    }
}

bool ParticleFilter::estimate( Particle &state )
{
    int N = getParcticlesCount();

    normalize();
    state = Particle( 0 );

    double h_x = 0; double h_y = 0;
    state.kl = 0;
    for( int i = 0; i < N; ++i )
    {
        state.pos += state_est[i].pos * state_est[i].w;
        h_x += cos( state_est[i].h ) * state_est[i].w;
        h_y += sin( state_est[i].h ) * state_est[i].w;
        state.rbf.b += state_est[i].rbf.b * state_est[i].w;
        state.rbf.P += state_est[i].rbf.P * state_est[i].w;
        state.rbf.S += state_est[i].rbf.S * state_est[i].w;
        state.rbf.z += state_est[i].rbf.z * state_est[i].w;
        state.rbf.C += state_est[i].rbf.C * state_est[i].w;
        state.kl += state_est[i].kl * state_est[i].w;
        state.w += state_est[i].w;
    }

    state.h = atan2(h_y, h_x);

    double sig_h = 0;

    for( int i = 0; i < N; ++i )
    {
        double h_pi = state_est[i].h - state.h;
        double dH = fmod( h_pi, M_PI );
        sig_h += dH * dH * state_est[i].w;
    }

    sig_h = sqrt( sig_h );
    inTracking = ( sig_h < tracking_sigH_threshold && isInitialized ) ? true : false;
    return ( inTracking );
}

bool ParticleFilter::estimate( Particle &stateMean, Particle &stateStd )
{
    int N = getParcticlesCount();

    normalize();
    stateMean = Particle( 0 );

    estimate( stateMean );

    stateStd = Particle( 0 );

    Particle stateDelta;

    for( int i = 0; i < N; ++i )
    {
        //compute differences of angles in range (-pi/2,pi/2)
        double dH = angle_diff( state_est[i].h, stateMean.h );
        stateStd.h += dH * dH * state_est[i].w;

        stateDelta.pos = state_est[i].pos - stateMean.pos;
        stateStd.pos += stateDelta.pos * stateDelta.pos * state_est[i].w;


        stateDelta.rbf.b = state_est[i].rbf.b - stateMean.rbf.b;
        stateStd.rbf.b += stateDelta.rbf.b.cwiseProduct( stateDelta.rbf.b ) * state_est[i].w;
    }

    stateStd.pos.x = sqrt( stateStd.pos.x );
    stateStd.pos.y = sqrt( stateStd.pos.y );
    stateStd.pos.level = sqrt( stateStd.pos.level );
    stateStd.h = sqrt( stateStd.h );

    inTracking = ( stateStd.h < tracking_sigH_threshold );
    return ( inTracking && isInitialized );
}

bool ParticleFilter::estimatePositionOnFloor(int floor, Particle &stateMean, Particle &stateStd)
{
    int N = getParcticlesCount();

    normalize();
    stateMean = Particle(0);

    estimate(stateMean);

    stateStd = Particle(0);

    Particle stateDelta;

    for (int i = 0; i < N; ++i)
    {
        double floor = state_est[i].pos.level;
        int int_floor = round(floor);
        if (int_floor == floor)
        {
            //compute differences of angles in range (-pi/2,pi/2)
            double dH = angle_diff(state_est[i].h, stateMean.h);
            stateStd.h += dH * dH * state_est[i].w;

            stateDelta.pos = state_est[i].pos - stateMean.pos;
            stateStd.pos += stateDelta.pos * stateDelta.pos * state_est[i].w;


            stateDelta.rbf.b = state_est[i].rbf.b - stateMean.rbf.b;
            stateStd.rbf.b += stateDelta.rbf.b.cwiseProduct(stateDelta.rbf.b) * state_est[i].w;
        }
    }

    stateStd.pos.x = sqrt(stateStd.pos.x);
    stateStd.pos.y = sqrt(stateStd.pos.y);
    stateStd.pos.level = sqrt(stateStd.pos.level);
    stateStd.h = sqrt(stateStd.h);

    inTracking = (stateStd.h < tracking_sigH_threshold);
    return (inTracking && isInitialized);
}

#if 1
void ParticleFilter::estimate(std::list<std::tuple<Particle, Particle, double>> &position_std_weight)
{
    if (!isInitialized)
    {
        return;
    }

    int N = getParcticlesCount();
    normalize();

    // getting all floor numbers from particles array
    std::vector<int> floor_numbers;

    Float estimated_floor_float = 0;

    for (int i = 0; i < N; ++i)
    {
        double floor = state_est[i].pos.level;
        int int_floor = round(floor);
        if (!(std::find(floor_numbers.begin(), floor_numbers.end(), int_floor) != floor_numbers.end()))
        {
            floor_numbers.push_back(int_floor);            
        }

        estimated_floor_float += state_est[i].pos.level * state_est[i].w;
    }
    
    // additional floor estimation logic (to prevent floor flickering)

    const int M = 10;
    const float beta = 0.6;

    if (!inTracking)
    {
        floor_queue_counter = 0;
        floor_latched = false;
        previous_floor_float = 0.0;
        previous_floor_int = 0;
    }

    floor_queue_counter += 1;
    floor_queue_counter = std::min(floor_queue_counter, M);

#if 0 //it works bad for elevetors, it should be applied just with elevators, and commented because of woriyng about reliability of transition_mode of TPN
    // this must be improved in future with using reliable portal being flag
    estimated_floor_float = previous_floor_float * (1.0 - (1.0 / floor_queue_counter)) + (1.0 / floor_queue_counter) * estimated_floor_float;
#endif
    if ((floor_queue_counter >= M) && (!floor_latched))
    {
        // checking if floor_latched can be set to true
        floor_latched = std::abs(estimated_floor_float - round(estimated_floor_float)) < (1 - beta)/2;
    }

    int estimated_floor_int = round(estimated_floor_float);
    if (floor_latched)
    {
        Float delta_f = std::abs(estimated_floor_float - previous_floor_int);
        if (delta_f < beta)
        {
            estimated_floor_int = previous_floor_int;
        }
    }

    previous_floor_int = std::max(estimated_floor_int, 0);
    previous_floor_float = std::max(estimated_floor_float, 0.0);
    //std::cout << "dbg: setting prev floor float, estimated was: " << estimated_floor_float << " prev float set to : " << previous_floor_float << std::endl;

    //int estimated_floor_int = round(estimated_floor_float);
    // moving the estimated floor to the front of floor numbers vector

    /*for (auto it = floor_numbers.begin(); it != floor_numbers.end(); ++it)
    {
        if (*it == estimated_floor_int)
        {
            auto x = *it;
            floor_numbers.erase(it);
            floor_numbers.insert(floor_numbers.begin(), x);
            break;
        }
    }*/
        
    // for each floor calculating position and weight
    // we need to save sum of weights of particles on current floor for output
    for (std::vector<int>::iterator it = floor_numbers.begin(); it != floor_numbers.end(); ++it)
    {
        Particle state;
        state = Particle(0);

        double h_x = 0; double h_y = 0;
        int floor_dbg = *it;
        state.kl = 0;
        int particle_num_on_floor = 0;
        for (int i = 0; i < N; ++i)
        {
            if (round(state_est[i].pos.level) == *it)
            {
                state.pos += state_est[i].pos * state_est[i].w;
                h_x += cos(state_est[i].h) * state_est[i].w;
                h_y += sin(state_est[i].h) * state_est[i].w;
                state.rbf.b += state_est[i].rbf.b * state_est[i].w;
                state.rbf.P += state_est[i].rbf.P * state_est[i].w;
                state.rbf.S += state_est[i].rbf.S * state_est[i].w;
                state.rbf.z += state_est[i].rbf.z * state_est[i].w;
                state.rbf.C += state_est[i].rbf.C * state_est[i].w;
                state.kl += state_est[i].kl * state_est[i].w;
                state.w += state_est[i].w;
                particle_num_on_floor++;
            }
        }

        // dividing everything by summary weight for current floor (normalizing within one floor)
        if (state.w > 0)
        {
            state.pos.x /= state.w;
            state.pos.y /= state.w;
            state.pos.level /= state.w;
            h_x /= state.w;
            h_y /= state.w;
            state.rbf.b /= state.w;
            state.rbf.P /= state.w;
            state.rbf.S /= state.w;
            state.rbf.z /= state.w;
            state.rbf.C /= state.w;
            state.kl /= state.w;
        }
                
        state.h = atan2(h_y, h_x);
        
        // ************DEBUG
#if ESTIMATE_DEBUG
        state.ph = particle_num_on_floor;
        state.kl = floor_dbg;
#endif
        // ************DEBUG-end

        // calculating STD for current floor
        Particle stateStd;
        stateStd = Particle(0);

        Particle stateDelta;

        for (int i = 0; i < N; ++i)
        {
            if (round(state_est[i].pos.level) == *it)
            {
                //compute differences of angles in range (-pi/2,pi/2)
                double dH = angle_diff(state_est[i].h, state.h);
                stateStd.h += dH * dH * state_est[i].w;

                stateDelta.pos = state_est[i].pos - state.pos;
                stateStd.pos += stateDelta.pos * stateDelta.pos * state_est[i].w;
                
                stateDelta.rbf.b = state_est[i].rbf.b - state.rbf.b;
                stateStd.rbf.b += stateDelta.rbf.b.cwiseProduct(stateDelta.rbf.b) * state_est[i].w;
            }
        }

        stateStd.pos.x = sqrt(stateStd.pos.x);
        stateStd.pos.y = sqrt(stateStd.pos.y);
        stateStd.pos.level = sqrt(stateStd.pos.level);
        stateStd.h = sqrt(stateStd.h);

        std::tuple<Particle, Particle, double> one_floor_tuple(state, stateStd, state.w);
        position_std_weight.push_back(one_floor_tuple);
    }
    
    //sort list of tuples by floor weight
    struct
    {
        bool operator()(std::tuple<Particle, Particle, double> const &t1, std::tuple<Particle, Particle, double> const &t2) const
        {
            return std::get<2>(t1) > std::get<2>(t2);
        }
    } customLess;
    position_std_weight.sort(customLess);
    

    // move forvard tuple with filtered estimated_floor_int
    auto found = std::find_if(position_std_weight.begin(), position_std_weight.end(),
        [&estimated_floor_int](const std::tuple<Particle, Particle, double> &t1)
    { return round(std::get<0>(t1).pos.level) == estimated_floor_int; });

    if (found != position_std_weight.end() && (std::get<2>(*found) > 0.3)) // move tupple only if weight is significant
    {
        position_std_weight.splice(position_std_weight.begin(), position_std_weight, found);
    }
    else
    {
        floor_latched = false;
        previous_floor_float = std::get<0>(position_std_weight.front()).pos.level;
        previous_floor_int = round(previous_floor_float);
    }
}
#endif

int ParticleFilter::estimateFloor()
{
    if (!isInitialized)
    {
        return -1;
    }

    int N = getParcticlesCount();
    normalize();

    // getting all floor numbers from particles array
    std::vector<int> floor_numbers;

    Float estimated_floor_float = 0;

    for (int i = 0; i < N; ++i)
    {
        estimated_floor_float += state_est[i].pos.level * state_est[i].w;
    }

    // additional floor estimation logic (to prevent floor flickering)

    const int M = 10;
    const float beta = 0.6;

    if (!inTracking)
    {
        floor_queue_counter = 0;
        floor_latched = false;
        previous_floor_float = 0.0;
        previous_floor_int = 0;
    }

    floor_queue_counter += 1;
    floor_queue_counter = std::min(floor_queue_counter, M);

#if 0 //it works bad for elevetors, it should be applied just with elevators, and commented because of woriyng about reliability of transition_mode of TPN
    // this must be improved in future with using reliable portal being flag
    estimated_floor_float = previous_floor_float * (1.0 - (1.0 / floor_queue_counter)) + (1.0 / floor_queue_counter) * estimated_floor_float;
#endif
    if ((floor_queue_counter >= M) && (!floor_latched))
    {
        // checking if floor_latched can be set to true
        floor_latched = std::abs(estimated_floor_float - round(estimated_floor_float)) < (1 - beta) / 2;
    }

    int estimated_floor_int = round(estimated_floor_float);
    if (floor_latched)
    {
        Float delta_f = std::abs(estimated_floor_float - previous_floor_int);
        if (delta_f < beta)
        {
            estimated_floor_int = previous_floor_int;
        }
    }

    previous_floor_int = std::max(estimated_floor_int, 0);
    previous_floor_float = std::max(estimated_floor_float, 0.0);

    return  estimated_floor_int;
}

void ParticleFilter::set_in_tracking_flag(std::list<std::tuple<Particle, Particle, double>> position_std_weight)
{
    // currently we take the most "heavy" solution out of all the floors, then repeat logic from old estimate function
    std::tuple<Particle, Particle, double> max_floor_tuple = position_std_weight.front();
    Particle stateStd = std::get<1>(max_floor_tuple);

    inTracking = ( stateStd.h < tracking_sigH_threshold );
}

Float ParticleFilter::weightsSum( Particle *state )
{
    Float sum = 0;
    int N = getParcticlesCount();

    for( int i = 0; i < N ; ++i )
    {
        sum += state[i].w;
    }

    return sum;
}

Float ParticleFilter::weightsSum(Particle *state, int floor)
{
    Float sum = 0;
    int N = getParcticlesCount();

    for (int i = 0; i < N; ++i)
    {
        double floor = state[i].pos.level;
        int int_floor = round(floor);
        
        if (int_floor == floor)
            sum += state[i].w;
    }

    return sum;
}

/*
Float ParticleFilter::weightsSumKahan( Particle *state )
{
    //Kahan sum, accurate sum calculation to avoid IEEE754 rounding error
    Float sum = 0;                             // var sum = 0.0
    Float c = 0;                               // var c = 0.0

    int N = getParcticlesCount();

    for( int i = 0; i < N; ++i )               // for i = 1 to Len(input) do
    {
        volatile Float y = state[i].w - c;     //     y = input[i] - c
        volatile Float t = sum + y;            //     t = sum + y
        c = ( t - sum ) - y;                   //     c = (t - sum) - y
        sum = t;
    }                                          // return sum

    return sum;
}
*/
void ParticleFilter::normalize()
{
    Float sum = weightsSum( state_est );
    int N = getParcticlesCount();
	if (!(sum > 0.))
		return;
    for( int i = 0; i < N; ++i )
    {
        state_est[i].w /= sum;
    }
}

int ParticleFilter::getParcticlesCount()
{
    return ParcticlesCount;
}

ParticleFilter::~ParticleFilter()
{
    delete[] state_est;
    delete[] state_prop;
    delete[] cumSum;
}

void ParticleFilter::resize( int particleCount )
{
    //TODO more accurate size reduction

    if( particleCount != getParcticlesCount() )
    {
        Particle *state_est_new  = new Particle[particleCount];
        Particle *state_prop_new = new Particle[particleCount];
        Float    *cumSum_new     = new Float[particleCount];

        //maybe these are not needed - constructor set all fields to zero
        //memset( state_est_new,  0, sizeof( Particle ) * particleCount );
        //memset( state_prop_new, 0, sizeof( Particle ) * particleCount );
        //memset( cumSum_new,     0, sizeof( Float ) * particleCount );

        int N = std::min( getParcticlesCount(), particleCount );

        memcpy( state_est_new,  state_est,  sizeof( Particle ) * N );
        memcpy( state_prop_new, state_prop, sizeof( Particle ) * N );

        delete [] state_est;
        delete [] state_prop;
        delete [] cumSum;

        state_est = state_est_new;
        state_prop = state_prop_new;
        cumSum = cumSum_new;
        this->ParcticlesCount = particleCount;

        normalize(); // normalization is recquired to make correct weights after resize
    }
}

void ParticleFilter::setEnabled( bool enable )
{
    IFilter::setEnabled( enable );

    if ( enable == false )
    {
        isInitialized = false;
    }
}

void ParticleFilter::printSettings(std::ostream &os) const
{
    os << name << "<ParticleFilter>:params:";
    os << " seed=" << rngn->get_seed();
    os << std::endl;
}

void ParticleFilter::printParticles(std::ostream &os, Particle *particles, int ParcticlesCount) const
{
    for (int i = 0; i < ParcticlesCount; ++i)
    {
        os << i << " " << particles[i].pos.x << " " << particles[i].pos.y
            << " " << particles[i].pos.level << " " << particles[i].h
            << " " << particles[i].w << std::endl;
    }
}

void ParticleFilter::printState( std::ostream &os ) const
{
    os << "call: " << callCounter << std::endl;
    os << "resampler: " << callResampler << std::endl;

    os << "state_prop: " << std::endl;
    printParticles(os, state_prop, ParcticlesCount);

    os << "state_est: " << std::endl;
    printParticles(os, state_est, ParcticlesCount);

}

void ParticleFilter::print( std::ostream &os ) const
{
    if (callCounter == 0)
    {
        printSettings(os);
    }

    os << name << "<ParticleFilter>: " << (inTracking ? 1 : 0) << " " << (isInitialized ? 1 : 0) << " " << ParcticlesCount << " "
        << (callResampler ? 1 : 0) << " " << callCounter << " " << (inj_is_enable ? 1 : 0) << " " << w_slow << " " << w_fast << " " << p_inj << " "
       << reject_percent_inst << " " << reject_percent_aver << std::endl;

    if (initializer != 0)
    {
        os << name << "<ParticleFilter> initializers: ";
        initializer->print_status(os);
    }

	if (injector != 0)
	{
		os << name << "<ParticleFilter> injectors: ";
		injector->print_status(os);
	}

    if (pPredictionModel != 0)
    {
        os << name << "<ParticleFilter> " << "predictModel: " << (*pPredictionModel);
    }
    else
    {
        os << name << "<ParticleFilter> " << "predictModel: " << "none";
    }

    for( std::set<UpdateSource *>::const_iterator it = Mappers.begin(); it != Mappers.end(); ++it )
    {
        os << name << "<ParticleFilter> " << "Mapper: " << *(*it);
    }

    for( std::set<UpdateSource *>::const_iterator it = UpdateSources.begin(); it != UpdateSources.end(); ++it )
    {
        os << name << "<ParticleFilter> " << "Updater: " << *(*it);
    }

    for ( std::set<RBFModel*>::const_iterator it = rbf.begin(); it != rbf.end(); ++it )
    {
        os << name << "<ParticleFilter> " << "RBF: " << *(*it);
    }
}

void ParticleFilter::setReinitThresholdInstant( Float thresh )
{
    reinit_threshold_inst = thresh;
}

void ParticleFilter::setReinitThresholdAverage( Float thresh )
{
    reinit_threshold_aver = thresh;
}

void ParticleFilter::setTrackingThresholdSigH( Float thresh_sigH )
{
    tracking_sigH_threshold = thresh_sigH;
}


std::ostream& operator<< ( std::ostream& out, const PF::PredictionModel& pm )
{
    pm.print( out );
    return out;
}

std::ostream& operator<< ( std::ostream& out, const PF::UpdateSource& us )
{
    us.print( out );
    return out;
}

std::ostream& operator<< ( std::ostream& out, const PF::FilterInitializer& fi )
{
    fi.print( out );
    return out;
}

std::ostream& operator<< ( std::ostream& out, const IFilter& flt )
{
    flt.print( out );
    return out;
}

std::ostream& operator<< ( std::ostream& out, const RBFModel& rbf )
{
    rbf.print( out );
    return out;
}
