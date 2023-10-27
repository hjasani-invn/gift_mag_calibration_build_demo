/**
* \copyright       Copyright (C) SPIRIT Navigation, LLC., 2014
* \brief           Particle filter declaration.
* \ingroup         PF
* \file            PF.hpp
* \author          M.Frolov
* \date            28.11.2014
*/

#ifndef PARTICLE_FILTER_H
#define PARTICLE_FILTER_H
#define _USE_MATH_DEFINES

#include <vector>
#include <ostream>
#include <string>
#include <math.h>
#include <algorithm>

#include "IPF.hpp"
#include "uniform_random.hpp"
#include "normal_random.hpp"

std::ostream& operator<< ( std::ostream& out, const PF::PredictionModel& pm );
std::ostream& operator<< ( std::ostream& out, const PF::UpdateSource& us );
std::ostream& operator<< ( std::ostream& out, const PF::FilterInitializer& fi );
std::ostream& operator<< ( std::ostream& out, const PF::IFilter& flt );
std::ostream& operator<< ( std::ostream& out, const PF::RBFModel& rbf );

#define ESTIMATE_DEBUG  0

namespace PF
{

    //static const Float default_reinit_w_threshold = 0.05;
    static const Float default_reinit_threshold_inst = 0.90;  // rejected partical percent
    static const Float default_reinit_threshold_aver = 0.75;  // rejected partical percent

    static const Float default_tracking_sigH_threshold = 45 * M_PI / 180;

    /**
    * General PF class with RBPF support
    * uses predict/update models which implements interfaces from IPF.hpp
    */
    class ParticleFilter : public IFilter
    {
        public:

            ParticleFilter();
            ParticleFilter( int particleCount );
            virtual ~ParticleFilter();

            virtual void restart();
            virtual void predict();
            virtual void update();
            virtual void do_resample();

            virtual bool estimate( Particle &state );
            virtual bool estimate( Particle &stateMean, Particle &stateStd );
            virtual void estimate(std::list<std::tuple<Particle, Particle, double>> &position_std_weight);
            virtual int estimateFloor();
            virtual bool estimatePositionOnFloor(int floor, Particle &stateMean, Particle &stateStd);
            virtual void set_in_tracking_flag(std::list<std::tuple<Particle, Particle, double>> position_std_weight);

            virtual void setRandomSeeds();

            virtual bool initialized()
            {
                return isInitialized;
            }
            virtual bool is_in_tracking()
            {
                return inTracking;
            }
            virtual Particle* getState()
            {
                return state_est;
            }
            virtual Particle* getPrediction()
            {
                return state_prop;
            }
            virtual Float rand()
            {
                return rng->randf();
            }

            virtual Float randn()
            {
                return rngn->randnf();
            }
            virtual uint32_t randi()
            {
                return rng->uniform_rand();
            }

            virtual int getParcticlesCount();
            virtual void resize( int particleCount );
            virtual void setWeff( Float weff )
            {
                w_eff = weff;
            }
            virtual Float getWeff()
            {
                return w_eff;
            }
            virtual void setEnabled( bool enable );

            void setReinitThresholdInstant( Float thresh_w );
            void setReinitThresholdAverage( Float thresh_w );
            void setTrackingThresholdSigH( Float thresh_sigH );

            void getReinitParameters( double &w_slow, double &w_fast, double &p_inj, double &reject_percent_inst, double &reject_percent_aver );

            virtual void setInjection( bool enable )
            {
                inj_is_enable = enable;
            };

            virtual void setInjection(bool enable, double _p_inj)
            {
                inj_is_enable = enable;
                p_inj = _p_inj;
            };
            //            friend std::ostream &operator<<( std::ostream&,  const ParticleFilter&);

            void setName( const std::string &objName )
            {
                name = objName;
            }
            virtual void print( std::ostream &os ) const;
            void printState( std::ostream &os ) const;
            void printSettings(std::ostream &os) const;
            void printParticles(std::ostream &os, Particle *particles, int ParcticlesCount) const;

            virtual Float weightsSum(Particle *state, int floor);
    private:

            bool check_rejected();
            bool check_est();
            void swap_states();
            void resample();
            void reinit();
            Float weightsSum(Particle *state);
            //Float weightsSumKahan( Particle *state );
            void normalize();
            void calcCumSum( Float *C, const Particle *state );
            bool skipResample();
            void initialize( int N );



            Particle *state_est;
            Particle *state_prop;
            Float *cumSum;

            int ParcticlesCount;
            Float upd_thresh;
            //pf_random::lfsr113 rng;
            pf_random::uniform_random *rng;
            pf_random::normal_random  *rngn;
            bool inTracking;
            bool isInitialized;
            bool callResampler;
            int callCounter;
            int floor_queue_counter;
            Float previous_floor_float;
            int previous_floor_int;
            bool floor_latched;

            Float reinit_threshold_inst;
            Float reinit_threshold_aver;
            Float tracking_sigH_threshold;

            Float w_slow;
            Float w_fast;
            Float p_inj;
            Float w_eff;
            bool  inj_is_enable;
            bool  injection_self_adjustment;

            Float reject_percent_inst;
            Float reject_percent_aver;

            std::string name;
    };

}
#endif
