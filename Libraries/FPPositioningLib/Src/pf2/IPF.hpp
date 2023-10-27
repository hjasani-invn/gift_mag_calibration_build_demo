/**
* \copyright       Copyright (C) SPIRIT Navigation, LLC., 2014
* \brief           particle filter interface.
* \ingroup         PF
* \file            IPF.hpp
* \author          M.Frolov, D. Churikov
* \date            28.11.2014
* \update          26.02.2019
*/

#ifndef INTERFACE_PARTICLE_FILTER_H
#define INTERFACE_PARTICLE_FILTER_H

#include "Particle.hpp"
#include <set>
#include <ostream>
#include <stdint.h>
#include <list>
#include <tuple>
#include "Venue.h"
#include "ff_config.h"
#include "IFFSettings.hpp"
#include <eigen/Eigenvalues>

namespace PF
{
    class IFilter;

    /**
    * This interface is used for generalization of logging operations
    */
    class IPrinted
    {
        public:
            virtual ~IPrinted() {};
            virtual void print(std::ostream &os) const = 0;
    };

	struct MotionModelParams
	{
		double sig_x, sig_y, sig_h, sig_f, lkh_ratio;
	};

    /**
    * This interface is used for generalization of prediction models
    * PF prediction model must implement this interface
    */
    class PredictionModel : public virtual IPrinted
    {
        public:
            virtual ~PredictionModel() {} ;
            virtual void propagate( IFilter *pFilter ) = 0;
            //virtual void print( std::ostream &os ) const = 0;
            //virtual void print();
    };

    /**
    * This interface is used for generalization of update models
    * PF update model must implement this interface
    */
    class UpdateSource : public virtual IPrinted
    {
        public:
            virtual ~UpdateSource() {};
            virtual void update( IFilter *pF ) = 0;
            virtual bool enabled() = 0;
            //virtual void print( std::ostream &os ) const = 0;
    };

    /**
    * This interface is used for generalization of Rao-Blackwellized models
    * linear part of RBF filter must implement this interface
    * provides interfece for predict and update operations
    */
    class RBFModel : public PredictionModel, public UpdateSource
    {
    };


    /**
    * This abstract class is used for generalization of filter initialization
    * implements chain of responsibility pattern
    */
    class FilterInitializer
    {
        public:

            FilterInitializer(FilterInitializer *_next) : next(_next), enabled(true), triggered(false), name("no_name_initializer"), triggered_injection(0), position{}, max_start_position_uncertainty(25.0), current_position_uncertainty(0.0) {};

            FilterInitializer() : FilterInitializer(0) {};

            virtual ~FilterInitializer() {};

            void enable(bool state)
            {
                enabled = state; 
            }

            bool isEnabled()
            { 
                return enabled; 
            }

            bool init( IFilter *pF, Float p )
            {
                bool result(false);
                
                if (enabled)
                {
                    triggered = result = tryInit(pF, p);
                }

                if( result == false )
                {
                    if( next != 0 )
                    {
                        result = next->init( pF, p );
                    }
                }
                else
                {
                    triggered_injection = p; // save injection percentage for logging
                }

                return result;
            }

			void get_best_initializer(FilterInitializer** current_best_init, double& current_best_uncertainty)
			{
				if (enabled)
				{
					tryGetBestInit(current_best_init, current_best_uncertainty);
				}

				if (next != 0)
				{
					next->get_best_initializer(current_best_init, current_best_uncertainty);
				}
			}

            void print_status(std::ostream &os) const
            {
                os << name << "( " << (enabled ? 1 : 0) << ", " << (triggered ? 1 : 0) << ", " << triggered_injection << ", " << current_position_uncertainty << " ) ";
                if (next != 0)
                {
                    next->print_status(os);
                }
                else
                {
                    os << std::endl;
                }
            }

            void clear_status()
            {
                triggered = false;
                triggered_injection = 0.;
                if (next != 0)
                {
                    next->clear_status();
                }
            }

			void set_timestamp(int64_t _timestamp)
			{
				timestamp = _timestamp;
				if (next != 0)
					next->set_timestamp(_timestamp);
			}

			void set_max_pos_uncertainty(double max_position_uncertainty)
			{
				max_start_position_uncertainty = max_position_uncertainty;
			}

            virtual void print( std::ostream &os ) const = 0;
            
            FilterInitializer* getNext() const{ return next;    }

        protected:
			
			bool evaluateUncertainty(double& pos_uncertainty)
			{
				current_position_uncertainty = 0.0;
				if (!position.valid)
					return false;

				double delta_t = (timestamp - position.timestamp) / 1000.0; // delta_t in seconds
				if (delta_t < 0.0)
					return false;
							   
				Eigen::MatrixXd cov_xy = Eigen::MatrixXd::Zero(2, 2);
				cov_xy(0, 0) = position.cov_xy[0][0];
				cov_xy(0, 1) = position.cov_xy[0][1];
				cov_xy(1, 0) = position.cov_xy[1][0];
				cov_xy(1, 1) = position.cov_xy[1][1];

				//double vel = 1.0; // 1/ms

				cov_xy(0, 0) += delta_t * delta_t; // * vel^2, omitted since vel = 1 m/s
				cov_xy(1, 1) += delta_t * delta_t;

				Eigen::EigenSolver<Eigen::MatrixXd> es;

				es.compute(cov_xy, false);

				double eigen_value_1_real = es.eigenvalues().transpose()(0, 0).real();
				double eigen_value_2_real = es.eigenvalues().transpose()(0, 1).real();
				double eigen_value_1_imag = es.eigenvalues().transpose()(0, 0).imag();
				double eigen_value_2_imag = es.eigenvalues().transpose()(0, 1).imag();

				if ((std::abs(eigen_value_1_imag) > std::numeric_limits<double>::epsilon())
					|| (std::abs(eigen_value_2_imag) > std::numeric_limits<double>::epsilon()))

					return false;					
				
				pos_uncertainty = std::sqrt(std::max(eigen_value_1_real, eigen_value_2_real));
				current_position_uncertainty = pos_uncertainty;
				if (pos_uncertainty > max_start_position_uncertainty)
					return false;

				return true;
			}

			bool updateUncertaintyForCurrentTime()
			{
				current_position_uncertainty = 0.0;
				if (!position.valid)
					return false;

				double delta_t = (timestamp - position.timestamp) / 1000.0; // delta_t in seconds
				if (delta_t < 0.0)
					return false;

				Eigen::MatrixXd cov_xy = Eigen::MatrixXd::Zero(2, 2);
				cov_xy(0, 0) = position.cov_xy[0][0];
				cov_xy(0, 1) = position.cov_xy[0][1];
				cov_xy(1, 0) = position.cov_xy[1][0];
				cov_xy(1, 1) = position.cov_xy[1][1];

				//double vel = 1.0; // 1/ms

				cov_xy(0, 0) += delta_t * delta_t; // * vel^2, omitted since vel = 1 m/s
				cov_xy(1, 1) += delta_t * delta_t;

				Eigen::EigenSolver<Eigen::MatrixXd> es;

				es.compute(cov_xy, false);

				double eigen_value_1_real = es.eigenvalues().transpose()(0, 0).real();
				double eigen_value_2_real = es.eigenvalues().transpose()(0, 1).real();
				double eigen_value_1_imag = es.eigenvalues().transpose()(0, 0).imag();
				double eigen_value_2_imag = es.eigenvalues().transpose()(0, 1).imag();

				if ((std::abs(eigen_value_1_imag) > std::numeric_limits<double>::epsilon())
					|| (std::abs(eigen_value_2_imag) > std::numeric_limits<double>::epsilon()))

					return false;

				double pos_uncertainty = std::sqrt(std::max(eigen_value_1_real, eigen_value_2_real));
				current_position_uncertainty = pos_uncertainty;
				if (pos_uncertainty > max_start_position_uncertainty)
					return false;
				
				position.cov_xy[0][0] += delta_t * delta_t; // * vel^2, omitted since vel = 1 m/s
				position.cov_xy[1][1] += delta_t * delta_t;

				position.timestamp = timestamp; // updating time of current position since we applied time correction to it

				return true;
			}

            FilterInitializer *next;
            bool enabled;
            bool triggered;
            PF::Float triggered_injection;
            std::string name;
			int64_t timestamp;
			double max_start_position_uncertainty;
			double current_position_uncertainty;
            virtual bool tryInit( IFilter *pF, Float p ) = 0;
            virtual bool postInit( Particle &state ) = 0;
			virtual void tryGetBestInit(FilterInitializer** current_best_initializer, double& current_best_uncertainty) = 0;
			PositionWithUncertainties position;
    };


    /**
    * This abstract class is a base class for "general" PF
    * provides base PF interface and implements some core PF functional
    * TODO! refactoring is required: extract  general filter interface and PF-interface
    */
    class IFilter
    {
        public:
            IFilter() : initializer(NULL), injector(NULL), pPredictionModel(NULL), isEnabled(true) { inPulling = false; };
            virtual ~IFilter() {};
            virtual void restart() = 0;
            virtual void predict() = 0;
            virtual void update() = 0;
            //estimate weighted solution
            virtual bool estimate( Particle &state ) = 0;
            //estimate weighted solution and standart deviation
            virtual bool estimate( Particle &stateMean, Particle &stateStd ) = 0;
            virtual void estimate(std::list<std::tuple<Particle, Particle, double>> &position_std_weight) = 0;
            virtual int estimateFloor() = 0;
            virtual bool estimatePositionOnFloor(int floor, Particle &stateMean, Particle &stateStd) = 0;
            virtual void set_in_tracking_flag(std::list<std::tuple<Particle, Particle, double>> position_std_weight) = 0;
            virtual bool initialized() = 0;
            virtual bool is_in_tracking() = 0;
            virtual Particle *getState() = 0;
            virtual Particle *getPrediction() = 0;
            // uniform noise in range (0, 1) strictly except 0 and 1
            virtual Float rand() = 0;
            // gaussian noise with sig=1
            virtual Float randn() = 0;
            // uniform uint rng
            virtual uint32_t randi() = 0;
            virtual int getParcticlesCount() = 0;
            virtual void setInjection( bool enable ) = 0;
            virtual void setInjection( bool enable, double _p_inj) = 0;
            virtual void resize( int particleCount ) = 0;
            virtual void setWeff( Float weff ) = 0;
            virtual Float getWeff() = 0;
            virtual void getReinitParameters(double &w_slow, double &w_fast, double &p_inj, double &reject_percent_inst, double &reject_percent_aver) = 0;
            virtual void print( std::ostream &os ) const = 0;
            virtual void setRandomSeeds() = 0;
            virtual Float weightsSum(Particle *state, int floor) = 0;


            bool inPulling;

			void setTimeStamp(int64_t _timestamp)
			{
				timestamp = _timestamp;
			}

            void setPredictionModel( PredictionModel *pPM )
            {
                pPredictionModel = pPM;
            }
            void addUpdateModel( UpdateSource *pUM )
            {
                UpdateSources.insert( pUM );
            }

            void removeUpdateModel( UpdateSource *pUM )
            {
                UpdateSources.erase( pUM );
            }

            void clearUpdateModels()
            {
                UpdateSources.clear();
            }
            void addMapper( UpdateSource *pMapper )
            {
                Mappers.insert( pMapper );
            }
            void removeMapper( UpdateSource *pMapper )
            {
                Mappers.erase( pMapper );
            }
            void clearMappers()
            {
                Mappers.clear();
            }

            void addRBFModel( RBFModel *pUM )
            {
                rbf.insert( pUM );
            }

            void removeRBFModel( RBFModel *pUM )
            {
                rbf.erase( pUM );
            }

            void clearRBFs()
            {
                rbf.clear();
            }

            void setInitializer( FilterInitializer *pInit )
            {
                initializer = pInit;
            }

            void setInjector(FilterInitializer *pInj)
            {
                injector = pInj;
            }

            virtual void setEnabled( bool enable )
            {
                isEnabled = enable;
            }

            virtual bool getEnabled()
            {
                return isEnabled;
            }
			void setVenueType(VenueType venuetype)
			{
				venue_type = venuetype;
			}
			VenueType getVenueType()
			{
				return venue_type;
			}
			void SetMotionParams(double sx, double sy, double sh, double sf, double lkh_ratio)
			{

				motion_params.sig_x = sx;
				motion_params.sig_y = sy;
				motion_params.sig_h = sh;
				motion_params.sig_f = sf;
				motion_params.lkh_ratio = lkh_ratio;
			}
			MotionModelParams* GetMotionParams()
			{
				return (&motion_params);
			}

			PredictionModel* GetPredictionModel()
			{
				return pPredictionModel;
			}
            //  virtual std::ostream &operator<<( std::ostream &os);

        protected:
            FilterInitializer *initializer;
			FilterInitializer *injector;
            PredictionModel   *pPredictionModel;
			MotionModelParams motion_params;
            std::set<UpdateSource*> UpdateSources;
            std::set<UpdateSource*> Mappers;
            std::set<RBFModel*> rbf;
            bool isEnabled;
			VenueType venue_type;
			int64_t timestamp;
    };
}

#endif