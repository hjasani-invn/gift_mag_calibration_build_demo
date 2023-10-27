#ifndef MFP_3_BUILDER_HPP
#define  MFP_3_BUILDER_HPP
#define _USE_MATH_DEFINES
#include "IMfpBuilder.hpp"
#include "IStdEstimator.hpp"
#include <map>
#include <list>
#include <math.h>
#include <algorithm>
#include <iostream>
#include <string>
#include <armadillo>
#include "stat_func.hpp"
#include "StdEstimatorCreator.hpp"
#include "fpeDataTypes.h"

//static int qq = 0;
class Mfp3Builder : public Fpbl::IMfpBuilder
{
    public:
        Mfp3Builder() {}
        ~Mfp3Builder() {}

        virtual void setStdEstimatorType(Fpbl::eStdEstimatorType stdEstimatorType){}

        virtual void setDefaultMagdata(double mag_x, double mag_y, double mag_z)
        {
            default_mag_x = mag_x;
            default_mag_y = mag_y;
            default_mag_z = mag_z;
        }

        //this procedure may work with different cell size between the grid and the fingerprint
        virtual Fpbl::ReturnStatus buildFingerprint(const Fpbl::MagneticGrid &mfpGrid, Fpbl::MfpBuilder::LocalDB *mfpFp)
        {
            //mesurementsInCell - dictionary with a Fp iterator as the key and allready processed measurements
            std::map<Fpbl::MfpBuilder::LocalDB::iterator, uint64_t> mesurementsInCell;
            //initialize fingerprint with initial distribution
            initializeFp( mfpFp );

            //loop for all cells in the grid
            for ( auto it = mfpGrid.cbegin(); it != mfpGrid.cend(); ++it )
            {
                auto position = it->coordinates;
                auto fp_it = mfpFp->findRecord( position ); //find cell(in FP) which contains position(in the grid) of the measurement

                if ( fp_it == mfpFp->cend() ) continue;

                uint64_t  measurementCount = 0;

                auto m_it = mesurementsInCell.find( fp_it );

                if ( m_it != mesurementsInCell.end() )
                {
                    measurementCount = m_it->second; //allready processed measurements
                }

                Fpbl::MfpBuilder::DBRecord::GaussMix m_x = fp_it->x; // init with zero
                Fpbl::MfpBuilder::DBRecord::GaussMix m_y = fp_it->y; // init with zero
                Fpbl::MfpBuilder::DBRecord::GaussMix m_z = fp_it->z; // init with zero

                m_x.s1 = pow( m_x.s1, 2 );
                m_y.s1 = pow( m_y.s1, 2 );
                m_z.s1 = pow( m_z.s1, 2 );

                double x_s1_smmothed = m_x.s1;
                double y_s1_smmothed = m_y.s1;
                double z_s1_smmothed = m_z.s1;

                auto estmate_iter = [&]( float & mean, float & sig2, double & sig2s, const double & meas, const size_t &iteration )
                {
                    double m1 = recursiveMean( mean, meas, iteration );
                    double s1 = recursiveVar( sig2, mean, m1, meas, iteration );
                    mean = (float)m1;
                    sig2 = (float)s1;
                    sig2s += 0.1 * ( sig2 - sig2s );
                };

                for ( auto m_it = it->magData.cbegin(); m_it != it->magData.cend(); ++m_it )
                {
                    ++measurementCount;
                    estmate_iter(m_x.mu1, m_x.s1, x_s1_smmothed, m_it->mX, (size_t)measurementCount);
                    estmate_iter(m_y.mu1, m_y.s1, y_s1_smmothed, m_it->mY, (size_t)measurementCount);
                    estmate_iter(m_z.mu1, m_z.s1, z_s1_smmothed, m_it->mZ, (size_t)measurementCount);
                }

                mesurementsInCell[fp_it] = measurementCount;

                //this is not accurate when esimation is called twice or more
                //not smoothed std should be saved between iterations
                m_x.s1 = (float)sqrt( x_s1_smmothed );
                m_y.s1 = (float)sqrt(y_s1_smmothed);
                m_z.s1 = (float)sqrt(z_s1_smmothed);

                if (measurementCount >= min_data_count)
                {
                    fp_it->x = m_x;
                    fp_it->y = m_y;
                    fp_it->z = m_z;
                }

            }

            return Fpbl::ReturnStatus::STATUS_SUCCESS;
        }

        virtual Fpbl::ReturnStatus updateFingerprint(const Fpbl::PortalsGrid &portalsGrid, Fpbl::MfpBuilder::LocalDB *mfpFp)
        {
            std::ofstream f_map("portal.map", std::ios::out);
            f_map << "position.x" << " " << "position.y" << " " << "position.floor" << " " << "mode_of_transit" << " " << "height1" << " "<< "weight1" << " " << "height2" << " " << "weight2" << std::endl;
            Fpbl::CoordinatesInGrid position;
            //loop for all cells in the grid
            for (auto it = portalsGrid.begin(); it != portalsGrid.end(); ++it)
            {
                if ( ! it->positionInPortal.is_valid)
                    continue;

                std::vector<FfPosition> results = it->resultsInPortal;
                if (results.size() < min_portal_data_count)
                    continue;

                PortalType portalType = k_NoPortal;
                switch ((int)it->positionInPortal.mode_of_transit)
                {
                case kElevator:
                    portalType = k_Elevator;
                    break;
                case kStairs:
                    portalType = k_Stairs;
                    break;
                case kEscalatorWalking:
                case kEscalatorStanding:
                    portalType = k_Escalator;
                    break;
                case kConveyorWalking:
                case kConveyorStanding:
                    portalType = k_Conveyor;
                }
                if (portalType == k_NoPortal)
                    continue;

                position.x = it->positionInPortal.x;
                position.y = it->positionInPortal.y;
                position.floor = it->positionInPortal.floor;

                auto fp_it = mfpFp->findRecord(position); //find cell (in FP) which contains position (in the grid) of the measurement
                if (fp_it == mfpFp->cend())
                    continue;

                double height1;
                double height2;
                double weight1;
                double weight2;
#if 0
                //average mean model
                int N = 0;
                double sum = 0;
                for (auto itt = results.begin(); itt != results.cend(); ++itt)
                { 
                    if (itt->altitude_std > 0)
                    {
                        sum += itt->altitude;
                        N++;
                    }
                }
                if (N == 0)
                    continue;
                height1 = sum / N;
                height2 = 0;//sum / N;
                weight1 = 1;//0.5;
                weight2 = 0;//0.5;
#else
                //Gaussian mixture model
                std::vector<double> heights;
                for (auto itt = results.begin(); itt != results.cend(); ++itt)
                {
                    if(itt->altitude_std > 0)
                        heights.push_back(itt->altitude);
                }
                if (heights.size() == 0)
                    continue;

                std::sort(heights.begin(), heights.end());

                arma::gmm_diag model;
                arma::rowvec data(heights);
                //data.print();
                model.learn(data, 1, arma::eucl_dist, arma::static_spread, 0, 10, 1e-10, false);

                //model.dcovs.print();
                if (model.dcovs(0.0) > 1. && data.size() > 10)
                {
                    model.learn(data, 2, arma::eucl_dist, arma::static_spread, 0, 10, 1e-10, false);
                }

                arma::colvec m(2), d(2), h(2);
                m.zeros(); d.fill(5.0); h.zeros();

                for (arma::uword i = 0; i < model.hefts.size(); ++i)
                {
                    double w = model.hefts(i);

                    if (w > 0.1)
                    {
                        h(i) = w;
                    }

                    m(i) = model.means(0, i);
                }
                h = h / arma::sum(h);
                height1 = m(0);
                height2 = m(1);
                weight1 = h(0);
                weight2 = h(1);
#endif
                
                // To do: Move portal grid output outside of MFP-builder
                f_map << position.x << " " << position.y << " " << position.floor << " "
                    << portalType << " "
                    << height1 << " " << weight1 << " " << height2 << " " << weight2 << std::endl;

                fp_it->x.reserved[0] = (float)(portalType);

                // this block was diasbled as unused, this should be output in portal/mm map
                //fp_it->y.reserved[0] = height1; // 1st height of transit
                //fp_it->y.reserved[1] = weight1; // weight of 1st height of transit
                //fp_it->z.reserved[0] = height2; // 2nd height of transit
                //fp_it->z.reserved[1] = weight2; // weight of 2nd height of transit

                double mag_val1 = sqrt((fp_it->x.mu1 * fp_it->x.mu1) + (fp_it->y.mu1 * fp_it->y.mu1) + (fp_it->y.mu1 * fp_it->y.mu1));
                double mag_val2 = sqrt((fp_it->x.mu2 * fp_it->x.mu2) + (fp_it->y.mu2 * fp_it->y.mu2) + (fp_it->y.mu2 * fp_it->y.mu2));
                if ( (portalType == k_Elevator) &&
                     (mag_val1 < 0.1) && (mag_val2 < 0.1) && // any small value
                     ((std::abs(fp_it->x.s1 - UNKNOWN_VALUE_STD) < 0.01) &&
                      (std::abs(fp_it->y.s1 - UNKNOWN_VALUE_STD) < 0.01) &&
                      (std::abs(fp_it->z.s1 - UNKNOWN_VALUE_STD) < 0.01) /* &&
                       (results.size() < 100) */ )
                    )
                {
                    fp_it->x.mu1 = default_mag_x;
                    fp_it->x.w1 = 1;  
                    fp_it->x.s1 = 19.0;
                    fp_it->x.mu2 = default_mag_x;
                    fp_it->x.w2 = 0;
                    fp_it->x.s2 = 19.0;

                    fp_it->y.mu1 = default_mag_y;
                    fp_it->y.w1 = 1;
                    fp_it->y.s1 = 19.0;
                    fp_it->y.mu2 = default_mag_y;
                    fp_it->y.w2 = 0;
                    fp_it->y.s2 = 19.0;

                    fp_it->z.mu1 = default_mag_z;
                    fp_it->z.w1 = 1;
                    fp_it->z.s1 = 19.0;
                    fp_it->z.mu2 = default_mag_z;
                    fp_it->z.w2 = 0;
                    fp_it->z.s2 = 19.0;
                }
            }
            f_map.close();
            return Fpbl::ReturnStatus::STATUS_SUCCESS;
        }

        virtual Fpbl::ReturnStatus updateFingerprintForCrowdsourced(const Fpbl::CSGrid &csGrid, Fpbl::MfpBuilder::LocalDB *mfpFp)
        {
            Fpbl::CoordinatesInGrid position;
            //loop for all cells in the grid
            for (auto it = csGrid.begin(); it != csGrid.end(); ++it)
            {
                if (!it->position.is_valid)
                    continue;

                bool is_crowdsourced = it->position.is_crowdsourced;
                double earth_mag_x = it->position.mag_x;
                double earth_mag_y = it->position.mag_y;
                double earth_mag_z = it->position.mag_z;

                position.x = it->position.x;
                position.y = it->position.y;
                position.floor = it->position.floor;

                auto fp_it = mfpFp->findRecord(position); //find cell (in FP) which contains position (in the grid) of the measurement
                if (fp_it == mfpFp->cend())
                    continue;

                if ((std::abs(fp_it->x.s1 - UNKNOWN_VALUE_STD) < 0.01) && 
                    (std::abs(fp_it->y.s1 - UNKNOWN_VALUE_STD) < 0.01) && 
                    (std::abs(fp_it->z.s1 - UNKNOWN_VALUE_STD) < 0.01) && 
                    is_crowdsourced)
                {
                    // filling empty crowdsourced magnetic cells with Earth mag field values
                    fp_it->x.mu1 = earth_mag_x;
                    fp_it->y.mu1 = earth_mag_y;
                    fp_it->z.mu1 = earth_mag_z;

                    fp_it->x.s1 = fp_it->y.s1 = fp_it->z.s1 = 10.0;
                }
            }
            return Fpbl::ReturnStatus::STATUS_SUCCESS;
        }

    protected:
        virtual void initializeFp( Fpbl::MfpBuilder::LocalDB *mfpFp )
        {
            Fpbl::MfpBuilder::DBRecord::GaussMix defaultMix = { 0, 0, 0, 0, 0, 0, {0, 0} }; // init with zero
            defaultMix.mu1 = (float)UNKNOWN_VALUE_MEAN;
            defaultMix.s1  = (float)UNKNOWN_VALUE_STD;
            defaultMix.w1  = (float)1;

            if (mfpFp->min_data_count > MIN_CELL_DATA_COUNT)
                this->min_data_count = mfpFp->min_data_count;
            else
                this->min_data_count = MIN_CELL_DATA_COUNT;

            this->min_portal_data_count = MIN_CELL_PORTAL_DATA_COUNT;

            Fpbl::MfpBuilder::DBRecord defualtRec = { defaultMix, defaultMix, defaultMix };

            for ( auto it = mfpFp->records.begin(); it != mfpFp->records.end(); ++it )
            {
                ( *it ) = defualtRec;
            }
        }

        double recursiveMean( double oldMean, double newValue, size_t n )
        {
            double result = oldMean + ( newValue - oldMean ) / n;
            return result;
        }

        double recursiveVar( double oldVar, double oldMean, double newMean, double newValue, size_t n )
        {
            double oldMean2 = oldMean * oldMean;
            double newMean2 = newMean * newMean;
            double newValue2 = newValue * newValue;
            double result = oldVar + oldMean2 - newMean2 + ( newValue2 - oldVar - oldMean2 ) / n;

            return result;
        }

        const double UNKNOWN_VALUE_STD = 60;
        const double UNKNOWN_VALUE_MEAN = 0;
        const double DENUMERATOR_VALUE_MIN = 1e-6;
        const int    MIN_CELL_DATA_COUNT = 30; ///< minimal data count in cell for FP parameters calculation in this cell
        int          min_data_count;  
        const int    MIN_CELL_PORTAL_DATA_COUNT = 3; ///< minimal data count in cell for FP parameters calculation in this cell
        int          min_portal_data_count;
        double default_mag_x;
        double default_mag_y;
        double default_mag_z;

};

class Mfp3BuilderRobustRef : public Mfp3Builder
{
    public:

        virtual void setStdEstimatorType(Fpbl::eStdEstimatorType stdEstimatorType)
        {
            this->stdEstimatorType = stdEstimatorType;
        }

        virtual Fpbl::ReturnStatus buildFingerprint (const Fpbl::MagneticGrid &mfpGrid, Fpbl::MfpBuilder::LocalDB *mfpFp)
        {
            //initialize fingerprint with initial distribution
            initializeFp( mfpFp );
            ptrEstimator = stdEstimatorCreator.create(stdEstimatorType);
            ptrQualityEstimator = stdEstimatorCreator.create_mag_quality_estimator();
            //cells - dictionary with a Fp iterator as the key and grid cells list as value
            FpCells cells;

            //loop for all cells in the grid
            for ( auto it = mfpGrid.cbegin(); it != mfpGrid.cend(); ++it )
            {
                auto position = it->coordinates;
                auto mag_data = it->magData;
                if (mag_data.size() == 0)
                    continue;

                auto fp_it = mfpFp->findRecord( position ); //find cell(in FP) which contains position(in the grid) of the measurement

                if ( fp_it == mfpFp->cend() ) 
                    continue;

                auto m_it = cells.find( fp_it );

                if ( m_it != cells.end() )
                {
                    m_it->second.push_back( it ); //allready processed measurements
                }
                else
                {
                    GridCellsList c;
                    c.push_back( it );
                    cells[fp_it] = c;
                }
            }

            for ( auto it = cells.cbegin(); it != cells.cend(); ++it )
            {
                ////////////////
                // Set data type for fingerprint cell as follows
                // All data has robotic_survey_data : set robotic_survey_data
                // Otherwise set a type which is presented mostly(except RoboticMode)
                {
                    const GridCellsList& cell_list = it->second;
                    int dataset_type = set_dataset_type_for_cell(cell_list);
                    it->first->y.reserved[0] = (float)dataset_type;
                }
                ////////////////

                auto f_x = []( const MagneticData & mag_data )
                {
                    SimpleMeas m = { 0, mag_data.mX, mag_data.covarianceMatrix[0][0] };
                    return m;
                };
                auto f_y = []( const MagneticData & mag_data )
                {
                    SimpleMeas m = { 0, mag_data.mY, mag_data.covarianceMatrix[1][1] };
                    return m;
                };
                auto f_z = []( const MagneticData & mag_data )
                {
                    SimpleMeas m = { 0, mag_data.mZ, mag_data.covarianceMatrix[2][2] };
                    return m;
                };

                double mean_x( 0 ), mean_y( 0 ), mean_z( 0 );
                double std_x( 0 ), std_y( 0 ), std_z( 0 );
                double width_x(0), width_y(0), width_z(0);
                bool   flag_alternative_x(false), flag_alternative_y(false), flag_alternative_z(false);
                bool   result_x(false), result_y(false), result_z(false);

				int64_t num_of_meas = 0;

				result_x = estimateGaussianInCell(f_x, it, &mean_x, &std_x, &width_x, &flag_alternative_x, num_of_meas);
				result_y = estimateGaussianInCell(f_y, it, &mean_y, &std_y, &width_y, &flag_alternative_y, num_of_meas);
				result_z = estimateGaussianInCell(f_z, it, &mean_z, &std_z, &width_z, &flag_alternative_z, num_of_meas);

                bool flag_alternative =  flag_alternative_x || flag_alternative_y || flag_alternative_z;

				it->first->x.reserved[1] = num_of_meas;

                if (result_x)
                {					
                    it->first->x.mu1 = (float)mean_x;
                    if(flag_alternative) 
                        it->first->x.s1 = (float)width_x;
                    else
                        it->first->x.s1 = (float)std_x;
                    it->first->x.w1 = 1.0;
                }
                if (result_y)
                {
                    it->first->y.mu1 = (float)mean_y;
                    if (flag_alternative)
                        it->first->y.s1 = (float)width_y;
                    else
                        it->first->y.s1 = (float)std_y;
                    it->first->y.w1 = 1.0;
                }
                if (result_z)
                {
                    it->first->z.mu1 = (float)mean_z;
                    if (flag_alternative)
                        it->first->z.s1 = (float)width_z;
                    else
                        it->first->z.s1 = (float)std_z;
                    it->first->z.w1 = 1.0;
                }
				
				// mag quality estimation 
				double quality_x(0), quality_y(0), quality_z(0);
				bool quality_result_x = estimateQualityInCell(f_x, it, &quality_x);
				bool quality_result_y = estimateQualityInCell(f_y, it, &quality_y);
				bool quality_result_z = estimateQualityInCell(f_z, it, &quality_z);

				if (quality_result_x)
				{
					it->first->x.s2 = (float)quality_x;				
				}

				if (quality_result_y)
				{
					it->first->y.s2 = (float)quality_y;
				}

				if (quality_result_z)
				{
					it->first->z.s2 = (float)quality_z;
				}

#if 0           // debug output: check of robust estimations
                if (it->first->x.mu1 != 0)
                {
                    static int flag = 0;
                    std::string path_to_file = "mag_mfp.txt";
                    std::ofstream log;
                    if (flag == 0)
                    {
                        flag = 1;
                        log.open(path_to_file, std::ofstream::out | std::ofstream::trunc);
                    }
                    else
                        log.open(path_to_file, std::ofstream::out | std::ofstream::app);
                    auto it1 = it->second.back();
                    auto position = it1->coordinates;
                    log << std::fixed;
                    log.precision(1);
                    log << position.x << " " << position.y << "  ";
                    log.precision(5);
                    log << it->first->x.mu1 << " " << it->first->x.s1 << "  ";
                    log << it->first->y.mu1 << " " << it->first->y.s1 << "  ";
                    log << it->first->z.mu1 << " " << it->first->z.s1;
                    log << std::endl;
                    log.close();
                }
#endif
            }
            return Fpbl::ReturnStatus::STATUS_SUCCESS;
        }

    protected:

        Fpbl::StdEstimatorCreator  stdEstimatorCreator;
        Fpbl::IStdEstimator* ptrEstimator;
		Fpbl::IStdEstimator* ptrQualityEstimator;
        Fpbl::eStdEstimatorType stdEstimatorType;

        typedef std::list<Fpbl::MagneticGrid::const_iterator> GridCellsList;
        typedef std::map<Fpbl::MfpBuilder::LocalDB::iterator, GridCellsList> FpCells;

		bool estimateQualityInCell(SimpleMeas(*f)(const MagneticData &mag_data), const FpCells::const_iterator &cell_it, double *quality)
		{
			bool result = false;
			std::vector<SimpleMeas> processed_cell;
			const GridCellsList &cell_list = cell_it->second;

			for (auto it = cell_list.cbegin(); it != cell_list.cend(); ++it)
			{
				if (it == cell_list.cbegin())
				{
					processed_cell.reserve((*it)->magData.size() * cell_list.size());
				}

				for (auto it_mag = (*it)->magData.cbegin(); it_mag != (*it)->magData.cend(); ++it_mag)
				{
					SimpleMeas m = f(*it_mag);
					processed_cell.push_back(m);
				}
			}

			if (processed_cell.size() >= 2)
			{
				std::vector<double> v(processed_cell.size());
				size_t i = 0;

				for (auto it = processed_cell.cbegin(); it != processed_cell.cend(); ++it, ++i)
				{
					v[i] = it->v;
				}

				double mean_1 = median(v.begin(), v.end());
				double err = std::numeric_limits<double>::max();
				auto f_tukey = [](const double & x)
				{
					//double const K = M_PI / 2; // pi is a perfect number

					double const K = 5; // this value was set because it increases KPI for MFP built by FBMC data
										// test environment: irl dataset 416+451, FBMC version 4, mag class limit 2, weak validator set. on 28.02.2017

					double result = (std::abs(x) < K) ? (1 - pow(x / K, 2)) : 0;
					result = result * result;

					return result;
				};

				for (auto i = 0; ((i < MAX_ITERATIONS) && (err > FINAL_ERROR)); ++i)
				{
					iterationEstimateWeight(f_tukey, mean_1, &processed_cell);
					double mean = iterationEstimateMean(processed_cell);
					err = fabs(mean - mean_1);
					mean_1 = mean;
				}
				double width_1;
				bool flag_alternative_1;

				*quality = ptrQualityEstimator->estimate(processed_cell, mean_1, width_1, flag_alternative_1);

				result = true;
			}

			return result;
		}

        bool estimateGaussianInCell( SimpleMeas( *f )( const MagneticData &mag_data ), const FpCells::const_iterator &cell_it, double *mean, double *std, double *width, bool *flag_alternative, int64_t &num_of_meas)
        {
            bool result = false;
            std::vector<SimpleMeas> processed_cell;
            const GridCellsList &cell_list = cell_it->second;

            for ( auto it = cell_list.cbegin(); it != cell_list.cend(); ++it )
            {
                if ( it == cell_list.cbegin() )
                {
                    processed_cell.reserve( ( *it )->magData.size() * cell_list.size() );
                }

                for ( auto it_mag = ( *it )->magData.cbegin(); it_mag != ( *it )->magData.cend(); ++it_mag )
                {
                    SimpleMeas m = f( *it_mag );
                    processed_cell.push_back( m );
                }
            }

			num_of_meas = processed_cell.size();

            if (processed_cell.size() >= min_data_count)
            {
                estimateGaussian( &processed_cell, mean, std, width, flag_alternative);
                result = true;
            }

            return result;
        }
				
        void estimateGaussian( std::vector<SimpleMeas> *processed_cell, double *mean, double *std, double *width, bool *flag_alternative)
        {
            std::vector<double> v( processed_cell->size() );
            size_t i = 0;

            for ( auto it = processed_cell->cbegin(); it != processed_cell->cend(); ++it, ++i )
            {
                v[i] = it->v;
            }

            double mean_1 = median( v.begin(), v.end() );
            double err = std::numeric_limits<double>::max();
            auto f_tukey = []( const double & x )
            {
                //double const K = M_PI / 2; // pi is a perfect number
                
                double const K = 5; // this value was set because it increases KPI for MFP built by FBMC data
                                    // test environment: irl dataset 416+451, FBMC version 4, mag class limit 2, weak validator set. on 28.02.2017

                double result = ( std::abs(x) < K ) ? ( 1 - pow( x / K, 2 ) ) : 0;
                result = result * result;

                return result;
            };

            for ( auto i = 0; ( ( i < MAX_ITERATIONS ) && ( err > FINAL_ERROR ) ); ++i )
            {
                iterationEstimateWeight( f_tukey, mean_1, processed_cell );
                double mean = iterationEstimateMean( *processed_cell );
                err = fabs( mean - mean_1 );
                mean_1 = mean;
            }

            //double s2 = robustEstimateVar( *processed_cell, mean_1 );
            //*std = sqrt(s2);
            double width_1;
            bool flag_alternative_1;
            *std = ptrEstimator->estimate(*processed_cell, mean_1, width_1, flag_alternative_1);
            //std::cout << "*std: " << *std << std::endl;
            *width = width_1;
            *flag_alternative = flag_alternative_1;
            *mean = mean_1;
        }


        void iterationEstimateWeight( double( *weight_func )( const double & ), const double &mean, std::vector<SimpleMeas> *processed_cell )
        {
            for ( auto it = processed_cell->begin(); it != processed_cell->end(); ++it )
            {
                double x = ( it->v - mean ) / sqrt( it->s2 );
                double fx = weight_func( x );
                it->w = fx / ( it->s2 );
            }

        }


        double iterationEstimateMean( const std::vector<SimpleMeas> &processed_cell )
        {

            double numerator = 0.;
            double denumerator = 0.;
            double result = 0.;

            for ( auto it = processed_cell.cbegin(); it != processed_cell.cend(); ++it )
            {
                double v = it->v;
                double w = it->w;
                numerator += v * w;
                denumerator += w;
            }

            if (denumerator < DENUMERATOR_VALUE_MIN)
            {
                std::vector<double> vec(0);
                for (auto it = processed_cell.cbegin(); it != processed_cell.cend(); ++it)
                {
                    vec.push_back(it->v);
                }
                result = median(vec.begin(), vec.end());
            }
            else
            {
                result = numerator / denumerator;
            }

            return result;
        }

        // Set data type for fingerprint cell as follows
        // All data has robotic_survey_data : set robotic_survey_data
        // Otherwise set a type which is presented mostly(except RoboticMode)
        int set_dataset_type_for_cell(const GridCellsList& cell_list)
        {
            // vector will contain number of meas having defined data set type
            std::vector<std::pair<Fpbl::DataSetType, int>> dataset_types;
            size_t size_mag = 0;
            for (auto it_grid = cell_list.cbegin(); it_grid != cell_list.cend(); ++it_grid)
            {
                size_mag += (*it_grid)->magData.size();
                for (int i = 0; i < (*it_grid)->magData.size(); i++)
                {
                    Fpbl::DataSetType dataset_type = (Fpbl::DataSetType)((*it_grid)->magData[i].dataset_type);
                    // find dataset type in vector of dataset types or add if it is not present
                    auto it = std::find_if(dataset_types.begin(), dataset_types.end(),
                        [&](const std::pair<Fpbl::DataSetType, int>& p) { return  (p.first == dataset_type); });
                    if (it == dataset_types.cend())
                    {
                        dataset_types.push_back(std::pair<Fpbl::DataSetType, int>(dataset_type, 1));
                    }
                    else
                    {
                        it->second++;
                    }
                }
            }

            // sort vector by number of meas having defined data set type from the greatest to least
            std::sort(dataset_types.begin(), dataset_types.end(),
                [](const std::pair<Fpbl::DataSetType, int>& a,
                    const std::pair<Fpbl::DataSetType, int>& b)
                {
                    return (a.second > b.second);
                });

            // it will be used data set type with max number of meas
            // or Fpbl::kRoboticData if all meas have this type
            int dataset_type = (int)Fpbl::kUnKnown;
            Fpbl::DataSetType prefer_type = Fpbl::kUnKnown;
            if (dataset_types.size() > 0)
            {
                Fpbl::DataSetType prefer_type = dataset_types[0].first;
                if (prefer_type != Fpbl::kRoboticData || dataset_types.size() == 1)
                    dataset_type = (int)prefer_type;
                else
                {
                    if (dataset_types[0].first == size_mag) // it is possible to use (dataset_types[0].first >= k * size_mag) where k < 1  
                        dataset_type = (int)prefer_type;
                    else
                        dataset_type = (int)dataset_types[1].first;
                }
            }
            return dataset_type;
        }

        const int MAX_ITERATIONS = 20;
        const double FINAL_ERROR = 5e-3;// 1E-2;
};

/*  Cureently code below is not used
class Mfp3BuilderRobust : public Mfp3Builder
{
    public:
        Fpbl::ReturnStatus buildFingerprint( const Fpbl::MagneticGrid &mfpGrid, Fpbl::MfpBuilder::LocalDB *mfpFp )
        {
            initializeFp( mfpFp );
            //double err = std::numeric_limits<double>::max();


            for ( auto it = cells.begin(); it != cells.end(); ++it )
            {

            }

            //loop for all cells in the grid
            for ( auto it = mfpGrid.cbegin(); it != mfpGrid.cend(); ++it )
            {
                auto position = it->coordinates;
                auto fp_it = mfpFp->findRecord( position ); //find cell(in FP) which contains position(in the grid) of the measurement

                if ( fp_it == mfpFp->cend() ) continue;


                for ( auto m_it = it->magData.cbegin(); m_it != it->magData.cend(); ++m_it )
                {
                }
            }

            return Fpbl::ReturnStatus::STATUS_SUCCESS;
        }
    protected:

        const int MAX_ITERATIONS = 20;
        const double FINAL_ERROR = 1E-2;

        typedef std::list<Fpbl::MagneticGrid::const_iterator> GridCellsList;
        //mesurementsInCell - dictionary with a Fp iterator as the key and allready processed measurements
        std::map<Fpbl::MfpBuilder::LocalDB::iterator, GridCellsList> cells;
        typedef double Weight;
        struct Mean : public MagneticVector
        {
            Mean()
            {
                ;
            }
            Mean( const Mean &v )
            {
                *this = v;
            }
            Mean( const MagneticVector &v )
            {
                *this = v;
            }

            const Mean operator*( const Weight &w )
            {
                Mean res;
                res.mX = this->mX * w;
                res.mY = this->mY * w;
                res.mZ = this->mZ * w;
                return res;
            }
            Mean & operator+=( const Mean &R )
            {
                this->mX += R.mX;
                this->mY += R.mY;
                this->mZ += R.mZ;

                return *this;
            }
            const Mean operator-( const Mean &R )
            {
                Mean res;
                res.mX = this->mX - R.mX;
                res.mY = this->mY - R.mY;
                res.mZ = this->mZ - R.mZ;
                return res;
            }
            Mean & componentMult( const Mean &R )
            {
                this->mX *= R.mX;
                this->mY *= R.mY;
                this->mZ *= R.mZ;
                return *this;
            }

        };
        typedef Mean Variance;



        typedef std::pair<Weight, MagneticData> WeightedMeas;
        typedef std::vector<WeightedMeas> ProcessedMeas;
        //initialize fingerprint with initial distribution

        void initializeLocals( const Fpbl::MagneticGrid &mfpGrid, const Fpbl::MfpBuilder::LocalDB &mfpFp )
        {

        }


        void iterationEstimateWeight( Mean ( *psi_func )( const Mean & ), ProcessedMeas &processed_cell )
        {
        }

        Mean iterationEstimateMean( const ProcessedMeas &processed_cell )
        {

            Mean numerator = {};
            double denumerator = 0;

            for ( auto it = processed_cell.cbegin(); it != processed_cell.cend(); ++it )
            {
                Mean v = it->second;
                Weight w = it->first;
                numerator += v * w;
                denumerator += w;
            }

            auto inv_denum = 1 / denumerator;
            return numerator * inv_denum;
        }

        Variance robustEstimateVar( const ProcessedMeas &processed_cell, Mean mean )
        {
            Variance numerator = {};
            double denumerator = 0;

            for ( auto it = processed_cell.cbegin(); it != processed_cell.cend(); ++it )
            {
                Mean d = it->second;
                d = d - mean;
                Weight w = it->first;
                numerator += d.componentMult( d ) * w;
                denumerator += w;
            }

            auto inv_denum = 1 / denumerator;
            return numerator * inv_denum;
        }


        template<class RandAccessIter>
        double median( RandAccessIter begin, RandAccessIter end )    throw()
        {
            if ( begin == end )
            {
                throw  std::string("empty_container");
            }

            std::size_t size = end - begin;
            std::size_t middleIdx = size / 2;
            RandAccessIter target = begin + middleIdx;
            std::nth_element( begin, target, end );

            if ( size % 2 != 0 ) //Odd number of elements
            {
                return *target;
            }
            else             //Even number of elements
            {
                double a = *target;
                RandAccessIter targetNeighbor = target - 1;
                std::nth_element( begin, targetNeighbor, end );
                return ( a + *targetNeighbor ) / 2.0;
            }
        }
};
*/

#endif //MFP_3_BUILDER_HPP
