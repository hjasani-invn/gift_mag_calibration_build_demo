
#include "TrackCheckers.hpp"
#include "TrackProcessor.hpp"
#include "Venue.h"

#include <math.h>
#include <iomanip>
#include <iostream>

#include <sstream>
#include <vector>

#include <eigen/LU>
#include <eigen/Geometry>

using namespace TrackPreProcessing;

template <typename T> int sgn( T val )
{
    return ( T( 0 ) < val ) - ( val < T( 0 ) );
}

//-------------------------------------------------------------------------
/// Base class of magnetic checker
ValidationResult* MagChecker::ValidateDataSet(DataSet &data_set, const MFPVenue &venue)
{
    ValidationResult* result = CheckDatasetAndCreateResult(data_set, venue);
    if (result->is_passed() != true)
        InvalidateIrlMagData(data_set);
    return result;
}

void MagChecker::InvalidateIrlMagData(DataSet &data_set)
{
    for (std::vector <TpnOutput>::iterator irl_data_item = data_set.irl_data_.begin(); irl_data_item != data_set.irl_data_.end(); ++irl_data_item)
        irl_data_item->mag_meas.is_valid = false;
}


//-------------------------------------------------------------------------
// Class Checker_MaxMagModule
/// Method implements "maximal magnetic module" check
CheckResult Checker_MaxMagModule::CheckDataset(const DataSet &data_set, const MFPVenue &venue)
{
    CheckResult result;
    result.set_name( "max_mag_module" );
    result.set_threshold( max_mag_module_threshold_ );

    double max_module = std::numeric_limits<double>::lowest();

    std::vector <TpnOutput>::const_iterator  irl_item;
    std::vector <MagneticData>::const_iterator mag_data;
    for (mag_data = data_set.mag_data_mfpf_.begin(), irl_item = data_set.irl_data_.begin();
         (mag_data != data_set.mag_data_mfpf_.end()) && (irl_item != data_set.irl_data_.end());
         ++mag_data, ++irl_item)
    {
            double module = sqrt(mag_data->mX * mag_data->mX + mag_data->mY * mag_data->mY + mag_data->mZ * mag_data->mZ);

            if ((module > max_module) && irl_item->mag_meas.is_valid)
                max_module = module;
    }

    if ( max_module > 0 )
        result.set_criterion( max_module );

    result.EstimateValidity();
    return result;
}

//-------------------------------------------------------------------------
// Checker_MeanMagZ
/// Method implements "Mean mean mag Z in mfp frame " check
CheckResult Checker_MeanMagZ::CheckDataset(const DataSet &data_set, const MFPVenue &venue)
{
    CheckResult result;
    result.set_name( "mean_mag_Z_dev" );
    result.set_threshold( mean_mag_Z_deviation_threshold_ );

    double mean_mag_Z = 0;
    long cnt = 0;

    std::vector <TpnOutput>::const_iterator  irl_item;
    std::vector <MagneticData>::const_iterator mag_data;
    for (mag_data = data_set.mag_data_mfpf_.begin(), irl_item = data_set.irl_data_.begin();
        (mag_data != data_set.mag_data_mfpf_.end()) && (irl_item != data_set.irl_data_.end());
        ++mag_data, ++irl_item)
    {
        if (irl_item->mag_meas.is_valid && irl_item->attitude.is_valid) // check both validity due to dependings Z companents form mag and attitude
        {
            if (irl_item->pdr.is_valid && (irl_item->pdr.stride_length > 0.))
            {
                double mag_Z = mag_data->mZ;
                mean_mag_Z = (1.0 - (1.0 / (cnt + 1.0))) * mean_mag_Z + 1.0 / (cnt + 1.0) * mag_Z;
                cnt++;
            }
        }
    }

    if (cnt > 0 && sqrt(venue.magX * venue.magX + venue.magY * venue.magY + venue.magZ * venue.magZ) > 0.1) // any small value
        result.set_criterion(fabs( mean_mag_Z - venue.magZ ));
    else
        result.set_criterion(0);

    result.EstimateValidity();
    return result;

}

//-------------------------------------------------------------------------
// Checker_MeanMagHor
/// Method implements "Mean mean mag plane projection in mfp frame " check
CheckResult Checker_MeanMagHor::CheckDataset(const DataSet &data_set, const MFPVenue &venue)
{
    CheckResult result;
    result.set_name( "mean_mag_hor_dev" );
    result.set_threshold( mean_mag_hor_deviation_threshold_ );

    double mean_mag_hor = 0;
    long cnt = 0;

    std::vector <TpnOutput>::const_iterator  irl_item;
    std::vector <MagneticData>::const_iterator mag_data;
    for (mag_data = data_set.mag_data_mfpf_.begin(), irl_item = data_set.irl_data_.begin();
        (mag_data != data_set.mag_data_mfpf_.end()) && (irl_item != data_set.irl_data_.end());
        ++mag_data, ++irl_item)
    {
        if (irl_item->mag_meas.is_valid && irl_item->attitude.is_valid) // check both validity due to dependings plane companents form mag and attitude
        {
            if (irl_item->pdr.is_valid && (irl_item->pdr.stride_length > 0.))
            {
                double mag_hor = sqrt(mag_data->mX * mag_data->mX + mag_data->mY * mag_data->mY);
                mean_mag_hor = (1.0 - (1.0 / (cnt + 1.0))) * mean_mag_hor + 1.0 / (cnt + 1.0) * mag_hor;
                cnt++;
            }
        }
    }

    if(cnt > 0 && sqrt(venue.magX * venue.magX + venue.magY * venue.magY + venue.magZ * venue.magZ) > 0.1) // any small value
        result.set_criterion(fabs( mean_mag_hor - sqrt( venue.magX * venue.magX + venue.magY * venue.magY ) ));
    else
        result.set_criterion(0);

    result.EstimateValidity();
    return result;
}

//-------------------------------------------------------------------------
// Checker_MagBiasCovariation
/// Method implements "Max magnetic bias covariation" check
CheckResult Checker_MagBiasCovariation::CheckDataset(const DataSet &data_set, const MFPVenue &venue)
{
    CheckResult result;
    result.set_name( "mag_bias_cov" );
    result.set_threshold( mag_bias_covariance_threshold_ );

    double max_covariance = std::numeric_limits<double>::lowest();

    for ( std::vector <MagneticMeasurement>::const_iterator mag_data = data_set.mag_data_udf_.begin(); mag_data != data_set.mag_data_udf_.end(); ++mag_data )
    {
        max_covariance = std::max(max_covariance, mag_data->covarianceMatrix[0][0]);
        max_covariance = std::max(max_covariance, mag_data->covarianceMatrix[1][1]);
        max_covariance = std::max(max_covariance, mag_data->covarianceMatrix[2][2]);
    }

    if ( max_covariance > 0 )
        result.set_criterion( max_covariance );

    result.EstimateValidity();
    return result;
}

//-------------------------------------------------------------------------
// Checker_MagBiasLevel
/// Method implements "Max magnetic bias level" check
CheckResult_MinThreshold Checker_MagBiasLevel::CheckDataset(const DataSet &data_set, const MFPVenue &venue)
{
    CheckResult_MinThreshold result;
    result.set_name("mag_bias_level");
    result.set_threshold(mag_bias_level_threshold_);

    uint8_t min_level = std::numeric_limits<uint8_t>::max();

    for (std::vector <TpnOutput>::const_iterator irl_data_item = data_set.irl_data_.begin(); irl_data_item != data_set.irl_data_.end(); ++irl_data_item)
    {
        if (irl_data_item->mag_meas.is_valid)
            min_level = std::min(min_level, irl_data_item->mag_meas.level_of_calibration);
    }

    if (min_level < std::numeric_limits<uint8_t>::max())
        result.set_criterion(min_level);

    result.EstimateValidity();
    return result;
}

//-------------------------------------------------------------------------
// Checker_B_value
/// Method implements "B-value criterion" check
CheckResult Checker_B_value::CheckDataset(const DataSet &data_set, const MFPVenue &venue)
{
    CheckResult result;
    result.set_name( "B_value" );
    result.set_threshold( B_threshold_ );

    size_t mag_meas_num = data_set.mag_data_mfpf_.size();
    size_t ang_meas_num = data_set.irl_data_.size();

    if ((mag_meas_num <= N_window_) || (ang_meas_num == N_window_))
    {
        return result; // no data for analyses
    }

    assert( mag_meas_num == ang_meas_num );

    size_t meas_num = mag_meas_num;

    Eigen::MatrixXd mag_data( meas_num, 3 );
    Eigen::MatrixXd angular_data( meas_num, 3 );

    Eigen::MatrixXd smoothed_mag_data( meas_num, 3 );
    Eigen::MatrixXd smoothed_angular_data( meas_num, 3 );

    Eigen::MatrixXd diff_mag( meas_num, 3 );
    Eigen::MatrixXd diff_angular( meas_num, 3 );

    mag_data( 0, 0 ) = data_set.mag_data_mfpf_[0].mX;
    mag_data( 0, 1 ) = data_set.mag_data_mfpf_[0].mY;
    mag_data( 0, 2 ) = data_set.mag_data_mfpf_[0].mZ;

    angular_data( 0, 0 ) = data_set.irl_data_[0].attitude.roll;
    angular_data( 0, 1 ) = data_set.irl_data_[0].attitude.pitch;
    angular_data( 0, 2 ) = data_set.irl_data_[0].attitude.heading;

    // filling the matrices and converting angles
    for (size_t i = 1; i < meas_num; ++i)
    {
        mag_data( i, 0 ) = data_set.mag_data_mfpf_[i].mX;
        mag_data( i, 1 ) = data_set.mag_data_mfpf_[i].mY;
        mag_data( i, 2 ) = data_set.mag_data_mfpf_[i].mZ;

        angular_data( i, 0 ) = data_set.irl_data_[i].attitude.roll;
        angular_data( i, 1 ) = data_set.irl_data_[i].attitude.pitch;
        angular_data( i, 2 ) = data_set.irl_data_[i].attitude.heading;

        double delta_heading = angular_data( i, 2 ) - angular_data( i - 1, 2 );

        if ( std::abs( delta_heading ) > 300 )
        {
            delta_heading = -sgn( delta_heading ) * 360;
            angular_data( i, 2 ) += delta_heading;
        }

        if ( std::abs( angular_data( i, 2 ) ) > ( 360 + 45 ) )
        {
            delta_heading = -sgn( angular_data( i, 2 ) ) * 360;
            angular_data( i, 2 ) += delta_heading;
        }

        double delta_pitch = angular_data( i, 1 ) - angular_data( i - 1, 1 );

        if ( std::abs( delta_pitch ) > 300 )
        {
            delta_pitch = -sgn( delta_pitch ) * 360;
            angular_data( i, 1 ) += delta_pitch;
        }

        if ( std::abs( angular_data( i, 1 ) ) > ( 360 + 45 ) )
        {
            delta_pitch = -sgn( angular_data( i, 1 ) ) * 360;
            angular_data( i, 1 ) += delta_pitch;
        }

        double delta_roll = angular_data( i, 0 ) - angular_data( i - 1, 0 );

        if ( std::abs( delta_roll ) > 300 )
        {
            delta_roll = -sgn( delta_roll ) * 360;
            angular_data( i, 0 ) += delta_roll;
        }

        if ( std::abs( angular_data( i, 0 ) ) > ( 360 + 45 ) )
        {
            delta_roll = -sgn( angular_data( i, 0 ) ) * 360;
            angular_data( i, 0 ) += delta_roll;
        }

        angular_data( i, 0 ) *= M_PI / 180.0;
        angular_data( i, 1 ) *= M_PI / 180.0;
        angular_data( i, 2 ) *= M_PI / 180.0;
    }

    double mean_magX = 0;
    double mean_magY = 0;
    double mean_magZ = 0;

    double mean_angularX = 0;
    double mean_angularY = 0;
    double mean_angularZ = 0;

    int it = 0;
    int end_idx = meas_num - 1 - N_window_ / 2;

    for (unsigned int i = 0; i <= N_window_; i++)
    {
        mean_magX = ( 1.0 - ( 1.0 / ( it + 1.0 ) ) ) * mean_magX + 1.0 / ( it + 1.0 ) * mag_data( i, 0 );
        mean_magY = ( 1.0 - ( 1.0 / ( it + 1.0 ) ) ) * mean_magY + 1.0 / ( it + 1.0 ) * mag_data( i, 1 );
        mean_magZ = ( 1.0 - ( 1.0 / ( it + 1.0 ) ) ) * mean_magZ + 1.0 / ( it + 1.0 ) * mag_data( i, 2 );

        mean_angularX = ( 1.0 - ( 1.0 / ( it + 1.0 ) ) ) * mean_angularX + 1.0 / ( it + 1.0 ) * angular_data( i, 0 );
        mean_angularY = ( 1.0 - ( 1.0 / ( it + 1.0 ) ) ) * mean_angularY + 1.0 / ( it + 1.0 ) * angular_data( i, 1 );
        mean_angularZ = ( 1.0 - ( 1.0 / ( it + 1.0 ) ) ) * mean_angularZ + 1.0 / ( it + 1.0 ) * angular_data( i, 2 );

        it++;

        smoothed_mag_data( i, 0 ) = mag_data( i, 0 ); smoothed_mag_data( i, 1 ) = mag_data( i, 1 ); smoothed_mag_data( i, 2 ) = mag_data( i, 2 );
        smoothed_angular_data( i, 0 ) = angular_data( i, 0 ); smoothed_angular_data( i, 1 ) = angular_data( i, 1 ); smoothed_angular_data( i, 2 ) = angular_data( i, 2 );

        diff_mag( i, 0 ) = 0; diff_mag( i, 1 ) = 0; diff_mag( i, 2 ) = 0;
        diff_angular( i, 0 ) = 0; diff_angular( i, 1 ) = 0; diff_angular( i, 2 ) = 0;

        int j = meas_num - 1 - i;

        smoothed_mag_data( j, 0 ) = mag_data( j, 0 ); smoothed_mag_data( j, 1 ) = mag_data( j, 1 ); smoothed_mag_data( j, 2 ) = mag_data( j, 2 );
        smoothed_angular_data( j, 0 ) = angular_data( j, 0 ); smoothed_angular_data( j, 1 ) = angular_data( j, 1 ); smoothed_angular_data( j, 2 ) = angular_data( j, 2 );

        diff_mag( j, 0 ) = 0; diff_mag( j, 1 ) = 0; diff_mag( j, 2 ) = 0;
        diff_angular( j, 0 ) = 0; diff_angular( j, 1 ) = 0; diff_angular( j, 2 ) = 0;
    }

    // now we have mean values calculated for index = N/2

    for ( int i = N_window_ / 2; i < end_idx; ++i )
    {
        smoothed_mag_data( i, 0 ) = mean_magX;
        smoothed_mag_data( i, 1 ) = mean_magY;
        smoothed_mag_data( i, 2 ) = mean_magZ;

        smoothed_angular_data( i, 0 ) = mean_angularX;
        smoothed_angular_data( i, 1 ) = mean_angularY;
        smoothed_angular_data( i, 2 ) = mean_angularZ;

        diff_mag( i, 0 ) = mag_data( i, 0 ) - smoothed_mag_data( i, 0 );
        diff_mag( i, 1 ) = mag_data( i, 1 ) - smoothed_mag_data( i, 1 );
        diff_mag( i, 2 ) = mag_data( i, 2 ) - smoothed_mag_data( i, 2 );

        diff_angular( i, 0 ) = angular_data( i, 0 ) - smoothed_angular_data( i, 0 );
        diff_angular( i, 1 ) = angular_data( i, 1 ) - smoothed_angular_data( i, 1 );
        diff_angular( i, 2 ) = angular_data( i, 2 ) - smoothed_angular_data( i, 2 );

        //calculate_new_window_mean( mag_data, i + 1, mean_magX, mean_magY, mean_magZ ); // next mean value calculation
        //calculate_new_window_mean( angular_data, i + 1, mean_angularX, mean_angularY, mean_angularZ );

        // ! function calls took too much time, moved calculation here:

        int leaving_idx = i + 1 - N_window_ / 2 - 1;
        int new_idx = i + 1 + N_window_ / 2;
        mean_magX = mean_magX + ( mag_data( new_idx, 0 ) - mag_data( leaving_idx, 0 ) ) / ( N_window_ + 1 );
        mean_magY = mean_magY + ( mag_data( new_idx, 1 ) - mag_data( leaving_idx, 1 ) ) / ( N_window_ + 1 );
        mean_magZ = mean_magZ + ( mag_data( new_idx, 2 ) - mag_data( leaving_idx, 2 ) ) / ( N_window_ + 1 );

        mean_angularX = mean_angularX + ( angular_data( new_idx, 0 ) - angular_data( leaving_idx, 0 ) ) / ( N_window_ + 1 );
        mean_angularY = mean_angularY + ( angular_data( new_idx, 1 ) - angular_data( leaving_idx, 1 ) ) / ( N_window_ + 1 );
        mean_angularZ = mean_angularZ + ( angular_data( new_idx, 2 ) - angular_data( leaving_idx, 2 ) ) / ( N_window_ + 1 );
    }

    Eigen::MatrixXd t_diff_mag( 3, meas_num );
    Eigen::MatrixXd t_diff_angular( 3, meas_num );

    t_diff_mag = diff_mag.transpose();
    t_diff_angular = diff_angular.transpose();

    Eigen::Matrix3d m_cov;
    Eigen::Matrix3d a_cov;

    m_cov = t_diff_mag * diff_mag / meas_num;
    a_cov = t_diff_angular * diff_angular / meas_num;

    double B = ( m_cov( 0, 0 ) + m_cov( 1, 1 ) + m_cov( 2, 2 ) ) / ( a_cov( 0, 0 ) + a_cov( 1, 1 ) + a_cov( 2, 2 ) );

    result.set_criterion( B );
    result.EstimateValidity();

    return result;
}

//-------------------------------------------------------------------------
CheckResult Checker_MaxUserSpeed::CheckDataset(const DataSet &data_set, const MFPVenue &venue)
{
	CheckResult result;
	result.set_name("max_user_speed");
	result.set_threshold(user_speed_limit_);

	double max_speed = std::numeric_limits<double>::min();
    if (data_set.position_data_ff_.size() > 0)
    {
        for (auto prev_pos_data_item = data_set.position_data_ff_.begin(), pos_data_item = data_set.position_data_ff_.begin() + 1; (pos_data_item != data_set.position_data_ff_.end()); ++pos_data_item, ++prev_pos_data_item)
        {
            if (pos_data_item->is_valid && prev_pos_data_item->is_valid)
            {
                //calculate speed
                double dx = (pos_data_item->x - prev_pos_data_item->x) / (pos_data_item->timestamp - prev_pos_data_item->timestamp) * 1000.;
                double dy = (pos_data_item->y - prev_pos_data_item->y) / (pos_data_item->timestamp - prev_pos_data_item->timestamp) * 1000.;
                double speed = std::sqrt(dx * dx + dy * dy);
                max_speed = std::max(speed, max_speed);
            }
        }
    }

	result.set_criterion(max_speed);
	result.EstimateValidity();

	return result;
}

//-------------------------------------------------------------------------
CheckResult Checker_MaxMagDataDerivative::CheckDataset(const DataSet &data_set, const MFPVenue &venue)
{
	CheckResult result;
	result.set_name("max_mag_derivative");
	result.set_threshold(mag_derivative_limit_);

    size_t meas_num = data_set.irl_data_.size();

	double max_derivative = std::numeric_limits<double>::min();
    if (data_set.position_data_ff_.size() > 0)
    {
        for (int i = 0; i < meas_num - 1; ++i)
        {
            if (data_set.irl_data_[i].mag_meas.is_valid && data_set.irl_data_[i + 1].mag_meas.is_valid)
            {
                // calculating mag meas derivative in MFP frame
                double abs_mag_x_derivative = std::abs(data_set.mag_data_mfpf_[i + 1].mX - data_set.mag_data_mfpf_[i].mX) / (data_set.mag_data_mfpf_[i + 1].timestamp - data_set.mag_data_mfpf_[i].timestamp) * 1000;
                double abs_mag_y_derivative = std::abs(data_set.mag_data_mfpf_[i + 1].mY - data_set.mag_data_mfpf_[i].mY) / (data_set.mag_data_mfpf_[i + 1].timestamp - data_set.mag_data_mfpf_[i].timestamp) * 1000;
                double abs_mag_z_derivative = std::abs(data_set.mag_data_mfpf_[i + 1].mZ - data_set.mag_data_mfpf_[i].mZ) / (data_set.mag_data_mfpf_[i + 1].timestamp - data_set.mag_data_mfpf_[i].timestamp) * 1000;

                max_derivative = std::max({ max_derivative, abs_mag_x_derivative, abs_mag_y_derivative, abs_mag_z_derivative });
            }
        }
    }

	result.set_criterion(max_derivative);
	result.EstimateValidity();

	return result;
}

//-------------------------------------------------------------------------
CheckResult Checker_DataSetScoreForPosMag::CheckDataset(const DataSet &data_set, const MFPVenue &venue)
{
    CheckResult result;
    result.set_name("data_set_score_for_mag");
    result.set_criterion(0);
    result.set_ready(true);
    result.set_passed(true);

    size_t meas_num = data_set.irl_data_.size();

    if (meas_num != 0)
    {
        double dataset_score = data_set.dataset_params_.dataset_score_;
        // for correct log printing
        result.set_criterion(dataset_score);
        result.set_ready(true);
        if (dataset_score < min_data_set_score_)
            result.set_passed(false);
    }

    //result.EstimateValidity();

    return result;
}

//-------------------------------------------------------------------------
CheckResult Checker_MagDataPercent::CheckDataset(const DataSet &data_set, const MFPVenue &venue)
{
    CheckResult result;
    result.set_name("invalid_mag_data_percent");
    result.set_criterion(0);
    result.set_threshold(min_percent_);
    result.set_ready(true);
    result.set_passed(true);

    size_t meas_num = data_set.irl_data_.size();
    if (meas_num != 0)
    {
        size_t invalid_counter = 0;
        for (size_t i = 0; i < meas_num; ++i)
        {
            if (data_set.irl_data_[i].mag_meas.is_valid == false)
                invalid_counter++;
        }
        result.set_criterion(invalid_counter / (double)meas_num);
    }
    //else
    //    result.set_criterion(min_percent_ + 0.001); // set false validation for empty  irl_data_

    result.EstimateValidity();

    return result;
}

//-------------------------------------------------------------------------
CheckResult Checker_PositionPercent::CheckDataset(const DataSet &data_set, const MFPVenue &venue)
{
    CheckResult result;
    result.set_name("invalid_position_percent");
    result.set_criterion(0);
    result.set_threshold(min_percent_);
    result.set_ready(true);
    result.set_passed(true);

    size_t meas_num = data_set.irl_data_.size();
    if (meas_num != 0)
    {
        size_t invalid_counter = 0;
        for (size_t i = 0; i < meas_num; ++i)
        {
            if (data_set.irl_data_[i].position.is_valid == false)
                invalid_counter++;
        }
        result.set_criterion(invalid_counter / (double)meas_num);
    }
    else
        result.set_criterion(min_percent_ + 0.001); // set false validation for empty  irl_data_

    result.EstimateValidity();

    return result;
}

//-------------------------------------------------------------------------
CheckResult Checker_DatasetTypeInformer::CheckDataset(const DataSet &data_set, const MFPVenue &venue)
{
    CheckResult result;
    result.set_name("data_set_type_info");
    Fpbl::DataSetType dataset_type_ = data_set.dataset_params_.dataset_type_;
    result.set_criterion(dataset_type_);
    result.set_ready(true);
    result.set_passed(true);

    return result;
}

