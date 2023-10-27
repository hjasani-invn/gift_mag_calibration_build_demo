#include "TrackCorrectors.hpp"
#include "TrackProcessor.hpp"
#include "tpn_converter.hpp"
#include <algorithm>
#include <assert.h>

#include <math.h>
#include <fstream>

#include <eigen/LU>
#include <eigen/Geometry>

using namespace TrackPreProcessing;

template <typename SampleType>
class  SlideWindowFilter
{
public:
    SlideWindowFilter(size_t length) : length_(length) { reset(); };

    SampleType get_flt_value() { return flt_value; }
    double get_delay()  { return (sample_list.size() > 0) ? (sample_list.size() - 1) / 2. : 0; }
    bool is_full()      { return (sample_list.size() >= length_); }

    void reset()
    {
        flt_value = {0};
        sample_list.clear();
    }

    bool push_sample(SampleType sample)
    {
        sample_list.push_back(sample);

        if (sample_list.size() > length_)
        {
            sample = sample - *sample_list.begin();
            sample_list.pop_front();
            flt_value = flt_value + sample * (1. / sample_list.size());
        }
        else
        {
            flt_value = (flt_value * (sample_list.size() - 1) + sample)* (1. / sample_list.size());
        }

        return is_full();
    }

private:
    size_t length_;
    SampleType flt_value;
    std::list <SampleType> sample_list;
};

//-------------------------------------------------------------------------------------------------
// Corrector_FilterMagData
CorrectionResult Corrector_FilterMagData::CorrectDataset(DataSet &data_set, const MFPVenue &venue)
{
    CorrectionResult result;
    result.set_name("mag_data_filter");

    size_t meas_num = data_set.irl_data_.size();
   
    SlideWindowFilter < MagneticVector> filter((N_window_ >> 1) * 2 + 1); // nearest odd number

#if 0
    {// debug output
        std::string path_to_file = "dbg_mag_data_raw.csv";
        std::ofstream irl_log(path_to_file, std::ofstream::out | std::ofstream::trunc);
        
        for (size_t i = 0; i < meas_num; i++)
        {
            irl_log << data_set.irl_data_[i].timestamp;

            irl_log << ", ";
            irl_log << ", " << data_set.attitude_data_udf2mfpf_[i].quaternion[0];
            irl_log << ", " << data_set.attitude_data_udf2mfpf_[i].quaternion[1];
            irl_log << ", " << data_set.attitude_data_udf2mfpf_[i].quaternion[2];
            irl_log << ", " << data_set.attitude_data_udf2mfpf_[i].quaternion[3];
            irl_log << ", " << data_set.attitude_data_udf2mfpf_[i].is_valid;

            irl_log << ", ";
            irl_log << ", " << data_set.irl_data_[i].attitude.roll;
            irl_log << ", " << data_set.irl_data_[i].attitude.pitch;
            irl_log << ", " << data_set.irl_data_[i].attitude.heading;
            irl_log << ", " << data_set.irl_data_[i].attitude.is_valid;

            irl_log << ", ";
            irl_log << ", " << data_set.irl_data_[i].mag_meas.mX;
            irl_log << ", " << data_set.irl_data_[i].mag_meas.mY;
            irl_log << ", " << data_set.irl_data_[i].mag_meas.mZ;
            irl_log << ", " << data_set.irl_data_[i].mag_meas.is_valid;

            irl_log << ", ";
            irl_log << ", " << data_set.mag_data_mfpf_[i].mX;
            irl_log << ", " << data_set.mag_data_mfpf_[i].mY;
            irl_log << ", " << data_set.mag_data_mfpf_[i].mZ;

            irl_log << ", ";
            irl_log << ", " << data_set.irl_data_[i].position.is_valid;
            irl_log << ", " << data_set.irl_data_[i].pdr.stride_length;
            irl_log << std::endl;
        }
    }
#endif

    for (size_t i = 0; i < meas_num; i++)
        if (data_set.irl_data_[i].mag_meas.is_valid && data_set.irl_data_[i].mag_meas.is_valid)
        {
            MagneticVector mag_meas = (MagneticVector) data_set.mag_data_mfpf_[i];

            // smoothing mag data in MFF 
            filter.push_sample(mag_meas);
            mag_meas = filter.get_flt_value();

            // transform MFF mag data to UDF
            double delay = filter.get_delay();
            double *q = data_set.attitude_data_udf2mfpf_[i - (size_t)delay].quaternion;
            //Eigen::Quaternion<double> q_mff2udf (q[0], -q[1], -q[2], -q[3]);
            //Eigen::Quaternion<double> q_mag(0., mag_meas.mX, mag_meas.mY, mag_meas.mZ);
            Eigen::Quaternion<double> q_mag_udf = Eigen::Quaternion<double> (q[0],       -q[1],       -q[2],       -q[3]) * 
                                                  Eigen::Quaternion<double> (  0., mag_meas.mX, mag_meas.mY, mag_meas.mZ) * 
                                                  Eigen::Quaternion<double>( q[0],       -q[1],       -q[2],        -q[3]).conjugate();

            // save filtered mag data
            const double mGauss_per_uTesla = 10.0;
            data_set.irl_data_[i - (size_t)delay].mag_meas.mX = q_mag_udf.x()*mGauss_per_uTesla;
            data_set.irl_data_[i - (size_t)delay].mag_meas.mY = q_mag_udf.y()*mGauss_per_uTesla;
            data_set.irl_data_[i - (size_t)delay].mag_meas.mZ = q_mag_udf.z()*mGauss_per_uTesla;
            data_set.irl_data_[i - (size_t)delay].mag_meas.is_valid = filter.is_full();
            
            result.IncCorrectionCount();
        }
        else
        {
            for (int32_t delay = (int32_t)filter.get_delay(); delay > 0; delay--)
            {
                data_set.irl_data_[i - delay].mag_meas.is_valid = false;
            }
            filter.reset();
        }

#if 0
        {// debug output
            std::string path_to_file = "dbg_mag_data_flt.csv";
            std::ofstream irl_log(path_to_file, std::ofstream::out | std::ofstream::trunc);

            for (size_t i = 0; i < meas_num; i++)
            {
                irl_log << data_set.irl_data_[i].timestamp;

                irl_log << ", ";
                irl_log << ", " << data_set.attitude_data_udf2mfpf_[i].quaternion[0];
                irl_log << ", " << data_set.attitude_data_udf2mfpf_[i].quaternion[1];
                irl_log << ", " << data_set.attitude_data_udf2mfpf_[i].quaternion[2];
                irl_log << ", " << data_set.attitude_data_udf2mfpf_[i].quaternion[3];
                irl_log << ", " << data_set.attitude_data_udf2mfpf_[i].is_valid;

                irl_log << ", ";
                irl_log << ", " << data_set.irl_data_[i].attitude.roll;
                irl_log << ", " << data_set.irl_data_[i].attitude.pitch;
                irl_log << ", " << data_set.irl_data_[i].attitude.heading;
                irl_log << ", " << data_set.irl_data_[i].attitude.is_valid;

                irl_log << ", ";
                irl_log << ", " << data_set.irl_data_[i].mag_meas.mX;
                irl_log << ", " << data_set.irl_data_[i].mag_meas.mY;
                irl_log << ", " << data_set.irl_data_[i].mag_meas.mZ;
                irl_log << ", " << data_set.irl_data_[i].mag_meas.is_valid;

                double *q = data_set.attitude_data_udf2mfpf_[i].quaternion;
                Eigen::Quaternion<double> q_udf2mff(q[0], q[1], q[2], q[3]);
                Eigen::Quaternion<double> q_mag(0., data_set.irl_data_[i].mag_meas.mX, data_set.irl_data_[i].mag_meas.mY, data_set.irl_data_[i].mag_meas.mZ);
                Eigen::Quaternion<double> q_mag_udf = q_udf2mff * q_mag * q_udf2mff.conjugate();

                const double mGauss_per_uTesla = 10.0;
                irl_log << ", ";
                irl_log << ", " << q_mag_udf.x() / mGauss_per_uTesla;
                irl_log << ", " << q_mag_udf.y() / mGauss_per_uTesla;
                irl_log << ", " << q_mag_udf.z() / mGauss_per_uTesla;

                irl_log << ", ";
                irl_log << ", " << data_set.irl_data_[i].position.is_valid;
                irl_log << ", " << data_set.irl_data_[i].pdr.stride_length;
                irl_log << std::endl;
            }
        }
# endif

    result.set_ready(true);
    result.set_passed(true);
    return result;
}

//-------------------------------------------------------------------------------------------------
// Corrector_FilterMagData
/*CorrectionResult Corrector_FilterMagData::CorrectDataset( DataSet &data_set, const Venue &venue )
{
    CorrectionResult result;
    result.set_name( "mag_data_filter" );

    size_t meas_num = data_set.mag_data_mfpf_.size();
    
    if (meas_num <= N_window_)
    { // no data for correction
        result.reset();
        return result;
    }

    Eigen::MatrixXd smoothed_mag_data( meas_num, 3 );

    double mean_magX = 0;    double mean_magY = 0;    double mean_magZ = 0;

    int it = 0;
    int end_idx = meas_num - 1 - N_window_ / 2;

    for ( int i = 0; i <= N_window_; i++ )
    {
        mean_magX = ( 1.0 - ( 1.0 / ( it + 1.0 ) ) ) * mean_magX + 1.0 / ( it + 1.0 ) * data_set.mag_data_mfpf_[i].mX;
        mean_magY = ( 1.0 - ( 1.0 / ( it + 1.0 ) ) ) * mean_magY + 1.0 / ( it + 1.0 ) * data_set.mag_data_mfpf_[i].mY;
        mean_magZ = ( 1.0 - ( 1.0 / ( it + 1.0 ) ) ) * mean_magZ + 1.0 / ( it + 1.0 ) * data_set.mag_data_mfpf_[i].mZ;

        it++;

        smoothed_mag_data( i, 0 ) = data_set.mag_data_mfpf_[i].mX;
        smoothed_mag_data( i, 1 ) = data_set.mag_data_mfpf_[i].mY;
        smoothed_mag_data( i, 2 ) = data_set.mag_data_mfpf_[i].mZ;

        int j = meas_num - 1 - i;

        smoothed_mag_data( j, 0 ) = data_set.mag_data_mfpf_[j].mX;
        smoothed_mag_data( j, 1 ) = data_set.mag_data_mfpf_[j].mY;
        smoothed_mag_data( j, 2 ) = data_set.mag_data_mfpf_[j].mZ;

        if ( i < N_window_ / 2 )
        {
            data_set.irl_data_[i].mag_meas.is_valid = false;
            result.IncCorrectionCount();
        }

        if ( j > end_idx )
        {
            data_set.irl_data_[j].mag_meas.is_valid = false;
            result.IncCorrectionCount();
        }
    }

    // now we have mean values calculated for index = N/2

    for ( int i = N_window_ / 2; i < end_idx; ++i )
    {
        smoothed_mag_data( i, 0 ) = mean_magX;
        smoothed_mag_data( i, 1 ) = mean_magY;
        smoothed_mag_data( i, 2 ) = mean_magZ;

        int leaving_idx = i + 1 - N_window_ / 2 - 1;
        int new_idx = i + 1 + N_window_ / 2;
        // calculating mean values for the next element
        mean_magX = mean_magX + ( data_set.mag_data_mfpf_[new_idx].mX - data_set.mag_data_mfpf_[leaving_idx].mX ) / ( N_window_ + 1 );
        mean_magY = mean_magY + ( data_set.mag_data_mfpf_[new_idx].mY - data_set.mag_data_mfpf_[leaving_idx].mY ) / ( N_window_ + 1 );
        mean_magZ = mean_magZ + ( data_set.mag_data_mfpf_[new_idx].mZ - data_set.mag_data_mfpf_[leaving_idx].mZ ) / ( N_window_ + 1 );
    }

    // changing values of mag_mfpf in data_set to filtered values
    // also changing irl_data to rotated mag vectors and 0 attitude angles

    const double mGauss_per_uTesla = 10.0;

    Eigen::Matrix3d R = Eigen::Quaternion<double>( 0, -sqrt( 2 ) / 2., -sqrt( 2 ) / 2., 0 ).matrix();

    double az = venue.origin_azimuth;

    Eigen::Matrix3d R_az;
    R_az.setZero();

    R_az( 0, 0 ) = cos( az ); R_az( 0, 1 ) = sin( az );
    R_az( 1, 0 ) = -sin( az ); R_az( 1, 1 ) = cos( az );
    R_az( 2, 2 ) = 1;

    for ( auto mag_data = data_set.mag_data_mfpf_.begin(); mag_data != data_set.mag_data_mfpf_.end(); ++mag_data )
    {
        unsigned int current_index = mag_data - data_set.mag_data_mfpf_.begin();

        // const FLOAT32 q_ned2mfp[4] = { 0, sqrt_2_2, sqrt_2_2, 0 };

        Eigen::Vector3d mag( smoothed_mag_data( current_index, 0 ), smoothed_mag_data( current_index, 1 ), smoothed_mag_data( current_index, 2 ) );

        mag = R * R_az * mag;

        data_set.mag_data_mfpf_[current_index].mX = mag( 0 );
        data_set.mag_data_mfpf_[current_index].mY = mag( 1 );
        data_set.mag_data_mfpf_[current_index].mZ = mag( 2 );

        data_set.irl_data_[current_index].mag_meas.mX = data_set.mag_data_mfpf_[current_index].mX * mGauss_per_uTesla;
        data_set.irl_data_[current_index].mag_meas.mY = data_set.mag_data_mfpf_[current_index].mY * mGauss_per_uTesla;
        data_set.irl_data_[current_index].mag_meas.mZ = data_set.mag_data_mfpf_[current_index].mZ * mGauss_per_uTesla;

        data_set.irl_data_[current_index].attitude.roll = 0.0;
        data_set.irl_data_[current_index].attitude.pitch = 0.0;
        data_set.irl_data_[current_index].attitude.heading = 0.0;

        data_set.irl_data_[current_index].attitude.orientation_id = 0;

        result.IncCorrectionCount();
    }

    result.set_ready( true );
    result.set_passed( true );

    return result;
}*/

//-------------------------------------------------------------------------------------------------
// Corrector_RejectMagSpikes
CorrectionResult Corrector_RejectMagSpikes::CorrectDataset(DataSet &data_set, const MFPVenue &venue)
{
    CorrectionResult result;
    result.set_name( "reject_mag_spikes" );

    const std::size_t data_size = data_set.irl_data_.size();

    if ( ( data_size != 0) &&
         ( data_size == data_set.attitude_data_udf2mfpf_.size() ) &&
         ( data_size == data_set.mag_data_udf_.size() ) &&
         ( data_size == data_set.mag_data_mfpf_.size() ) )
    {
        double module_ref = sqrt(venue.magX * venue.magX + venue.magY * venue.magY + venue.magZ * venue.magZ);

        for ( std::size_t idx = 0; idx < data_size; idx++ )
        {
            if ( data_set.irl_data_[idx].mag_meas.is_valid )
            {
                MagneticData mag_data = data_set.mag_data_mfpf_[idx];
                double module = sqrt( mag_data.mX * mag_data.mX + mag_data.mY * mag_data.mY + mag_data.mZ * mag_data.mZ );

                if ( std::fabs( module_ref - module ) > mag_spike_threshold_ )
                {
                    InvalidateRangeInIrlData( data_set.irl_data_, idx, invalidated_range_size_ );
                    result.IncCorrectionCount();
                }
            }
        }

        result.set_passed(true);
        result.set_ready( true );
    }
    else
    {
        // correction was not provided
        result.reset();
    }

    return result;
}

// mag and attitude data invalidated in both side from specified irl data item
// invalidated_range_size - defines invalidated data items in samples in each direction from specified item
void Corrector_RejectMagSpikes::InvalidateRangeInIrlData( std::vector<TpnOutput> &irl_data, const std::size_t index0, const std::size_t invalidated_range_size )
{
    std::size_t irl_data_size = irl_data.size();
    std::size_t index_end = std::min(index0 + invalidated_range_size, irl_data_size - 1);

    for ( std::size_t idx = index0; idx <= index_end; idx++ )
    {
        assert( idx < irl_data_size );
        irl_data[idx].mag_meas.is_valid = false;
    }

    std::size_t index_start = (index0 > invalidated_range_size) ? (index0 - invalidated_range_size) : 0;

    //for ( std::size_t idx = index0; idx >= index_end; idx-- )
    for (std::size_t idx = index_start; idx <= index0; idx++)
    {
        assert( idx < irl_data_size );
        irl_data[idx].mag_meas.is_valid = false;
    }
}

//-------------------------------------------------------------------------------------------------
// Corrector_SetFixMagBiasCovMatrix
CorrectionResult Corrector_SetFixMagMeasCovMatrix::CorrectDataset(DataSet &data_set, const MFPVenue &venue)
{
    CorrectionResult result;
    result.set_name( "set_fix_mag_meas_cov_matrix" );

    size_t meas_num = data_set.irl_data_.size();

    if (meas_num == 0)
    { // no data for correction
        result.reset();
        return result;
    }

    double mag_meas_covariance_mgaus_2 = mag_meas_sigma_ / k_uTesla_per_mGauss * mag_meas_sigma_ / k_uTesla_per_mGauss;

    for ( auto irl_item = data_set.irl_data_.begin(); irl_item != data_set.irl_data_.end(); irl_item++ )
    {
        irl_item->mag_meas.covarianceMatrix[0][0] = mag_meas_covariance_mgaus_2;
        irl_item->mag_meas.covarianceMatrix[1][1] = mag_meas_covariance_mgaus_2;
        irl_item->mag_meas.covarianceMatrix[2][2] = mag_meas_covariance_mgaus_2;
        irl_item->mag_meas.covarianceMatrix[0][1] = irl_item->mag_meas.covarianceMatrix[0][2] = 0.;
        irl_item->mag_meas.covarianceMatrix[1][0] = irl_item->mag_meas.covarianceMatrix[1][2] = 0.;
        irl_item->mag_meas.covarianceMatrix[2][0] = irl_item->mag_meas.covarianceMatrix[2][1] = 0.;
        result.IncCorrectionCount();
    }

    result.set_ready( true );
    result.set_passed( true );
    return result;
}

//-------------------------------------------------------------------------------------------------
// Corrector_SetMagMeasCovMatrixDependsOnCalibrationLevel
CorrectionResult Corrector_SetMagMeasCovMatrixDependsOnCalibrationLevel::CorrectDataset(DataSet& data_set, const MFPVenue& venue)
{
    CorrectionResult result;
    result.set_name("set_mag_meas_cov_matrix_on_calibration_level");

    size_t meas_num = data_set.mag_data_mfpf_.size();

    if (meas_num == 0)
    { // no data for correction
        result.reset();
        return result;
    }

    for (auto irl_item = data_set.irl_data_.begin(); irl_item != data_set.irl_data_.end(); irl_item++)
    {
        irl_item->mag_meas.covarianceMatrix[0][1] = irl_item->mag_meas.covarianceMatrix[0][2] = 0.;
        irl_item->mag_meas.covarianceMatrix[1][0] = irl_item->mag_meas.covarianceMatrix[1][2] = 0.;
        irl_item->mag_meas.covarianceMatrix[2][0] = irl_item->mag_meas.covarianceMatrix[2][1] = 0.;
    }

    const double bias_uncertainty_calibration_level_0 = 1e7; // mGauss 
    const double bias_uncertainty_calibration_level_1 = 250; // mGauss 
    const double bias_uncertainty_calibration_level_2 = 50;  // mGauss 
    const double bias_uncertainty_calibration_level_3 = 30;  // mGauss 
    const double bias_uncertainty_calibration_level_4 = 10;  // mGauss 
    const double bias_uncertainty_calibration_level_5 = 5;   // mGauss 
    const double bias_uncertainty_calibration_level_6 = 2.5; // mGauss 
    const double bias_uncertainty_calibration_level_7 = 1;   // mGauss 

    for (auto irl_item = data_set.irl_data_.begin(); irl_item != data_set.irl_data_.end(); irl_item++)
    {
        double bias_uncertainty_calibration_cov;
        switch (irl_item->mag_meas.level_of_calibration)
        {
        case 1:
            bias_uncertainty_calibration_cov = bias_uncertainty_calibration_level_1 * bias_uncertainty_calibration_level_1;
            break;
        case 2:
            bias_uncertainty_calibration_cov = bias_uncertainty_calibration_level_2 * bias_uncertainty_calibration_level_2;
            break;
        case 3:
            bias_uncertainty_calibration_cov = bias_uncertainty_calibration_level_3 * bias_uncertainty_calibration_level_3;
            break;
        case 4:
            bias_uncertainty_calibration_cov = bias_uncertainty_calibration_level_4 * bias_uncertainty_calibration_level_4;
            break;
        case 5:
            bias_uncertainty_calibration_cov = bias_uncertainty_calibration_level_5 * bias_uncertainty_calibration_level_5;
            break;
        case 6:
            bias_uncertainty_calibration_cov = bias_uncertainty_calibration_level_6 * bias_uncertainty_calibration_level_6;
            break;
        case 7:
            bias_uncertainty_calibration_cov = bias_uncertainty_calibration_level_7 * bias_uncertainty_calibration_level_7;
            break;
        default:
            bias_uncertainty_calibration_cov = bias_uncertainty_calibration_level_7 * bias_uncertainty_calibration_level_7;
        }
            irl_item->mag_meas.covarianceMatrix[0][0] = bias_uncertainty_calibration_cov;
            irl_item->mag_meas.covarianceMatrix[1][1] = bias_uncertainty_calibration_cov;
            irl_item->mag_meas.covarianceMatrix[2][2] = bias_uncertainty_calibration_cov;
        result.IncCorrectionCount();
    }

    result.set_ready(true);
    result.set_passed(true);
    return result;
}

//-------------------------------------------------------------------------------------------------
// Corrector_SetMagMeasCovMatrixToMaxValues
CorrectionResult Corrector_SetMagMeasCovMatrixToMaxValues::CorrectDataset(DataSet &data_set, const MFPVenue &venue)
{
    CorrectionResult result;
    result.set_name( "set_mag_meas_cov_matrix_to_max" );

    size_t meas_num = data_set.mag_data_mfpf_.size();

    if (meas_num <= N_window_)
    { // no data for correction
        result.reset();
        return result;
    }

    Eigen::MatrixXd mag_data( meas_num, 3 );
    Eigen::MatrixXd smoothed_mag_data( meas_num, 3 );
    Eigen::MatrixXd diff_mag( meas_num, 3 );

    mag_data( 0, 0 ) = data_set.mag_data_mfpf_[0].mX;
    mag_data( 0, 1 ) = data_set.mag_data_mfpf_[0].mY;
    mag_data( 0, 2 ) = data_set.mag_data_mfpf_[0].mZ;

    // filling the matrix 
    for (size_t i = 1; i < meas_num; ++i)
    {
        mag_data( i, 0 ) = data_set.mag_data_mfpf_[i].mX;
        mag_data( i, 1 ) = data_set.mag_data_mfpf_[i].mY;
        mag_data( i, 2 ) = data_set.mag_data_mfpf_[i].mZ;
    }

    double mean_magX = 0;
    double mean_magY = 0;
    double mean_magZ = 0;

    int it = 0;
    int end_idx = meas_num - 1 - N_window_ / 2;

    for (size_t i = 0; i <= N_window_; i++)
    {
        mean_magX = ( 1.0 - ( 1.0 / ( it + 1.0 ) ) ) * mean_magX + 1.0 / ( it + 1.0 ) * mag_data( i, 0 );
        mean_magY = ( 1.0 - ( 1.0 / ( it + 1.0 ) ) ) * mean_magY + 1.0 / ( it + 1.0 ) * mag_data( i, 1 );
        mean_magZ = ( 1.0 - ( 1.0 / ( it + 1.0 ) ) ) * mean_magZ + 1.0 / ( it + 1.0 ) * mag_data( i, 2 );
           
        it++;

        smoothed_mag_data( i, 0 ) = mag_data( i, 0 ); smoothed_mag_data( i, 1 ) = mag_data( i, 1 ); smoothed_mag_data( i, 2 ) = mag_data( i, 2 );

        diff_mag( i, 0 ) = 0; diff_mag( i, 1 ) = 0; diff_mag( i, 2 ) = 0;

        int j = meas_num - 1 - i;

        smoothed_mag_data( j, 0 ) = mag_data( j, 0 ); smoothed_mag_data( j, 1 ) = mag_data( j, 1 ); smoothed_mag_data( j, 2 ) = mag_data( j, 2 );

        diff_mag( j, 0 ) = 0; diff_mag( j, 1 ) = 0; diff_mag( j, 2 ) = 0;
    }

    // now we have mean values calculated for index = N/2

    for ( int i = N_window_ / 2; i < end_idx; ++i )
    {
        smoothed_mag_data( i, 0 ) = mean_magX;
        smoothed_mag_data( i, 1 ) = mean_magY;
        smoothed_mag_data( i, 2 ) = mean_magZ;

        diff_mag( i, 0 ) = mag_data( i, 0 ) - smoothed_mag_data( i, 0 );
        diff_mag( i, 1 ) = mag_data( i, 1 ) - smoothed_mag_data( i, 1 );
        diff_mag( i, 2 ) = mag_data( i, 2 ) - smoothed_mag_data( i, 2 );
        
        int leaving_idx = i + 1 - N_window_ / 2 - 1;
        int new_idx = i + 1 + N_window_ / 2;
        mean_magX = mean_magX + ( mag_data( new_idx, 0 ) - mag_data( leaving_idx, 0 ) ) / ( N_window_ + 1 );
        mean_magY = mean_magY + ( mag_data( new_idx, 1 ) - mag_data( leaving_idx, 1 ) ) / ( N_window_ + 1 );
        mean_magZ = mean_magZ + ( mag_data( new_idx, 2 ) - mag_data( leaving_idx, 2 ) ) / ( N_window_ + 1 );
    }

    Eigen::MatrixXd t_diff_mag( 3, meas_num );
    t_diff_mag = diff_mag.transpose();
    Eigen::Matrix3d m_cov;
    m_cov = t_diff_mag * diff_mag / meas_num;

    double max_sigmaM = std::max( m_cov( 0, 0 ), m_cov( 1, 1 ) );
    max_sigmaM = std::max( max_sigmaM, m_cov( 2, 2 ) );

    double mag_meas_covariance_mgaus_2 = max_sigmaM / k_uTesla_per_mGauss * max_sigmaM / k_uTesla_per_mGauss;

    for ( auto irl_item = data_set.irl_data_.begin(); irl_item != data_set.irl_data_.end(); irl_item++ )
    {
        irl_item->mag_meas.covarianceMatrix[0][0] = mag_meas_covariance_mgaus_2;
        irl_item->mag_meas.covarianceMatrix[1][1] = mag_meas_covariance_mgaus_2;
        irl_item->mag_meas.covarianceMatrix[2][2] = mag_meas_covariance_mgaus_2;
        irl_item->mag_meas.covarianceMatrix[0][1] = irl_item->mag_meas.covarianceMatrix[0][2] = 0.;
        irl_item->mag_meas.covarianceMatrix[1][0] = irl_item->mag_meas.covarianceMatrix[1][2] = 0.;
        irl_item->mag_meas.covarianceMatrix[2][0] = irl_item->mag_meas.covarianceMatrix[2][1] = 0.;
        result.IncCorrectionCount();
    }

    result.set_ready( true );
    result.set_passed( true );
    return result;
}


//-------------------------------------------------------------------------------------------------
// Corrector_RejectPositionsOutsideOfVenue
CorrectionResult_RejectPosition Corrector_RejectPositionsOutsideOfVenue::CorrectDataset(DataSet &data_set, const MFPVenue &venue)
{
    size_t meas_num = data_set.position_data_ff_.size();
    CorrectionResult_RejectPosition result(meas_num);
    result.set_name("reject_outside_position");

    if (meas_num == 0)
    { // no data for correction
        return result;
    }

    for (size_t i = 0; i < meas_num; ++i)
    {
        FfPosition ff_position = data_set.position_data_ff_[i];
        
        if ( ( ff_position.x < 0 ) || ( ff_position.x >= venue.size_x ) || ( ff_position.y < 0 ) ||
            ( ff_position.y >= venue.size_y ) || ( ff_position.floor_number >= venue.floors_count ) || ( ff_position.floor_number < 0 ) )
        {
            data_set.position_data_ff_[i].is_valid = false;
            data_set.irl_data_[i].position.is_valid = false;
            result.IncCorrectionCount();
        }
    }

    result.EstimateValidity();
    return result;
}

//-------------------------------------------------------------------------------------------------
// CorrectionResult_RejectPercentOutput
CorrectionResult_RejectPercentOutput Corrector_RejectMagMeasByPosUncertainty::CorrectDataset(DataSet &data_set, const MFPVenue &venue)
{
    size_t meas_num = data_set.position_data_ff_.size();
    CorrectionResult_RejectPercentOutput result(meas_num);
    result.set_name("reject_mag_by_pos_uncertainty");

    if (meas_num != 0)
    {
        for (size_t i = 0; i < meas_num; ++i)
        {
            FfPosition ff_position = data_set.position_data_ff_[i];
            double pos_unc = std::sqrt(ff_position.covariance_xy[0][0] + ff_position.covariance_xy[1][1]);
            if (pos_unc > max_position_uncertainty_)
            {
                data_set.irl_data_[i].mag_meas.is_valid = false;
                result.IncCorrectionCount();
            }
        }
    }

    result.EstimateValidity();
    return result;
}

//-------------------------------------------------------------------------------------------------
// CorrectionResult_RejectPercentOutput
CorrectionResult_RejectPercentOutput Corrector_RejectDataByPosUncertainty::CorrectDataset(DataSet &data_set, const MFPVenue &venue)
{
    size_t meas_num = data_set.position_data_ff_.size();
    CorrectionResult_RejectPercentOutput result(meas_num);
    result.set_name("reject_data_by_pos_uncertainty");

    if (meas_num != 0)
    { 
        for (size_t i = 0; i < meas_num; ++i)
        {
            FfPosition ff_position = data_set.position_data_ff_[i];
            double pos_unc = std::sqrt(ff_position.covariance_xy[0][0] + ff_position.covariance_xy[1][1]);
            if (pos_unc > max_position_uncertainty_)
            {
                //data_set.irl_data_[i].mag_meas.is_valid = false;
                data_set.irl_data_[i].position.is_valid = false;
                result.IncCorrectionCount();
            }
        }
    }

    result.EstimateValidity();
    return result;
}

//-------------------------------------------------------------------------------------------------
// Corrector_RejectMagDataInStops
CorrectionResult_RejectPercentOutput Corrector_RejectMagDataInStops::CorrectDataset(DataSet &data_set, const MFPVenue &venue)
{
    size_t meas_num = data_set.position_data_ff_.size();
    CorrectionResult_RejectPercentOutput result(meas_num);
    result.set_name("reject_mag_in_stops");

    if (meas_num != 0)
    {
        for (size_t i = 0; i < meas_num; ++i)
        {
            //MagneticData mag_data_mfpf = data_set.mag_data_mfpf_[i];

            if ( (data_set.irl_data_[i].position.mode_of_transit != kElevator) && (data_set.irl_data_[i].position.mode_of_transit != kEscalatorStanding) )
            {
                if (data_set.irl_data_[i].pdr.is_valid && (data_set.irl_data_[i].pdr.stride_length <= 0.))
                {
                    data_set.irl_data_[i].mag_meas.is_valid = false;
                    result.IncCorrectionCount();
                }
            }
        }
    }

    result.EstimateValidity();
    return result;
}

//-------------------------------------------------------------------------------------------------
// Corrector_InterpolationInElevator
CorrectionResult_RejectPercentOutput Corrector_InterpolationInElevator::CorrectDataset(DataSet &data_set, const MFPVenue &venue)
{
    size_t meas_num = data_set.irl_data_.size();
    CorrectionResult_RejectPercentOutput result(meas_num);
    result.set_name("interpolate_floor_numbers_in_elevator");

    if (meas_num > 0)
    {

        std::vector<TpnOutput>::iterator it = std::find_if(data_set.irl_data_.begin(), data_set.irl_data_.end(), [](TpnOutput irl_package) { return ((int)irl_package.position.mode_of_transit > kWalking); });

        if (it != data_set.irl_data_.end())
        {
            //std::cout << std::endl << "mode_of_transit = " << (int)it->position.mode_of_transit << std::endl;
            if ((int)it->position.mode_of_transit == kElevator
                //||
                //data_set.irl_data_[0].position.mode_of_transit == kEscalatorStanding 
                //||
                //data_set.irl_data_[0].position.mode_of_transit == kEscalatorWalking
                )
                // interpolate the floor numbers in elevator 
            {
                int16_t floor_start = data_set.irl_data_[0].position.floor;
                int16_t floor_finish = data_set.irl_data_[meas_num - 1].position.floor;
                if (floor_start != floor_finish)
                {
                    int16_t floor_value = std::abs(floor_start - floor_finish) + 1;
                    int16_t floor_change = ((floor_start - floor_finish) < 0 ? 1 : -1);
                    int16_t meas_value_for_floor = (int16_t)floor((double)meas_num / floor_value + 0.5);
                    int16_t current_floor = floor_start;
                    int16_t floor_counter = 1;
                    for (size_t i = 0; i < meas_num; ++i)
                    {
                        if (i >= meas_value_for_floor * floor_counter)
                        {
                            current_floor += floor_change;
                            floor_counter++;
                        }

                        data_set.irl_data_[i].position.floor = current_floor;
                    }
                }
            }
        }
    }

    result.set_ready(true);
    result.set_passed(true);
    return result;
}

//-------------------------------------------------------------------------------------------------
// Corrector_RejectTrackByHighSpeed
CorrectionResult_RejectPosition Corrector_RejectTrackByHighSpeed::CorrectDataset(DataSet &data_set, const MFPVenue &venue)
{
    size_t meas_num = data_set.irl_data_.size();
    CorrectionResult_RejectPosition  result(meas_num);
    result.set_name("reject_track_by_high_speed");

    const int64_t skipped_start_interval = 20000; // ms
    
    if (data_set.position_data_ff_.size() > 0)
    {

        bool track_validity = true;
        for (size_t i = 0; i < meas_num - 1; ++i)
        {
            bool navigation_phase = (data_set.position_data_ff_[i + 1].navigation_phase > 0) && (data_set.position_data_ff_[i].navigation_phase > 0);

            if (data_set.position_data_ff_[i].is_valid && data_set.position_data_ff_[i + 1].is_valid && navigation_phase)
            {
                double dx = (data_set.position_data_ff_[i + 1].x - data_set.position_data_ff_[i].x) / (data_set.position_data_ff_[i + 1].timestamp - data_set.position_data_ff_[i].timestamp) * 1000;
                double dy = (data_set.position_data_ff_[i + 1].y - data_set.position_data_ff_[i].y) / (data_set.position_data_ff_[i + 1].timestamp - data_set.position_data_ff_[i].timestamp) * 1000;
                double speed = std::sqrt(dx * dx + dy * dy);

                if ((speed > max_speed_) && (data_set.position_data_ff_[i].timestamp > skipped_start_interval))
                {
                    result.SetCorrectionCount(meas_num);
                    track_validity = false;
                    break;
                }
            }
        }
        if(!track_validity)
            for (size_t i = 0; i < meas_num; ++i)
            {
                data_set.irl_data_[i].mag_meas.is_valid = false;
                data_set.irl_data_[i].position.is_valid = false;
            }
    }

    result.EstimateValidity();
    return result;
}

//-------------------------------------------------------------------------------------------------
// Corrector_RejectTrackByVeryManyInvalidPositions
CorrectionResult_RejectPercentOutput Corrector_RejectTrackByMultipleSpeedErrors::CorrectDataset(DataSet &data_set, const MFPVenue &venue)
{
    size_t meas_num = data_set.irl_data_.size();
    CorrectionResult_RejectPercentOutput result(meas_num, threshold_);
    result.set_name("reject_track_by_multiple_speed_errors");

    std::vector <double> median_filter;
    const int median_filter_length = 3;

    size_t valid_size = 0;

    if (data_set.position_data_ff_.size() > 0)
    {
        for (size_t i = 0; i < meas_num - 1; ++i)
        {
            bool navigation_phase = (data_set.position_data_ff_[i + 1].navigation_phase > 0) && (data_set.position_data_ff_[i].navigation_phase > 0);

            bool mode_of_portal_transit = (
                (data_set.position_data_ff_[i].mode_of_transit == kElevator) ||
                (data_set.position_data_ff_[i].mode_of_transit == kStairs) ||
                (data_set.position_data_ff_[i].mode_of_transit == kEscalatorWalking) ||
                (data_set.position_data_ff_[i].mode_of_transit == kEscalatorStanding));

            if (mode_of_portal_transit)
                navigation_phase = true;

            if (data_set.position_data_ff_[i].is_valid && data_set.position_data_ff_[i + 1].is_valid && navigation_phase)
            {
                valid_size++;
                if (mode_of_portal_transit)
                    continue;

                double dx = (data_set.position_data_ff_[i + 1].x - data_set.position_data_ff_[i].x) / (data_set.position_data_ff_[i + 1].timestamp - data_set.position_data_ff_[i].timestamp) * 1000;
                double dy = (data_set.position_data_ff_[i + 1].y - data_set.position_data_ff_[i].y) / (data_set.position_data_ff_[i + 1].timestamp - data_set.position_data_ff_[i].timestamp) * 1000;
                double speed = std::sqrt(dx * dx + dy * dy);

                if (median_filter.size() < median_filter_length)
                    median_filter.push_back(speed);
                if (median_filter.size() == median_filter_length)
                {
                    std::vector <double> tmp = median_filter;
                    std::sort(tmp.begin(), tmp.end());
                    speed = tmp[median_filter_length / 2];
                    median_filter.erase(median_filter.begin());
                }
                if (((speed < min_speed_) && (speed > 0.01)) || (speed > max_speed_))
                    result.IncCorrectionCount();
            }
        }

        if (valid_size == 0)
            valid_size = meas_num;
        {
            result.set_total_data_size(valid_size);

            result.EstimateValidity();

            if (!result.is_passed())
                for (size_t i = 0; i < meas_num; ++i)
                {
                    //std::cout << "final_valid: " << validity_vector[i] << std::endl;
                    data_set.irl_data_[i].mag_meas.is_valid = false;
                    data_set.irl_data_[i].position.is_valid = false;
                }
        }
    }

    return result;
}

//-------------------------------------------------------------------------------------------------
// Corrector_RejectPositionsBySpeed
CorrectionResult_RejectPercentOutput Corrector_RejectPositionsBySpeed::CorrectDataset(DataSet &data_set, const MFPVenue &venue)
{
    size_t meas_num = data_set.irl_data_.size();
    CorrectionResult_RejectPercentOutput result(meas_num, 1.0);
    result.set_name("reject_positions_by_speed");

    std::vector <double> median_filter;
    const int median_filter_length = 3;

    if (data_set.position_data_ff_.size() > 0)
    {
        std::vector<bool> validity_vector;
        //std::cout << std::endl;

        for (size_t i = 0; i < meas_num; ++i)
        {
            validity_vector.push_back(data_set.position_data_ff_[i].is_valid);
            //std::cout << "init_valid: " << data_set.position_data_ff_[i].is_valid << std::endl;
        }
        
        for (int i = 0; i < meas_num - 1; ++i)
        {
            bool navigation_phase = (data_set.position_data_ff_[i + 1].navigation_phase > 0) && (data_set.position_data_ff_[i].navigation_phase > 0);

            if (data_set.position_data_ff_[i].is_valid && data_set.position_data_ff_[i + 1].is_valid && navigation_phase)
            {
                double dx = (data_set.position_data_ff_[i + 1].x - data_set.position_data_ff_[i].x) / (data_set.position_data_ff_[i + 1].timestamp - data_set.position_data_ff_[i].timestamp) * 1000;
                double dy = (data_set.position_data_ff_[i + 1].y - data_set.position_data_ff_[i].y) / (data_set.position_data_ff_[i + 1].timestamp - data_set.position_data_ff_[i].timestamp) * 1000;
                double speed = std::sqrt(dx * dx + dy * dy);

                //std::cout << "speed , " << data_set.position_data_ff_[i + 1].timestamp << " , " << speed << " , ";

                if (median_filter.size() < median_filter_length)
                    median_filter.push_back(speed);
                if (median_filter.size() == median_filter_length)
                {
                    std::vector <double> tmp = median_filter;
                    std::sort(tmp.begin(), tmp.end());
                    speed = tmp[median_filter_length / 2];
                    median_filter.erase(median_filter.begin());
                }
                //std::cout << speed << std::endl;

                if ( ((speed < min_speed_) && (speed > 0.01)) || speed > max_speed_)
                {
                    // removing validity for areas around suspicious speed
                    int min_idx = std::max(0, i - delta_);
                    int max_idx = std::min(i + delta_, (int)meas_num - 1);

                    for (int j = min_idx; j < max_idx + 1; ++j)
                    {
                        if (validity_vector[j] == true)
                        {
                            result.IncCorrectionCount();
                        }
                        validity_vector[j] = false;
                    }
                }
            }
        }

        for (size_t i = 0; i < meas_num; ++i)
        {
            //std::cout << "final_valid: " << validity_vector[i] << std::endl;
            data_set.irl_data_[i].mag_meas.is_valid &= validity_vector[i];
            data_set.irl_data_[i].position.is_valid &= validity_vector[i];
        }
    }

    result.EstimateValidity();
    return result;
}

//-------------------------------------------------------------------------------------------------
// Corrector_RejectSegmentByHighSpeed
CorrectionResult_RejectPercentOutput Corrector_RejectSegmentByHighSpeed::CorrectDataset(DataSet &data_set, const MFPVenue &venue)
{
    size_t meas_num = data_set.irl_data_.size();
    CorrectionResult_RejectPercentOutput result(meas_num, 1.0);
    result.set_name("reject_segment_by_speed");

    std::vector <double> median_filter;
    const int median_filter_length = 3;

    if (data_set.position_data_ff_.size() > 0)
    {
        std::vector<bool> validity_vector;
        std::vector<size_t> segments; // boundaries of segments
        segments.push_back(0);

        for (size_t i = 0; i < meas_num; ++i)
        {
            validity_vector.push_back(data_set.position_data_ff_[i].is_valid);
        }
        validity_vector.push_back(false);

        double angle = 0;
        double angle_prev = 0;
        double d_angle = 0;
        int    counter = 0;
        for (int i = 0; i < meas_num - 1; ++i)
        {
            bool navigation_phase = (data_set.position_data_ff_[i + 1].navigation_phase > 0) && (data_set.position_data_ff_[i].navigation_phase > 0);

            if (data_set.position_data_ff_[i].is_valid && data_set.position_data_ff_[i + 1].is_valid && navigation_phase)
            {
                double dx = (data_set.position_data_ff_[i + 1].x - data_set.position_data_ff_[i].x);
                double dy = (data_set.position_data_ff_[i + 1].y - data_set.position_data_ff_[i].y);
                if (dx == 0 && dy == 0)
                    continue;
                angle = atan2(dy, dx);
                if (median_filter.size() < median_filter_length)
                    median_filter.push_back(angle);
                if (median_filter.size() == median_filter_length)
                {
                    std::vector <double> tmp = median_filter;
                    std::sort(tmp.begin(), tmp.end());
                    angle = tmp[median_filter_length / 2];
                    median_filter.erase(median_filter.begin());
                }
                d_angle = angle - angle_prev;
                if (counter > 0)
                {
                    if (std::abs(d_angle) > min_angle_for_new_segment_)
                        segments.push_back(i);
                }
                angle_prev = angle;
                counter++;
            }
        }
        segments.push_back(meas_num);
        for (auto it = segments.begin(); it != segments.end()-1; ++it)
        {
            //std::cout << "segment: " << *it << "   " << *(it + 1) << "   " << std::endl;
            int min_idx = *it;
            int max_idx = *(it + 1);

            // exclude first and last segments
            if (0)//(min_idx == 0 || max_idx == meas_num)
            {
                for (int j = min_idx; j < max_idx + 1; ++j)
                {
                    if (validity_vector[j] == true)
                        result.IncCorrectionCount();
                    validity_vector[j] = false;
                }
                continue;
            }

            median_filter.clear();
            int high_speed_for_segment_counter = 0;
            bool invalid_segment = false;
            double max_speed = 0;
            double min_speed = 100000;
            for (int i = min_idx; i < max_idx - 1; ++i)
            {
                bool navigation_phase = (data_set.position_data_ff_[i + 1].navigation_phase > 0) && (data_set.position_data_ff_[i].navigation_phase > 0);

                if (data_set.position_data_ff_[i].is_valid && data_set.position_data_ff_[i + 1].is_valid && navigation_phase)
                {
                    double dx = (data_set.position_data_ff_[i + 1].x - data_set.position_data_ff_[i].x) / (data_set.position_data_ff_[i + 1].timestamp - data_set.position_data_ff_[i].timestamp) * 1000;
                    double dy = (data_set.position_data_ff_[i + 1].y - data_set.position_data_ff_[i].y) / (data_set.position_data_ff_[i + 1].timestamp - data_set.position_data_ff_[i].timestamp) * 1000;
                    double speed = std::sqrt(dx * dx + dy * dy);
                    if (median_filter.size() < median_filter_length)
                        median_filter.push_back(speed);
                    if (median_filter.size() == median_filter_length)
                    {
                        std::vector <double> tmp = median_filter;
                        std::sort(tmp.begin(), tmp.end());
                        speed = tmp[median_filter_length / 2];
                        median_filter.erase(median_filter.begin());
                    }
                    max_speed = std::max(max_speed, speed);
                    min_speed = std::min(min_speed, speed);

                    if (speed > high_speed_for_segment_ /*|| speed < 0.5*/)
                        high_speed_for_segment_counter++;
                    if (high_speed_for_segment_counter >= number_high_speed_)
                    {
                        invalid_segment = true;
                        //break;
                    }
                }
            }
            //std::cout << "min_speed: " << min_speed << "   max_speed: " << max_speed << std::endl;
            if (invalid_segment)
            {
                for (int j = min_idx; j < max_idx + 1; ++j)
                {
                    if (validity_vector[j] == true)
                    {
                        result.IncCorrectionCount();
                    }
                    validity_vector[j] = false;
                }
            }
        }

        for (size_t i = 0; i < meas_num; ++i)
        {
            data_set.irl_data_[i].mag_meas.is_valid &= validity_vector[i];
            //data_set.irl_data_[i].position.is_valid &= validity_vector[i];
            if (data_set.position_data_ff_[i].navigation_phase == 0)
                data_set.irl_data_[i].mag_meas.is_valid = false;
        }
    }

    result.EstimateValidity();
    return result;
}

//-------------------------------------------------------------------------------------------------
// Corrector_DebugRejection
CorrectionResult Corrector_DebugRejection::CorrectDataset(DataSet &data_set, const MFPVenue &venue)
{
    size_t meas_num = data_set.position_data_ff_.size();
    CorrectionResult result;
    result.set_name("dbg_rejection");

    if (meas_num != 0)
    {
        for (size_t i = 0; i < meas_num; ++i)
        {
            FfPosition ff_position = data_set.position_data_ff_[i];
            MagneticData mag_data_mfpf = data_set.mag_data_mfpf_[i];

            /*if ((ff_position.x < 50) && (ff_position.y < 50) && (mag_data_mfpf.mY < -10))
            {
                data_set.irl_data_[i].mag_meas.is_valid = false;
                result.IncCorrectionCount();
            }*/
            if (data_set.irl_data_[i].pdr.is_valid && (data_set.irl_data_[i].pdr.stride_length <= 0.))
            {
                data_set.irl_data_[i].mag_meas.is_valid = false;
                result.IncCorrectionCount();
        }
    }
    }

    result.EstimateValidity();
    return result;
}

//-------------------------------------------------------------------------------------------------
// Corrector_RejectMagShifts
CorrectionResult_RejectPercentOutput Corrector_RejectMagShifts::CorrectDataset(DataSet &data_set, const MFPVenue &venue)
{
    const std::size_t data_size = data_set.irl_data_.size();

    CorrectionResult_RejectPercentOutput result(data_size, 1.0);
    result.set_name( "reject_mag_shifts" );

    double threshold = mag_shift_threshold_ * 20.0; // converting from (uT/50ms) to uT/sec
    
    if ( ( data_size != 0) &&
         ( data_size == data_set.attitude_data_udf2mfpf_.size() ) &&
         ( data_size == data_set.mag_data_udf_.size() ) &&
         ( data_size == data_set.mag_data_mfpf_.size() ) )
    {
        std::vector<uint32_t> mag_shift_indexes;

        for (size_t i = 0; i < data_size - 1; ++i)
        {
            double max_derivative = std::numeric_limits<double>::min();
            if (data_set.irl_data_[i].mag_meas.is_valid && data_set.irl_data_[i + 1].mag_meas.is_valid)
            {
                // calculating mag meas derivative in MFP frame
                double abs_mag_x_derivative = std::abs(data_set.mag_data_mfpf_[i + 1].mX - data_set.mag_data_mfpf_[i].mX) / (data_set.mag_data_mfpf_[i + 1].timestamp - data_set.mag_data_mfpf_[i].timestamp) * 1000;
                double abs_mag_y_derivative = std::abs(data_set.mag_data_mfpf_[i + 1].mY - data_set.mag_data_mfpf_[i].mY) / (data_set.mag_data_mfpf_[i + 1].timestamp - data_set.mag_data_mfpf_[i].timestamp) * 1000;
                double abs_mag_z_derivative = std::abs(data_set.mag_data_mfpf_[i + 1].mZ - data_set.mag_data_mfpf_[i].mZ) / (data_set.mag_data_mfpf_[i + 1].timestamp - data_set.mag_data_mfpf_[i].timestamp) * 1000;

                max_derivative = std::max({ abs_mag_x_derivative, abs_mag_y_derivative, abs_mag_z_derivative });

                if (max_derivative > threshold)
                {
                    mag_shift_indexes.push_back(i);
                }
            }
        }

        std::vector<std::pair<uint32_t, uint32_t>> intervals;

        if (mag_shift_indexes.size() > 0)
        {
            int32_t next_shift_index = 0;
            bool finished = false;

            while (!finished)
            {
                // creating pairs of indexes <start, end> between which mag data is invalidated
                if (next_shift_index + 1 < mag_shift_indexes.size()) // we have at least 2 shifts left
                {
                    std::pair<uint32_t, uint32_t> one_interval = std::make_pair(mag_shift_indexes[next_shift_index], mag_shift_indexes[next_shift_index + 1]);
                    intervals.push_back(one_interval);

                    if (next_shift_index + 2 >= mag_shift_indexes.size())
                    {
                        finished = true;
                    }
                    else
                    {
                        next_shift_index += 2;
                    }
                }

                else if (next_shift_index + 1 == mag_shift_indexes.size()) // just one shift is left
                {
                    std::pair<uint32_t, uint32_t> one_interval = std::make_pair(mag_shift_indexes[next_shift_index], data_size);
                    intervals.push_back(one_interval);
                    finished = true;
                }
            }
        }

        for (std::vector<std::pair<uint32_t, uint32_t>>::const_iterator it = intervals.begin(); it != intervals.end(); ++it)
        {
            // invalidating mag data in intervals
            for (size_t i = it->first; i < it->second; ++i)
            {
                data_set.irl_data_[i].mag_meas.is_valid = false;
                result.IncCorrectionCount();
            }
        }

        result.set_passed( true );
        result.set_ready( true );
    }
    else
    {
        // correction was not provided
        result.reset();
    }

    return result;
}

//-------------------------------------------------------------------------------------------------
// Corrector_InvalidNavFlag
CorrectionResult_RejectPosition Corrector_MinNavFlag::CorrectDataset(DataSet &data_set, const MFPVenue &venue)
{
    const std::size_t data_size = data_set.irl_data_.size();

    CorrectionResult_RejectPosition result(data_size);
    result.set_name("min_nav_flag");

    if ( data_size != 0 )
    {
        for (size_t i = 0; i < data_size - 1; ++i)
        {
            bool mode_of_portal_transit = (
                (data_set.position_data_ff_[i].mode_of_transit == kElevator) ||
                (data_set.position_data_ff_[i].mode_of_transit == kStairs) ||
                (data_set.position_data_ff_[i].mode_of_transit == kEscalatorWalking) ||
                (data_set.position_data_ff_[i].mode_of_transit == kEscalatorStanding));
            if (!mode_of_portal_transit && data_set.irl_data_[i].position.navigation_phase < min_nav_flag_)
            {
                data_set.irl_data_[i].mag_meas.is_valid = false;
                if(invalid_nav_flag_all_data_)
                    data_set.irl_data_[i].position.is_valid = false;
                result.IncCorrectionCount();
            }
        }
    }

    result.set_passed(true);
    result.set_ready(true);
    return result;
}

//-------------------------------------------------------------------------------------------------
// Corrector_UpdateNavPhaseForRoboticData
CorrectionResult Corrector_UpdateNavPhaseForRoboticData::CorrectDataset(DataSet &data_set, const MFPVenue &venue)
{
    const std::size_t data_size = data_set.irl_data_.size();

    CorrectionResult_RejectPosition result(data_size);
    result.set_name("update_nav_phase_for_robotic_data");
    bool ok = true;

    if ( data_size != 0 )
    {
        size_t i, start_index, index;
        for (i = 0; i < data_size; ++i)
        {
            if (data_set.irl_data_[i].position.navigation_phase >= nav_phase_threshold_)
                break;
        }
        start_index = i;
        for (i = start_index; i < data_size; ++i)
        {
            if (data_set.irl_data_[i].position.navigation_phase < nav_phase_threshold_)
            {
                ok = false;
                break;
            }
        }
        if (ok)
        {
            uint64_t start_time = data_set.irl_data_[start_index].timestamp;
            for (i = start_index; i < data_size; ++i)
            {
                if ((data_set.irl_data_[i].timestamp - data_set.irl_data_[start_index].timestamp) >= update_delay_)
                {
                    break;
                }
            }
            index = i;
            for (i = index; i < data_size; ++i)
            {
                if (data_set.irl_data_[i].position.navigation_phase < nav_desired_level_)
                {
                    data_set.irl_data_[i].position.navigation_phase = nav_desired_level_;
                    result.IncCorrectionCount();
                }
                else
                {
                    break; // it is done to stop the flag correction after it reached nav_desired_level_
                }
            }
        }
    }

    result.set_passed(ok);
    result.set_ready(true);
    return result;
}

//-------------------------------------------------------------------------------------------------
// Corrector_InterpolatesHeight
CorrectionResult_RejectPosition Corrector_InterpolatesHeight::CorrectDataset(DataSet &data_set, const MFPVenue &venue)
{
    const std::size_t data_size = data_set.irl_data_.size();

    CorrectionResult_RejectPosition result(data_size);
    result.set_name("interpolate_floor_height");
    int16_t start_floor;
    int16_t finish_floor;
    int16_t current_floor;
    double delta_altitude;

    if (data_size != 0)
    {
        current_floor = start_floor = data_set.irl_data_[0].position.floor;
        finish_floor = data_set.irl_data_[data_size - 1].position.floor;
        if (finish_floor != start_floor)
            //delta_altitude = floor_height_ / ( ((double)data_size) / ((double)finish_floor - start_floor) );
            delta_altitude = floor_height_ * double(finish_floor - start_floor) / double(data_size);
        else
            delta_altitude = 0;
        for (size_t i = 0; i < data_size; ++i)
        {
            if (i > 0)
            {
                if (current_floor > data_set.irl_data_[i].position.floor)
                {
                    current_floor = start_floor = data_set.irl_data_[i].position.floor;
                    data_set.irl_data_[i].position.altitude = data_set.irl_data_[i - 1].position.altitude + floor_height_;
                }
                else
                {
                    if (current_floor < data_set.irl_data_[i].position.floor)
                    {
                        current_floor = start_floor = data_set.irl_data_[i].position.floor;
                        data_set.irl_data_[i].position.altitude = data_set.irl_data_[i - 1].position.altitude - floor_height_;
                    }
                    else
                        data_set.irl_data_[i].position.altitude = data_set.irl_data_[i - 1].position.altitude + delta_altitude;
                }
            }
            else
                data_set.irl_data_[i].position.altitude = 0;

            data_set.position_data_ff_[i].altitude = data_set.irl_data_[i].position.altitude;
            data_set.position_data_ff_[i].altitude_std = data_set.irl_data_[i].position.sigma_altitude = 1;
            //std::cout << "altitude =  = " << data_set.position_data_ff_[i].altitude << std::endl;
            result.IncCorrectionCount();
        }
    }

    result.set_passed(true);
    result.set_ready(true);
    return result;
}

//-------------------------------------------------------------------------------------------------
// Corrector_CalculatesHeight
CorrectionResult_RejectPosition Corrector_CalculatesHeight::CorrectDataset(DataSet &data_set, const MFPVenue &venue)
{
    const std::size_t data_size = data_set.irl_data_.size();

    CorrectionResult_RejectPosition result(data_size);
    result.set_name("calculate_floor_height");
    int16_t start_floor;
    int16_t finish_floor;
    int16_t current_floor;
    double delta_altitude;
    int16_t number_of_floor_transit;

    if (data_size != 0)
    {
        current_floor = start_floor = data_set.irl_data_[0].position.floor;
        finish_floor = data_set.irl_data_[data_size - 1].position.floor;

        number_of_floor_transit = std::abs(start_floor - finish_floor);

        std::vector<double> heights;
        for (size_t i = 0; i < data_size; ++i)
        {
            if (data_set.position_data_ff_[i].altitude_std > 0)
                heights.push_back(data_set.position_data_ff_[i].altitude);
        }
        std::sort(heights.begin(), heights.end());
        double min_altitude = heights[0];
        int baro_number = heights.size();
        for (size_t i = 0; i < data_size; ++i)
        {
            if (data_set.position_data_ff_[i].altitude_std > 0)
            {
                data_set.position_data_ff_[i].altitude -= min_altitude;
                data_set.irl_data_[i].position.altitude = data_set.position_data_ff_[i].altitude;
                result.IncCorrectionCount();
            }
        }
#if 1
        //  start height
        double start_height = 0;
        int k = 1;
        int N = baro_number / 10;
        size_t n1;
        {
            double sum = 0;
            int N1 = 0;
            for (n1 = 0; n1 < k * N; n1++)
            {
                if (data_set.position_data_ff_[n1].altitude_std > 0)
                {
                    sum += data_set.position_data_ff_[n1].altitude;
                    N1++;
                }
            }
            start_height = sum / N1;
            double current_height = 0;
            bool start_moving_flag = false;
            while (!start_moving_flag && ((k + 1) * N < data_size))
            {
                sum = 0;
                N1 = 0;
                for (n1 = k * N; n1 < (k + 1) * N; n1++)
                {
                    if (data_set.position_data_ff_[n1].altitude_std > 0)
                    {
                        sum += data_set.position_data_ff_[n1].altitude;
                        N1++;
                    }
                }
                current_height = sum / N1;
                if (std::abs(start_height - current_height) > 1.0)    // 1 m
                    start_moving_flag = true;
                k++;
            }
        }
        
        //  finish height
        double finish_height = 0;
        size_t n2;
        {
            double sum = 0;
            N = baro_number / 10;
            k = 1;
            int N1 = 0;
            for (n2 = data_size - 1; n2 > data_size - k * N; n2--)
            {
                if (data_set.position_data_ff_[n2].altitude_std > 0)
                {
                    sum += data_set.position_data_ff_[n2].altitude;
                    N1++;
                }
            }
            finish_height = sum / N1;
            double current_height = 0;
            bool finish_moving_flag = false;
            while (!finish_moving_flag && ((data_size - (k + 1) * N) > 0))
            {
                sum = 0;
                N1 = 0;
                for (n2 = data_size - k * N; n2 > data_size - (k + 1) * N; n2--)
                {
                    if (data_set.position_data_ff_[n2].altitude_std > 0)
                    {
                        sum += data_set.position_data_ff_[n2].altitude;
                        N1++;
                    }
                }
                current_height = sum / N1;
                if (std::abs(finish_height - current_height) > 1.0)    // 1 m
                    finish_moving_flag = true;
                k++;
            }
        }

        double all_floors_height = finish_height - start_height;
        double floor_height = std::abs(all_floors_height) / number_of_floor_transit;

        // correct the floors
        
        for (size_t i = 0; i < n1; i++)
        {
            data_set.irl_data_[i].position.floor = start_floor;
            data_set.position_data_ff_[i].floor_number = start_floor;
        }
        
        double cur_height = start_height;
        current_floor = start_floor;
        int16_t delta_floor = finish_floor - start_floor;

        if (start_floor > finish_floor)
        {
            for (size_t i = n1; i < n2; i++)
            {
                if (data_set.position_data_ff_[i].altitude_std > 0)
                {
                    if (data_set.position_data_ff_[i].altitude < (cur_height - floor_height))
                    {
                        current_floor--;
                        cur_height -= floor_height;
                        std::cout << std::endl << data_set.position_data_ff_[i].altitude << "   " << data_set.position_data_ff_[i].floor_number;
                    }
                    data_set.irl_data_[i].position.floor = current_floor;
                    data_set.position_data_ff_[i].floor_number = current_floor;
                }
            }
        }
         
        for (size_t i = n2; i < data_size; i++)
        {
            data_set.irl_data_[i].position.floor = finish_floor;
            data_set.position_data_ff_[i].floor_number = finish_floor;
        }
#endif
    }

    result.set_passed(true);
    result.set_ready(true);
    return result;
}

//-------------------------------------------------------------------------------------------------
// Corrector_RejectPositionInStops
CorrectionResult_RejectPosition Corrector_RejectPositionInStops::CorrectDataset(DataSet &data_set, const MFPVenue &venue)
{
    size_t meas_num = data_set.position_data_ff_.size();
    CorrectionResult_RejectPosition result(meas_num);
    result.set_name("reject_position_in_stop");

    if (meas_num != 0)
    {
        for (size_t i = 0; i < meas_num; ++i)
        {
            if ((data_set.irl_data_[i].position.mode_of_transit != kElevator) && (data_set.irl_data_[i].position.mode_of_transit != kEscalatorStanding))
            {
                if (data_set.irl_data_[i].pdr.is_valid && (data_set.irl_data_[i].pdr.stride_length <= 0.))
                {
                    data_set.irl_data_[i].position.is_valid = false;
                    result.IncCorrectionCount();
                }
            }
        }
    }

    result.EstimateValidity();
    return result;
}

//-------------------------------------------------------------------------------------------------
// Corrector_DataSetScoreForPosition
CorrectionResult Corrector_DataSetScoreForPosition::CorrectDataset(DataSet &data_set, const MFPVenue &venue)
{
    size_t meas_num = data_set.position_data_ff_.size();
    CorrectionResult result;
    result.set_name("data_set_score_for_all");
    result.set_ready(false);
    result.set_passed(true);
    if (meas_num != 0)
    {
        double dataset_score = data_set.dataset_params_.dataset_score_;
        // for correct log printing  only
        for (int i = 0; i < (int)dataset_score; i++)
        //for (int i = 0; i < (int)min_data_set_score_; i++)
            result.IncCorrectionCount();
        
        if (dataset_score < min_data_set_score_)
        {
            for (size_t i = 0; i < meas_num; ++i)
                data_set.irl_data_[i].position.is_valid = false;
            result.set_passed(false);
        }
    }
    result.set_ready(true);
    //result.EstimateValidity();
    return result;
}

//-------------------------------------------------------------------------------------------------
// Corrector_UpdateRouteType
CorrectionResult Corrector_UpdateRouteType::CorrectDataset(DataSet &data_set, const MFPVenue &venue)
{
    size_t meas_num = data_set.position_data_ff_.size();
    CorrectionResult result;
    result.set_name("update_route_type");
    result.set_ready(false);
    result.set_passed(true);

    if (meas_num != 0)
    {
        Fpbl::RouteMode_t route_type = data_set.dataset_params_.route_type_;
        for (size_t i = 0; i < meas_num; ++i)
        {
            switch (route_type)
            {
            case Fpbl::NormalMode:
                data_set.irl_data_[i].position.mode_of_transit = kWalking;
                break;
            case Fpbl::ElevatorMode:
                data_set.irl_data_[i].position.mode_of_transit = kElevator;
                break;
            case Fpbl::EscalatorMode:
                data_set.irl_data_[i].position.mode_of_transit = kEscalatorStanding;
                break;
            case Fpbl::StairsMode:
                data_set.irl_data_[i].position.mode_of_transit = kStairs;
                break;
            case Fpbl::ConveyorBeltMode:
                data_set.irl_data_[i].position.mode_of_transit = kConveyorStanding;
                break;
            case Fpbl::SingleCellMode:
                data_set.irl_data_[i].position.mode_of_transit = kFidgeting; // ??
                break;
            default:
                data_set.irl_data_[i].position.mode_of_transit = kWalking;
            }
			data_set.position_data_ff_[i].mode_of_transit = data_set.irl_data_[i].position.mode_of_transit;
        }
    }
    result.set_ready(true);
    //result.EstimateValidity();
    return result;
}


