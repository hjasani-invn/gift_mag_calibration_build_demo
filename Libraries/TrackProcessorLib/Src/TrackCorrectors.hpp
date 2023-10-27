#ifndef TRACK_CORRECTORS
#define TRACK_CORRECTORS

#include<list>
#include<vector>
#include<fstream>
#include<math.h>
#include<iomanip>
#include<ios>

#include "TpnData.hpp"
#include "fpVenue.hpp"

#include "TrackValidator.hpp"

namespace TrackPreProcessing
{
    // Corrector_FilterMagData  default data
    const unsigned int k_default_filter_window_size = 40;

    // Corrector_RejectMagSpikes default data
    const double k_default_mag_spike_threshold = 30.; /// default mag spike threshold, uT
    const std::size_t k_default_invalidated_range_size = 200; /// default invalidate range size, samples;

    // Corrector_SetFixMagBiasCovMatrix default data
    const double k_default_mag_bias_sigma = 10.; /// default mag bias sigma, uT
    const double k_uTesla_per_mGauss = 0.1;     /// uTesla per mGauss, scale factor

    // Corrector_RejectMagMeasByPosUncertainty
    const double k_default_max_position_uncertainty_for_mag = 4.; /// default max avaliable position uncertanty for validity of mag data
    // Corrector_RejectDataByPosUncertainty
    const double k_default_max_position_uncertainty_for_all = 4.; /// default max avaliable position uncertanty for validity of mag data

    // Corrector_RejectPositionsBySpeed
    const double k_default_min_speed = 0.0; // m/s
    const double k_default_max_speed = 2.0; // m/s
    const int    k_default_delta = 20;      // positions

    // Corrector_RejectTrackByMultipleSpeedErrors
    const double k_default_threshold = 0.5; // percent / 100

    // Corrector_RejectTrackByHighSpeed
    const double k_default_high_speed = 5; // m/s

    // Corrector_RejectSegmentByHighSpeed
    const double k_default_high_speed_for_segment = 2.0; // m/s
    const double k_default_min_angle_for_new_segment = 0.5; // radian
    const int    k_default_number_high_speed = 1;

    // Corrector RejectMagShifts default data
    const double k_default_mag_shift_threshold = 15.; /// default mag shift threshold, uT/(50 ms)

    //Corrector_InvalidNavFlag
    const int k_default_min_invalid_nav_flag = 0;
    const bool  k_default_invalid_nav_flag_all_data = false;

    /// Class implements using zero nav_phase flag for robot
    const uint8_t k_default_nav_phase_threshold = 0;
    const uint8_t k_default_nav_desired_level = 1;
    const double k_default_update_delay = 2;

    //Corrector_InterpolatesHeight
    const double k_default_floor_height = 5;

    //Corrector_DataSetScoreForPosition
    const double k_default_min_data_set_score_for_all = 60; /// min data set score for position: 0 - 100

    class TrackProcessor;   /// Processor class declaration
    struct DataSet;          /// DataSet class declaration

    //-------------------------------------------------------------------------
    /// Base class of correction results
    class CorrectionResult : public ValidationResult
    {
    public:
        CorrectionResult() : correction_count_(0), ValidationResult("unknown_corrector") {};
        /// Reset validation result
        virtual void reset()
        {
            f_ready_ = false;
            f_passed_ = true; // dataset/track is valid by default
            correction_count_ = 0;
        }
        /// The method prints criterion value and validity status in to stream
        virtual void Print(std::ofstream& fout)
        {
            if (f_ready_)
                fout << separator_.c_str() << (int)correction_count_ << separator_.c_str() << (int)f_passed_;
            else
                fout << separator_.c_str() << separator_.c_str();
        }
        /// The method prints header with information about criterion
        virtual void PrintHeader(std::ofstream& fout)
        {
            fout << separator_.c_str() << name_.c_str() << separator_.c_str() << "correction_state";
        }
        /// The method increments correction count if correction is not finished
        /// @return: current correction_count_ value
        size_t IncCorrectionCount()
        {
            if (!f_ready_) correction_count_++;
            return correction_count_;
        }
        size_t GetCorrectionCount()
        {
            return correction_count_;
        }
        size_t SetCorrectionCount(size_t correction_count)
        {
            correction_count_ = correction_count;
            return correction_count_;
        }

    protected:
        /// The method estimates validity.
        virtual bool EstimateValidity_Method()
        {
            return f_passed_;
        };
    protected:
        size_t correction_count_;   ///< count of provided corrections
    };

    //-------------------------------------------------------------------------
    /// Base class of corrector - implements correction interface
    class Corrector_Base : public Validator_Base
    {
    public:
        virtual ValidationResult* ValidateDataSet(DataSet& data_set, const MFPVenue& venue)
        {
            ValidationResult* result = CorrectDatasetAndCreateResult(data_set, venue);
            return result;
        }
    protected:
        virtual CorrectionResult* CorrectDatasetAndCreateResult(DataSet& data_set, const MFPVenue& venue) = 0;
    };

    //----------------------------------------------------------------------------------------
    /// Class implements correction - magnetic data filtering using sliding window
    class Corrector_FilterMagData : public Corrector_Base
    {
    public:
        Corrector_FilterMagData() : Corrector_FilterMagData(k_default_filter_window_size) {};
        Corrector_FilterMagData(const unsigned int N_window) : N_window_(N_window) {};
    protected:
        virtual CorrectionResult* CorrectDatasetAndCreateResult(DataSet& data_set, const MFPVenue& venue)
        {
            CorrectionResult* result = new (CorrectionResult);
            if (result)
                *result = CorrectDataset(data_set, venue);
            return result;
        }
    private:
        CorrectionResult CorrectDataset(DataSet& data_set, const MFPVenue& venue);
        unsigned int N_window_;
    };

    //--------------------------------------------------------------------------------------------
    /// Class implements rejection of spikes in magnetic data
    class Corrector_RejectMagSpikes : public Corrector_Base
    {
    public:
        Corrector_RejectMagSpikes() : Corrector_RejectMagSpikes(k_default_mag_spike_threshold, k_default_invalidated_range_size) {};
        Corrector_RejectMagSpikes(const double mag_spike_threshold, const std::size_t invalidated_range_size) :
            mag_spike_threshold_(mag_spike_threshold), invalidated_range_size_(invalidated_range_size) {};
    protected:
        virtual CorrectionResult* CorrectDatasetAndCreateResult(DataSet& data_set, const MFPVenue& venue)
        {
            CorrectionResult* result = new (CorrectionResult);
            if (result)
                *result = CorrectDataset(data_set, venue);
            return result;
        }
    private:
        CorrectionResult CorrectDataset(DataSet& data_set, const MFPVenue& venue);
        void InvalidateRangeInIrlData(std::vector<TpnOutput>& irl_data, const std::size_t index0, const std::size_t invalidated_range_size);
        std::size_t invalidated_range_size_;
        double mag_spike_threshold_;
    };

    //--------------------------------------------------------------------------------------------
    /// Class implements correction of mag meassurement covariance matrix: set fix covariance matrix for all mag data
    class Corrector_SetFixMagMeasCovMatrix : public Corrector_Base
    {
    public:
        Corrector_SetFixMagMeasCovMatrix() : Corrector_SetFixMagMeasCovMatrix(k_default_mag_bias_sigma) {};
        Corrector_SetFixMagMeasCovMatrix(const double mag_meas_sigma) : mag_meas_sigma_(mag_meas_sigma) {};
    protected:
        virtual CorrectionResult* CorrectDatasetAndCreateResult(DataSet& data_set, const MFPVenue& venue)
        {
            CorrectionResult* result = new (CorrectionResult);
            if (result)
                *result = CorrectDataset(data_set, venue);
            return result;
        }
    private:
        CorrectionResult CorrectDataset(DataSet& data_set, const MFPVenue& venue);
        double mag_meas_sigma_; /// < default mag bias sigma, uT
    };

    //--------------------------------------------------------------------------------------------
    /// Class implements correction of mag covarianceMatrix of mag meas depends on calibration level
    class Corrector_SetMagMeasCovMatrixDependsOnCalibrationLevel : public Corrector_Base
    {
    public:
        Corrector_SetMagMeasCovMatrixDependsOnCalibrationLevel() {};
    protected:
        virtual CorrectionResult* CorrectDatasetAndCreateResult(DataSet& data_set, const MFPVenue& venue)
        {
            CorrectionResult* result = new (CorrectionResult);
            if (result)
                *result = CorrectDataset(data_set, venue);
            return result;
        }
    private:
        CorrectionResult CorrectDataset(DataSet& data_set, const MFPVenue& venue);
    };

    //--------------------------------------------------------------------------------------------
    /// Class implements correction of mag meassurement covariance matrix: set fix covariance matrix for all mag data
    class Corrector_SetMagMeasCovMatrixToMaxValues : public Corrector_Base
    {
    public:
        Corrector_SetMagMeasCovMatrixToMaxValues() : Corrector_SetMagMeasCovMatrixToMaxValues(k_default_filter_window_size) {};
        Corrector_SetMagMeasCovMatrixToMaxValues(const unsigned int N_window) : N_window_(N_window) {};
    protected:
        virtual CorrectionResult* CorrectDatasetAndCreateResult(DataSet& data_set, const MFPVenue& venue)
        {
            CorrectionResult* result = new (CorrectionResult);
            if (result)
                *result = CorrectDataset(data_set, venue);
            return result;
        }
    private:
        CorrectionResult CorrectDataset(DataSet& data_set, const MFPVenue& venue);
        unsigned int N_window_;
    };

    //----------------------------------------------------------------------------------------
    /// Correction results with modified EstimateValidity_Method
    class CorrectionResult_RejectPosition : public CorrectionResult
    {
    public:
        CorrectionResult_RejectPosition(size_t total_data_size) : total_data_size_(total_data_size) {};
        void set_total_data_size(size_t new_total_data_size)
        {
            total_data_size_ = new_total_data_size;
        }
    protected:
        /// The method estimates validity.
        virtual bool EstimateValidity_Method()
        {
            return ((total_data_size_ > 0) && (total_data_size_ > correction_count_));
        };
    protected:
        size_t total_data_size_;   ///< total count of data items
    };

    //----------------------------------------------------------------------------------------
    /// Class implements correction - rejection position that is not from current venue
    class Corrector_RejectPositionsOutsideOfVenue : public Corrector_Base
    {
    public:
        Corrector_RejectPositionsOutsideOfVenue() {};
    protected:
        virtual CorrectionResult* CorrectDatasetAndCreateResult(DataSet& data_set, const MFPVenue& venue)
        {
            CorrectionResult_RejectPosition* result = new CorrectionResult_RejectPosition(0);
            if (result)
                *result = CorrectDataset(data_set, venue);
            return result;
        }
    private:
        CorrectionResult_RejectPosition CorrectDataset(DataSet& data_set, const MFPVenue& venue);
    };

    //----------------------------------------------------------------------------------------
    /// Correction results with modified EstimateValidity_Method
    class CorrectionResult_RejectPercentOutput : public CorrectionResult_RejectPosition
    {
    public:
        CorrectionResult_RejectPercentOutput(size_t total_data_size) : CorrectionResult_RejectPosition(total_data_size), threshold_(1.0) {}; // WARNING: do not change 1.0 to another value!
                                                                                                                                             // otherwise it can be that dataset will be rejected 
                                                                                                                                             // and Wi-Fi, BLA data will be lost
        CorrectionResult_RejectPercentOutput(size_t total_data_size, double threshold) : CorrectionResult_RejectPosition(total_data_size), threshold_(threshold) {};

        /// The method prints criterion value and validity status in to stream
        virtual void Print(std::ofstream& fout)
        {
            if (f_ready_)
            {
                fout << std::fixed;
                fout << std::setprecision(2);
                if (total_data_size_ == 0)
                    fout << separator_.c_str() << (double)total_data_size_;
                else
                    fout << separator_.c_str() << (double)correction_count_ / total_data_size_;
                //fout << std::defaultfloat;
                fout << separator_.c_str() << (int)f_passed_;
            }
            else
                fout << separator_.c_str() << separator_.c_str();
        }
    protected:
        /// The method estimates validity.
        virtual bool EstimateValidity_Method()
        {
            double correction_ratio = (total_data_size_ > 0) ? ((double)correction_count_ / (double)total_data_size_) : 1.;
            return correction_ratio <= threshold_;
        };
    protected:
        double   threshold_;

    };

    //----------------------------------------------------------------------------------------
    /// Class implements correction - rejection mag meas by position uncertainty
    class Corrector_RejectMagMeasByPosUncertainty : public Corrector_Base
    {
    public:
        Corrector_RejectMagMeasByPosUncertainty() : Corrector_RejectMagMeasByPosUncertainty(k_default_max_position_uncertainty_for_mag) {};
        Corrector_RejectMagMeasByPosUncertainty(const double max_position_uncertainty) : max_position_uncertainty_(max_position_uncertainty) {};
    protected:
        virtual CorrectionResult* CorrectDatasetAndCreateResult(DataSet& data_set, const MFPVenue& venue)
        {
            CorrectionResult_RejectPercentOutput* result = new CorrectionResult_RejectPercentOutput(0);
            if (result)
                *result = CorrectDataset(data_set, venue);
            return result;
        }
    private:
        CorrectionResult_RejectPercentOutput CorrectDataset(DataSet& data_set, const MFPVenue& venue);
        double max_position_uncertainty_; // max avaliable position uncertanty for validity of mag data
    };

    //----------------------------------------------------------------------------------------
    /// Class implements correction - rejection data by position uncertainty
    class Corrector_RejectDataByPosUncertainty : public Corrector_Base
    {
    public:
        Corrector_RejectDataByPosUncertainty() : Corrector_RejectDataByPosUncertainty(k_default_max_position_uncertainty_for_all) {};
        Corrector_RejectDataByPosUncertainty(const double max_position_uncertainty) : max_position_uncertainty_(max_position_uncertainty) {};
    protected:
        virtual CorrectionResult* CorrectDatasetAndCreateResult(DataSet& data_set, const MFPVenue& venue)
        {
            CorrectionResult_RejectPercentOutput* result = new CorrectionResult_RejectPercentOutput(0);
            if (result)
                *result = CorrectDataset(data_set, venue);
            return result;
        }
    private:
        CorrectionResult_RejectPercentOutput CorrectDataset(DataSet& data_set, const MFPVenue& venue);
        double max_position_uncertainty_; // max avaliable position uncertanty for validity of mag data
    };

    //----------------------------------------------------------------------------------------
    /// Class implements correction - rejection mag data during stops
    class Corrector_RejectMagDataInStops : public Corrector_Base
    {
    public:
        Corrector_RejectMagDataInStops() {};
    protected:
        virtual CorrectionResult* CorrectDatasetAndCreateResult(DataSet& data_set, const MFPVenue& venue)
        {
            CorrectionResult_RejectPercentOutput* result = new CorrectionResult_RejectPercentOutput(0);
            if (result)
                *result = CorrectDataset(data_set, venue);
            return result;
        }
    private:
        CorrectionResult_RejectPercentOutput CorrectDataset(DataSet& data_set, const MFPVenue& venue);
    };

    //----------------------------------------------------------------------------------------
    /// Class implements correction - interpolation of the floor numbers in elevator
    class Corrector_InterpolationInElevator : public Corrector_Base
    {
    public:
        Corrector_InterpolationInElevator() {};
    protected:
        virtual CorrectionResult* CorrectDatasetAndCreateResult(DataSet& data_set, const MFPVenue& venue)
        {
            CorrectionResult_RejectPercentOutput* result = new CorrectionResult_RejectPercentOutput(0);
            if (result)
                *result = CorrectDataset(data_set, venue);
            return result;
        }
    private:
        CorrectionResult_RejectPercentOutput CorrectDataset(DataSet& data_set, const MFPVenue& venue);
    };

    //----------------------------------------------------------------------------------------
    /// Class implements correction - rejection of all track if there is segments with very high user speed
    class Corrector_RejectTrackByHighSpeed : public Corrector_Base
    {
    public:
        Corrector_RejectTrackByHighSpeed() : Corrector_RejectTrackByHighSpeed(k_default_high_speed) {};
        Corrector_RejectTrackByHighSpeed(double max_speed) : max_speed_(max_speed) {};

    protected:
        virtual CorrectionResult* CorrectDatasetAndCreateResult(DataSet& data_set, const MFPVenue& venue)
        {
            //CorrectionResult_RejectPercentOutput *result = new CorrectionResult_RejectPercentOutput(0);
            CorrectionResult_RejectPosition* result = new CorrectionResult_RejectPosition(0);
            if (result)
                *result = CorrectDataset(data_set, venue);
            return result;
        }
    private:
        CorrectionResult_RejectPosition CorrectDataset(DataSet& data_set, const MFPVenue& venue);
        double max_speed_;
    };

    //----------------------------------------------------------------------------------------
    /// Class implements correction - rejection of all track if there are too many invalid positions
    class Corrector_RejectTrackByMultipleSpeedErrors : public Corrector_Base
    {
    public:
        Corrector_RejectTrackByMultipleSpeedErrors() : Corrector_RejectTrackByMultipleSpeedErrors(k_default_min_speed, k_default_max_speed, k_default_threshold) {};
        Corrector_RejectTrackByMultipleSpeedErrors(double min_speed, double max_speed, double threshold) : min_speed_(min_speed), max_speed_(max_speed), threshold_(threshold) {};

    protected:
        virtual CorrectionResult* CorrectDatasetAndCreateResult(DataSet& data_set, const MFPVenue& venue)
        {
            CorrectionResult_RejectPercentOutput* result = new CorrectionResult_RejectPercentOutput(0, threshold_);
            if (result)
                *result = CorrectDataset(data_set, venue);
            return result;
        }
    private:
        CorrectionResult_RejectPercentOutput CorrectDataset(DataSet& data_set, const MFPVenue& venue);
        double min_speed_;
        double max_speed_;
        double threshold_;
    };

    //----------------------------------------------------------------------------------------
    /// Class implements correction - rejection of track segments with unrealistic user speed
    class Corrector_RejectPositionsBySpeed : public Corrector_Base
    {
    public:
        Corrector_RejectPositionsBySpeed() : Corrector_RejectPositionsBySpeed(k_default_min_speed, k_default_max_speed, k_default_delta) {};
        Corrector_RejectPositionsBySpeed(double min_speed, double max_speed, int delta) :
            min_speed_(min_speed), max_speed_(max_speed), delta_(delta) {};

    protected:
        virtual CorrectionResult* CorrectDatasetAndCreateResult(DataSet& data_set, const MFPVenue& venue)
        {
            CorrectionResult_RejectPercentOutput* result = new CorrectionResult_RejectPercentOutput(0);
            if (result)
                *result = CorrectDataset(data_set, venue);
            return result;
        }
    private:
        CorrectionResult_RejectPercentOutput CorrectDataset(DataSet& data_set, const MFPVenue& venue);
        double min_speed_;
        double max_speed_;
        int    delta_;
    };

    //----------------------------------------------------------------------------------------
    /// Class implements correction - rejection of straight segment of track with unrealistic user speed
    class Corrector_RejectSegmentByHighSpeed : public Corrector_Base
    {
    public:
        Corrector_RejectSegmentByHighSpeed() : Corrector_RejectSegmentByHighSpeed(k_default_high_speed_for_segment,
            k_default_min_angle_for_new_segment,
            k_default_number_high_speed) {};
        Corrector_RejectSegmentByHighSpeed(double high_speed_for_segment, double min_angle_for_new_segment, int number_high_speed) :
            high_speed_for_segment_(high_speed_for_segment),
            min_angle_for_new_segment_(min_angle_for_new_segment),
            number_high_speed_(number_high_speed) {};

    protected:
        virtual CorrectionResult* CorrectDatasetAndCreateResult(DataSet& data_set, const MFPVenue& venue)
        {
            CorrectionResult_RejectPercentOutput* result = new CorrectionResult_RejectPercentOutput(0);
            if (result)
                *result = CorrectDataset(data_set, venue);
            return result;
        }
    private:
        CorrectionResult_RejectPercentOutput CorrectDataset(DataSet& data_set, const MFPVenue& venue);
        double high_speed_for_segment_;
        double min_angle_for_new_segment_;
        int    number_high_speed_;
    };

    //----------------------------------------------------------------------------------------
    /// Class implements correction - Debug corrector - any debug rejection
    class Corrector_DebugRejection : public Corrector_Base
    {
    public:
        Corrector_DebugRejection() {};
    protected:
        virtual CorrectionResult* CorrectDatasetAndCreateResult(DataSet& data_set, const MFPVenue& venue)
        {
            CorrectionResult* result = new CorrectionResult_RejectPercentOutput(0);
            if (result)
                *result = CorrectDataset(data_set, venue);
            return result;
        }
    private:
        CorrectionResult CorrectDataset(DataSet& data_set, const MFPVenue& venue);
    };

    //--------------------------------------------------------------------------------------------
    /// Class implements rejection of shifts in magnetic data (abrupt change of mag measurements due to phone orientation errors)
    class Corrector_RejectMagShifts : public Corrector_Base
    {
    public:
        Corrector_RejectMagShifts() : Corrector_RejectMagShifts(k_default_mag_shift_threshold) {};
        Corrector_RejectMagShifts(const double mag_shift_threshold) : mag_shift_threshold_(mag_shift_threshold) {};
    protected:
        virtual CorrectionResult* CorrectDatasetAndCreateResult(DataSet& data_set, const MFPVenue& venue)
        {
            CorrectionResult_RejectPosition* result = new CorrectionResult_RejectPercentOutput(0);
            if (result)
                *result = CorrectDataset(data_set, venue);
            return result;
        }
    private:
        CorrectionResult_RejectPercentOutput CorrectDataset(DataSet& data_set, const MFPVenue& venue);
        double mag_shift_threshold_;
    };

    //--------------------------------------------------------------------------------------------
    /// Class implements rejection of navigation data
    class Corrector_MinNavFlag : public Corrector_Base
    {
    public:
        Corrector_MinNavFlag() : Corrector_MinNavFlag(k_default_min_invalid_nav_flag, k_default_invalid_nav_flag_all_data) {};
        Corrector_MinNavFlag(const int min_nav_flag, const bool invalid_nav_flag_all_data) : min_nav_flag_(min_nav_flag), invalid_nav_flag_all_data_(invalid_nav_flag_all_data) {};
    protected:
        virtual CorrectionResult* CorrectDatasetAndCreateResult(DataSet& data_set, const MFPVenue& venue)
        {
            CorrectionResult_RejectPosition* result = new CorrectionResult_RejectPosition(0);
            if (result)
                *result = CorrectDataset(data_set, venue);
            return result;
        }
    private:
        CorrectionResult_RejectPosition CorrectDataset(DataSet& data_set, const MFPVenue& venue);
        int  min_nav_flag_;
        bool invalid_nav_flag_all_data_;
    };

    //--------------------------------------------------------------------------------------------
    /// Class implements using zero nav_phase flag for robot
    class Corrector_UpdateNavPhaseForRoboticData : public Corrector_Base
    {
    public:
        Corrector_UpdateNavPhaseForRoboticData() : Corrector_UpdateNavPhaseForRoboticData(k_default_nav_phase_threshold, k_default_nav_desired_level, k_default_update_delay) {};
        Corrector_UpdateNavPhaseForRoboticData(const int8_t nav_phase_threshold, const int8_t nav_desired_level, const double update_delay) : nav_phase_threshold_(nav_phase_threshold), nav_desired_level_(nav_desired_level), update_delay_(update_delay) {};
    protected:
        virtual CorrectionResult* CorrectDatasetAndCreateResult(DataSet& data_set, const MFPVenue& venue)
        {
            CorrectionResult* result = new CorrectionResult();
            if (result)
                *result = CorrectDataset(data_set, venue);
            return result;
        }
    private:
        CorrectionResult CorrectDataset(DataSet& data_set, const MFPVenue& venue);
        int8_t nav_phase_threshold_;
        int8_t nav_desired_level_;
        double update_delay_;
    };

    //--------------------------------------------------------------------------------------------
    /// Class interpolates height between the floors
    class Corrector_InterpolatesHeight : public Corrector_Base
    {
    public:
        Corrector_InterpolatesHeight() : Corrector_InterpolatesHeight(k_default_floor_height) {};
        Corrector_InterpolatesHeight(const double floor_height) : floor_height_(floor_height) {};
    protected:
        virtual CorrectionResult* CorrectDatasetAndCreateResult(DataSet& data_set, const MFPVenue& venue)
        {
            CorrectionResult_RejectPosition* result = new CorrectionResult_RejectPosition(0);
            if (result)
                *result = CorrectDataset(data_set, venue);
            return result;
        }
    private:
        CorrectionResult_RejectPosition CorrectDataset(DataSet& data_set, const MFPVenue& venue);
        double floor_height_;
    };

    //--------------------------------------------------------------------------------------------
    /// Class calculates height of the floors
    class Corrector_CalculatesHeight : public Corrector_Base
    {
    public:
        Corrector_CalculatesHeight() : Corrector_CalculatesHeight(k_default_floor_height) {};
        Corrector_CalculatesHeight(const double floor_height) : floor_height_(floor_height) {};
    protected:
        virtual CorrectionResult* CorrectDatasetAndCreateResult(DataSet& data_set, const MFPVenue& venue)
        {
            CorrectionResult_RejectPosition* result = new CorrectionResult_RejectPosition(0);
            if (result)
                *result = CorrectDataset(data_set, venue);
            return result;
        }
    private:
        CorrectionResult_RejectPosition CorrectDataset(DataSet& data_set, const MFPVenue& venue);
        double floor_height_;
    };

    //----------------------------------------------------------------------------------------
    /// Class implements correction - rejection ble data during stops
    class Corrector_RejectPositionInStops : public Corrector_Base
    {
    public:
        Corrector_RejectPositionInStops() {};
    protected:
        virtual CorrectionResult* CorrectDatasetAndCreateResult(DataSet& data_set, const MFPVenue& venue)
        {
            CorrectionResult_RejectPosition* result = new CorrectionResult_RejectPosition(0);
            if (result)
                *result = CorrectDataset(data_set, venue);
            return result;
        }
    private:
        CorrectionResult_RejectPosition CorrectDataset(DataSet& data_set, const MFPVenue& venue);
    };

    //----------------------------------------------------------------------------------------
    /// Class implements correction - invalid positions if data set score is small
    class Corrector_DataSetScoreForPosition : public Corrector_Base
    {
    public:
        Corrector_DataSetScoreForPosition() : Corrector_DataSetScoreForPosition(k_default_min_data_set_score_for_all) {};
        Corrector_DataSetScoreForPosition(const double min_data_set_score) : min_data_set_score_(min_data_set_score) {};
    protected:
        virtual CorrectionResult* CorrectDatasetAndCreateResult(DataSet& data_set, const MFPVenue& venue)
        {
            CorrectionResult* result = new CorrectionResult;
            if (result)
                *result = CorrectDataset(data_set, venue);
            return result;
        }
    private:
        CorrectionResult CorrectDataset(DataSet& data_set, const MFPVenue& venue);
        double min_data_set_score_;
    };

    //----------------------------------------------------------------------------------------
    /// Class implements correction - update rout type
    class Corrector_UpdateRouteType : public Corrector_Base
    {
    public:
        Corrector_UpdateRouteType() {};
    protected:
        virtual CorrectionResult* CorrectDatasetAndCreateResult(DataSet& data_set, const MFPVenue& venue)
        {
            CorrectionResult* result = new CorrectionResult;
            if (result)
                *result = CorrectDataset(data_set, venue);
            return result;
        }
    private:
        CorrectionResult CorrectDataset(DataSet& data_set, const MFPVenue& venue);
    };
}

#endif
