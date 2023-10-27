#ifndef TRACK_CHECKERS
#define TRACK_CHECKERS

#include<list>
#include<vector>
#include<fstream>
#include<math.h>
#include<limits> // std::numeric_limits

#include "fpVenue.hpp"
#include "TrackValidator.hpp"

namespace TrackPreProcessing
{
    const double k_default_max_mag_module_threshold = 80.0; // Maximal magnetic vector module in track, uT
    const double k_default_mean_mag_Z_deviation_threshold = 35.0; /// Mean vertical magnetic component deviation in MFP frame, uT
    const double k_default_mean_mag_hor_deviation_threshold = 20.0; /// Mean horizontal magnetic component deviation in MFP frame, uT
    const double k_default_mag_bias_covariance_threshold = 9.0; /// Magnetic bias covariance threshold, uT
    const double k_default_mag_bias_level_threshold = 1; /// Minimal magnetic bias covariance level threshold
    const double k_default_B_threshold = 40.0; /// B-value threshold
    const unsigned int k_default_N_window = 40; /// size of the sliding window for filtering data in B-value calculation
	const double k_default_user_speed_limit = 2.; /// user speed limit, m/sec
    const double k_default_mag_derivative_limit = 800; /// mag derivative limit, uT/sec    
    const double k_default_min_data_set_score_for_magnetic = 60; /// min data set score for magnetic: 0 - 100

    struct DataSet;          /// DataSet class declaration

    //-------------------------------------------------------------------------
    /// Base class of validation/checker results
    class CheckResult: public ValidationResult
    {
        public:
            /// Constructor
            CheckResult() : criterion_(0), threshold_((std::numeric_limits<double>::max)()), ValidationResult("unknown_checker") {};

            /// Copy constructor
            CheckResult(CheckResult & src)
            {
                name_ = src.name_;
                f_ready_ = src.f_ready_;
                f_passed_ = src.f_passed_;
                criterion_ = src.criterion_;
                threshold_ = src.threshold_;
            };

            /// Virtual destructor
            virtual ~CheckResult() {};

            /// Reset validation result
            virtual void reset()
            {
                f_ready_ = false;
                f_passed_ = true; // dataset/track is valid by default
                set_criterion(0);
            }

            /// Set validation threshold
            void set_threshold( double threshold )
            {
                threshold_ = threshold;
            }

            /// Set criterion value and set f_ready to false
            void set_criterion( double criterion )
            {
                criterion_ = criterion;
                f_ready_ = false;
            };

            /// Get criterion value
            double get_criterion()
            {
                return criterion_;
            };

            /// The method prints criterion value and validity status in to stream
            virtual void Print( std::ofstream &fout )
            {
                if ( f_ready_ )
                    fout << separator_.c_str() << criterion_ << separator_.c_str() << (int)f_passed_;
                else
                    fout << separator_.c_str() << "0" << separator_.c_str() << "-1";
            };

            /// The method prints header with information about criterion
            virtual void PrintHeader( std::ofstream &fout )
            {
                fout << separator_.c_str() << name_.c_str() << separator_.c_str() << "check state";
            };

			/// The method reads header info
			virtual void ReadHeader(std::istream &in, std::string &val)
			{
				std::getline(in, val, separator_.c_str()[0]);
				std::string dummy;
				std::getline(in, dummy, separator_.c_str()[0]);
			};

        protected:
            virtual bool EstimateValidity_Method()
            {
                f_ready_ = true;

                if (fabs(criterion_) <= threshold_)
                    f_passed_ = true;
                else
                    f_passed_ = false;

                return f_passed_;
            };

        protected:
            double criterion_;  /// criterion value
            double threshold_;  /// threshold for criterion checking
    };

    //-------------------------------------------------------------------------
    /// Class of validation results, with validity estimation: valid if |criterion| => threshold
    class CheckResult_MinThreshold : public CheckResult
    {
      public:
        /// The method check criterion and estimates validity. Valid if |criterion| => threshold
        virtual bool EstimateValidity_Method()
        {
            f_ready_ = true;

            if (fabs(criterion_) >= threshold_)
                f_passed_ = true;
            else
                f_passed_ = false;

            return f_passed_;
        };
    };

    //-------------------------------------------------------------------------
    /// Base class of magnetic checker
    class MagChecker : public Validator_Base
    {
    public:
        virtual ValidationResult* ValidateDataSet(DataSet &data_set, const MFPVenue &venue);
    protected:
        virtual CheckResult* CheckDatasetAndCreateResult(const DataSet &data_set, const MFPVenue &venue) = 0;
    private:
        void InvalidateIrlMagData(DataSet &data_set);
    };

    //-------------------------------------------------------------------------
    /// Class Implements "maximal magnetic module" check
    class Checker_MaxMagModule : public MagChecker
    {
    public:
        Checker_MaxMagModule() : Checker_MaxMagModule(k_default_max_mag_module_threshold) {};
        Checker_MaxMagModule(const double max_mag_module_threshold) : max_mag_module_threshold_(max_mag_module_threshold) {};
    protected:
        CheckResult* CheckDatasetAndCreateResult(const DataSet &data_set, const MFPVenue &venue)
        {
            CheckResult *result = new (CheckResult);
            if (result)
                *result = this->CheckDataset(data_set, venue);
            return result;
        }
    private:
        CheckResult CheckDataset(const DataSet &data_set, const MFPVenue &venue);
        double max_mag_module_threshold_; // Maximal avaliable magnetic vector module in track, uT
    };

    //-------------------------------------------------------------------------
    /// Class Implements "Mean mean mag Z in mfp frame " check
    class Checker_MeanMagZ : public MagChecker
    {
    public:
        Checker_MeanMagZ() : Checker_MeanMagZ(k_default_mean_mag_Z_deviation_threshold) {};
        Checker_MeanMagZ(const double mean_mag_Z_deviation_threshold) : mean_mag_Z_deviation_threshold_(mean_mag_Z_deviation_threshold) {};
    protected:
        CheckResult* CheckDatasetAndCreateResult(const DataSet &data_set, const MFPVenue &venue)
        {
            CheckResult *result = new (CheckResult);
            if (result)
                *result = this->CheckDataset(data_set, venue);
            return result;
        }
    private:
        CheckResult CheckDataset(const DataSet &data_set, const MFPVenue &venue);
        double mean_mag_Z_deviation_threshold_; /// Mean vertical magnetic component deviation in MFP frame, uT
    };

    //-------------------------------------------------------------------------
    /// Class Implements "Mean mean mag plane projection in mfp frame " check
    class Checker_MeanMagHor : public MagChecker
    {
    public:
        Checker_MeanMagHor() : Checker_MeanMagHor(k_default_mean_mag_hor_deviation_threshold) {};
        Checker_MeanMagHor(const double mean_mag_hor_deviation_threshold) : mean_mag_hor_deviation_threshold_(mean_mag_hor_deviation_threshold) {};
    protected:
        CheckResult* CheckDatasetAndCreateResult(const DataSet &data_set, const MFPVenue &venue)
        {
            CheckResult *result = new (CheckResult);
            if (result)
                *result = this->CheckDataset(data_set, venue);
            return result;
        }
    private:
        CheckResult CheckDataset(const DataSet &data_set, const MFPVenue &venue);
        double mean_mag_hor_deviation_threshold_; /// Mean horizontal magnetic component deviation in MFP frame, uT
    };

    //-------------------------------------------------------------------------
    /// Class implements "Max magnetic bias covariation" check
    class Checker_MagBiasCovariation : public MagChecker
    {
    public:
        Checker_MagBiasCovariation() : Checker_MagBiasCovariation(k_default_mag_bias_covariance_threshold) {}
        Checker_MagBiasCovariation(const double mag_bias_covariance_threshold) : mag_bias_covariance_threshold_(mag_bias_covariance_threshold) {};
    protected:
        CheckResult* CheckDatasetAndCreateResult(const DataSet &data_set, const MFPVenue &venue)
        {
            CheckResult *result = new (CheckResult);
            if (result)
                *result = this->CheckDataset(data_set, venue);
            return result;
        }
    private:
        CheckResult CheckDataset(const DataSet &data_set, const MFPVenue &venue);
        double mag_bias_covariance_threshold_; /// Magnetic bias covariance threshold, uT
    };

    //-------------------------------------------------------------------------
    /// Class implements "Max magnetic bias level" check
    class Checker_MagBiasLevel : public MagChecker
    {
    public:
        Checker_MagBiasLevel() : Checker_MagBiasLevel(k_default_mag_bias_level_threshold) {};
        Checker_MagBiasLevel(const double mag_bias_level_threshold) : mag_bias_level_threshold_(mag_bias_level_threshold) {};
    protected:
        virtual CheckResult* CheckDatasetAndCreateResult(const DataSet &data_set, const MFPVenue &venue)
        {
            CheckResult_MinThreshold *result = new (CheckResult_MinThreshold);
            if (result)
                *result = CheckDataset(data_set, venue);
            return result;
        }
    private:
        CheckResult_MinThreshold CheckDataset(const DataSet &data_set, const MFPVenue &venue);
        double mag_bias_level_threshold_; /// Minimal magnetic bias covariance level threshold
    };

    //-------------------------------------------------------------------------
    /// Class implements "B-value criterion" check
    class Checker_B_value : public MagChecker
    {
    public:
        Checker_B_value() : Checker_B_value(k_default_N_window, k_default_B_threshold) {};
        Checker_B_value(const unsigned int N_window, const double B_threshold) : N_window_(N_window), B_threshold_(B_threshold) {};
    protected:
        CheckResult* CheckDatasetAndCreateResult(const DataSet &data_set, const MFPVenue &venue)
        {
            CheckResult *result = new (CheckResult);
            if (result)
                *result = this->CheckDataset(data_set, venue);
            return result;
        }
    private:
        CheckResult CheckDataset(const DataSet &data_set, const MFPVenue &venue);
        unsigned int N_window_; /// size of the sliding window for filtering data in B-value calculation
        double B_threshold_;    /// B-value threshold
    };

	//-------------------------------------------------------------------------
	
	/// Class implements user max speed checker
	class Checker_MaxUserSpeed : public MagChecker
	{
	public:
		Checker_MaxUserSpeed() : Checker_MaxUserSpeed(k_default_user_speed_limit) {};
		Checker_MaxUserSpeed(const double user_speed_limit) : user_speed_limit_(user_speed_limit) {};
	protected:
		CheckResult* CheckDatasetAndCreateResult(const DataSet &data_set, const MFPVenue &venue)
		{
			CheckResult *result = new (CheckResult);
			if (result)
				*result = this->CheckDataset(data_set, venue);
			return result;
		}
	private:
		CheckResult CheckDataset(const DataSet &data_set, const MFPVenue &venue);
		double user_speed_limit_;    /// user speed limit
	};

	//-------------------------------------------------------------------------
	
	/// Class implements user max speed checker
	class Checker_MaxMagDataDerivative : public MagChecker
	{
	public:
		Checker_MaxMagDataDerivative() : Checker_MaxMagDataDerivative(k_default_mag_derivative_limit) {};
		Checker_MaxMagDataDerivative(const double mag_derivative_limit) : mag_derivative_limit_(mag_derivative_limit) {};
	protected:
		CheckResult* CheckDatasetAndCreateResult(const DataSet &data_set, const MFPVenue &venue)
		{
			CheckResult *result = new (CheckResult);
			if (result)
				*result = this->CheckDataset(data_set, venue);
			return result;
		}
	private:
		CheckResult CheckDataset(const DataSet &data_set, const MFPVenue &venue);
		double mag_derivative_limit_;    /// mag derivative limit
	};

    //-------------------------------------------------------------------------

    /// Class implements min data set score checker
    class Checker_DataSetScoreForPosMag : public MagChecker
    {
    public:
        Checker_DataSetScoreForPosMag() : Checker_DataSetScoreForPosMag(k_default_min_data_set_score_for_magnetic) {};
        Checker_DataSetScoreForPosMag(const double min_data_set_score) : min_data_set_score_(min_data_set_score) {};
    protected:
        CheckResult* CheckDatasetAndCreateResult(const DataSet &data_set, const MFPVenue &venue)
        {
            CheckResult *result = new (CheckResult);
            if (result)
                *result = this->CheckDataset(data_set, venue);
            return result;
        }
    private:
        CheckResult CheckDataset(const DataSet &data_set, const MFPVenue &venue);
        double min_data_set_score_;
    };

    //-------------------------------------------------------------------------

    /// Class implements invalid mag percent checker
    class Checker_MagDataPercent : public MagChecker
    {
    public:
        Checker_MagDataPercent() : min_percent_(1.00001) {};
    protected:
        CheckResult* CheckDatasetAndCreateResult(const DataSet &data_set, const MFPVenue &venue)
        {
            CheckResult *result = new (CheckResult);
            if (result)
                *result = this->CheckDataset(data_set, venue);
            return result;
        }
    private:
        CheckResult CheckDataset(const DataSet &data_set, const MFPVenue &venue);
        double min_percent_;
    };

    //-------------------------------------------------------------------------

    /// Class implements invalid position percent checker
    class Checker_PositionPercent : public MagChecker
    {
    public:
        Checker_PositionPercent() : min_percent_(1.00001) {};
    protected:
        CheckResult* CheckDatasetAndCreateResult(const DataSet &data_set, const MFPVenue &venue)
        {
            CheckResult *result = new (CheckResult);
            if (result)
                *result = this->CheckDataset(data_set, venue);
            return result;
        }
    private:
        CheckResult CheckDataset(const DataSet &data_set, const MFPVenue &venue);
        double min_percent_;
    };

	/// Class implements invalid position percent checker
    class Checker_DatasetTypeInformer : public MagChecker
    {
    public:
        Checker_DatasetTypeInformer() {};
    protected:
        CheckResult* CheckDatasetAndCreateResult(const DataSet &data_set, const MFPVenue &venue)
        {
            CheckResult *result = new (CheckResult);
            if (result)
                *result = this->CheckDataset(data_set, venue);
            return result;
        }
    private:
        CheckResult CheckDataset(const DataSet &data_set, const MFPVenue &venue);
    };
       
}

#endif
