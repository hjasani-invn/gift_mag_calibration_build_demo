#ifndef TRACK_VALIDATOR
#define TRACK_VALIDATOR

#include "fpVenue.hpp"
#include <string>
#include<fstream>

namespace TrackPreProcessing
{
    struct DataSet;          // DataSet struct declaration
    class ValidationResult;  // ValidationResult class declaration

    //-------------------------------------------------------------------------
    /// Base class of validator object- implements validation interface
    class Validator_Base
    {
    public:
        Validator_Base() {};
        ~Validator_Base() {};
        virtual ValidationResult* ValidateDataSet(DataSet &data_set, const MFPVenue &venue) = 0;
    };


    //-------------------------------------------------------------------------
    /// Base class of validation/checker results
    class ValidationResult
    {
    public:
        /// Constructor
        ValidationResult() : ValidationResult("unknown_validator") { };
        ValidationResult(const std::string name) : name_(name), separator_(", ") { reset(); };

        /// Copy constructor
        ValidationResult(ValidationResult & src)
        {
            name_ = src.name_;
            f_ready_ = src.f_ready_;
            f_passed_ = src.f_passed_;
        };

        /// Virtual destructor
        virtual ~ValidationResult() {};

        /// Reset validation result
        virtual void reset()
        {
            f_ready_ = false;
            f_passed_ = true; // dataset/track is valid by default
        }

        /// Set validator name
        void set_name(std::string name)        { name_ = name; };

		/// Get validator name
		std::string get_name() { return name_; };

        /// Set passed status
        void set_passed(bool f_passed)        { f_passed_ = f_passed; };

        /// Set ready status
        void set_ready(bool f_ready)          { f_ready_ = f_ready; };

        /// Set ready status
        void set_separator(std::string separator)          { separator_ = separator; };

        /// Return validation readyness status
        bool is_ready()        { return f_ready_;};

        /// Return validation passed status
        bool is_passed()       { return f_passed_;};

        /// The method prints criterion value and validity status in to stream
        virtual void Print(std::ofstream &fout)
        {
            if (f_ready_)
                fout << separator_.c_str() << (int)f_ready_ << separator_.c_str() << (int)f_passed_;
            else
                fout << separator_.c_str() << separator_.c_str();
        }

		/// The method reads criterion value and validity status from the stream
		virtual void Read(std::istream &in)
		{
			std::string str;

			std::getline(in, str, separator_.c_str()[0]);
			f_ready_ = std::stoi(str);

			std::getline(in, str, separator_.c_str()[0]);
			f_passed_ = std::stoi(str);
		}

        /// The method prints header with information about criterion
        virtual void PrintHeader(std::ofstream &fout)
        {
            fout << separator_.c_str() << name_.c_str() << separator_.c_str() << "validation_state";
        }

		/// The method reads header info
		virtual void ReadHeader(std::istream &in, std::string &val)
		{
			std::getline(in, val, separator_.c_str()[0]);
			std::string dummy;
			std::getline(in, dummy, separator_.c_str()[0]);
		}

        /// The method check criterion and estimates validity. Valid if |criterion| <= threshold
        virtual bool EstimateValidity()
        {
            f_ready_ = true;
            f_passed_ = EstimateValidity_Method();
            return f_passed_;
        }

    protected:
        /// The method estimates validity.
        virtual bool EstimateValidity_Method() = 0;

    protected:
        std::string name_;  /// validation / checking name
        bool f_ready_;      /// readyness status
        bool f_passed_;     /// checking / validation result
        std::string separator_ ; /// output data separator
    };
}


#endif
