#ifndef TRACK_PROCESSOR
#define TRACK_PROCESSOR

#include<list>
#include<vector>
#include<fstream>

#include "TpnData.hpp"
#include "Fpbl.hpp"
#include "fpVenue.hpp"
#include "TrackValidator.hpp"
#include "MagData.hpp"
#include "LocalData.hpp"

#define DEBUG_OUTPUT__IRL_MAG_AND_ORIENTATION_DATA          0

namespace TrackPreProcessing
{
    enum DefaultValidatorSet { eIrlData, eMapperData, eCoursaSurveyData, eIvlData, eSingleCellData, eRobotic, eNone };

    struct DataSetParams
    {
        double dataset_score_;                     ///<
        std::string dataset_name;
        Fpbl::DataSetType dataset_type_;
        Fpbl::RouteMode_t route_type_;

        // processing flags
        bool calibrated_mag_data;
    };

    struct DataSet
    {
        std::vector<TpnOutput> irl_data_;                     ///<
        std::vector<MagneticData> mag_data_mfpf_;
        std::vector<FfPosition> position_data_ff_;
        std::vector<MffAttitude> attitude_data_udf2mfpf_;
        std::vector<MagneticMeasurement> mag_data_udf_;
        DataSetParams   dataset_params_;
    };

    struct Fpp_pos_accuracy
    {
        double time;
        double sigma_x;
        double sigma_y;
    };

    //-------------------------------------------------------------------------
    /// Track processor class
    class TrackProcessor
    {
        public:
            TrackProcessor(std::string &default_mag_validators);
            ~TrackProcessor();

            void AddValidatorInstance(Validator_Base * validator); // add instance of checker; object can be deleted by destructor or ClearValidatorList function
            void ClearValidatorList();

            void SetDefaultValidators(std::string &default_mag_validators);
            // data set definition
            std::string GenerateDataSetName( const std::string irl_file_name ) const;
            void set_data_set_name( const std::string  data_set_name );

			std::string GetDataSetName() const;

            void set_mag_validators(std::vector<std::string> &mag_validators);

            // data processing members
            bool ValidateDataSet(std::vector<TpnOutput> &irl_data, const MFPVenue venue, DataSetParams dataset_params, std::ofstream &fout);
            bool GetDataSetValidity();
			void SetDataSetValidity(bool val);
            bool InvalidateMagData(std::vector<TpnOutput> &irl_data);

            // console logging
            void echo( const std::string echo_string );
            void set_echo_on( const bool echo_on );

            // output members
            void PrintValidationReport( std::ofstream &fout );
            void PrintValidationReportHeader( std::ofstream &fout );
			void ReadValidationReportHeader(std::ifstream &fin);
			bool ReadValidationReport(std::ifstream &fin);
            void set_report_separator(std::string separator)     { separator_ = separator; };

    public:
            void ClearValidationReport();

    private:
            // Conversion and transformation members
        void FillDataSet(const std::vector<TpnOutput> irl_data, const MFPVenue venue, DataSet &data_set);
            void ConvertIrlData(const std::vector<TpnOutput> irl_data, const MFPVenue venue, std::vector<FfPosition> &position_data_ff, std::vector<MffAttitude> &attitude_data_udf2mfpf, std::vector<MagneticMeasurement> &mag_data_udf);
            void CalculateMfpfMagData(const std::vector<MffAttitude> &mff_attitude_data, const std::vector<MagneticMeasurement> &mag_data_udf, std::vector<MagneticData> &mag_data_mfpf);
            // create magnetic valigator from name and values
            bool create_mag_valigator(const std::string &name, const int number_of_values, const double values []);

    private:
            std::string data_set_name_;                         ///<  data set name
            bool echo_on_;                                      ///<  console echo status
			bool data_set_validity_;

            std::list<ValidationResult*> validation_report_;    ///<  Validation result list
            std::list<Validator_Base*> validator_list_;          ///<  List of validators
            std::vector<std::string> mag_validators;

            std::string separator_;
    private:
        DefaultValidatorSet input_data_type_;
    };

}

#endif
