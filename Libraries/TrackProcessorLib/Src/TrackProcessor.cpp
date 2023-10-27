
#include "TrackProcessor.hpp"
#include "tpn_converter.hpp"
#include "TrackCheckers.hpp"
#include "TrackCorrectors.hpp"
#include <math.h>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <vector>
#include <eigen/Geometry>

#define WEAK_THRESHOLDS 1

using namespace TrackPreProcessing;

TrackProcessor::TrackProcessor(std::string &default_mag_validators)
{
	set_report_separator(", ");
	set_echo_on(true);
	ClearValidationReport();

	SetDefaultValidators(default_mag_validators);
};

TrackProcessor::~TrackProcessor()
{
	ClearValidationReport();
	ClearValidatorList();
};


void TrackProcessor::SetDefaultValidators(std::string &default_mag_validators)
{

	if (default_mag_validators == "for_irl")
	{
		input_data_type_ = eIrlData;
	}
	else if (default_mag_validators == "for_mapper")
	{
		input_data_type_ = eMapperData;
	}
	else if (default_mag_validators == "for_coursa_survey_tool")
	{
		input_data_type_ = eCoursaSurveyData;
	}
	else if (default_mag_validators == "for_ivl")
	{
		input_data_type_ = eIvlData;
	}
	else if (default_mag_validators == "for_single-cell")
	{
		input_data_type_ = eSingleCellData;
	}
	else if (default_mag_validators == "for_robo_survey")
	{
		input_data_type_ = eRobotic;
	}
	else if (default_mag_validators == "none")
	{
		input_data_type_ = eNone;
	}
	else
	{
		input_data_type_ = eMapperData; // default validator set
	}

	AddValidatorInstance(new Checker_DatasetTypeInformer);

	// default general validators
	AddValidatorInstance(new Checker_MagDataPercent);
	AddValidatorInstance(new Checker_PositionPercent);

	if (input_data_type_ != eNone)
	{
		AddValidatorInstance(new Corrector_DataSetScoreForPosition);
		AddValidatorInstance(new Checker_DataSetScoreForPosMag);
	}

	if (input_data_type_ == eIrlData)
	{// default validator settings for IRL input data

		AddValidatorInstance(new Corrector_UpdateRouteType);
		AddValidatorInstance(new Corrector_RejectPositionsOutsideOfVenue);
		AddValidatorInstance(new Corrector_RejectDataByPosUncertainty(4));
		//AddValidatorInstance(new Corrector_RejectMagMeasByPosUncertainty(2.5));
		AddValidatorInstance(new Checker_MagBiasLevel(2));
		AddValidatorInstance(new Checker_MagBiasCovariation(25));

		// Week and strong thresholds explanation
		// B-value validator:
		//      weak threshold: 100 - 95% data from class 1-5 are utilyzed
		//      strong threshold: 50 - all data from class 4,5 are utilized
		// Mean mag vertical deviation validator:
		//      weak threshold: 10 uT - all data from class 1-5 are utilyzed
		//      strong threshold: 6 uT - all data from class 4,5 are utilized
		//      thresholsds was defined by Lowe's datasets
		// Mean mag horizontal deviation validator:
		//      weak threshold:50 uT - all data from class 1-5 are utilyzed
		//      strong threshold: 30 uT - all data from class 4,5 are utilyzed

#if (WEAK_THRESHOLDS == 1)
		AddValidatorInstance(new Checker_B_value(k_default_N_window, 100));
        AddValidatorInstance(new Checker_MeanMagHor(30));
        AddValidatorInstance(new Checker_MeanMagZ(35));
#else
		AddValidatorInstance(new Checker_B_value(k_default_N_window, 50));
		AddValidatorInstance(new Checker_MeanMagHor(6));
		AddValidatorInstance(new Checker_MeanMagZ(30));
#endif

		// default correctors
		AddValidatorInstance(new Corrector_RejectMagSpikes);
		AddValidatorInstance(new Corrector_SetMagMeasCovMatrixToMaxValues);
		AddValidatorInstance(new Corrector_RejectMagDataInStops);
		//AddValidatorInstance(new Corrector_RejectPositionsBySpeed);
	}
	else if (input_data_type_ == eMapperData)
	{
		AddValidatorInstance(new Corrector_RejectPositionsOutsideOfVenue());
        AddValidatorInstance(new Checker_MeanMagZ(35));
        AddValidatorInstance(new Checker_MeanMagHor(30));

		// default correctors
		AddValidatorInstance(new Corrector_SetFixMagMeasCovMatrix(5));
		AddValidatorInstance(new Corrector_RejectMagDataInStops());
	}
	else if (input_data_type_ == eCoursaSurveyData)
	{
		AddValidatorInstance(new Corrector_UpdateRouteType);
		// default checkers
		AddValidatorInstance(new Corrector_RejectPositionsOutsideOfVenue());
        AddValidatorInstance(new Checker_MeanMagZ(35));
        AddValidatorInstance(new Checker_MeanMagHor(30));
		
		// default correctors
		AddValidatorInstance(new Corrector_RejectDataByPosUncertainty(10));
		AddValidatorInstance(new Corrector_RejectMagMeasByPosUncertainty(4));
		AddValidatorInstance(new Corrector_MinNavFlag((int)1, k_default_invalid_nav_flag_all_data));
		AddValidatorInstance(new Corrector_SetFixMagMeasCovMatrix(5));
        //AddValidatorInstance(new Corrector_RejectMagDataInStops()); // removed in 1.4.8
		AddValidatorInstance(new Corrector_RejectMagShifts());

		AddValidatorInstance(new Corrector_RejectTrackByHighSpeed());
		AddValidatorInstance(new Corrector_RejectTrackByMultipleSpeedErrors());
		AddValidatorInstance(new Corrector_RejectSegmentByHighSpeed());
		//AddValidatorInstance(new Corrector_RejectPositionsBySpeed());

		AddValidatorInstance(new Corrector_InterpolatesHeight());
	}
	else if (input_data_type_ == eIvlData)
	{
		AddValidatorInstance(new Corrector_UpdateRouteType);
		AddValidatorInstance(new Corrector_RejectPositionsOutsideOfVenue);
		AddValidatorInstance(new Corrector_RejectDataByPosUncertainty(4));
		AddValidatorInstance(new Corrector_RejectMagMeasByPosUncertainty(2.5));
		AddValidatorInstance(new Corrector_RejectDataByPosUncertainty);
		AddValidatorInstance(new Checker_MagBiasLevel(4));
		//AddValidatorInstance(new Checker_MagBiasCovariation(25));

		// Week and strong thresholds explanation
		// B-value validator:
		//      weak threshold: 100 - 95% data from class 1-5 are utilyzed
		//      strong threshold: 50 - all data from class 4,5 are utilized
		// Mean mag vertical deviation validator:
		//      weak threshold: 10 uT - all data from class 1-5 are utilyzed
		//      strong threshold: 6 uT - all data from class 4,5 are utilized
		//      thresholsds was defined by Lowe's datasets
		// Mean mag horizontal deviation validator:
		//      weak threshold:50 uT - all data from class 1-5 are utilyzed
		//      strong threshold: 30 uT - all data from class 4,5 are utilyzed

#if (WEAK_THRESHOLDS == 1)
		//AddValidatorInstance(new Checker_B_value(k_default_N_window, 100));
        AddValidatorInstance(new Checker_MeanMagHor(30));
        AddValidatorInstance(new Checker_MeanMagZ(35));
#else
		//AddValidatorInstance(new Checker_B_value(k_default_N_window, 50));
		AddValidatorInstance(new Checker_MeanMagHor(6));
		AddValidatorInstance(new Checker_MeanMagZ(30));
#endif

		// default correctors
		AddValidatorInstance(new Corrector_RejectMagSpikes);
		AddValidatorInstance(new Corrector_SetFixMagMeasCovMatrix(5));
		AddValidatorInstance(new Corrector_RejectMagDataInStops);

		//AddValidatorInstance(new Corrector_RejectTrackByHighSpeed());
		AddValidatorInstance(new Corrector_RejectTrackByMultipleSpeedErrors());
		AddValidatorInstance(new Corrector_RejectPositionsBySpeed());
		//AddValidatorInstance(new Corrector_InterpolatesHeight());
	}
	else if (input_data_type_ == eSingleCellData)
	{
		AddValidatorInstance(new Corrector_UpdateRouteType);
		// default checkers
		AddValidatorInstance(new Corrector_RejectPositionsOutsideOfVenue());

		// default correctors
		AddValidatorInstance(new Corrector_SetFixMagMeasCovMatrix(5));
		AddValidatorInstance(new Corrector_RejectMagShifts());
	}
	else if (input_data_type_ == eRobotic)
	{

		AddValidatorInstance(new Corrector_UpdateRouteType);
		AddValidatorInstance(new Corrector_RejectPositionsOutsideOfVenue());
		AddValidatorInstance(new Corrector_UpdateNavPhaseForRoboticData(0, 1, 2));
		AddValidatorInstance(new Checker_MeanMagZ(35));
		AddValidatorInstance(new Checker_MeanMagHor(30));

		AddValidatorInstance(new Checker_MagBiasLevel(5)); // means mag uncertainty less than 0.5 uT
		// default correctors
		AddValidatorInstance(new Corrector_RejectDataByPosUncertainty(0.5));
		AddValidatorInstance(new Corrector_RejectMagMeasByPosUncertainty(0.5));
		AddValidatorInstance(new Corrector_MinNavFlag((int)1, k_default_invalid_nav_flag_all_data));
		AddValidatorInstance(new Corrector_SetMagMeasCovMatrixDependsOnCalibrationLevel());
		AddValidatorInstance(new Corrector_RejectMagDataInStops());
		AddValidatorInstance(new Corrector_RejectMagShifts());
		AddValidatorInstance(new Corrector_RejectTrackByHighSpeed(2));
		AddValidatorInstance(new Corrector_RejectTrackByMultipleSpeedErrors(2, k_default_max_speed, k_default_threshold));
	}
	else
	{ /*no validators*/
	}
	 
	AddValidatorInstance(new Checker_MagDataPercent);
	AddValidatorInstance(new Checker_PositionPercent);
}

std::string TrackProcessor::GenerateDataSetName(const std::string irl_file_name) const
{
	std::string data_set_name = irl_file_name; // to do: add more sophisticated data name generation

	return data_set_name;
}


void TrackProcessor::set_data_set_name(const std::string  data_set_name)
{
	this->data_set_name_ = data_set_name;
}

std::string TrackProcessor::GetDataSetName() const
{
	return this->data_set_name_;
}

void TrackProcessor::FillDataSet(const std::vector<TpnOutput> irl_data, const MFPVenue venue, DataSet &data_set)
{
	data_set.irl_data_ = irl_data;
	ConvertIrlData(irl_data, venue, data_set.position_data_ff_, data_set.attitude_data_udf2mfpf_, data_set.mag_data_udf_);
	CalculateMfpfMagData(data_set.attitude_data_udf2mfpf_, data_set.mag_data_udf_, data_set.mag_data_mfpf_);
}

void TrackProcessor::set_mag_validators(std::vector<std::string> &mag_validators)
{
	this->mag_validators = mag_validators;

	ClearValidationReport();
	//ClearValidatorList();

	int length = mag_validators.size();
	std::string validator;
	const char delim1 = ':';
	const char delim2 = ',';

	for (auto it_validator = mag_validators.begin(); it_validator != mag_validators.end(); it_validator++)
	{
		validator = *it_validator;
		//std::cout << validator << std::endl;

		std::stringstream buf(validator);
		std::string name = "";
		std::string valuestr = "";
		int number_of_values = 0;
		double values[4] = { 0, 0, 0, 0 };

		std::getline(buf, name, delim1);
		// remove all spaces from string
		name.erase(remove_if(name.begin(), name.end(), isspace), name.end());

		std::getline(buf, valuestr, delim2);
		// remove all spaces from string
		valuestr.erase(remove_if(valuestr.begin(), valuestr.end(), isspace), valuestr.end());
		while (valuestr != "")
		{
			double value;
			// double from string
			value = std::stod(valuestr);
			values[number_of_values] = value;
			number_of_values++;
			valuestr = "";
			std::getline(buf, valuestr, delim2);
			// remove all spaces from string
			valuestr.erase(remove_if(valuestr.begin(), valuestr.end(), isspace), valuestr.end());
		}
		create_mag_valigator(name, number_of_values, values);
	}
}

// create magnetic valigator from name and values
bool TrackProcessor::create_mag_valigator(const std::string &name, const int number_of_values, const double values[])
{
	if (name == "corr_data_set_type_info")
	{
		AddValidatorInstance(new Checker_DatasetTypeInformer);
		return true;
	}
	if (name == "corr_reject_outside_pos")
	{
		AddValidatorInstance(new Corrector_RejectPositionsOutsideOfVenue);
		return true;
	}
	if (name == "corr_reject_mag_by_pos_uncertainty")
	{
		if (number_of_values == 0)
			AddValidatorInstance(new Corrector_RejectMagMeasByPosUncertainty);
		else
			AddValidatorInstance(new Corrector_RejectMagMeasByPosUncertainty(values[0]));
		return true;
	}
	if (name == "corr_reject_data_by_pos_uncertainty")
	{
		if (number_of_values == 0)
			AddValidatorInstance(new Corrector_RejectDataByPosUncertainty);
		else
			AddValidatorInstance(new Corrector_RejectDataByPosUncertainty(values[0]));
		return true;
	}
	if (name == "check_mag_bias_level")
	{
		if (number_of_values == 0)
			AddValidatorInstance(new Checker_MagBiasLevel);
		else
			AddValidatorInstance(new Checker_MagBiasLevel(values[0]));
		return true;
	}
	if (name == "check_mag_bias_cov")
	{
		if (number_of_values == 0)
			AddValidatorInstance(new Checker_MagBiasCovariation);
		else
			AddValidatorInstance(new Checker_MagBiasCovariation(values[0]));
		return true;
	}
	if (name == "check_b_value")
	{
		if (number_of_values == 0)
			AddValidatorInstance(new Checker_B_value);
		else if (number_of_values == 1)
			AddValidatorInstance(new Checker_B_value(k_default_N_window, values[0]));
		else
			AddValidatorInstance(new Checker_B_value(values[0], values[1]));
		return true;
	}
	if (name == "check_mean_mag_hor")
	{
		if (number_of_values == 0)
			AddValidatorInstance(new Checker_MeanMagHor);
		else
			AddValidatorInstance(new Checker_MeanMagHor(values[0]));
		return true;
	}
	if (name == "check_mean_mag_z")
	{
		if (number_of_values == 0)
			AddValidatorInstance(new Checker_MeanMagZ);
		else
			AddValidatorInstance(new Checker_MeanMagZ(values[0]));
		return true;
	}
	if (name == "corr_reject_mag_spikes")
	{
		if (number_of_values == 0)
			AddValidatorInstance(new Corrector_RejectMagSpikes);
		else if (number_of_values == 1)
			AddValidatorInstance(new Corrector_RejectMagSpikes(values[0], k_default_invalidated_range_size));
		else
			AddValidatorInstance(new Corrector_RejectMagSpikes(values[0], values[1]));
		return true;
	}
	if (name == "corr_set_mag_meas_cov_matrix_to_max" || name == "corr_set_max_mag_cov")
	{
		if (number_of_values == 0)
			AddValidatorInstance(new Corrector_SetMagMeasCovMatrixToMaxValues);
		else
			AddValidatorInstance(new Corrector_SetMagMeasCovMatrixToMaxValues(values[0]));
		return true;
	}
	if (name == "corr_reject_mag_in_stops" || name == "corr_reject_data_in_dwell")
	{
		AddValidatorInstance(new Corrector_RejectMagDataInStops);
		return true;
	}
	if (name == "corr_mag_data_filter" || name == "corr_filtration")
	{
		if (number_of_values == 0)
			AddValidatorInstance(new Corrector_FilterMagData);
		else
			AddValidatorInstance(new Corrector_FilterMagData(values[0]));
		return true;
	}
	if (name == "floor_interpolation_for_elevator")
	{
		AddValidatorInstance(new Corrector_InterpolationInElevator());
		return true;
	}

	if (name == "corr_reject_track_by_high_speed")
	{
		switch (number_of_values)
		{
		case 0:
			AddValidatorInstance(new Corrector_RejectTrackByHighSpeed());
			break;
		case 1:
			AddValidatorInstance(new Corrector_RejectTrackByHighSpeed(values[0]));
			break;
		default:
			AddValidatorInstance(new Corrector_RejectTrackByHighSpeed(values[0]));
		}
		return true;
	}

	if (name == "corr_reject_track_by_multiple_speed_errors")
	{
		switch (number_of_values)
		{
		case 0:
			AddValidatorInstance(new Corrector_RejectTrackByMultipleSpeedErrors());
			break;
		case 1:
			AddValidatorInstance(new Corrector_RejectTrackByMultipleSpeedErrors(values[0], k_default_max_speed, k_default_threshold));
			break;
		case 2:
			AddValidatorInstance(new Corrector_RejectTrackByMultipleSpeedErrors(values[0], values[1], k_default_threshold));
			break;
		case 3:
			AddValidatorInstance(new Corrector_RejectTrackByMultipleSpeedErrors(values[0], values[1], values[2]));
			break;
		default:
			AddValidatorInstance(new Corrector_RejectTrackByMultipleSpeedErrors(values[0], values[1], values[2]));
		}
		return true;
	}

	if (name == "corr_reject_positions_by_speed")
	{
		switch (number_of_values)
		{
		case 0:
			AddValidatorInstance(new Corrector_RejectPositionsBySpeed());
			break;
		case 1:
			AddValidatorInstance(new Corrector_RejectPositionsBySpeed(values[0], k_default_max_speed, k_default_delta));
			break;
		case 2:
			AddValidatorInstance(new Corrector_RejectPositionsBySpeed(values[0], values[1], k_default_delta));
			break;
		case 3:
			AddValidatorInstance(new Corrector_RejectPositionsBySpeed(values[0], values[1], (int)values[2]));
			break;
		default:
			AddValidatorInstance(new Corrector_RejectPositionsBySpeed(values[0], values[1], (int)values[2]));
		}
		return true;
	}
	if (name == "corr_reject_segment_by_speed")
	{
		switch (number_of_values)
		{
		case 0:
			AddValidatorInstance(new Corrector_RejectSegmentByHighSpeed());
			break;
		case 1:
			AddValidatorInstance(new Corrector_RejectSegmentByHighSpeed(values[0], k_default_min_angle_for_new_segment, k_default_number_high_speed));
			break;
		case 2:
			AddValidatorInstance(new Corrector_RejectSegmentByHighSpeed(values[0], values[1], k_default_number_high_speed));
			break;
		case 3:
			AddValidatorInstance(new Corrector_RejectSegmentByHighSpeed(values[0], values[1], (int)values[2]));
			break;
		default:
			AddValidatorInstance(new Corrector_RejectSegmentByHighSpeed(values[0], values[1], (int)values[2]));
		}
		return true;
	}

	if (name == "corr_set_fix_mag_meas_cov_matrix")
	{
		if (number_of_values == 0)
			AddValidatorInstance(new Corrector_SetFixMagMeasCovMatrix);
		else
			AddValidatorInstance(new Corrector_SetFixMagMeasCovMatrix(values[0]));
		return true;
	}

	if (name == "check_max_mag_derivative")
	{
		if (number_of_values == 0)
			AddValidatorInstance(new Checker_MaxMagDataDerivative);
		else
			AddValidatorInstance(new Checker_MaxMagDataDerivative(values[0]));
		return true;
	}

	if (name == "corr_reject_mag_shifts")
	{
		if (number_of_values == 0)
			AddValidatorInstance(new Corrector_RejectMagShifts);
		else
			AddValidatorInstance(new Corrector_RejectMagShifts(values[0]));
		return true;
	}

	if (name == "corr_min_nav_flag")
	{
		if (number_of_values == 0)
			AddValidatorInstance(new Corrector_MinNavFlag);
		if (number_of_values == 1)
			AddValidatorInstance(new Corrector_MinNavFlag((int)values[0], k_default_invalid_nav_flag_all_data));
		else
			AddValidatorInstance(new Corrector_MinNavFlag((int)values[0], (bool)values[1]));
		return true;
	}

	if (name == "corr_interpolate_floor_height")
	{
		switch (number_of_values)
		{
		case 0:
			AddValidatorInstance(new Corrector_InterpolatesHeight());
			break;
		case 1:
			AddValidatorInstance(new Corrector_InterpolatesHeight(values[0]));
			break;
		default:
			AddValidatorInstance(new Corrector_InterpolatesHeight(values[0]));
		}
		return true;
	}

	if (name == "corr_calculate_floor_height")
	{
		switch (number_of_values)
		{
		case 0:
			AddValidatorInstance(new Corrector_CalculatesHeight());
			break;
		case 1:
			AddValidatorInstance(new Corrector_CalculatesHeight(values[0]));
			break;
		default:
			AddValidatorInstance(new Corrector_CalculatesHeight(values[0]));
		}
		return true;
	}

	if (name == "corr_reject_position_in_stop")
	{
		AddValidatorInstance(new Corrector_RejectPositionInStops);
		return true;
	}

	if (name == "corr_data_set_score_for_all" || name == "check_data_set_score_for_all")
	{
		if (number_of_values == 0)
			AddValidatorInstance(new Corrector_DataSetScoreForPosition);
		else
			AddValidatorInstance(new Corrector_DataSetScoreForPosition(values[0]));
		return true;
	}

	if (name == "check_data_set_score_for_mag")
	{
		if (number_of_values == 0)
			AddValidatorInstance(new Checker_DataSetScoreForPosMag);
		else
			AddValidatorInstance(new Checker_DataSetScoreForPosMag(values[0]));
		return true;
	}

	if (name == "check_invalid_mag_data_percent")
	{
		AddValidatorInstance(new Checker_MagDataPercent);
		return true;
	}
          
    if (name == "check_invalid_position_percent")
	{
		AddValidatorInstance(new Checker_PositionPercent);
		return true;
	}

	if (name == "check_max_user_speed")
	{
		if (number_of_values == 0)
			AddValidatorInstance(new Checker_MaxUserSpeed);
		else
			AddValidatorInstance(new Checker_MaxUserSpeed(values[0]));
		return true;
	}

	if (name == "check_max_mag_module")
	{
		if (number_of_values == 0)
			AddValidatorInstance(new Checker_MaxMagModule);
		else
			AddValidatorInstance(new Checker_MaxMagModule(values[0]));
		return true;
	}

	if (name == "corr_set_mag_meas_cov_matrix_on_calibration_level")
	{
		AddValidatorInstance(new Corrector_SetMagMeasCovMatrixDependsOnCalibrationLevel);
		return true;
	}
 
	if (name == "corr_update_nav_phase_for_robotic_data")
	{
		switch (number_of_values)
		{
		case 0:
			AddValidatorInstance(new Corrector_UpdateNavPhaseForRoboticData());
			break;
		case 1:
			AddValidatorInstance(new Corrector_UpdateNavPhaseForRoboticData(values[0], k_default_nav_desired_level, k_default_update_delay));
			break;
		case 2:
			AddValidatorInstance(new Corrector_UpdateNavPhaseForRoboticData(values[0], values[1], k_default_update_delay));
			break;
		case 3:
			AddValidatorInstance(new Corrector_UpdateNavPhaseForRoboticData(values[0], values[1], (int)values[2]));
			break;
		default:
			AddValidatorInstance(new Corrector_UpdateNavPhaseForRoboticData(values[0], values[1], (int)values[2]));
		}
		return true;
	}

	if (name == "corr_update_route_type")
	{
		AddValidatorInstance(new Corrector_UpdateRouteType);
		return true;
	}
          
	return false;
}

// data processing members
bool TrackProcessor::ValidateDataSet(std::vector<TpnOutput> &irl_data, const MFPVenue venue, DataSetParams dataset_params, std::ofstream &fout)
{
	std::string dataset_name = dataset_params.dataset_name;
	set_data_set_name(dataset_name);
	//echo( dataset_name + " : " );
	bool all_validators_res = true;

	DataSet data_set;
	FillDataSet(irl_data, venue, data_set);
	data_set.dataset_params_ = dataset_params;

#if DEBUG_OUTPUT__IRL_MAG_AND_ORIENTATION_DATA
	int N = data_set.irl_data_.size();

	for (int i = 0; i < N; ++i)
	{
		if (data_set.irl_data_[i].mag_meas.is_valid)
		{
			fout << std::setprecision(10);

			fout << data_set.position_data_ff_[i].timestamp << " , ";
			fout << data_set.position_data_ff_[i].x << " , " << data_set.position_data_ff_[i].y << " , " << data_set.position_data_ff_[i].floor_number << " , ";
			fout << data_set.mag_data_mfpf_[i].mX << " , " << data_set.mag_data_mfpf_[i].mY << " , " << data_set.mag_data_mfpf_[i].mZ << " , ";
			fout << data_set.irl_data_[i].attitude.roll << " , " << data_set.irl_data_[i].attitude.pitch << " , " << data_set.irl_data_[i].attitude.heading << " , ";
			fout << data_set.mag_data_udf_[i].mX << " , " << data_set.mag_data_udf_[i].mY << " , " << data_set.mag_data_udf_[i].mZ << " , ";
			fout << data_set.attitude_data_udf2mfpf_[i].quaternion[0] << " , " << data_set.attitude_data_udf2mfpf_[i].quaternion[1] << " , ";
			fout << data_set.attitude_data_udf2mfpf_[i].quaternion[2] << " , " << data_set.attitude_data_udf2mfpf_[i].quaternion[3] << " , ";
			fout << (int)data_set.irl_data_[i].attitude.orientation_id << std::endl;
		}
	}

#endif

	ClearValidationReport();
	if (validator_list_.size() > 0)
	{
		echo("mag validation:");

		for (auto validator = validator_list_.begin(); validator != validator_list_.end(); ++validator)
		{
			ValidationResult *result = (*validator)->ValidateDataSet(data_set, venue);
			validation_report_.push_back(result);
			echo(result->is_passed() ? "1" : "0");
			if (!result->is_passed())
				break;
		}

		all_validators_res = GetDataSetValidity();

		// invalidate mag data for wrong tracks
		if (all_validators_res == false)
		{
			for (auto item : data_set.irl_data_)
			{
				item.mag_meas.is_valid = false;
			}
		}

		echo(" - ");
		echo(all_validators_res ? "ok" : "fail");
	}
	else
	{
		echo(" no mag validation");
	}

	// copy corrected irl data
	irl_data = data_set.irl_data_;

	// invalidate all magnetic data data in dataset
	if (all_validators_res == false)
	{
		InvalidateMagData(irl_data);
	}

	return all_validators_res; // mag validation status
}

bool TrackProcessor::InvalidateMagData(std::vector<TpnOutput> &irl_data)
{
	for (auto it = irl_data.begin(); it != irl_data.end(); ++it)
		it->mag_meas.is_valid = false;
	return true;
}


void TrackProcessor::AddValidatorInstance(Validator_Base *validator)
{
	if (validator)
	{
		validator_list_.push_back(validator);
	}
}


void TrackProcessor::ClearValidatorList()
{
	while (validator_list_.size() > 0)
	{
		Validator_Base *ptr_validator = validator_list_.back();
		validator_list_.pop_back();
		delete ptr_validator;
	}
}

void TrackProcessor::ClearValidationReport()
{
	while (validation_report_.size() > 0)
	{
		ValidationResult * ptr_validation_result = validation_report_.back();
		validation_report_.pop_back();
		delete ptr_validation_result;
	}
}

// logging and output members
void TrackProcessor::set_echo_on(const bool echo_on)
{
	echo_on_ = echo_on;
}

void TrackProcessor::echo(std::string echo_string)
{
	if (echo_on_)
	{
		std::cout << echo_string.c_str();
	}
}

void TrackProcessor::PrintValidationReportHeader(std::ofstream &fout)
{
	fout << "data_set_name" << separator_.c_str() << "data_set_validity";

	DataSet data_set; // empty DataSet
	MFPVenue venue;

	for (auto validator = validator_list_.begin(); validator != validator_list_.end(); ++validator)
	{
		ValidationResult *result = (*validator)->ValidateDataSet(data_set, venue);
		result->set_separator(separator_);
		result->PrintHeader(fout);
		delete result;
	}

	fout << std::endl;
}

void TrackProcessor::ReadValidationReportHeader(std::ifstream &fin)
{
	std::string line, col_header;
	DataSet data_set; // dummy DataSet
	Venue venue = { 0 }; // dummy venue

	std::getline(fin, line);
	std::istringstream line_stream(line);

	std::getline(line_stream, col_header, separator_.c_str()[0]);
	assert(col_header == "data_set_name");

	std::getline(line_stream, col_header, separator_.c_str()[0]);
	col_header = col_header.substr(1); // remove white space at the beginning
	assert(col_header == "data_set_validity");

	for (auto validator = validator_list_.begin(); validator != validator_list_.end(); ++validator)
	{
		ValidationResult *result = (*validator)->ValidateDataSet(data_set, venue);
		result->set_separator(separator_);
		result->ReadHeader(line_stream, col_header);
		col_header = col_header.substr(1); // remove white space at the beginning
		assert(col_header == result->get_name());
		delete result;
	}
}

void TrackProcessor::PrintValidationReport(std::ofstream &fout)
{
	fout << data_set_name_.c_str() << separator_.c_str() << (int)GetDataSetValidity();

	for (auto validation_result = validation_report_.begin(); validation_result != validation_report_.end(); ++validation_result)
	{
		(*validation_result)->set_separator(separator_.c_str());
		(*validation_result)->Print(fout);
	}

	fout << std::endl;
}

bool TrackProcessor::ReadValidationReport(std::ifstream &fin)
{
	ClearValidationReport();
	bool success = true;
	DataSet data_set; // dummy DataSet
	Venue venue = { 0 }; // dummy venue
	std::string line, val;

	if (fin.eof())
	{
		success = false;
		return success;
	}

	std::getline(fin, line);
	if (line.empty())
	{
		success = false;
		return success;
	}
	std::istringstream line_stream(line);

	std::getline(line_stream, data_set_name_, separator_.c_str()[0]);

	std::getline(line_stream, val, separator_.c_str()[0]);
	SetDataSetValidity(stoi(val));

	for (auto validator = validator_list_.begin(); validator != validator_list_.end(); ++validator)
	{
		ValidationResult *validation_result = (*validator)->ValidateDataSet(data_set, venue);
		validation_result->set_separator(separator_.c_str());
		validation_result->Read(line_stream);
		validation_report_.push_back(validation_result);
		if (!validation_result->is_passed())   break;
	}

	return success;
}

bool TrackProcessor::GetDataSetValidity()
{
	bool result = true;

	for (auto validation_result = validation_report_.begin(); validation_result != validation_report_.end(); ++validation_result)
	{
		if (result && (*validation_result)->is_ready())
			result = result && (*validation_result)->is_passed();
	}

	return result;
}

void TrackProcessor::SetDataSetValidity(bool val)
{
	data_set_validity_ = val;
}

void TrackProcessor::ConvertIrlData(const std::vector<TpnOutput> irl_data,
	const MFPVenue venue,
	std::vector<FfPosition> &position_data_ff,
	std::vector<MffAttitude> &attitude_data_udf2mfpf,
	std::vector<MagneticMeasurement> &mag_data_udf)
{
	TpnConverter converter(venue);

	for (std::vector<TpnOutput>::const_iterator irl_data_item = irl_data.begin(); irl_data_item != irl_data.end(); ++irl_data_item)
	{
		FfPosition ff_position = converter.ConvertPositionData(irl_data_item->timestamp, irl_data_item->position);
		position_data_ff.push_back(ff_position);
		MffAttitude mff_attitude = converter.ConvertAttitudeData(irl_data_item->timestamp, irl_data_item->attitude);
		attitude_data_udf2mfpf.push_back(mff_attitude);
		MagneticMeasurement udf_mag = converter.ConvertMagneticData(irl_data_item->timestamp, irl_data_item->mag_meas);
		mag_data_udf.push_back(udf_mag);
	}
}

void TrackProcessor::CalculateMfpfMagData(const std::vector<MffAttitude> &mff_attitude_data, const std::vector<MagneticMeasurement> &mag_data_udf, std::vector<MagneticData> &mag_data_mfpf)
{
	// rotating mag data to local frame
	for (uint32_t i = 0; i < mff_attitude_data.size(); i++)
	{
		/*MffAttitude mff_attitude = mff_attitude_data[i];
		MagneticData mag = mag_data_udf[i];

		Eigen::Quaternion<double> q_udf2MFF = Eigen::Quaternion<double>( mff_attitude.quaternion[0], mff_attitude.quaternion[1], mff_attitude.quaternion[2], mff_attitude.quaternion[3] );
		Eigen::Matrix<double, 3, 3> C = Eigen::Matrix<double, 3, 3>::Zero();
		C = q_udf2MFF.matrix();
		Eigen::Matrix<double, 3, 1> mg_vec;
		mg_vec( 0 ) = mag.mX;
		mg_vec( 1 ) = mag.mY;
		mg_vec( 2 ) = mag.mZ;

		mg_vec = C * mg_vec;

		mag.mX = mg_vec( 0 );
		mag.mY = mg_vec( 1 );
		mag.mZ = mg_vec( 2 );
		*/
		//double q_mag_mff = q_udf2mff * q_mag_udf * conj(q_udf2mff);
		Eigen::Quaternion<double> q_mag_mff =
			Eigen::Quaternion<double>(mff_attitude_data[i].quaternion[0], mff_attitude_data[i].quaternion[1], mff_attitude_data[i].quaternion[2], mff_attitude_data[i].quaternion[3]) *
			Eigen::Quaternion<double>(0., mag_data_udf[i].mX, mag_data_udf[i].mY, mag_data_udf[i].mZ) *
			Eigen::Quaternion<double>(mff_attitude_data[i].quaternion[0], mff_attitude_data[i].quaternion[1], mff_attitude_data[i].quaternion[2], mff_attitude_data[i].quaternion[3]).conjugate();

		MagneticData mag = mag_data_udf[i];
		mag.mX = q_mag_mff.x();
		mag.mY = q_mag_mff.y();
		mag.mZ = q_mag_mff.z();

		mag_data_mfpf.push_back(mag);
	}
}
