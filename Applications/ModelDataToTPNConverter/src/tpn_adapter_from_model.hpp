#ifndef TPN_ADAPTOR_FROM_MODEL_HPP
#define TPN_ADAPTOR_FROM_MODEL_HPP

#include "model_output_data_struct.hpp"
#include "tpn_data_struct.hpp"

const double ms_per_sec = 1000.;

bool tpn_adapter_from_model(const model_output_data &mod_output, tpn_data &data_for_tpn)
{
    data_for_tpn.tpn_entity_time.timetag_ = ((double)mod_output.time_tag) / ms_per_sec;

	data_for_tpn.tpn_entity_position.latitude_ = mod_output.geo_lat;
	data_for_tpn.tpn_entity_position.longitude_ = mod_output.geo_lon;
	data_for_tpn.tpn_entity_position.height_ = mod_output.height;
	
	data_for_tpn.tpn_entity_position_std.position_north_standard_deviation_ = mod_output.sigma_n;
	data_for_tpn.tpn_entity_position_std.position_east_standard_deviation_ = mod_output.sigma_e;
	data_for_tpn.tpn_entity_position_std.height_standard_deviation_ = mod_output.sigma_height;

	data_for_tpn.tpn_entity_attitude.pitch_ = mod_output.pitch;
	data_for_tpn.tpn_entity_attitude.roll_ = mod_output.roll;
	data_for_tpn.tpn_entity_attitude.heading_ = mod_output.heading;
	
	data_for_tpn.tpn_entity_attitude_standard_deviation.pitch_standard_deviation_ = mod_output.pitch_std;
	data_for_tpn.tpn_entity_attitude_standard_deviation.roll_standard_deviation_ = mod_output.roll_std;
	data_for_tpn.tpn_entity_attitude_standard_deviation.heading_standard_deviation_ = mod_output.heading_std;

	data_for_tpn.tpn_entity_magnetometer_data.raw_data_accuracy_flag_ = 0x02;
  data_for_tpn.tpn_entity_magnetometer_data.raw_data_available_ = mod_output.mag_valid;
	data_for_tpn.tpn_entity_magnetometer_data.raw_data_x_ = mod_output.mag_x;
	data_for_tpn.tpn_entity_magnetometer_data.raw_data_y_ = mod_output.mag_y;
	data_for_tpn.tpn_entity_magnetometer_data.raw_data_z_ = mod_output.mag_z;
	
	data_for_tpn.tpn_entity_magnetometer_data.calibrated_data_accuracy_flag_= 0x02;
	data_for_tpn.tpn_entity_magnetometer_data.calibrated_data_available_ = mod_output.mag_valid;
	data_for_tpn.tpn_entity_magnetometer_data.calibrated_data_x_ = mod_output.mag_x;
	data_for_tpn.tpn_entity_magnetometer_data.calibrated_data_y_ = mod_output.mag_y;
	data_for_tpn.tpn_entity_magnetometer_data.calibrated_data_z_ = mod_output.mag_z;
	
	data_for_tpn.tpn_entity_orientation_pitch_d.m_orientation = 0;;
	data_for_tpn.tpn_entity_grp_07.m_nav_phase = mod_output.navigation_phase;

	data_for_tpn.tpn_entity_device_heading.device_heading_available_ = 0x01;
	data_for_tpn.tpn_entity_device_heading.device_heading_ = mod_output.user_heading;
	data_for_tpn.tpn_entity_device_heading.platform_heading_ = 0;
	data_for_tpn.tpn_entity_device_heading.misalignment_angle_ = mod_output.misalignment;


	data_for_tpn.tpn_entity_internal_dbg.m_att_filter_roll = mod_output.roll;
	data_for_tpn.tpn_entity_internal_dbg.m_att_filter_pitch = mod_output.pitch;
	data_for_tpn.tpn_entity_internal_dbg.m_att_filter_heading = mod_output.heading;

	//irlOutput.attitude.roll = e_dbg.m_att_filter_roll;
	//irlOutput.attitude.pitch = e_dbg.m_att_filter_pitch;
	//irlOutput.attitude.heading = e_dbg.m_att_filter_heading;
 
	data_for_tpn.tpn_entity_internal_dbg.m_att_filter_pitch_std = mod_output.pitch_std;
	data_for_tpn.tpn_entity_internal_dbg.m_att_filter_roll_std = mod_output.roll_std;
	data_for_tpn.tpn_entity_internal_dbg.m_att_filter_heading_std = mod_output.heading_std;
 
	//irlOutput.attitude.sigma_roll = e_dbg.m_att_filter_roll_std;
	//irlOutput.attitude.sigma_pitch = e_dbg.m_att_filter_pitch_std;
	//irlOutput.attitude.sigma_heading = e_dbg.m_att_filter_heading_std;

	data_for_tpn.tpn_entity_internal_dbg.m_orientation_on_pitch = mod_output.frame_flag;
  data_for_tpn.tpn_entity_internal_dbg.walking_fidgeting_flag = mod_output.fidgeting_flag;

	//irlOutput.attitude.orientation_id = e_dbg.m_orientation_on_pitch;

	data_for_tpn.tpn_entity_internal_dbg.m_imu_smp_proc_time = 0;
	data_for_tpn.tpn_entity_internal_dbg.m_internal_pitch = 0;
	data_for_tpn.tpn_entity_internal_dbg.m_internal_roll = 0;

	data_for_tpn.tpn_entity_heading_misalignment.heading_misalignment_ = mod_output.misalignment;

	data_for_tpn.tpn_entity_floor_number.floor_number_ = mod_output.floor;

	data_for_tpn.tpn_entity_stride_information.stride_distance_ = mod_output.stride_length;
	data_for_tpn.tpn_entity_stride_information.stride_velocity_ = 0; // UNUSED 

	return true;
}

#endif
