#ifndef TPN_DATA_STRUCT_HPP
#define TPN_DATA_STRUCT_HPP

#include "tpn_packet_parser.hpp"

struct tpn_data
{
	//struct tpn_entity_time_t   // see tpn_packet_parser.hpp
	//{
	//	double timetag_;    ///< time tag, sec
	//}
	tpn_entity_time_t tpn_entity_time;
	//int64_t time_tag; // ms

	//struct tpn_entity_position_t // see tpn_packet_parser.hpp
	//{
	//	double latitude_;   ///< Lattitude, deg
	//	double longitude_;  ///< Longitude, deg
	//	float height_;      ///< Heeght, m
	//}
	tpn_entity_position_t tpn_entity_position;
	//double  geo_lat;  // deg
	//double  geo_lon;  // deg
	//double  height;   // m - height from sea level


	//struct tpn_entity_position_std_t  // see tpn_packet_parser.hpp
	//{
	//	float position_north_standard_deviation_;   ///> north position standard deviation, m
	//	float position_east_standard_deviation_;    ///> east position standard deviation, m
	//	float height_standard_deviation_;           ///> heightstandard deviation, m
	//}
	tpn_entity_position_std_t tpn_entity_position_std;
	//double  sigma_n;  // m - uncertainty in North direction
	//double  sigma_e;  // m - uncertainty in East direction
	//double  sigma_height; // m


	//struct tpn_entity_attitude_t		// see tpn_packet_parser.hpp
	//{
	//	float roll_;      ///< Device roll, deg
	//	float pitch_;     ///< Device pitch, deg
	//	float heading_;   ///< Device heading, deg
	//}
	tpn_entity_attitude_t tpn_entity_attitude;
	//double  roll; // deg
	//double  pitch; // deg
	//double  heading; // deg - device heading

	//struct tpn_entity_attitude_standard_deviation_t // see tpn_packet_parser.hpp
	//{
	//	float roll_standard_deviation_;      ///< Device roll, deg
	//	float pitch_standard_deviation_;     ///< Device pitch, deg
	//	float heading_standard_deviation_;   ///< Device heading, deg
	//}
	tpn_entity_attitude_standard_deviation_t tpn_entity_attitude_standard_deviation;
	//double  roll_std; // deg
	//double  pitch_std; // deg
	//double  heading_std; // deg - device heading

	//struct tpn_entity_magnetometer_data_t			  // see tpn_packet_parser.hpp
	//{
	//	uint8_t raw_data_available_; 	///< 0x00: raw data is not available, 0x01 : raw data is available
	//	float raw_data_x_;      ///< raw magnetic sensor x data, mG
	//	float raw_data_y_;      ///< raw magnetic sensor y data, mG
	//	float raw_data_z_;      ///< raw magnetic sensor z data, mG
	//	uint8_t raw_data_accuracy_flag_;    ///< 0x00 : Flag not available, 0x01 : Invalid data, 0x02 : Valid data

	//	uint8_t calibrated_data_available_;    ///<	0x00 : calibrated data is not available, 0x01 : calibrated data is available
	//	float calibrated_data_x_;      ///< raw magnetic sensor x data, mG
	//	float calibrated_data_y_;      ///< raw magnetic sensor x data, mG
	//	float calibrated_data_z_;      ///< raw magnetic sensor x data, mG
	//	uint8_t calibrated_data_accuracy_flag_;    ///< 0x00 : Flag not available, 0x01 : Unreliable, 0x02 : Low Accuracy, 0x03 : Medium Accuracy, 0x04 : High Accuracy
	//}
	tpn_entity_magnetometer_data_t tpn_entity_magnetometer_data;
	// raw data
	//double  mag_x; // mG - Magnetic vector in BF
	//double  mag_y; // mG
	//double  mag_z; // mG

	//struct tpn_device_heading_t					  // see tpn_packet_parser.hpp
	//{
	//	int8_t device_heading_available_; //0x00: Device heading value is not available directly and will be computed from the platform_heading_ and misalignment_angle_ fields.
		//0x01: Device heading value is available directly from the device_heading_ field.
	//	float device_heading_;
	//	float platform_heading_;
	//	float misalignment_angle_;
	//}
  tpn_entity_device_heading_t tpn_entity_device_heading;

	//double  user_heading; //deg
	//double  sigma_user_heading; //deg

	//struct tpn_entity_grp_07_t					  // see tpn_packet_parser.hpp
	//{
	//	int8_t m_nav_phase;    ///< navigation phase flag
	//}
	tpn_entity_grp_07_t tpn_entity_grp_07;
	//int     frame_flag; // -1, 0, 1

	//struct tpn_entity_orientation_pitch_d_t		  // see tpn_packet_parser.hpp
	//{
	//	int8_t m_orientation;       ///< it orientation based on pitch flag
	//}
	tpn_entity_orientation_pitch_d_t tpn_entity_orientation_pitch_d;


	//struct tpn_entity_internal_dbg_t				  // see tpn_packet_parser.hpp
	//{
	//	float m_imu_smp_proc_time;            //1
		// ...
	//	int8_t m_orientation_on_pitch;         //6  ///< it orientation based on pitch flag
		// ...
	//	double m_internal_roll;                //29
	//	double m_internal_pitch;               //30
		// ...
	//	float m_att_filter_roll;              //49
	//	float m_att_filter_pitch;             //50
	//	float m_att_filter_heading;           //51
	//	float m_att_filter_roll_std;          //52
	//	float m_att_filter_pitch_std;         //53
	//	float m_att_filter_heading_std;       //54
		// ...
	//};

	tpn_entity_internal_dbg_t tpn_entity_internal_dbg;

	//struct tpn_heading_misalignment			 // see tpn_packet_parser.hpp
	//{
	//	float heading_misalignment_; //deg
	//};
  tpn_entity_heading_misalignment_t tpn_entity_heading_misalignment;


  //struct tpn_entity_floor_number_t			 // see tpn_packet_parser.hpp
  //{
  //   uint16_t floor_number_;             ///< Floor number
  //};
  tpn_entity_floor_number_t tpn_entity_floor_number;

  tpn_entity_stride_information_t tpn_entity_stride_information;

	//
	//int     fidgeting_flag; // optional parameter can absent
};

#endif
