/**
* \file LocalData.hpp
* \brief Defines Local frame data structures
* \author Mikhail Frolov (mfrolov@invensense.com), D Churikov
* \version 0.1
* \copyright InvenSense, all rights reserved
* \date Jun 20, 2016
*/

#ifndef LOCAL_DATA_HPP
#define LOCAL_DATA_HPP

#include <stdint.h>


/** Position information in Filter Frame (FF) */
struct FfPosition
{
    int64_t timestamp;           /**< UNIX timestamp [msec]*/
    double x;                    /**< local frame x-coordinate, [m]*/
    double y;                    /**< local frame y-coordinate, [m] */
    double altitude;             /**< altitude from sea level, [m] */
    int16_t floor_number;        /**< discrete FF-floor number,starting from 0 */

    double covariance_xy[2][2];  /**< x-y covariance matrix in column order [m^2]\n
                                  * cov(x,x), cov(y,x)\n
                                  * cov(x,y), cov(y,y) */

    double altitude_std;        /**< altitude standard deviation [m]     */
    double floor_std;           /**< altitude standard deviation [floor] */

    int8_t navigation_phase;    ///< navigation phase flag; position is avaliable for navigation_phase > 0
    int8_t fidgeting_flag;      ///< fidgeting flag

	int8_t mode_of_transit;     ///< mode of transit, 1 - walking, 2 - elevator, 3 - stairs, 4 - escalator walking, 5 - escalator standing, 6 - fidgeting, 9 - running

    bool is_valid;              /**< position validity flag */
};

/** Attitude information in Filter Frame (FF) */
struct FfAttitude
{
	int64_t timestamp;           /**< UNIX timestamp [msec]*/
    double user_heading;        /**< speed vector dicrection [rad] [0..2PI] (user heading) */
    double user_heading_std;    /**< user heading std [rad] [0..2PI] (user heading) */
};


/** Attitude information for convertion from UDF to Magnetic Fingerprint Frame (MFF) */
struct MffAttitude
{
	int64_t timestamp;           /**< UNIX timestamp [msec]*/
    double quaternion[4];       /**< orientation quaternion [q0, q1, q2, q3], conversion from UDF to Magnetic Fingerprint Frame (MFF)
                                   * q = q(roll, pith, heading), where roll,pitch,heading ~ N(0,{rollStd^2, pithStd^2, headingStd^2}) */

    double roll_std;            /**< roll standard deviation in UDF, [rad] */
    double pitch_std;           /**< pitch standard deviation in UDF,  [rad] */
    double heading_std;         /**< heading standard deviationin in UDF, [rad] */

    bool is_valid;              /**< orientation validity flag */
};

#endif //TPN_DATA_HPP
