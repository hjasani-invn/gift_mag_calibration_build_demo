/**
* \file CMagData.h
* \brief C wrapper for definitions of basic MFP types
* \author Mostafa Elhoushi (melhoushi@invensense.com)
* \version 0.1
* \copyright InvenSense, all rights reserved
* \date Oct 19, 2016
*/

#ifndef C_MAGNETIC_DATA_H
#define C_MAGNETIC_DATA_H

#include <stdint.h>
#include <memory.h>

/** single magnetometer measurement */
typedef struct CMagneticVectorTag
{
    double mX;          /**<  magnetic field on x-axis [uT] */
    double mY;          /**<  magnetic field on y-axis [uT] */
    double mZ;          /**<  magnetic field on z-axis [uT] */
} CMagneticVector;

/** defines mag data with uncertainties */
typedef struct CMagneticDataTag
{
	double mX;          /**<  magnetic field on x-axis [uT] */
	double mY;          /**<  magnetic field on y-axis [uT] */
	double mZ;          /**<  magnetic field on z-axis [uT] */

	int64_t timestamp;  /**< unix time [ms] */

    double covarianceMatrix[3][3]; /**< mag meassurement covariance matrix [uT^2], column order\n
                                   * cov(mX, mX), cov(mY, mX), cov(mZ, mX)\n
                                   * cov(mX, mY), cov(mY, mY), cov(mZ, mY)\n
                                   * cov(mX, mZ), cov(mY, mZ), cov(mZ, mZ) */
} CMagneticData;

int CMagneticData_clone(int c);



/** defines mag data with uncertainties */
typedef struct CMagneticMeasurementTag
{
	double mX;          /**<  magnetic field on x-axis [uT] */
	double mY;          /**<  magnetic field on y-axis [uT] */
	double mZ;          /**<  magnetic field on z-axis [uT] */

	int64_t timestamp;  /**< unix time [ms] */

	double covarianceMatrix[3][3]; /**< mag meassurement covariance matrix [uT^2], column order\n
								   * cov(mX, mX), cov(mY, mX), cov(mZ, mX)\n
								   * cov(mX, mY), cov(mY, mY), cov(mZ, mY)\n
								   * cov(mX, mZ), cov(mY, mZ), cov(mZ, mZ) */

    double sigma_mX;          /**<  x-axis magnetometer meassurement noise [uT] */
    double sigma_mY;          /**<  y-axis magnetometer meassurement noise [uT] */
    double sigma_mZ;          /**<  z-axis magnetometer meassurement noise [uT] */
} CMagneticMeasurement;


#endif //C_MAGNETIC_DATA_H

