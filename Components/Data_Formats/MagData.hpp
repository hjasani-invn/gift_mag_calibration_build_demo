/**
* \file MagData.hpp
* \brief Defines basic MFP types
* \author Mikhail Frolov (mfrolov@invensense.com)
* \version 0.1
* \copyright InvenSense, all rights reserved
* \date Jun 2, 2016
*/

#ifndef MAGNETIC_DATA_HPP
#define MAGNETIC_DATA_HPP

#include <stdint.h>
#include <memory.h>

/** single magnetometer measurement */
struct MagneticVector
{
    double mX;          /**<  magnetic field on x-axis [uT] */
    double mY;          /**<  magnetic field on y-axis [uT] */
    double mZ;          /**<  magnetic field on z-axis [uT] */

    const MagneticVector operator + (const MagneticVector &mv) const
    {
        MagneticVector res;
        res.mX = this->mX + mv.mX;
        res.mY = this->mY + mv.mY;
        res.mZ = this->mZ + mv.mZ;
        return res;
    };
    const MagneticVector operator - (const MagneticVector &mv) const
    {
        MagneticVector res;
        res.mX = this->mX - mv.mX;
        res.mY = this->mY - mv.mY;
        res.mZ = this->mZ - mv.mZ;
        return res;
    };
    const MagneticVector operator * (const double lamda) const
    {
        MagneticVector res;
        res.mX = this->mX * lamda;
        res.mY = this->mY * lamda;
        res.mZ = this->mZ * lamda;
        return res;
    };
};

/** defines mag data with uncertainties */
struct MagneticData : public MagneticVector
{
    //MagneticData()
    //{
    //}
    //MagneticData(const MagneticData &c)
    //{
    //	this->timestamp = c.timestamp;
    //	this->mX = c.mX;
    //	this->mY = c.mY;
    //	this->mZ = c.mZ;
    //	memcpy(this->covarianceMatrix, c.covarianceMatrix, sizeof(c.covarianceMatrix));
    //}
    int64_t timestamp;  /**< unix time [ms] */

    double covarianceMatrix[3][3]; /**< mag meassurement covariance matrix [uT^2], column order\n
                                   * cov(mX, mX), cov(mY, mX), cov(mZ, mX)\n
                                   * cov(mX, mY), cov(mY, mY), cov(mZ, mY)\n
                                   * cov(mX, mZ), cov(mY, mZ), cov(mZ, mZ) */
};


/** defines mag data with uncertainties */
struct MagneticMeasurement : public MagneticData
{
    double sigma_mX;          /**<  x-axis magnetometer meassurement noise [uT] */
    double sigma_mY;          /**<  y-axis magnetometer meassurement noise [uT] */
    double sigma_mZ;          /**<  z-axis magnetometer meassurement noise [uT] */
    MagneticMeasurement(): sigma_mX(0), sigma_mY(0), sigma_mZ(0) {}
};


#endif //MAGNETIC_DATA_HPP

