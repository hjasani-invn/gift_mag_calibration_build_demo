/**
 *   \copyright       Copyright (C) SPIRIT Navigation, LLC., 2013
 *   \brief           Magnetic sensor parameters interface
 *   \file            mc_bias.h
 *   \author          D.Churikov
 *   \date            21.06.2014
 */
/*****************************************************************************/

#ifndef __MC_BIAS_H
#define __MC_BIAS_H


#define MC_PARAMS_DEBUG   1 // enable-disable debug information

double extern const MC_MEAS_COV;              ///< sensor measurement covariance
double extern const MC_BIAS_FINE_COV;         ///< initial covariance for fine bias
double extern const MC_ROUGH_BIAS_THRESHOLD;  ///< rough bias threshold
int64_t extern const MC_MAX_FINE_BIAS_AGE;    ///< maximal age of fine bias


namespace PF
{
    /**
    * Magnetic sensor parameters class
    * \ingroup MFP
    */
    class tMagneticSensorParams
    {
        public:

            /**
            * Constructor
            */
            tMagneticSensorParams();

            /**
            * Destructor
            */
            ~tMagneticSensorParams() {};

            /**
            * The method returns calibration-sleep flag
            * \return calibration-sleep flag
            */
            bool isCalibrationSleep()
            {
                return bCalibrationSleep;
            };

            /**
            * The method set calibration-sleep flag
            * \param[in] value - new value of calibration-sleep flag
            */
            void setCalibrationSleep( bool value )
            {
                bCalibrationSleep = value;
            };

            /**
            * The method returns calibration-enable flag
            * \return calibration-enable flag
            */
            bool isCalibrationEnable()
            {
                return bCalibrationEnabled;
            };

            /**
            * The method set calibration-enable flag
            * \param[in] value - new value of calibration-enable flag
            */
            void setCalibrationEnable( bool value )
            {
                bCalibrationEnabled = value;
            };

            /**
            * The method returns bias-fix flag
            * \return bias-fix flag
            */
            bool isBiasFixed()
            {
                return bFixed ;
            };

            /**
            * The method set bias-fix flag
            * \param[in] value - new value of bias-fix flag
            */
            void setBiasFixed( bool value )
            {
                bFixed = value;
            };

            /**
            * The method set specified magnetic bias vector or default magnetic bias vector
            * Additionally method set default value for magnetic bias covariance
            * The method shell used for start when bias accuracy is unknown or poor (first start)
            * \param[in] mcBias - magnetic bias vector
            */
            void SetDefaultBias( double* mcBias );

            /**
            * The method set specified magnetic bias vector
            * Additionally method set initial value for magnetic bias covariance
            * The method shell used for start when accuracy of specified bias is not good
            * \param[in] cur_sys_time - magnetic bias vector
            * \param[in] mcBias - magnetic bias vector
            */
            void SetInitialBias( int64_t cur_sys_time, double* mcBias );

            /**
            * The method set specified magnetic bias vector
            * Additionally method set initial fine value for magnetic bias covariance
            * The method shell used for start when bias accuracy is good
            * \param[in] cur_sys_time - magnetic bias vector
            * \param[in] mcBias - magnetic bias vector
            */
            void SetFineBias( int64_t cur_sys_time, double* mcBias );

            /**
            * The method set specified magnetic bias vector
            * Additionally method set initial value for magnetic bias covariance
            * The method shell used for bias obtained from Android/IOS calibration procedure
            * \param[in] cur_sys_time - magnetic bias vector
            * \param[in] mcBias - magnetic bias vector
            */
            void SetBiasFromStream( int64_t cur_sys_time, double *mcBias );

            /**
            * The method set specified magnetic bias vector
            * \param[in] cur_sys_time - time stamp
            * \param[in] mcBias - magnetic bias vector
            */
            void SetBias( int64_t cur_sys_time, double* mcBias );

            /**
            * The method get magnetic bias vector
            * \param[out] time  - time stamp
            * \param[out] mcBias - magnetic bias vector
            * \return bias validity flag
            */
            bool GetBias( int64_t *time, double* mcBias );

            /**
            * The method set specified magnetic bias covariance (dispersion) vector
            * \param[in] mcBias - magnetic bias covariance (dispersion) vector
            */
            void SetBiasCov( double* mcBiasCov );

            /**
            * The method increases magnetic bias covariance by 4
            */
            void IncreaseBiasCov();

            /**
            * The method get magnetic bias covariance (dispersion) vector
            * \param[out] mcBias - magnetic bias vector
            * \return bias validity flag
            */
            bool GetBiasCov( double* mcBiasCov );


            /**
            * The method returns bias validity
            * \return calibration-sleep flag
            */
            bool IsBiasValid()
            {
                return bValid;
            };

            /**
            * The method returns bias initialization flag
            * \return calibration-sleep flag
            */
            bool IsBiasInited()
            {
                return bInited;
            };

            /**
            * The method returns magnetic bias age from
            * \param[in] time  - current time stamp
            * \return magnetic bias age
            */
            int64_t GetBiasAge( int64_t cur_sys_time )
            {
                return cur_sys_time - sys_time;
            };

            /**
            * The method update magnetic bias value by magnetic bias estimation
            * \param[in] cur_sys_time  - current time stamp
            * \param[in] meas_bias_delta  - measurement bias innovation
            * \param[in] meas_bias_cov  - bias innovation
            * \return magnetic bias age
            */
            void UpdateBias( int64_t cur_sys_time, double *meas_bias_delta, double *meas_bias_cov );

            /**
            * The method calculates consistency of magnetic bias over current measurement
            * \param[in] meas_bias_delta  - measurement bias innovation
            * \return true if consistant
            */
            bool CheckBiasConsistancy( double *meas_bias_delta );


        private:
            void EKFIteration( double *x, double *P, double y, double R );

        private:
            bool bCalibrationSleep;   ///< sleep calibration flag
            bool bCalibrationEnabled; ///< enable calibration flag
            bool bValid;              ///< validity flag
            bool bInited;             ///< initialization flag
            bool bFixed;              ///< fix flag (bias is not changed by calibration and from stream)
            int64_t sys_time;         ///< ms, system time of last bias update
            double bias[3];           ///< bias estimation in phone frame
            double bias_cov[3];       ///< dispersions of bias estimation
            double bias_error[3];     ///< bias error estimation
    };
};

#endif //__MC_BIAS_H
