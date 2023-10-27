#include "SensorsCalibrator.h"
#include <iostream>
#include <stdio.h>
#include <iomanip>


void Calibration::SensorsCalibrator::ClearSensorsData()
{
    // clear all internal sensor data vectors
    rawMagneticData.clear();
    rawGyroData.clear();
    rawAccelData.clear();
    calibratedMagneticData.clear();
    calibratedGyroData.clear();
    calibratedAccelData.clear();

    calibration_status = Calibration::ReturnStatus::STATUS_SUCCESS;
}

uint32_t Calibration::SensorsCalibrator::AddAccelData(const SensorDataSample* pDataSamples, const uint32_t n)
{
    for (uint32_t i = 0; i < n; ++i)
    {
        rawAccelData.push_back(pDataSamples[i]);
    }

    return n;
}

uint32_t Calibration::SensorsCalibrator::AddGyroData(const SensorDataSample* pDataSamples, const uint32_t n)
{
    for (uint32_t i = 0; i < n; ++i)
    {
        rawGyroData.push_back(pDataSamples[i]);
    }

    return n;
}

uint32_t Calibration::SensorsCalibrator::AddMagneticData(const SensorDataSample* pDataSamples, const uint32_t n)
{
    for (uint32_t i = 0; i < n; ++i)
    {
        rawMagneticData.push_back(pDataSamples[i]);
    }

    return n;
}

Calibration::ReturnStatus Calibration::SensorsCalibrator::EstimateAccelCalibrationParams(SensorCalibrationParams& ClbParams)
{
    calibration_status = Calibration::ReturnStatus::STATUS_SUCCESS;
    Calibration::ReturnStatus resulting_status = Calibration::ReturnStatus::STATUS_SUCCESS;

    const size_t N = rawAccelData.size();

    if (N < 1000)
    {
        resulting_status = Calibration::ReturnStatus::STATUS_INSUFFICIENT_DATA;
        updateReturnStatus(resulting_status);
        return calibration_status;
    }

    double max_delta_time = estimateSensorDataDivergency(rawAccelData);

    if (calibration_status != Calibration::ReturnStatus::STATUS_SUCCESS)
    {
        return calibration_status; // this is important, since failed status at this point means that library can't even try to perform calibration
    }

    if (max_delta_time > max_calibration_time)
    {
        resulting_status = Calibration::ReturnStatus::STATUS_TOO_LONG_CALIBRATION_TIME;
        updateReturnStatus(resulting_status);
    }

    double temp_variation = estimateTemperatureVariations(rawAccelData);

    if (temp_variation > max_temp_variation)
    {
        resulting_status = Calibration::ReturnStatus::STATUS_UNSTABLE_TEMPERATURE;
        updateReturnStatus(resulting_status);
    }

    std::vector<SensorDataSample> calibrationData;
    bool result = selectSensorDataForCalibration(rawAccelData, 1.0, 1.0, calibrationData);

    if (calibrationData.size() < optimal_data_count)
    {
        resulting_status = Calibration::ReturnStatus::STATUS_INSUFFICIENT_DATA;
        updateReturnStatus(resulting_status);
    }

    Calibration::ReturnStatus LSEF_result = LSEF_calibration(calibrationData, ClbParams);

    if (LSEF_result == Calibration::ReturnStatus::STATUS_SUCCESS)
    {
        std::vector<double> axis_dop;
        axis_dop.push_back(dop_x);
        axis_dop.push_back(dop_y);
        axis_dop.push_back(dop_z);

        double SD_axis_DOP = calculateVectorSD(axis_dop);

        if ((ClbParams.DOP > max_DOP) || (dop_x > max_axis_DOP) || (dop_y > max_axis_DOP) || (dop_z > max_axis_DOP) || (SD_axis_DOP > max_SD_axis_DOP))
        {
            resulting_status = Calibration::ReturnStatus::STATUS_HIGH_DOP;
            updateReturnStatus(resulting_status);
        }

        double max_scale_factor = std::max(ClbParams.CM[0][0], ClbParams.CM[1][1]);
        max_scale_factor = std::max(max_scale_factor, ClbParams.CM[2][2]);

        double min_scale_factor = std::min(ClbParams.CM[0][0], ClbParams.CM[1][1]);
        min_scale_factor = std::min(min_scale_factor, ClbParams.CM[2][2]);

        if ((max_scale_factor > max_allowed_scale_factor) || (min_scale_factor < min_allowed_scale_factor))
        {
            resulting_status = Calibration::ReturnStatus::STATUS_UNREALISTIC_SCALE_FACTOR;
            updateReturnStatus(resulting_status);
        }

        double biasUncertainty = sqrt(ClbParams.covB[0][0] * 3.0);
        ClbParams.calibrationLevel = calculateAccelerometerCalibrationLevel(biasUncertainty);
    }

    return calibration_status;
}

Calibration::ReturnStatus Calibration::SensorsCalibrator::EstimateMagneticCalibrationParams(SensorCalibrationParams& ClbParams)
{
    calibration_status = Calibration::ReturnStatus::STATUS_SUCCESS;
    Calibration::ReturnStatus resulting_status = Calibration::ReturnStatus::STATUS_SUCCESS;

    const size_t N = rawMagneticData.size();

    if (N < 1000)
    {
        resulting_status = Calibration::ReturnStatus::STATUS_INSUFFICIENT_DATA;
        updateReturnStatus(resulting_status);
    }

    double max_delta_time = estimateSensorDataDivergency(rawMagneticData);

    if (calibration_status != Calibration::ReturnStatus::STATUS_SUCCESS)
    {
        return calibration_status; // this is important, since failed status at this point means that library can't even try to perform calibration
    }

    if (max_delta_time > max_calibration_time)
    {
        resulting_status = Calibration::ReturnStatus::STATUS_TOO_LONG_CALIBRATION_TIME;
        updateReturnStatus(resulting_status);
    }

    double temp_variation = estimateTemperatureVariations(rawMagneticData);

    if (temp_variation > max_temp_variation)
    {
        resulting_status = Calibration::ReturnStatus::STATUS_UNSTABLE_TEMPERATURE;
        updateReturnStatus(resulting_status);
    }

    std::vector<SensorDataSample> calibrationData;
    bool result = selectSensorDataForCalibration(rawMagneticData, 1.0, 10.0, calibrationData);

    if (calibrationData.size() < optimal_data_count)
    {
        resulting_status = Calibration::ReturnStatus::STATUS_INSUFFICIENT_DATA;
        updateReturnStatus(resulting_status);
    }

    Calibration::ReturnStatus LSEF_result = LSEF_calibration(calibrationData, ClbParams);

    if (LSEF_result == Calibration::ReturnStatus::STATUS_SUCCESS)
    {
        std::vector<double> axis_dop;
        axis_dop.push_back(dop_x);
        axis_dop.push_back(dop_y);
        axis_dop.push_back(dop_z);

        double SD_axis_DOP = calculateVectorSD(axis_dop);

        if ((ClbParams.DOP > max_DOP) || (dop_x > max_axis_DOP) || (dop_y > max_axis_DOP) || (dop_z > max_axis_DOP) || (SD_axis_DOP > max_SD_axis_DOP))
        {
            resulting_status = Calibration::ReturnStatus::STATUS_HIGH_DOP;
            updateReturnStatus(resulting_status);
        }

        double max_scale_factor = std::max(ClbParams.CM[0][0], ClbParams.CM[1][1]);
        max_scale_factor = std::max(max_scale_factor, ClbParams.CM[2][2]);

        double min_scale_factor = std::min(ClbParams.CM[0][0], ClbParams.CM[1][1]);
        min_scale_factor = std::min(min_scale_factor, ClbParams.CM[2][2]);

        if ((max_scale_factor > max_allowed_scale_factor) || (min_scale_factor < min_allowed_scale_factor))
        {
            resulting_status = Calibration::ReturnStatus::STATUS_UNREALISTIC_SCALE_FACTOR;
            updateReturnStatus(resulting_status);
        }

        double biasUncertainty = sqrt(ClbParams.covB[0][0] * 3.0);
        ClbParams.calibrationLevel = calculateMagnetometerCalibrationLevel(biasUncertainty);
    }

    return calibration_status;
}

double Calibration::SensorsCalibrator::estimateSensorDataDivergency(std::vector<SensorDataSample>& sensorData)
{
    const size_t N = sensorData.size();
    std::vector<double>	time;

    bool backward_time_jump_detected = false;
    double previous_time = 0.0;

    for (size_t i = 0; i < N; ++i)
    {
        time.push_back(sensorData[i].time);

        if ((sensorData[i].time - previous_time) < 0.0)
            backward_time_jump_detected = true;

        previous_time = sensorData[i].time;
    }

    if (backward_time_jump_detected)
    {
        updateReturnStatus(Calibration::ReturnStatus::STATUS_ERRORS_IN_INPUT_DATA);
    }

    double min_time = (double)*std::min_element(time.begin(), time.end());
    double max_time = (double)*std::max_element(time.begin(), time.end());

    return (max_time - min_time);
}

double Calibration::SensorsCalibrator::estimateTemperatureVariations(std::vector<SensorDataSample>& sensorData)
{
    const size_t N = sensorData.size();

    std::vector<double>	temp;

    for (size_t i = 0; i < N; ++i)
    {
        temp.push_back(sensorData[i].temperature);
    }

    std::vector<double> temp_flt;
    meanSlideWindow_Filter(temp, 20, temp_flt);

    average_temperature = calculateVectorMean(temp_flt);
    double min_temp = (double)*std::min_element(temp_flt.begin(), temp_flt.end());
    double max_temp = (double)*std::max_element(temp_flt.begin(), temp_flt.end());
    
    return (max_temp - min_temp);
}

void Calibration::SensorsCalibrator::meanSlideWindow_Filter(std::vector<double>& v, uint32_t span, std::vector<double>& v_flt)
{
    int N = (int)v.size();
    v_flt.clear();
    int n = (int)floor(span / 2);

    for (int i = N; i > 0; --i)
    {
        int ni = std::min(std::min(i - 1, n), (N - i));
        std::vector<double> tmp_v = std::vector<double>(v.begin() + (i - ni - 1), v.begin() + (i + ni));
        double tmp_v_mean = calculateVectorMean(tmp_v);

        v_flt.push_back(tmp_v_mean);
    }
}

Calibration::ReturnStatus Calibration::SensorsCalibrator::LSEF_calibration(std::vector<SensorDataSample>& dataSelectedForCalibration, SensorCalibrationParams& calibrationParams)
{
    const size_t N = dataSelectedForCalibration.size();

    calibrationParams = {};

    Eigen::MatrixXd D = Eigen::MatrixXd::Zero(N, 10);
    Eigen::MatrixXd X = Eigen::MatrixXd::Zero(N, 4);
    Eigen::VectorXd Y = Eigen::VectorXd::Zero(N, 1);

    for (size_t i = 0; i < N; ++i)
    {
        double x = dataSelectedForCalibration[i].x;
        double y = dataSelectedForCalibration[i].y;
        double z = dataSelectedForCalibration[i].z;

        D(i, 0) = x * x;
        D(i, 1) = y * y;
        D(i, 2) = z * z;
        D(i, 3) = 2 * y * z;
        D(i, 4) = 2 * x * z;
        D(i, 5) = 2 * x * y;
        D(i, 6) = 2 * x;
        D(i, 7) = 2 * y;
        D(i, 8) = 2 * z;
        D(i, 9) = 1.0;

        X(i, 0) = x; X(i, 1) = y; X(i, 2) = z; X(i, 3) = 1.0;
        Y(i, 0) = x * x + y * y + z * z;
    }

    // bet - vector 4 x 1
    Eigen::Vector4d bet = Eigen::Vector4d::Zero(4, 1);

    bet = (X.transpose() * X).inverse() * X.transpose() * Y;

    Eigen::Vector3d best4246 = Eigen::Vector3d::Zero(3, 1);
    best4246 = bet.block(0, 0, 3, 1) / 2.0;	 // A.block(i, j, m, n) - matrix block starting at (i,j) having m rows, and n columns

    double BTB4246 = bet(3, 0) + best4246.dot(best4246);

    Eigen::MatrixXd P = Eigen::MatrixXd::Zero(10, 10);
    P = D.transpose() * D;
    // S11 - P(0:5, 0:5)
    // S22 - P(6:9, 6:9)
    // S12 = P(0:5, 6:9)
    Eigen::MatrixXd S11 = Eigen::MatrixXd::Zero(6, 6);
    Eigen::MatrixXd S22 = Eigen::MatrixXd::Zero(4, 4);
    Eigen::MatrixXd S12 = Eigen::MatrixXd::Zero(6, 4);

    S11 = P.block(0, 0, 6, 6);
    S22 = P.block(6, 6, 4, 4);
    S12 = P.block(0, 6, 6, 4);

    Eigen::MatrixXd S22_1 = S22.inverse();
    Eigen::Matrix3d S_1 = Eigen::MatrixXd::Identity(3, 3);

    double ql = 0.0;
    double qh = 0.0;
    double q = 100.0;

    // iterative ellipsoid fitting method

    Eigen::VectorXd a = Eigen::VectorXd::Zero(10, 1);
    Eigen::Matrix3d Q = Eigen::Matrix3d::Zero(3, 3);
    int cnd = 0;

    elfit(S11, S22_1, S12, q, a, Q, cnd);

    int wc = 1;
    if (cnd > 0)
        wc = 0;
    else
    {
        if (q > 3.0)
        {
            while (wc == 1)
            {
                q /= 2.0;
                ql = std::max(q, 3.0);
                qh = 2 * q;
                q = (ql + qh) / 2.0;

                elfit(S11, S22_1, S12, q, a, Q, cnd);
                
                if (cnd > 0)
                    ql = q;
                else
                    qh = q;

                if (std::abs(qh - ql) < 0.1)
                    break;
            }
        }
    }

    // estimate hard iron compensation vector (bias)
    Eigen::Vector3d best = -Q.inverse() * a.segment(6, 3);

    // estimate of B'B
    double BTB = std::abs(best.transpose() * Q * best - a(9));
    // estimate correction
    double coef = sqrt(BTB4246 / BTB);
    
    // Q_eigenlib = (-1) * Q_Matlab
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(Q, Eigen::ComputeThinU | Eigen::ComputeThinV);

    Eigen::Matrix3d Xq = svd.matrixU();
    //Eigen::Matrix3d Zq = svd.matrixV();
    Eigen::Matrix3d Yq = svd.singularValues().asDiagonal();

    Eigen::Matrix3d Ys = Yq.cwiseSqrt();

    // soft iron compensation matrix
    S_1 = Xq * Ys * Xq.transpose() * coef;

    calibrationParams.bx = best(0);
    calibrationParams.by = best(1);
    calibrationParams.bz = best(2);

    calibrationParams.CM[0][0] = S_1(0, 0); calibrationParams.CM[0][1] = S_1(0, 1); calibrationParams.CM[0][2] = S_1(0, 2);
    calibrationParams.CM[1][0] = S_1(1, 0); calibrationParams.CM[1][1] = S_1(1, 1); calibrationParams.CM[1][2] = S_1(1, 2);
    calibrationParams.CM[2][0] = S_1(2, 0); calibrationParams.CM[2][1] = S_1(2, 1); calibrationParams.CM[2][2] = S_1(2, 2);

    calibrationParams.temperature = average_temperature;
    calibrationParams.time = 0.0;

    Eigen::Vector3d magXYZ = Eigen::MatrixXd::Zero(3, 1);
    Eigen::Vector3d compensated_magXYZ = Eigen::MatrixXd::Zero(3, 1);
    Calibration::SensorDataSample one_sample = {};
    std::vector<SensorDataSample> calibratedData;

    for (size_t i = 0; i < N; ++i)
    {
        magXYZ(0) = dataSelectedForCalibration[i].x;
        magXYZ(1) = dataSelectedForCalibration[i].y;
        magXYZ(2) = dataSelectedForCalibration[i].z;

        compensated_magXYZ = S_1 * (magXYZ - best);

        one_sample.time = dataSelectedForCalibration[i].time;
        one_sample.temperature = dataSelectedForCalibration[i].temperature;
        one_sample.x = compensated_magXYZ(0);
        one_sample.y = compensated_magXYZ(1);
        one_sample.z = compensated_magXYZ(2);

        calibratedData.push_back(one_sample);
    }

    double accuracy = estimateBiasUncertainty(calibratedData);

    calibrationParams.DOP = estimateDOP(calibratedData, dop_x, dop_y, dop_z);

    calibrationParams.covB[0][0] = accuracy * accuracy / 3.0;
    calibrationParams.covB[1][1] = accuracy * accuracy / 3.0;
    calibrationParams.covB[2][2] = accuracy * accuracy / 3.0;

    return Calibration::ReturnStatus::STATUS_SUCCESS;
}

void Calibration::SensorsCalibrator::elfit(Eigen::MatrixXd& S11, Eigen::MatrixXd& S22_1, Eigen::MatrixXd& S12, double q, Eigen::VectorXd& a, Eigen::Matrix3d& Q, int& cnd)
{
    Eigen::MatrixXd C1 = Eigen::MatrixXd::Zero(6, 6);
    C1(0, 0) = -1;
    C1(0, 1) = q / 2 - 1;
    C1(0, 2) = q / 2 - 1;
    
    C1(1, 0) = q / 2 - 1;
    C1(1, 1) = - 1;
    C1(1, 2) = q / 2 - 1;

    C1(2, 0) = q / 2 - 1;
    C1(2, 1) = q / 2 - 1;
    C1(2, 2) = -1;

    C1(3, 3) = -q;
    C1(4, 4) = -q;
    C1(5, 5) = -q;

    Eigen::MatrixXd M = Eigen::MatrixXd::Zero(6, 6);
    M = C1.inverse() * (S11 - S12 * S22_1 * S12.transpose());

    // [V,E] = eig(M);
    Eigen::MatrixXcd V(6, 6);
    Eigen::MatrixXcd E(6, 1);

    // Note: E is 6x1 complex vector here, but in Matlab it is 6x6 real matrix
    Eigen::EigenSolver<Eigen::MatrixXd> es;
    es.compute(M, true);
    
    V = es.eigenvectors();
    E = es.eigenvalues();
    
    // find the only positive eigenvalue, ignore complex values if they exist
    // return cnd = -1 if failed to find it

    int k0 = -1;
    
    for (int i = 0; i < 6; ++i)
    {
        if ( (E(i).imag() < std::numeric_limits<double>::epsilon()) && (E(i).real() > 0.0) )
        {
            k0 = i;
            break;
        }
    }

    if (k0 == -1) // didn't find a real positive eigenvalue
    {
        cnd = -1;
        return;
    }

    // corresponding eigen vector
    Eigen::VectorXd v = Eigen::VectorXd::Zero(6, 1);
    v = V.col(k0).real();
    // Lagrange multiplier
    double mu = sqrt(1.0 / (v.transpose() * C1 * v));

    Eigen::VectorXd a1 = v * mu;
    Eigen::VectorXd a2 = -S22_1 * S12.transpose() * a1;

    a.segment(0, 6) = a1;
    a.segment(6, 4) = a2;

    double I = a(0) + a(1) + a(2);
    double J = a(0) * a(1) + a(1) * a(2) + a(0) * a(2) - a(3) * a(3) - a(4) * a(4) - a(5) * a(5);

    Q(0, 0) = a(0); Q(0, 1) = a(5); Q(0, 2) = a(4);
    Q(1, 0) = a(5); Q(1, 1) = a(1); Q(1, 2) = a(3);
    Q(2, 0) = a(4); Q(2, 1) = a(3); Q(2, 2) = a(2);

    if ((J > 0) && (I * Q.determinant() > 0))
        cnd = 1;
    else
        cnd = -1;
}

double Calibration::SensorsCalibrator::estimateDOP(std::vector<SensorDataSample>& calibratedData, double& DOP_x, double& DOP_y, double& DOP_z)
{
    const size_t N = calibratedData.size();
    double DOP = 0.0;
    // F - matrix 3xN, which consists of normalized meas vectors from calibratedData
    Eigen::MatrixXd F = Eigen::MatrixXd::Zero(3, N);
    double meas_x = 0.0;
    double meas_y = 0.0;
    double meas_z = 0.0;
    double norm = 0.0;

    for (size_t i = 0; i < N; ++i)
    {
        meas_x = calibratedData[i].x;
        meas_y = calibratedData[i].y;
        meas_z = calibratedData[i].z;

        norm = sqrt(meas_x * meas_x + meas_y * meas_y + meas_z * meas_z);

        F(0, i) = meas_x / norm;
        F(1, i) = meas_y / norm;
        F(2, i) = meas_z / norm;
    }

    Eigen::Matrix3d FFinv = (F * F.transpose()).inverse();
    DOP = sqrt(FFinv.trace());

    DOP_x = sqrt(FFinv(0, 0));
    DOP_y = sqrt(FFinv(1, 1));
    DOP_z = sqrt(FFinv(2, 2));

    return DOP;
}

double Calibration::SensorsCalibrator::estimateBiasUncertainty(std::vector<SensorDataSample> &calibratedData)
{
    // estimation of uncertainty
    // iterate vector and add results (deltas) to another vector
    const size_t N = calibratedData.size();
    std::vector<double> norms;

    Eigen::Vector3d magXYZ = Eigen::MatrixXd::Zero(3, 1);
    double d = 0.0;

    for (size_t i = 0; i < N; ++i)
    {
        magXYZ(0) = calibratedData[i].x;
        magXYZ(1) = calibratedData[i].y;
        magXYZ(2) = calibratedData[i].z;

        d = magXYZ.norm();
        norms.push_back(d);
    }

    double est_sd = calculateVectorSD(norms);

    return est_sd;
}

bool Calibration::SensorsCalibrator::selectSensorDataForCalibration(std::vector<SensorDataSample>& sensorData, double averagingInterval, double threshold, std::vector<SensorDataSample>& dataSelectedForCalibration)
{
    size_t sz = sensorData.size();

    std::vector<double> t_raw;
    
    for (size_t i = 0; i < sz; ++i)
    {
        t_raw.push_back(sensorData[i].time - sensorData[0].time);
    }

    size_t kb = 0;
    size_t ke = 0;
    double t_av = 0.0;
    std::vector<double> t;
    double v_sd_x = 0.0;
    double v_sd_y = 0.0;
    double v_sd_z = 0.0;
    std::vector<double> m;

    int dbg_idx = 0;

    for (size_t i = 1; i < sz - 1; ++i)
    {
        kb = findRightNeighbour(t_raw[i] - 0.5 * averagingInterval, t_raw);
        kb = std::max<size_t>(kb, 0);
        ke = findLeftNeighbour(t_raw[i] + 0.5 * averagingInterval, t_raw);
        ke = std::min<size_t>(ke, sz - 1);

        if ((ke - kb) >= min_averaging_sample_number)
        {
            dbg_idx += 1;
            t_av = t_raw[i];
            t.push_back(t_av);
            std::vector<SensorDataSample> sensorData_slice = std::vector<SensorDataSample>(sensorData.begin() + kb, sensorData.begin() + ke + 1);

            calculate_SensorDataSample_vector_SD(sensorData_slice, v_sd_x, v_sd_y, v_sd_z);

            m.push_back(sqrt(v_sd_x * v_sd_x + v_sd_y * v_sd_y + v_sd_z * v_sd_z));
        }
    }
    
    // find extremums
    double dm1 = 0.0;
    double dm2 = 0.0;
    double dm3 = 0.0;
    std::vector<double> m_min;
    std::vector<double> m_max;
    std::vector<double> t_min;
    std::vector<double> t_max;

    sz = m.size();

    for (size_t i = 2; i < sz - 2; ++i)
    {
        dm1 = m[i] - m[i - 1];
        dm2 = m[i + 1] - m[i];

        // minimums which are under 1.0
        if ((dm1 * dm2 < 0) && (dm1 < 0) && (m[i] < threshold))
        {
            m_min.push_back(m[i]);
            t_min.push_back(t[i]);
        }

        // maximums which are over 1.0
        if ((dm1 * dm2 < 0) && (dm1 > 0) && (m[i] > threshold))
        {
            m_max.push_back(m[i]);
            t_max.push_back(t[i]);
        }

        // one slope is zero
        if (dm1 * dm2 == 0)
        {
            dm1 = m[i - 1] - m[i - 2];
            dm2 = m[i] - m[i - 1];
            dm3 = m[i + 1] - m[i];

            // minimums which are under 1.0
            if ((dm1 * dm3 < 0) && (dm1 < 0) && (m[i] < threshold))
            {
                m_min.push_back(m[i]);
                t_min.push_back(t[i]);
            }

            // maximums which are over 1.0
            if ((dm1 * dm3 < 0) && (dm1 > 0) && (m[i] > threshold))
            {
                m_max.push_back(m[i]);
                t_max.push_back(t[i]);
            }
        }

    }

    // select optimal points

    std::vector<double> m_opt;
    std::vector<double> t_opt;
    size_t idx1 = 0;
    size_t idx2 = 0;

    sz = t_max.size();

    for (size_t i = 0; i < sz + 1; ++i)
    {
        if (i == 0)
            idx1 = 0;
        else
            idx1 = findRightNeighbour(t_max[i - 1], t_min);

        if (i == sz)
            idx2 = t_min.size() - 1;
        else
            idx2 = findLeftNeighbour(t_max[i], t_min);

        if (idx1 <= idx2)
        {
            double min_element = (double)*std::min_element(m_min.begin() + idx1, m_min.begin() + idx2 + 1);
            m_opt.push_back(min_element);

            // find index of element from mmin(idx1:idx2) which equals to m_opt_(end)
            auto iter = std::find(m_min.begin() + idx1, m_min.begin() + idx2 + 1, *(m_opt.end() - 1));

            if (iter != m_min.begin() + idx2 + 1)
            {
                size_t idx_opt = std::distance(m_min.begin() + idx1, iter);
                t_opt.push_back(t_min[idx1 + idx_opt]);
            }

            else
                continue;
        }
    }

    // calc average values
    sz = t_opt.size();
    size_t idx = 0;

    for (size_t i = 0; i < sz; ++i)
    {		
        auto iter = std::find(t_raw.begin(), t_raw.end(), t_opt[i]);
        if (iter != t_raw.end())
        {
            idx = std::distance(t_raw.begin(), iter);
        }
        else
            continue;

        kb = findRightNeighbour(t_raw[idx] - 0.5 * averagingInterval, t_raw);
        kb = std::max<size_t>(kb, 0);
        ke = findLeftNeighbour(t_raw[idx] + 0.5 * averagingInterval, t_raw);
        ke = std::min<size_t>(ke, (int)t_raw.size());
        
        double tav = 0.0;
        double m_av_x = 0.0;
        double m_av_y = 0.0;
        double m_av_z = 0.0;
        int j1 = 0;
                
        for (size_t j = kb; j < ke + 1; ++j)
        {
            tav = (double)j1/((double)j1 + 1) * tav + (1.0 / (j1 + 1.0)) * t_raw[j];
            m_av_x = (double)j1 / ((double)j1 + 1) * m_av_x + (1.0 / (j1 + 1.0)) * sensorData[j].x;
            m_av_y = (double)j1 / ((double)j1 + 1) * m_av_y + (1.0 / (j1 + 1.0)) * sensorData[j].y;
            m_av_z = (double)j1 / ((double)j1 + 1) * m_av_z + (1.0 / (j1 + 1.0)) * sensorData[j].z;

            ++j1;
        }

        Calibration::SensorDataSample result;
        result.time = tav;
        result.temperature = 0.0;
        result.x = m_av_x;
        result.y = m_av_y;
        result.z = m_av_z;

        dataSelectedForCalibration.push_back(result);
    }


    return true;
}

size_t Calibration::SensorsCalibrator::findRightNeighbour(double ti, std::vector<double>& v)
{
    // from vector sensorData find index of element with minimum element.time, where (element.time > t)
    // if each element time is < t, then return index = (N - 1)
    // Matlab always returns idx = -1 for zero size vector

    const size_t N = v.size();

    if (N < 1)
        return -1;

    bool found = false;
    size_t i = N;

    while (!found)
    {
        --i;

        if (v[i] <= ti)
        {
            found = true;
            break;
        }
    
        if (i <= 0)
        {
            break;
        }
    }

    if (found)
    {
        if (i == (N - 1))
            return i;
        else
            return i + 1;
    }
    else
    {
        return 0;
    }
}

size_t Calibration::SensorsCalibrator::findLeftNeighbour(double ti, std::vector<double>& v)
{
    // from vector sensorData find index of element with maximum element.time, where (element.time < t)
    // if each element time is > t, then return index = 0
    // if each element time is < t, return index = the last element index
    // Matlab always returns idx = 0 for zero size vector, maybe better to return -1 instead

    const size_t N = v.size();

    if (N < 1)
        return -1;

    bool found = false;
    size_t i = 0;
    
    while (!found)
    {
        if (v[i] >= ti)
        {
            found = true;
            break;
        }

        ++i;

        if (i == N)
        {
            break;
        }
    }

    if (found)
    {
        if (i == 0)
            return 0;
        else
            return i - 1;
    }
    else
    {
        return i - 1;
    }
}

double Calibration::SensorsCalibrator::calculateVectorSD(std::vector<double>& v)
{
    const size_t N = v.size();
    double mean = calculateVectorMean(v);

    double tmp_sum = 0.0;

    for (int i = 0; i < N; ++i)
    {
        tmp_sum += (v[i] - mean) * (v[i] - mean);
    }

    double est_sd = sqrt(tmp_sum / (N - 1));

    return est_sd;
}

double Calibration::SensorsCalibrator::calculateVectorMean(std::vector<double>& v)
{
    const size_t N = v.size();
    double mean = 0.0;

    for (size_t i = 0; i < N; ++i)
    {
        mean = ((double)i / ((double)i + 1)) * mean + (1.0 / (i + 1.0)) * v[i];
    }

    return mean;
}

void Calibration::SensorsCalibrator::calculate_SensorDataSample_vector_SD(std::vector<SensorDataSample>& sensorData, double& sd_x, double& sd_y, double& sd_z)
{
    const size_t N = sensorData.size();

    std::vector<double> v_x;
    std::vector<double> v_y;
    std::vector<double> v_z;

    for (size_t i = 0; i < N; ++i)
    {
        v_x.push_back(sensorData[i].x);
        v_y.push_back(sensorData[i].y);
        v_z.push_back(sensorData[i].z);
    }

    sd_x = calculateVectorSD(v_x);
    sd_y = calculateVectorSD(v_y);
    sd_z = calculateVectorSD(v_z);
}

void Calibration::SensorsCalibrator::updateReturnStatus(Calibration::ReturnStatus return_status)
{
    //std::cout << "new calibration status: " << int(return_status) << std::endl;
    calibration_status = std::min(calibration_status, return_status);
}

int8_t Calibration::SensorsCalibrator::calculateMagnetometerCalibrationLevel(double biasUncertainty)
{
    int8_t calibLevel = 0;

    if (calibration_status < Calibration::ReturnStatus::STATUS_SUCCESS)
        return 0;

    if (biasUncertainty > magnetometer_low_calib_accuracy_limit)
    {
        updateReturnStatus(Calibration::ReturnStatus::STATUS_LOW_CALIBRATION_ACCURACY);		
    }

    if (biasUncertainty <= 1.0)
        calibLevel = 7;
    else if (biasUncertainty <= 2.5)
        calibLevel = 6;
    else if (biasUncertainty <= 5.0)
        calibLevel = 5;
    else if (biasUncertainty <= 10.0)
        calibLevel = 4;
    else if (biasUncertainty <= 30.0)
        calibLevel = 3;
    else if (biasUncertainty <= 50.0)
        calibLevel = 2;
    else if (biasUncertainty <= 250.0)
        calibLevel = 1;
    else
        calibLevel = 0;

    return calibLevel;
}

int8_t Calibration::SensorsCalibrator::calculateAccelerometerCalibrationLevel(double biasUncertainty)
{
    int8_t calibLevel = 0;

    if (calibration_status < Calibration::ReturnStatus::STATUS_SUCCESS)
        return 0;

    if (biasUncertainty > accelerometer_low_calib_accuracy_limit)
    {
        updateReturnStatus(Calibration::ReturnStatus::STATUS_LOW_CALIBRATION_ACCURACY);
        calibLevel = 1;
    }

    else
        calibLevel = 2;

    return calibLevel;
}

bool Calibration::SensorsCalibrator::PrintData(std::ostream& pLog)
{
    size_t N = rawMagneticData.size();

    if (N > 0)
    {
        pLog << "Magnetometer data received: " << std::endl;
        for (size_t i = 0; i < N; ++i)
        {
            pLog << rawMagneticData[i].time << " " << rawMagneticData[i].x << " " << rawMagneticData[i].y << " " << rawMagneticData[i].z << " " << rawMagneticData[i].temperature << std::endl;
        }
    }

    N = rawAccelData.size();

    if (N > 0)
    {
        pLog << "Accelerometer data received: " << std::endl;
        for (size_t i = 0; i < N; ++i)
        {
            pLog << rawAccelData[i].time << " " << rawAccelData[i].x << " " << rawAccelData[i].y << " " << rawAccelData[i].z << " " << rawAccelData[i].temperature << std::endl;
        }
    }

    return true;
}