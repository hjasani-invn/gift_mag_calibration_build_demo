/**
* \copyright       Copyright (C) TDK, LLC., 2018
* \brief           BLE Proximity bias estimating
* \file            BLEProximityBiasEstimator.cpp
* \addtogroup      bias
* \author          V. Pentiukhov
* \date            20.01.2019
*/

#include <cmath>
#include <algorithm>
#include <vector>
#include <numeric>
#include "BleHash.h"
#include "BLEProximityBiasEstimator.hpp"
#include <eigen/Core>
#include <eigen/Dense>

#define Sqr(x) ( (x)*(x) )

void BLEProximityBiasEstimator::initializeBias(double bias, double bias_sigma) 
{
    previousBias = bias;
    currentBias = bias;
    this->biasSigma = bias_sigma;
}

bool  BLEProximityBiasEstimator::estimateBias(const std::map < BSSID, BLE_position> &ble_position_map, const Fppe::BleScanResult &measurement, std::vector <Location_and_time> &positions, unsigned long  &biasCounter, double &bias, double &biasSigma, std::ostream *log)
{
    const int fix_filter_steps = 50;
    //int filter_length = 50;
    const int cutting_level = -90;
    bool success = true;
    double deltaBias;
    std::vector<double> deltaBiases;
    std::vector<int> numbers;
    
    /*
    for (auto it = measurement.scanBle.begin(); it != measurement.scanBle.end(); ++it)
    {
        std::cout << getBleHash(it->major, it->minor, it->uuid) << "    ";
        std::cout << (int)it->rssi << "    ";
    }
    //std::cout << "rssi======="<<std::endl;
    */
    for (auto it = ble_position_map.begin(); it != ble_position_map.end(); ++it)
    {
        BSSID bssid = it->first;
        //std::cout << bssid << std::endl;
        BLE_position ble_position = it->second;
        auto found = std::find_if(measurement.scanBle.begin(), measurement.scanBle.end(),
            [&bssid](const Fppe::BleMeasurement& x)
            { return getBleHash(x.major, x.minor, x.uuid) == bssid; });

        if (found == measurement.scanBle.end())
            continue;
        if (found->rssi < cutting_level)
            continue;
            //std::cout << "=======   " << getBleHash(found->major, found->minor, found->uuid) << std::endl;
        int n = 0;
        Location_and_time location_time = positions[0];
        Location_and_time location_time1;
        Location_and_time location_time2;
        if (found->timestamp < location_time.t)
        {
            if ((location_time.t - found->timestamp) > 500)
                continue;
        }
        if (found->timestamp >= location_time.t)
        {
            for (auto itt = positions.begin(); itt != positions.end(); ++itt)
            {
                if (found->timestamp >= itt->t) //&& (found->timestamp < (itt + 1)->t))
                {
                    n++;
                    continue;
                }
                else
                {
                    //location_time = *itt;
                    location_time2 = *itt;
                    location_time1 = *(itt-1);
                    double k = (double)(found->timestamp - location_time2.t) / (double)(location_time1.t - location_time2.t);
                    location_time.location.x = location_time2.location.x + k * (location_time1.location.x - location_time2.location.x);
                    location_time.location.y = location_time2.location.y + k * (location_time1.location.y - location_time2.location.y);
                    location_time.t = location_time2.t + k * (location_time1.t - location_time2.t);

                    break;
                }
            }
        }
        if (n == positions.size())
            continue;
        numbers.push_back(n);

        double t1 = (ble_position.loc.x - location_time.location.x)*(ble_position.loc.x - location_time.location.x) + (ble_position.loc.y - location_time.location.y)*(ble_position.loc.y - location_time.location.y);
        double t2 = sqrt(t1);
        double expected_attenuation = 20 * log10(t2);
        double real_attenuation = ble_position.txPower - found->rssi;

        //std::cout << "real_attenuation = " << real_attenuation << std::endl;

        //std::cout << "deltaBias = " << deltaBias << std::endl;

        deltaBias = (expected_attenuation - real_attenuation - currentBias);
        deltaBiases.push_back(deltaBias);
        //std::cout << "deltaBias = " << deltaBias << std::endl;

    }

    if (!numbers.empty())
    {
        int  min_number = *min_element(numbers.begin(), numbers.end());
        if ((min_number > 0) && (min_number < positions.size() ))
            positions.erase(positions.begin(), positions.begin() + min_number - 1);
    }

    //std::cout << "deltaBias ==================== " <<  std::endl;
    std::sort(deltaBiases.begin(), deltaBiases.end());
    int deltaBiases_size = deltaBiases.size();
    //std::cout << "deltaBiases_size = " << deltaBiases_size << std::endl;
    if (deltaBiases_size >= 1)
    {
        double sum = 0;  // 0 - value of 3rd param
        for (auto x : deltaBiases)  
            sum += x;
        // median
        //currentBias += deltaBiases[deltaBiases_size / 2];
        // average
        currentBias += sum / deltaBiases_size;
    }
    else
    {
        success = false;
        currentBias = previousBias;
    }

    if (filter_step >= fix_filter_steps)
        filter_step = fix_filter_steps;

    //std::cout << "currentBias = " << currentBias << std::endl;

    double alpha = 1 / (double)filter_step;
    bias = currentBias * alpha + previousBias * (1 - alpha);
    biasSigma = biasSigma;

    previousBias = bias;
    filter_step++;

    return success;

}

bool GetDbItem(BSSID  ble_hash, const std::map < BSSID, BLE_position> &ble_map, BLE_position &db_item)
{
    bool IsItemPresentInDB = true;
    try
    {
        db_item = ble_map.at(ble_hash);
    }
    catch (std::out_of_range)
    {
        IsItemPresentInDB = false;
    }
    return IsItemPresentInDB;
}

double const default_kf_bias_covariation = 2 * 2; //3 * 3;

void BleProximityBiasEstimator_EKF::initializeBias(double bias, double bias_sigma)
{
    this->bias = bias;
    this->bias_cov = (bias_sigma > 0) ? (bias_sigma * bias_sigma) : default_kf_bias_covariation;
}

#define EKF_BLP_BIAS_DBG_OUT 0

#define MEAS_MODEL_DISTANCE 1
#define MEAS_MODEL_RSSI 2
#define MEAS_MODEL_COMBINED 3
#define MEAS_MODEL MEAS_MODEL_RSSI
bool BleProximityBiasEstimator_EKF::estimateBias(const std::map < BSSID, BLE_position> &ble_map, const Fppe::BleScanResult &measurement, std::vector <Location_and_time> &positions, unsigned long  &biasCounter, double &bias, double &biasSigma, std::ostream *log)
{
    bool result(false);
    std::vector<double> meas_rssi, meas_d;
    std::vector<double> sigma_rssi, sigma_d;

    double estimation_time = measurement.timestamp*1e-3;

    // calc vector of distances to herable beacons
    for (auto meas : measurement.scanBle)
    {
        //find beacon in DB
        BLE_position db_item;
        const int cutting_level = -90; //-95
        if (GetDbItem(getBleHash(meas.major, meas.minor, meas.uuid), ble_map, db_item) && (meas.rssi > cutting_level))
        {
#if EKF_BLP_BIAS_DBG_OUT
            std::cout << "t= " << estimation_time;
            std::cout << " meas(" << meas.major << ", " << meas.minor << ", " << (int)meas.rssi << ") ";
#endif
            // calc user position
            auto i_pos1 = std::find_if(positions.rbegin(), positions.rend(),
                [&meas](const Location_and_time& pos) { return pos.t <= meas.timestamp; });

            auto i_pos2 = std::find_if(positions.begin(), positions.end(),
                [&meas](const Location_and_time& pos) { return pos.t > meas.timestamp; });

            // calc distance
            if ((i_pos1 != positions.rend()) && (i_pos2 != positions.end()))
            {
                // consistancy checking: is it from one floor, is it adjascent positions
                double tau = (double)(i_pos2->t - i_pos1->t);
                bool is_position_consistent = (tau >= 50) && (tau <= 2500); // time interval limit

                double delta_x = (i_pos2->location.x - i_pos1->location.x);
                double delta_y = (i_pos2->location.y - i_pos1->location.y);
                
                double user_speed = is_position_consistent ? (sqrt(delta_x * delta_x + delta_y * delta_y) / (tau *1e-3)) : 0.;
                is_position_consistent = is_position_consistent && (user_speed >= 0.2) && (user_speed <= 2); // speed limit to reject noisy positions

                double k = is_position_consistent ? (double)(meas.timestamp - i_pos1->t) / tau : 0.;
                double x = i_pos1->location.x + k * delta_x;
                double y = i_pos1->location.y + k * delta_y;
                double rms_xy = std::max(i_pos1->location.rms_xy, i_pos2->location.rms_xy);
                //double rms_xy = i_pos1->location.rms_xy * i_pos1->location.rms_xy + i_pos2->location.rms_xy * i_pos2->location.rms_xy;
                //rms_xy = sqrt(rms_xy/2);
                
                // distance to beacon (propagated)
                double d = sqrt((db_item.loc.x - x) * (db_item.loc.x - x) + (db_item.loc.y - y) * (db_item.loc.y - y));

#if EKF_BLP_BIAS_DBG_OUT // debug output
                std::cout << "p1(" << i_pos1->t << ", " << i_pos1->location.x << ", " << i_pos1->location.y << ", " << i_pos1->location.z << ", " << i_pos1->location.rms_xy << "),, ";
                std::cout << "p2(" << i_pos2->t << ", " << i_pos2->location.x << ", " << i_pos2->location.y << ", " << i_pos2->location.z << ", " << i_pos2->location.rms_xy << "),, ";
                std::cout << "p(" << d << ", " << x << ", " << y << ", " << rms_xy << ") " << std::endl;
#endif
                // Rejection explanation:
                // (rms_xy < 5.) - to reject inaccurate positions
                // (rms_xy > 0.5) - to reject stucked and positions and positions with degraded clowd
                // (d < 15) - to reject far beacons - this limit can be changed to -10 or iven to -5 but this make bias convergency longer; -15 is optimal value adjusted for iPhones in TDK_HQ_Nihonbashi venue
                // (d > 1) - to avoid non-liniarity whed position is to close to the beacon
                if (is_position_consistent && (rms_xy < 5.) && (rms_xy > 0.5) && (d < 5) && (d > 0.5))
                {
                    meas_d.push_back(d);
                    sigma_d.push_back(rms_xy);

                    meas_rssi.push_back(meas.rssi - meas.txPower);
                    //double rssi_sigma = 1 + 0.1 * (-50 - std::max(-90, std::min(-50, int(meas.rssi))));
                    double delta = (-60 - std::min(-60, int(meas.rssi)));
                    sigma_rssi.push_back(1 + 0.2 * delta);
                }
            }
        }
    }

    const int N = sigma_rssi.size();
    if (true == (result = (N > 0)))
    {
        Eigen::MatrixXd F(1, 1), Q(1, 1);
        F << 1;
        Eigen::MatrixXd x = F * this->bias;

#if MEAS_MODEL == MEAS_MODEL_DISTANCE
    Q << 0.1 * 0.1;
    Eigen::MatrixXd y(N, 1), H(N, 1);
    Eigen::MatrixXd R = Eigen::MatrixXd::Zero(N, N);
    for (int i = 0; i < N; i++)
    {
        double h = pow(10, -(meas_rssi[i] - x(0, 0)) / 20);
        double z = meas_d[i];
        y(i, 0) = z - h; // y =  z-h(b)
        H(i, 0) = std::log(10) * h / 20;
        double meas_sigma = h * pow(10, sigma_rssi[i] / 20);
        R(i, i) = meas_sigma * meas_sigma + sigma_d[i] * sigma_d[i];
    }
#elif MEAS_MODEL == MEAS_MODEL_RSSI
        Q << 0.1 * 0.1;
        Eigen::MatrixXd h(N, 1), y(N, 1), H(N, 1);
        Eigen::MatrixXd R = Eigen::MatrixXd::Zero(N, N);
        for (int i = 0; i < N; i++)
        {
            double d0 = 1;
            double h_rssi = -20 * log10(meas_d[i] / d0) + x(0, 0);
            h(i, 0) = h_rssi;
            y(i, 0) = meas_rssi[i] - h(i, 0); // y =  z-h(b)
            H(i, 0) = 1;
            
            double pred_sigma = 20 * log10((meas_d[i] + sigma_d[i]) / d0) - 20 * log10(meas_d[i] / d0); // it is very rough equation
            R(i, i) = sigma_rssi[i] * sigma_rssi[i] + pred_sigma * pred_sigma;
#if EKF_BLP_BIAS_DBG_OUT
            std::cout << "EKF,," << estimation_time << ", " << i << ",, " << meas_d[i] << ", " << sigma_d[i] << ",, " << meas_rssi[i]<< ", " << h_rssi - x(0,0) << ", " << x(0, 0) << ",, " << sigma_rssi[i] << ", " << pred_sigma << ", " << R(i, i) << " " << std::endl;
#endif
        }
#elif MEAS_MODEL == MEAS_MODEL_COMBINED
        Q << 0.01 * 0.01;

        // calc h(b)
        Eigen::MatrixXd y(2*N, 1), H(2*N, 1);
        Eigen::MatrixXd R = Eigen::MatrixXd::Zero(2*N, 2*N);
        for (int i = 0; i < N; i++)
        {
            double h_d = pow(10, -(meas_rssi[i] - x(0, 0)) / 20); // d 
            double h_rssi = -20 * log10( meas_d[i]) + x(0, 0);

            y(2 * i, 0) = meas_d[i] - h_d; // y =  z-h(b)
            y(2 * i + 1, 0) = meas_rssi[i] - h_rssi; // y =  z-h(b)
            H(2 * i, 0) = std::log(10) * h_d / 20;
            H(2 * i + 1, 0) = 1;

            R(2 * i, 2 * i) = sigma_d[i] * sigma_d[i];
            R(2 * i + 1, 2 * i + 1) = sigma_rssi[i] * sigma_rssi[i];
        }
#endif


        double tau = estimation_time - this->t;
        Eigen::MatrixXd P = F * this->bias_cov *F.transpose() + tau * Q;

        Eigen::MatrixXd S = H * P *H.transpose() + R; // S = HPH' + R
        Eigen::MatrixXd K = P * H.transpose() * S.inverse(); // K = PH'inv(S)

        x = x + K * y; // x = x+Ky

        Eigen::MatrixXd I = Eigen::Matrix<double, 1, 1>::Identity();
        //P = P - K * H *P; // P = (I-KH)P - regular equation
        P = (I - K * H) * P * (I - K * H).transpose() + K * R * K.transpose(); // P = (I ? KH)P?(I ? KH)' + KRK' // Josep's form

        this->bias = bias = x(0, 0);
        this->bias_cov = P(0, 0);
        biasSigma = sqrt(this->bias_cov);
        this->t = estimation_time;

    }

    return result;
}
