/**
* \copyright       Copyright (C) TDK-Invensense, 2017
* \brief           Likelyhood position calculation
* \defgroup        LikelihoodSampling
* \file            LikelihoodPos.hpp
* \author          D Churikov
* \date            8.06.2017
*/

#ifndef _LIKELIHOODPOS_HPP
#define _LIKELIHOODPOS_HPP

#include "IFFSettings.hpp"
#include "Particle.hpp"

#include <stdint.h>
#include <algorithm>
#include <list>
#include <fstream>
#include <memory>

#include <iostream>

struct TrackPoint
{
    int64_t timestamp;      ///< timestamp [ms]

    double x;               ///< local x [m]
    double y;               ///< local y [m]
    double floor;           ///< floor number [-]
    double cov_xy[2][2];    ///< local coordinates covarinace matrix [m^2]
    double std_floor;       ///< floor number std [-]
    double h;         ///< frames missaligment [rad] // reserved

    double qi2l[4];

    bool valid;

    MagMeasurements <PF::Float> mag;
    MagMeasurements <PF::Float> bias;
};

const size_t k_default_history_length = 1; // 
const double k_default_pos_threshold = 3.; //m
const size_t k_default_lucky_cell_count = 10; // 

class LikelihoodPositionEstimator
{
public:
    LikelihoodPositionEstimator() : LikelihoodPositionEstimator(k_default_history_length, k_default_pos_threshold, k_default_lucky_cell_count) {};
    LikelihoodPositionEstimator(
        const size_t history_length,
        const double pos_threshold,
        const size_t lucky_cell_count) :
        history_length_(history_length), pos_threshold_(pos_threshold), lucky_cell_count_(lucky_cell_count), log_stream_(nullptr)
    {
        heading_estimation_enable = false;
    };

    ~LikelihoodPositionEstimator() 
    {
        // tmp debug output
        if (log_stream_ != nullptr)
            delete  log_stream_;
    };
        
    void set_pos_threshold(double pos_threshold)
    {
        pos_threshold_ = pos_threshold;
    }

    void set_history_length(size_t history_length)
    {
        history_length_ = history_length;
    }

    void set_log_stream(std::basic_ostream<char> *log_stream)
    {
        log_stream_ = log_stream;
    }

    void set_heading_estimation_enable(bool enable)
    {
        heading_estimation_enable = enable;
    }

    void ClearTrackHistory()
    {
        lhp_history_.clear();
    }
    
    void  AssignLkhCells(const unsigned long requested_cell_cnt)
    {
        if (max_cell_cnt != requested_cell_cnt)
        {
            max_cell_cnt = requested_cell_cnt;
            cells_list.assign(max_cell_cnt, tMFP_Particle3D());
            lh_cell_list.assign(max_cell_cnt, PF::Particle());
        }
    }

    void ProcessTrackPoint(const TrackPoint &track_point, const tMFP * pMfp, const PF::Particle& particle, std::size_t &cell_number);
    //void ProcessTrackPoint(const TrackPoint &track_point, const tMFP * pMfp, const PF::Particle& particle, std::vector <PF::Particle> &lh_cell_list);

    PositionWithUncertainties GetLikelihoodPosition();

    std::vector <PF::Particle>     lh_cell_list;

private:
    void GetFingerprintCellList(const TrackPoint &track_point, const tMFP * pMfp,  std::size_t &cell_cnt);
    double CalcLikelihoodForCell(
        tMFP_Particle3D cell,
        const MagMeasurements <PF::Float> mag_meas,
        const PF::Particle& particle,
        const double *quat, double h);
    double CalcLikelihoodForCell_v2(
        tMFP_Particle3D cell,
        const Eigen::Matrix<PF::Float, 3, 1> &mag_meas,
        const Eigen::Matrix<PF::Float, 3, 1> &mag_bias,
        const Eigen::Matrix<PF::Float, 3, 3> &mag_bias_cov,
        const Eigen::Quaternion<PF::Float> &quat,
        double h);

    void SelectLuckyCells(std::vector <PF::Particle> &lh_cell_list, std::size_t &cell_number, size_t lucky_cell_count);
    PositionWithUncertainties EstimateLikelyhoodPosition(int64_t timestamp, const std::vector <PF::Particle> & lh_cell_list, std::size_t &cell_number);

    /**
    * The function calculates frame missalignment angle (h) by solving rough minimization equation: arg(min(C(h)*Ci2l*(B-bm)*mu')) - minimal calculation consumption
    * C(h) = {{cos(h)  -sin(h)  0} ; {sin(h)  cos(h)  0}; {0 0 1}
    * B - mag measurement; bm - mag bias; mu - magnetic vector from MFP
    * param[in] cell - MFP cell instance
    * param[out] mag_meas - magnetic meassurement vector
    * param[in] particle - particle instance
    * param[in] qi2l - instrumental to local frame transformation quaternion
    * param[out] h_opt - solution of arg(min(C(h)*Ci2l*(B-bm)*mu'))
    * return value of LKH function for h_opt
    */
    double EstimateFMA_byEquation(   tMFP_Particle3D cell, const MagMeasurements <PF::Float> mag_meas, const PF::Particle& particle, const double *qi2l, double *h_opt);
    
    /**
    * The function calculates frame missalignment angle (h) solving of minimization equation: arg(min((C(h)*betta-mu)'*inv(Dbf)*(C(h)*betta-mu)))
    * C(h) = {{cos(h)  -sin(h)  0} ; {sin(h)  cos(h)  0}; {0 0 1}
    * betta = Ci2l*(B-bm)
    * B - mag measurement; bm - mag bias; mu - magnetic vector from MFP
    * Dbm =  Ci2l*S Ci2l' + Dm;     S - mag bias covariace matrix from RBF;     Dm - MFP cell covariance matrix
    * param[in] cell - MFP cell instance
    * param[out] mag_meas - magnetic meassurement vector
    * param[in] particle - particle instance
    * param[in] qi2l - instrumental to local frame transformation quaternion
    * param[out] h_opt - solution of arg(min((C(h)*betta-mu)'*inv(Dbf)*(C(h)*betta-mu)))
    * return value of LKH function for h_opt
    */
    double EstimateFMA_byEquation_v2(tMFP_Particle3D cell, const MagMeasurements <PF::Float> mag_meas, const PF::Particle& particle, const double *qi2l, double *h_opt);
    
    /**
    * The function calculates frame missalignment angle (h) by solving of minimization equation by sampling method - huge calculation consumption
    * minimization equation : arg(min((C(h)*betta-mu)'*inv(Dbf)*(C(h)*betta-mu)))
    * C(h) = {{cos(h)  -sin(h)  0} ; {sin(h)  cos(h)  0}; {0 0 1}
    * betta = Ci2l*(B-bm)
    * B - mag measurement; bm - mag bias; mu - magnetic vector from MFP
    * Dbm =  Ci2l*S Ci2l' + Dm;     S - mag bias covariace matrix from RBF;     Dm - MFP cell covariance matrix
    * param[in] cell - MFP cell instance
    * param[out] mag_meas - magnetic meassurement vector
    * param[in] particle - particle instance
    * param[in] qi2l - instrumental to local frame transformation quaternion
    * param[out] h_opt - solution of arg(min((C(h)*betta-mu)'*inv(Dbf)*(C(h)*betta-mu)))
    * return value of LKH function for h_opt
    */
    double EstimateFMA_bySampling(   tMFP_Particle3D cell, const MagMeasurements <PF::Float> mag_meas, const PF::Particle& particle, const double *qi2l, double *h_opt);
    
    double CalcDistance(PositionWithUncertainties pos1, PositionWithUncertainties pos2);
    
    void PrintCellData(const TrackPoint &track_point, const tMFP_Particle3D &cell, double h, double lh);

private:
    size_t history_length_;
    double pos_threshold_;

    size_t lucky_cell_count_;

    bool heading_estimation_enable;

    std::list <PositionWithUncertainties> lhp_history_;
    std::basic_ostream<char> *log_stream_;
    
    std::vector <tMFP_Particle3D > cells_list;
    unsigned long  max_cell_cnt = 0;

};

#endif
