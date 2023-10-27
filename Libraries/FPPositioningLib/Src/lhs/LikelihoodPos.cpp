/**
* \copyright       Copyright (C) TDK-Invensense, 2017
* \brief           Likelyhood position calculation
* \defgroup        LikelihoodSampling
* \file            LikelihoodPos.cpp
* \author          D Churikov
* \date            8.06.2017
*/
#include "LikelihoodPos.hpp"
#include <iostream>
#include <algorithm>
#include <functional>
#include <vector>
#include <list>
#include <utility>

int64_t cnt = 0;

//-----------------------------------------------------------------------------
double LikelihoodPositionEstimator::CalcLikelihoodForCell(
    tMFP_Particle3D cell,
    const MagMeasurements <PF::Float> mag_meas,
    const PF::Particle& particle,
    const double *quat,
    double h)
{
#define LOCAL_DEBUG_OUTPUT 0

    Eigen::Quaternion<PF::Float> qi2l(quat[0], quat[1], quat[2], quat[3]); //instrumental to local

    Eigen::Matrix<PF::Float, 3, 3> Ci2l = qi2l.toRotationMatrix();
    Eigen::Matrix<PF::Float, 3, 3> Rz = Eigen::Matrix<PF::Float, 3, 3>::Zero();
    
    PF::Float cos_h = cos(h);
    PF::Float sin_h = sin(h);
    Rz(2, 2) = 1;
    Rz(0, 0) = cos_h; Rz(0, 1) = -sin_h;
    Rz(1, 0) = sin_h; Rz(1, 1) = cos_h;
    
    Eigen::Matrix<PF::Float, 3, 3> C = Rz * Ci2l;
    Eigen::Matrix<PF::Float, 3, 3> C_inv = C.transpose();//C.inverse();
    Eigen::Matrix<PF::Float, 3, 1> map_mag(cell.v[0], cell.v[1], cell.v[2]);
    
    //Eigen::Matrix<PF::Float, 3, 1> delta_b = mag_meas.m - particle.rbf.b - C_inv * map_mag;
    Eigen::Matrix<PF::Float, 3, 1> delta_b;
    delta_b(0, 0) = mag_meas.m(0, 0) - particle.rbf.b(0, 0) - (C_inv(0, 0)*cell.v[0] + C_inv(0, 1)*cell.v[1] + C_inv(0, 2)*cell.v[2]);
    delta_b(1, 0) = mag_meas.m(1, 0) - particle.rbf.b(1, 0) - (C_inv(1, 0)*cell.v[0] + C_inv(1, 1)*cell.v[1] + C_inv(1, 2)*cell.v[2]);
    delta_b(2, 0) = mag_meas.m(2, 0) - particle.rbf.b(2, 0) - (C_inv(2, 0)*cell.v[0] + C_inv(2, 1)*cell.v[1] + C_inv(2, 2)*cell.v[2]);

    Eigen::Matrix<PF::Float, 3, 3> Dm = Eigen::Matrix<PF::Float, 3, 3>::Zero();
	double D_min = 10, s2_m = 1;
	Dm(0, 0) = std::max(cell.s[0], D_min);   Dm(1, 1) = std::max(cell.s[1], D_min);   Dm(2, 2) = std::max(cell.s[2], D_min);
	Eigen::Matrix<PF::Float, 3, 3> Sm = s2_m * Eigen::Matrix<PF::Float, 3, 3>::Identity(); //measurement covariance matrix
    //Dm(0, 0) = cell.s[0];   Dm(1, 1) = cell.s[1];   Dm(2, 2) = cell.s[2];
	Eigen::Matrix<PF::Float, 3, 3> S = C_inv * Dm * C + particle.rbf.P + Sm;//C_inv*Dm*C_inv.transpose() + particle.rbf.P;

    //double dm_det = S.determinant();
    double dm_det  =  S(0, 0) * S(1, 1) * S(2, 2) + S(0, 1) * S(1, 2) * S(2, 0) + S(1, 0) * S(2, 1) * S(0, 2);
    dm_det += -S(2, 0) * S(1, 1) * S(0, 2) - S(2, 1) * S(1, 2) * S(0, 0) - S(1, 0) * S(2, 2) * S(0, 1);

    double cov_mag = delta_b.transpose() * S.inverse() * delta_b;
    
    if ((cov_mag < 0) && (dm_det <= 0)) assert(0);

    dm_det *= 8.*M_PI*M_PI*M_PI;
    double lh_value = ((cov_mag >= 0) && (dm_det > 0)) ? exp(-cov_mag / 2.) / std::sqrt(dm_det) : std::numeric_limits<double>::min();
    //double lh_value = ((cov_mag >= 0) && (dm_det > 0)) ? exp(-cov_mag) / std::sqrt(dm_det) : std::numeric_limits<double>::min();
    lh_value = std::max(lh_value, std::numeric_limits<double>::min()); // correction for zero exponent values

#if LOCAL_DEBUG_OUTPUT
    std::cout << "\n\n cell pos:" << cell.X << ", " << cell.Y;
    std::cout << "\n mag_meas:" << mag_meas.m(0) << ", " << mag_meas.m(1) << ", " << mag_meas.m(2);
    std::cout << "\n mag_bias:" << particle.rbf.b(0) << ", " << particle.rbf.b(1) << ", " << particle.rbf.b(2);
    std::cout << "\n mag_map:" << map_mag(0) << ", " << map_mag(1) << ", " << map_mag(2);
    std::cout << "\n delta_b:" << delta_b(0) << ", " << delta_b(1) << ", " << delta_b(2);
    std::cout << "\n lh_value:" << lh_value;
#endif

#undef LOCAL_DEBUG_OUTPUT

    return (lh_value);
}


//-----------------------------------------------------------------------------
void LikelihoodPositionEstimator::GetFingerprintCellList(
    const TrackPoint &track_point,
    const tMFP * pMfp,
     std::size_t &cell_cnt)
{
#define LOCAL_DEBUG_OUTPUT 0
    MagMeasurements <PF::Float> mag_meas = track_point.mag;
    //bool ok = (pMfp != 0 && pMfp->getRandomPosCount() > 0); // to do:  !!! utilize this flag 
    if (pMfp == 0)
        return;
    if (pMfp->getRandomPosCount() == 0)
        return;

    // to do more accurate and speed up algoritm of cell list determination
    std::size_t cell_count = pMfp->getRandomPosCount();
    
    double area_radius_square = track_point.cov_xy[0][0] + track_point.cov_xy[1][1]; // to do more correct area bound calculation
    
    double cell_size = pMfp->GetMapCellSize() / 100.; // cell size in meters
    std::size_t max_cell_cnt = size_t(4 * area_radius_square / (cell_size*cell_size) + 4 * std::sqrt(area_radius_square) / cell_size + 1);

    tMFP_Particle3D cell;
#if LOCAL_DEBUG_OUTPUT
    std::cout << "\n ref pos:" << track_point.x << ", " << track_point.y;
    std::cout << "\n (rand_cell_count, area_radius_square, max_cell_cnt):" << cell_count << ", " << area_radius_square << ", " << max_cell_cnt << ", " << max_cell_cnt;
#endif

    std::size_t idx = 0;
    tMFP_CellDescrApprox vec = {};
    for (size_t i = 0; i < cell_count; ++i)
    {
        cell = pMfp->getRandomPos(i);
        
        double cell_X_m = cell.X / 100;
        double cell_Y_m = cell.Y / 100;

        int track_floor = static_cast<int> (std::floor(track_point.floor + 0.5)); // Attention: cells are selected frome NEAREST floor only
        if (max_cell_cnt <= std::size_t(1))
        {
            if ((std::abs(cell_X_m - track_point.x) <= 0.5*cell_size) && (std::abs(cell_Y_m - track_point.y) <= 0.5*cell_size))
            {
                pMfp->GetMgVector(&cell, &vec, 1);
                cell.v[0] = vec.x.mu1;            cell.v[1] = vec.y.mu1;            cell.v[2] = vec.z.mu1;
                const float map_s_limit = 0;
                cell.s[0] = std::max(vec.x.s1*vec.x.s1, map_s_limit);
                cell.s[1] = std::max(vec.y.s1*vec.y.s1, map_s_limit);
                cell.s[2] = std::max(vec.z.s1*vec.z.s1, map_s_limit);
                
                cells_list[0] = cell;
                idx = 1;
     //           cells.push_back(cell);
#if LOCAL_DEBUG_OUTPUT
                std::cout << "\n area pos:" << cell_X_m << ", " << cell_Y_m;
                std::cout << " ;" << cells[0].v[0] << ", " << cells[0].v[1] << ", " << cells[0].v[2];
                std::cout << " ;" << cells[0].s[0] << ", " << cells[0].s[1] << ", " << cells[0].s[2];
#endif
            }
        }
        else
        {
            double distance_square = (cell_X_m - track_point.x)*(cell_X_m - track_point.x) + (cell_Y_m - track_point.y)*(cell_Y_m - track_point.y);
            if ((cell.Floor == track_floor) && (area_radius_square > distance_square))
            {
                if (idx > max_cell_cnt)
                    break;

                pMfp->GetMgVector(&cell, &vec, 1);
                cell.v[0] = vec.x.mu1;            cell.v[1] = vec.y.mu1;            cell.v[2] = vec.z.mu1;
                cell.s[0] = vec.x.s1*vec.x.s1;    cell.s[1] = vec.y.s1*vec.y.s1;    cell.s[2] = vec.z.s1*vec.z.s1;

                cells_list[idx++] = cell;
                //cells.push_back(cell);

#if LOCAL_DEBUG_OUTPUT
                std::cout << "\n area pos:" << cell_X_m << ", " << cell_Y_m;
                std::cout << " ;" << cells[idx-1].v[0] << ", " << cells[idx-1].v[1] << ", " << cells[idx-1].v[2];
                std::cout << " ;" << cells[idx-1].s[0] << ", " << cells[idx-1].s[1] << ", " << cells[idx-1].s[2];
#endif
            }
        }
    }
    //cells.resize(idx);
    cell_cnt = idx;
#undef LOCAL_DEBUG_OUTPUT
}

//-----------------------------------------------------------------------------
void LikelihoodPositionEstimator::SelectLuckyCells(std::vector <PF::Particle> &cell_list, std::size_t &cell_number, size_t lucky_cell_count)
{
    size_t sort_count = std::min(cell_number, lucky_cell_count);
    std::partial_sort(cell_list.begin(), cell_list.begin() + sort_count, cell_list.begin() + cell_number, [](PF::Particle &p1, PF::Particle &p2) { return p1.w > p2.w; });
    cell_number = sort_count;
}

//-----------------------------------------------------------------------------
PositionWithUncertainties LikelihoodPositionEstimator::EstimateLikelyhoodPosition(int64_t timestamp, const std::vector <PF::Particle> & lh_cell_list, std::size_t &cell_number)
{
#define LOCAL_DEBUG_OUTPUT 0
    PositionWithUncertainties lh_position = { 0 };
    
    lh_position.timestamp = timestamp;
    
    if (lh_cell_list.size() > 0)
    {
        double w_sum = 0;
        for (std::size_t i = 0; i < cell_number; i++)
        {
            lh_position.x += lh_cell_list[i].pos.x * lh_cell_list[i].w;
            lh_position.y += lh_cell_list[i].pos.y * lh_cell_list[i].w;
            lh_position.z += lh_cell_list[i].pos.level * lh_cell_list[i].w;
            lh_position.heading += lh_cell_list[i].h *lh_cell_list[i].w;
            w_sum += lh_cell_list[i].w;
        }
        
        if (w_sum > 0)
        {
            lh_position.x /= w_sum;
            lh_position.y /= w_sum;
            lh_position.z /= w_sum;
            lh_position.heading /= w_sum; // to do: more accurate calculation

            double rms = 0;
            for (std::size_t i = 0; i < cell_number; i++)
            {
                rms += (lh_cell_list[i].pos.x - lh_position.x)*(lh_cell_list[i].pos.x - lh_position.x);
                rms += (lh_cell_list[i].pos.y - lh_position.y)*(lh_cell_list[i].pos.y - lh_position.y);
            }
            rms = (cell_number > 0) ? rms / cell_number : 0;

            lh_position.cov_xy[0][0] = rms / 2;
            lh_position.cov_xy[1][1] = rms / 2;
            
            lh_position.valid = true;
        }
    }

#if LOCAL_DEBUG_OUTPUT
    std::cout << "\n lh_position (t,v,x,y,f,h,cx,cy):" << lh_position.timestamp << ", " << lh_position.valid;
    std::cout << ", " << lh_position.x << ", " << lh_position.y << ", " << lh_position.z;
    std::cout << ", " << lh_position.heading << ", " << lh_position.cov_xy[0][0] << ", " << lh_position.cov_xy[1][1];
    std::cout << "\n";
#endif

    return lh_position;
#undef LOCAL_DEBUG_OUTPUT
}

//-----------------------------------------------------------------------------
void LikelihoodPositionEstimator::ProcessTrackPoint(const TrackPoint &track_point, const tMFP * pMfp, const PF::Particle& particle, std::size_t &cell_number)
{
#define LOCAL_DEBUG_OUTPUT 0
    std::size_t cell_cnt = 0;

    cnt++;
#if LOCAL_DEBUG_OUTPUT
    {
        std::cout << "\n trk: " << cnt << ", " << track_point.x << ", " << track_point.y << ", " << track_point.h << ", " << track_point.valid;
        std::cout << ", " << (double)(track_point.timestamp) / 1000;
    }
#endif

    AssignLkhCells(pMfp->GetMapCellsCount());

    if (track_point.valid)
    {
        GetFingerprintCellList(track_point, pMfp, cell_cnt);
        PrintCellData(track_point, tMFP_Particle3D{ track_point.x * 100, track_point.y * 100 }, track_point.h, -1); // print ref point
    }

#if LOCAL_DEBUG_OUTPUT 
    {
        std::cout << "\n Cell list";
        for (auto cell : cell_list)
        {
            std::cout << "\n(x,y,f,mvx3, msx3):" << cell.X << ", " << cell.Y << ", " << cell.Floor;
            std::cout << " ;" << cell.v[0] << ", " << cell.v[1] << ", " << cell.v[2];
            std::cout << " ;" << cell.s[0] << ", " << cell.s[1] << ", " << cell.s[2];
        }
        std::cout << "\n";
    }
#endif
    cell_number = cell_cnt;
    // Calculate Likelihood for all cells
    for (std::size_t i = 0; i < cell_cnt; i++)
    {
        PF::Particle lh_cell = { 0 };
        lh_cell.pos.x = cells_list[i].X / 100;
        lh_cell.pos.y = cells_list[i].Y / 100;
        lh_cell.pos.level = cells_list[i].Floor;
        if (heading_estimation_enable)
        {
            lh_cell.w = EstimateFMA_byEquation_v2(cells_list[i], track_point.mag, particle, track_point.qi2l, &lh_cell.h);
        }
        else
        {
            lh_cell.h = track_point.h; // heading inheritance
            lh_cell.w = CalcLikelihoodForCell(cells_list[i], track_point.mag, particle, track_point.qi2l, lh_cell.h);
        }
        lh_cell_list[i] = lh_cell;

    }

#if LOCAL_DEBUG_OUTPUT
        {
            std::cout << "\n All Cells";
            for (auto lh_cell : lh_cell_list)
            {
                std::cout << "\n(x,y,f,h,lh):" << lh_cell.pos.x << ", " << lh_cell.pos.y << ", " << lh_cell.pos.level;
                std::cout << " ," << lh_cell.h << ", " << lh_cell.w;
                std::cout << ", " << lh_cell.kl << " ," << lh_cell.ph;
            }
            std::cout << "\n";
#endif
            // Select cells with maximal Likelihood
            if (cell_number > 0)
            {
                SelectLuckyCells(lh_cell_list, cell_number, lucky_cell_count_);
            }

#if LOCAL_DEBUG_OUTPUT
    {
        std::cout << "\n Lucky Cells";
        for (auto lh_cell : lh_cell_list)
        {
            std::cout << "\n cell (x,y,f,h,lh):" << lh_cell.pos.x << ", " << lh_cell.pos.y << ", " << lh_cell.pos.level;
            std::cout << lh_cell.h << ", " << lh_cell.w;
            std::cout << ", " << lh_cell.kl << " ," << lh_cell.ph;
        }
        std::cout << "\n";
    }
#endif
    // Estimate Likelyhood position
    PositionWithUncertainties lh_position = EstimateLikelyhoodPosition(track_point.timestamp, lh_cell_list, cell_number);
#if LOCAL_DEBUG_OUTPUT
    {
        std::cout << "\n lh_position (t,v,x,y,f,h,cx,cy):" << lh_position.timestamp << ", " << lh_position.valid;
        std::cout << ", " << lh_position.x << ", " << lh_position.y << ", " << lh_position.z;
        std::cout << ", " << lh_position.heading << ", " << lh_position.cov_xy[0][0] << ", " << lh_position.cov_xy[1][1];
        std::cout << "\n";
    }
#endif
    if (lh_position.valid)
    {
        lhp_history_.push_front(lh_position);
        if (lhp_history_.size() > history_length_)
{
            lhp_history_.pop_back();
        }
#if LOCAL_DEBUG_OUTPUT
        std::cout << "\n\n track_position:" << track_point.x << ", " << track_point.y;
        std::cout << "\n lh_position:" << lh_position.x << ", " << lh_position.y;
        std::cout << "\n";
#endif
}
#undef LOCAL_DEBUG_OUTPUT

}
#if 0
//-----------------------------------------------------------------------------
void LikelihoodPositionEstimator::ProcessTrackPoint(const TrackPoint &track_point, const tMFP * pMfp, const PF::Particle& particle, std::vector <PF::Particle> &lh_cell_list)
{
#define LOCAL_DEBUG_OUTPUT 0
    std::size_t cell_cnt = 0;
    cnt++;
#if LOCAL_DEBUG_OUTPUT
    {
        std::cout << "\n trk: " << cnt << ", " << track_point.x << ", " << track_point.y << ", " << track_point.h << ", " << track_point.valid;
        std::cout << ", " << (double)(track_point.timestamp) / 1000;
    }
#endif

    if (track_point.valid )
    {
        GetFingerprintCellList(track_point, pMfp, cell_cnt);
        PrintCellData(track_point, tMFP_Particle3D{ track_point.x*100, track_point.y*100 }, track_point.h,  - 1); // print ref point
    }

#if LOCAL_DEBUG_OUTPUT 
    {
        std::cout << "\n Cell list";
        for (auto cell: cell_list)
        {
            std::cout << "\n(x,y,f,mvx3, msx3):" << cell.X << ", " << cell.Y << ", " << cell.Floor;
            std::cout << " ;" << cell.v[0] << ", " << cell.v[1] << ", " << cell.v[2];
            std::cout << " ;" << cell.s[0] << ", " << cell.s[1] << ", " << cell.s[2];
        }
        std::cout << "\n";
    }
#endif

    // Calculate Likelihood for all cells
    for (std::size_t i = 0; i < cell_cnt; i++)
    {
        PF::Particle lh_cell = {0};
        lh_cell.pos.x = cells_list[i].X / 100;
        lh_cell.pos.y = cells_list[i].Y / 100;
        lh_cell.pos.level = cells_list[i].Floor;
        if (heading_estimation_enable)
        {
            lh_cell.w = EstimateFMA_byEquation_v2(cells_list[i], track_point.mag, particle, track_point.qi2l, &lh_cell.h); 
        }
        else
        {
            lh_cell.h = track_point.h; // heading inheritance
            lh_cell.w = CalcLikelihoodForCell(cells_list[i], track_point.mag, particle, track_point.qi2l, lh_cell.h);
        }

        lh_cell_list.push_back(lh_cell);
    }

#if LOCAL_DEBUG_OUTPUT
        {
        std::cout << "\n All Cells";
        for (auto lh_cell: lh_cell_list)
            {
            std::cout << "\n(x,y,f,h,lh):" << lh_cell.pos.x << ", " << lh_cell.pos.y << ", " << lh_cell.pos.level;
            std::cout << " ," << lh_cell.h << ", " << lh_cell.w;
            std::cout << ", " << lh_cell.kl << " ," << lh_cell.ph;
        }
        std::cout << "\n";
#endif

    // Select cells with maximal Likelihood
    if (lh_cell_list.size() > 0)
    {
        SelectLuckyCells(lh_cell_list, lucky_cell_count_);
    }

#if LOCAL_DEBUG_OUTPUT
    {
        std::cout << "\n Lucky Cells";
        for (auto lh_cell : lh_cell_list)
            {
            std::cout << "\n cell (x,y,f,h,lh):" << lh_cell.pos.x << ", " << lh_cell.pos.y << ", " << lh_cell.pos.level;
            std::cout << lh_cell.h << ", " << lh_cell.w;
            std::cout << ", " << lh_cell.kl << " ," << lh_cell.ph;
            }
        std::cout << "\n";
        }
#endif
    // Estimate Likelyhood position
    PositionWithUncertainties lh_position = EstimateLikelyhoodPosition(track_point.timestamp, lh_cell_list);
#if LOCAL_DEBUG_OUTPUT
    {
        std::cout << "\n lh_position (t,v,x,y,f,h,cx,cy):" << lh_position.timestamp << ", " << lh_position.valid;
        std::cout << ", " << lh_position.x << ", " << lh_position.y << ", " << lh_position.z;
        std::cout << ", " << lh_position.heading << ", " << lh_position.cov_xy[0][0] << ", " << lh_position.cov_xy[1][1];
        std::cout << "\n";
    }
#endif
    if (lh_position.valid)
    {
        lhp_history_.push_front(lh_position);
        if (lhp_history_.size() > history_length_)
        {
            lhp_history_.pop_back();
        }
#if LOCAL_DEBUG_OUTPUT
        std::cout << "\n\n track_position:" << track_point.x << ", " << track_point.y;
        std::cout << "\n lh_position:" << lh_position.x << ", " << lh_position.y;
        std::cout << "\n";
#endif
    }
#undef LOCAL_DEBUG_OUTPUT
}
#endif
//-----------------------------------------------------------------------------
double LikelihoodPositionEstimator::CalcDistance(
    PositionWithUncertainties pos1,
    PositionWithUncertainties pos2)
{
    return sqrt((pos1.x - pos2.x)*(pos1.x - pos2.x) + (pos1.y - pos2.y)*(pos1.y - pos2.y));
}

//-----------------------------------------------------------------------------
PositionWithUncertainties LikelihoodPositionEstimator::GetLikelihoodPosition()
{
#define LOCAL_DEBUG_OUTPUT 0
    PositionWithUncertainties lh_position = { 0 };
    
    if (lhp_history_.size() > 0)
    {
#if LOCAL_DEBUG_OUTPUT
        std::cout << "\n lh_pos distance list(" << lhp_history_.size()-1 << "):";
        int count(0);
#endif
        double max_distance = -1;
        lh_position = lhp_history_.front();
        for (auto lhp_point = lhp_history_.begin(), __lhp_point = lhp_history_.begin(); lhp_point != lhp_history_.end(); lhp_point++)
        {
            double distance = CalcDistance(*__lhp_point, *lhp_point);
#if LOCAL_DEBUG_OUTPUT
            if (count++ > 0)    std::cout << "; " << distance;
#endif
            max_distance = std::max(distance, max_distance);
            __lhp_point = lhp_point;
        }
        lh_position.valid = ((max_distance >= 0) && (max_distance < pos_threshold_)) || (history_length_ == 1);
#if LOCAL_DEBUG_OUTPUT
        std::cout << "\n lh_position_out:" << lh_position.x << ", " << lh_position.y << "; valid=" << (lh_position.valid ? 1 : 0) << "; max distance="<< max_distance;
        std::cout << "\n";
#endif
    }
    
#undef LOCAL_DEBUG_OUTPUT
    return lh_position;
}

//-----------------------------------------------------------------------------
void LikelihoodPositionEstimator::PrintCellData(const TrackPoint &track_point, const tMFP_Particle3D &cell, double h, double lh)
{
    if (log_stream_ != nullptr)
    {
        //(*log_stream_) << "lhp_cell_data: ";
        (*log_stream_) << track_point.timestamp;
        (*log_stream_) << ", " << cell.X/100;
        (*log_stream_) << ", " << cell.Y/100;
        (*log_stream_) << ", " << h;
        (*log_stream_) << ", " << lh;
        (*log_stream_) << std::endl;
    }
}

//-----------------------------------------------------------------------------
double LikelihoodPositionEstimator::EstimateFMA_byEquation(tMFP_Particle3D cell, const MagMeasurements <PF::Float> mag_meas, const PF::Particle& particle, const double *quat, double *h_opt)
{
#define LOCAL_DEBUG_OUTPUT 0
    Eigen::Quaternion<PF::Float> qi2l(quat[0], quat[1], quat[2], quat[3]); //instrumental to local
    Eigen::Matrix<PF::Float, 3, 3> Ci2l = qi2l.toRotationMatrix();

    
    //Eigen::Matrix<PF::Float, 3, 1> betta = Ci2l*(mag_meas.m - mag_bias.m);// TO DO: use quaternion insted matrix
    Eigen::Matrix<PF::Float, 3, 1> betta = Ci2l*(mag_meas.m - particle.rbf.b);// TO DO: use quaternion insted matrix
    Eigen::Matrix<PF::Float, 3, 1> mu(cell.v[0], cell.v[1], cell.v[2]);

    auto a = mu(1) * betta(0) - mu(0) * betta(1);
    auto b = mu(0) * betta(0) + mu(1) * betta(1);

    auto max_test = [](double h, double a, double b)
    {
        return (bool)((b*cos(h)) > (-a*sin(h)));
    };

    double h = std::atan2(a, b);
    if (false == max_test(h, a, b))
    {
        h += M_PI;
    }

    // ranging
    const double k_2pi = M_PI + M_PI;
    *h_opt = (h > k_2pi) ? (h - k_2pi) : ((h < 0) ? (h + k_2pi) : h);
    double w = CalcLikelihoodForCell(cell, mag_meas, particle, quat, h);

#if LOCAL_DEBUG_OUTPUT
    std::cout << "\n\n cell pos:" << cell.X << ", " << cell.Y;
    std::cout << "\n mag_map:" << mu[0] << ", " << mu[1] << ", " << mu[2];
    std::cout << "\n mag_meas:" << mag_meas.m(0) << ", " << mag_meas.m(1) << ", " << mag_meas.m(2);
    std::cout << "\n mag_bias:" << particle.rbf.b(0) << ", " << particle.rbf.b(1) << ", " << particle.rbf.b(2);
    std::cout << "\n betta:" << betta(0) << ", " << betta(1) << ", " << betta(2);
    std::cout << "\n a,b:" << a << ", " << b;
    std::cout << "\n h0,h1,test0,test1,h:" << h << ", " << h + M_PI << ", " << max_test(h, a, b) << ", " << max_test(h + M_PI, a, b) << ", " << *h_opt;
    std::cout << "\n h_opt, lh_max:" << *h_opt << ", " << w;
#endif

#undef LOCAL_DEBUG_OUTPUT

    return (w);
}


double LikelihoodPositionEstimator::EstimateFMA_byEquation_v2(tMFP_Particle3D cell, const MagMeasurements <PF::Float> mag_meas, const PF::Particle& particle, const double *quat, double *h_opt)
{
#define LOCAL_DEBUG_OUTPUT 0
   
    Eigen::Quaternion<PF::Float> qi2l(quat[0], quat[1], quat[2], quat[3]); //instrumental to local
    Eigen::Matrix<PF::Float, 3, 3> Ci2l = qi2l.toRotationMatrix();

    Eigen::Matrix<PF::Float, 3, 1> betta = Ci2l*(mag_meas.m - particle.rbf.b);
    Eigen::Matrix<PF::Float, 3, 1> mu(cell.v[0], cell.v[1], cell.v[2]);

    Eigen::Matrix<PF::Float, 3, 3> Dm = Eigen::Matrix<PF::Float, 3, 3>::Zero();
    Dm(0, 0) = cell.s[0];   Dm(1, 1) = cell.s[1];   Dm(2, 2) = cell.s[2];
    Eigen::Matrix<PF::Float, 3, 3> Dbf = Dm + Ci2l * particle.rbf.P * Ci2l.transpose();
    Eigen::Matrix<PF::Float, 3, 3> Dbf_inv = Dbf.inverse();

    PF::Float dxx = Dbf_inv(0, 0), dyy = Dbf_inv(1, 1), dzz = Dbf_inv(2, 2);

    auto a =      (betta(1)*betta(1) - betta(0)*betta(0))*(dxx - dyy);
    auto b = 2. * betta(1)*betta(0) * (dyy - dxx);
    auto c = 2. * (mu(0) * betta(0) * dxx + mu(1) * betta(1) * dyy);
    auto d = 2. * (mu(0) * betta(1) * dxx - mu(1) * betta(0) * dyy);

#if LOCAL_DEBUG_OUTPUT
    std::cout << "\n\n cell pos:" << cell.X << ", " << cell.Y;
    std::cout << "\n mag_meas:" << mag_meas.m(0) << ", " << mag_meas.m(1) << ", " << mag_meas.m(2);
    std::cout << "\n mag_bias:" << particle.rbf.b(0) << ", " << particle.rbf.b(1) << ", " << particle.rbf.b(2);
    std::cout << "\n betta:" << betta(0) << ", " << betta(1) << ", " << betta(2);
    std::cout << "\n mag_map:" << mu[0] << ", " << mu[1] << ", " << mu[2];
    std::cout << "\n Dm:" << Dm;
    std::cout << "\n rbf.P:" << particle.rbf.P;
    std::cout << "\n Ci2l:" << Ci2l;
    std::cout << "\n Dbf:" << Dbf;
    std::cout << "\n Dbf_inv:" << Dbf_inv;
    std::cout << "\n a,b,c,d:" << a << ", " << b << ", " << c << ", " << d;
#endif


    double h = std::atan2(-d, c); // rough estimation of h

    // lamda function for digital solving of optimization equation by Newton method
    auto root_f = [](PF::Float h0, PF::Float a, PF::Float b, double c, PF::Float d)
    {
        // lamda function to calculete equation: (a*sin(2.*h) + b*cos(2.*h) + c*sin(h) + d*cos(h))
        auto f = [](PF::Float sinh, PF::Float cosh, PF::Float a, PF::Float b, PF::Float c, PF::Float d)
        {
            PF::Float tm1 = (2*a*cosh - b*sinh + c)*sinh;
            PF::Float tm2 = (b*cosh + d)*cosh;
            return tm1 + tm2;
        };

        // lamda function to calculete equation: (2.*a*cos(2.*h) - 2.*b*sin(2.*h) + c*cos(h) - d*sin(h))
        auto df = [](PF::Float sinh, PF::Float cosh, PF::Float a, PF::Float b, PF::Float c, PF::Float d)
        {
            PF::Float tm1 = (2.*(a*cosh - b*sinh) + c)*cosh;
            PF::Float tm2 = (-2.*a*sinh - d)*sinh;
            return tm1 + tm2;
        };

        PF::Float h(h0), dh(1.), dfn(1);
        int32_t n = 0;
        while ((std::abs(dh) > 0.001) && (std::abs(dfn) > 1.e-6) && (++n < 10))
        {
            PF::Float sinh = sin(h), cosh = cos(h);
            PF::Float fn = f(sinh, cosh, a, b, c, d);
            dfn = df(sinh, cosh, a, b, c, d);
            dh = fn / dfn;
            dh = std::max(std::min(dh, 0.5 / n), -0.5 / n);
            h -= dh;
        }

        return h;
    };

    // lamda function for calculation of exponent argument of Likelyhood function
    auto F = [](PF::Float h, const Eigen::Matrix<PF::Float, 3, 1> &betta, const Eigen::Matrix<PF::Float, 3, 1> &mu, PF::Float dxx, PF::Float dyy)
    {
        PF::Float sinh = sin(h);
        PF::Float cosh = cos(h);
        PF::Float tm1 = betta(0)*cosh - betta(1)*sinh;
        PF::Float tm2 = betta(0)*sinh + betta(1)*cosh;

        return dxx*tm1*(tm1 - 2 * mu(0)) + dyy*tm2*(tm2 - 2 * mu(1));
    };

    std::pair< PF::Float, PF::Float> extremum_min;
    extremum_min.first = root_f(h, a, b, c, d);
    extremum_min.second = F(extremum_min.first, betta, mu, dxx, dyy);

    for (int i = 1; i <= 3; i++)
    {
        std::pair< PF::Float, PF::Float> extremum;
        extremum.first = root_f(h + i*M_PI / 2, a, b, c, d);
        extremum.second = F(extremum.first, betta, mu, dxx, dyy);

        if (extremum.second < extremum_min.second)
        {
            extremum_min = extremum;
        }
    }
    // ranging
    const double k_2pi = M_PI + M_PI;
    *h_opt = extremum_min.first;
    *h_opt = (*h_opt > k_2pi) ? (*h_opt - k_2pi) : ((*h_opt < 0) ? (*h_opt + k_2pi) : *h_opt);

    double w = CalcLikelihoodForCell(cell, mag_meas, particle, quat, *h_opt);

#if LOCAL_DEBUG_OUTPUT
    std::cout << "\n h_opt, w: " << *h_opt << ", " << CalcLikelihoodForCell(cell, mag_meas, particle, quat, *h_opt);
    std::cout << std::endl;
#endif

#undef LOCAL_DEBUG_OUTPUT

    return (w);
}

//-----------------------------------------------------------------------------
double LikelihoodPositionEstimator::EstimateFMA_bySampling(
    tMFP_Particle3D cell,
    const MagMeasurements <PF::Float> mag_meas,
    const PF::Particle& particle,
    const double *quat,
    double *h_opt)
{
#define LOCAL_DEBUG_OUTPUT 0
    const double h_step = 2 * M_PI / 32;  //2 * M_PI / 1024;
    double w_max(-1);
    
    *h_opt = 0;

#if LOCAL_DEBUG_OUTPUT
    std::cout << "\n\n cell pos:" << cell.X << ", " << cell.Y;
    std::cout << "\n mag_map:" << cell.v[0] << ", " << cell.v[1] << ", " << cell.v[2];
    std::cout << "\n mag_meas:" << mag_meas.m(0) << ", " << mag_meas.m(1) << ", " << mag_meas.m(2);
    std::cout << "\n mag_bias:" << particle.rbf.b(0) << ", " << particle.rbf.b(1) << ", " << particle.rbf.b(2);
#endif

    //double max_lh_in_cell = -1;
    for (double h = 0/*, max_lh_in_cell = -1*/; h < (2 * M_PI); h += h_step)
    {
        double w = CalcLikelihoodForCell(cell, mag_meas, particle, quat, h);
        //double lh = CalcLikelihoodForCell(cell, mag_meas, mag_bias, mag_bias_cov, quat, h);

#if LOCAL_DEBUG_OUTPUT
        std::cout << "\n h, lh:" << h << ", " << w;
#endif
        if (w > w_max)
        {
            w_max = w;
            *h_opt = h;
        }
    }
#if LOCAL_DEBUG_OUTPUT
    std::cout << "\n h_opt, lh_max:" << *h_opt << ", " << w_max;
#endif

    return w_max;
}

