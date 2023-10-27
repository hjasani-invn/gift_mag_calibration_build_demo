#include <fstream>
#include <iomanip>
#include <string>
#include <stdint.h>
#include <list>
#include <vector>
#include "pdr_rt/matrix.h"
#include "TrackData2AttitudeData.h"


Matrix GetFullAttMatrix(const Matrix &Cp2b, double heading)
{
    Matrix Cp2n;
    Matrix C3;
    double cos_psi = cos(heading);
    double sin_psi = sin(heading);

    CreateMatrix(&C3, 3, 3);
    /*
    C3.Matr[0][0] = cos_psi;    C3.Matr[0][1] = sin_psi;    C3.Matr[0][2] = 0;
    C3.Matr[1][0] = -sin_psi;   C3.Matr[1][1] = cos_psi;    C3.Matr[1][2] = 0;
    C3.Matr[2][0] = 0;          C3.Matr[2][1] = 0;          C3.Matr[2][2] = 1;
    */
    // This notation according to TrackingDataToStaticData
    C3.Matr[0][0] = cos_psi;    C3.Matr[0][1] = -sin_psi;   C3.Matr[0][2] = 0;
    C3.Matr[1][0] = sin_psi;    C3.Matr[1][1] = cos_psi;    C3.Matr[1][2] = 0;
    C3.Matr[2][0] = 0;          C3.Matr[2][1] = 0;          C3.Matr[2][2] = 1;
    
    MultMatrix(&Cp2n, &C3, (Matrix*)&Cp2b);
    
    return Cp2n;
}

bool DCM2Quaternion(const Matrix &dcm, double q[4])
{
    bool result = true;

    // easy way for spesial ortogonal matrixes
    double qw = 1 + dcm.Matr[0][0] + dcm.Matr[1][1] + dcm.Matr[2][2];
    if (qw > 1.e-6)
    {
        q[0] = sqrt(qw) / 2;
        q[1] = (dcm.Matr[2][1] - dcm.Matr[1][2]) / (4 * q[0]);
        q[2] = (dcm.Matr[0][2] - dcm.Matr[2][0]) / (4 * q[0]);
        q[3] = (dcm.Matr[1][0] - dcm.Matr[0][1]) / (4 * q[0]);
    }
    else
        result = false;

    if (abs(sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]) - 1.) > 1.e-6)
        result = false;

    return result;
}


TrackLineType ParseTrackLine(const std::string str, TrackLine *track_line )
{
    char tag[100];
    TrackLine tmp;
    //bool valid = false;
    TrackLineType  line_type = eUnused;

    if (str.size() > 0)
    {
        //valid = false;
        int cnt = sscanf(str.c_str(), " %10s  %lf %lf %lf %lf  %I64d %I64d  %lf %lf  %d",
            tag,
            &tmp.x1, &tmp.y1,
            &tmp.x2, &tmp.y2,
            &tmp.st1,
            &tmp.st2,
            &tmp.pitch,
            &tmp.roll,
            &tmp.att_flag);
        //line 2008 510 2007 452 1453892952207 1453892952967 -0.2844276203412884 -0.03182967211552228 2
        if ((cnt == 10) && (!strcmp(tag, "line")))
        {
            //valid = true;
            line_type = eLine;
            tmp.x1 /= 100.;        tmp.y1 /= 100.;
            tmp.x2 /= 100.;        tmp.y2 /= 100.;
        }
        //setfloor 0// or 1 and more
        if ((cnt >= 2) && (!strcmp(tag, "setfloor")))
        {
            //valid = true;
            line_type = eFloor;
            tmp.floor = (int16_t)tmp.x1;
            tmp.x1 = 0.;        tmp.y1 = 0.;
            tmp.x2 = 0.;        tmp.y2 = 0.;
        }
    }
    if (line_type != eUnused)
        *track_line = tmp;
    return line_type;
}


void SaveAttPosData(std::ofstream &os, int64_t sys_t, double lattitude, double longitude, int16_t floor, double q[4])
{
    const double default_sigma_q = 0.123456;
    const double altitude = 0.01;        /**< altitude (sea level?) [m] */
    const double default_sigma_lat = (1.5e-7); 
    const double altitudeStd = 0.04;
    const double floorStd = 0.0004;        /**< altitude standard deviation [floor] */
    const int posValid = 1;
    const int attValid = 1;
    
    os << sys_t;
    os << " , " << posValid;
    os.precision(12);
    os << std::fixed;
    os << " , " << lattitude << " , " << longitude;
    os.precision(3);
    os << " , " << altitude;
    os << " , " << floor;
    os.precision(3);
    os << std::scientific;
    for (int i = 0; i < 2; i++) // covarianceLatLon[2][2]
        for (int j = 0; j < 2; j++)
            os << " , " << ((i == j) ? (default_sigma_lat*default_sigma_lat) : (0.0)); // quaternion covariance matrix
    os << std::fixed;
    os << " , " << altitudeStd;
    os << " , " << floorStd;

    os << "   , " << attValid;
    os.precision(12);
    for (int i = 0; i < 4; i++)  os << " , " << q[i]; // quaternion
    os.precision(3);
    os << std::scientific;
#if (OUT_QUATERNION_DISPERSION_ONLY)
    for (int i = 0; i < 4; i++)  os << " , " << (default_sigma_q*default_sigma_q); // quaternion dispersion
#else
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            os << " , " << ((i == j) ? (default_sigma_q*default_sigma_q) : (0.)); // quaternion covariance matrix
#endif
    os << '\n';
}

void SaveAttIncData(std::ofstream &os, int64_t sys_t, double dx, double dy, int16_t floor, double q[4], double mg[3])
{
    const double default_sigma_q = 0.123456;
    const double altitude = 0.00;        /**< altitude (sea level?) [m] */
    const double default_sigma_lat = (1.5e-7);
    const double altitudeStd = 0.04;
    const double floorStd = 0.0004;        /**< altitude standard deviation [floor] */
    const int posValid = 1;
    const int attValid = 1;

    os << sys_t;
    os.precision(3);
    os << std::fixed;
    os << " , " << dx << " , " << dy;
    os.precision(3);
    os << " , " << altitude;
        os.precision(3);
    os << std::scientific;
    for (int i = 0; i < 2; i++) // covarianceLatLon[2][2]
    for (int j = 0; j < 2; j++)
        os << " , " << ((i == j) ? (default_sigma_lat*default_sigma_lat) : (0.0)); // quaternion covariance matrix

    os << std::fixed;
    
    os.precision(12);
    for (int i = 0; i < 4; i++)  os << " , " << q[i]; // quaternion
    os.precision(3);
    os << std::scientific;
#if (OUT_QUATERNION_DISPERSION_ONLY)
    for (int i = 0; i < 4; i++)  os << " , " << (default_sigma_q*default_sigma_q); // quaternion dispersion
#else
    for (int i = 0; i < 4; i++)
    for (int j = 0; j < 4; j++)
        os << " , " << ((i == j) ? (default_sigma_q*default_sigma_q) : (0.)); // quaternion covariance matrix
#endif
    
    os << std::fixed;
    os << " , " << 1; // valid

    os << " , " << mg[0] << " , " << mg[1] << " , " << mg[2]; 
    os << '\n';
}


void SaveSensorData(std::ofstream &os, int64_t time_tag, int64_t call_counter, double sensor_data[3])
{
    os << time_tag << ", " << call_counter;
    os << ", " << sensor_data[0] << ", " << sensor_data[1] << ", " << sensor_data[2];
    os << std::endl;
}

void InterpolateHeading( TrackLine step, double &heading)
{
    heading = atan2((step.y2 - step.y1), (step.x2 - step.x1));    // calculate heading angle
}

void InterpolatePos(int64_t st, TrackLine step, double &x, double &y)
{
    x = step.x1 + (step.x2 - step.x1) * (st - step.st1) / (step.st2 - step.st1);
    y = step.y1 + (step.y2 - step.y1) * (st - step.st1) / (step.st2 - step.st1);
}


std::list<TrackLine> LoadTrackData(std::string TrackDataFileName)
{
    std::ifstream is;
    std::list<TrackLine> Track;

    Track.clear();

    // load track data
    is.open(TrackDataFileName.c_str(), std::ios::in);
    if (is.bad())   std::cout << "Can't open specified track file " << TrackDataFileName << std::endl;
    bool result = is.is_open();
    if (result)
    {
        int16_t current_floor = 0;
        while (!is.eof())
        {
            TrackLine track_line;
            std::string str;
            std::getline(is, str);
            //if (ParseTrackLine(str, &track_line))
            //    Track.insert(Track.end(), track_line);

            TrackLineType  line_type = ParseTrackLine(str, &track_line);
            if (line_type == eFloor)
                current_floor = track_line.floor;
            if (line_type == eLine)
            {
                track_line.floor = current_floor;
                Track.insert(Track.end(), track_line);
            }
        }
    }
    if (is.is_open()) is.close();

    return Track;
}
/*
{
    // generate time scale 
    std::vector<int64_t> sys_time;
    std::string cmd_param;
    if (CmdLine.FindCmd("ref_time_scale", cmd_param))
    {// generate from reference file
        is.open(cmd_param, std::ios::in);
        if (is.bad())   std::cout << "Can't open specified file " << cmd_param << std::endl;
        if (result = result && !is.bad())
            while (!is.eof())
            {
                std::string str;
                std::getline(is, str);
                int64_t time_tag;
                if (sscanf(str.c_str(), "%I64d", &time_tag) == 1)
                    sys_time.insert(sys_time.end(), time_tag);
                else
                    result = false;
            }
        if (is.is_open()) is.close();
    }
    else if (CmdLine.FindCmd("time_perion", cmd_param))
    {// generate with defined period
        int tau;
        sys_time.insert(sys_time.end(), Track.begin()->st1);
        if (sscanf(cmd_param.c_str(), "%d", &tau) == 1)
            while ((--Track.end())->st2 > (*(--sys_time.end()) + tau))
                sys_time.insert(sys_time.end(), *(--sys_time.end()) + tau);
        else
        {
            std::cout << "Incorect parameter in ""time_perion"" command" << std::endl;
            result = false;
        }
    }
    else
    {// generate from track
        for (std::list<TrackLine>::iterator item = Track.begin(); item != Track.end(); item++)
            sys_time.insert(sys_time.end(), item->st1);
    }

    //interpolate convert & output data
    os.open(AttitudeDataFileName.c_str(), std::ios::out);
    if (os.bad())   std::cout << "Can't open output attitude file " << AttitudeDataFileName << std::endl;
    if (result = result && !os.bad())
    {
        std::list<TrackLine>::iterator track_item = Track.begin();
        track_item++; // second step
        for (std::vector<int64_t> ::iterator st = sys_time.begin(); st != sys_time.end(); st++)
        {
            if (*st > track_item->st2)
                if ((track_item++) == Track.end())  break;
            
            std::list<TrackLine>::iterator prev_track_item = track_item;
            prev_track_item--;
            
            double heading = 0, pitch, roll;
#if !OUT_PITCH_ROLL_ONLY
            InterpolateHeading(*track_item, heading);
#endif
            InterpolatePichRoll(*st, *prev_track_item, *track_item, pitch, roll);
            
            double q[4];
            EylerAngles2Quaternion(roll, pitch, heading, q);

            //SaveAttData(os, *st, q, pitch, roll);
            SaveAttPosData(os, *st, track_item->x1*(6.28 / 4e7), track_item->y1*(6.28 / 4e7), track_item->floor, q);
        }
    }
    if (os.is_open()) os.close();

    return result;
}
*/
