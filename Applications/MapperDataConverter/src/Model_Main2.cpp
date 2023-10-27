/*****************************************************************************
 *    Copyright (c) 2013 Spirit Corp.
******************************************************************************/
/**
 *   @project                MapperDataConverter
 *   @brief                  Main 
 *   @file                   Model_Main2.cpp
 *   @author                 D Churikov
 *   @date                   18.02.2016
 *   @version                1.0
 */
/*****************************************************************************/

#include <stdio.h>
#include <crtdbg.h>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <string>
#include <list>
#include <cassert>
#include "Model_Main2.h"
#include "pdr_rt/Init.h"
#include "pdr_rt/PDR.h"
#include "LoadData.h"
#include "pdr_rt/report.h"
#include "pdr_rt/PDRWrap.h"
#include <eigen/Core>

#include "model_config.h"
#include "TrackData2AttitudeData.h"
#include "TrackData2TPNData.h"
#include "tpn_quat_lib.hpp"


#define DEBUG_OUTPUT    0

#ifdef _MSC_VER
#   pragma warning( push )
#   pragma warning( disable : 4201 )
#   include "dirent.h"
#   pragma warning( pop )
#   pragma warning( disable : 4505 )
#else
#   include <dirent.h>
#endif


using namespace std;


#define sqr(x) (x)*(x)

const double ms_per_sec = 1000.;

//FusionFilter *pFF;
static tLoadData LoadData;
std::list<TrackLine> Track;
std::string att_pos_file, att_inc_file, unbias_mg_file, att_pos_en_file, tpn_text_file, irl_text_file;
std::string mag_data_check_report = "mag_check_report.log";
GeoLocConverter2D geo2ff;

void PdrData2RtfpplData(tPFData &PFData, std::ofstream &fTpnData);

std::string GetFileGroupMarker(std::string input_folder_name)
{
    return "mt_converter";
}

void CreateFile(const std::string file_name)
{
    std::fstream ff(file_name, ios_base::out | ios_base::trunc);
    if (!ff || !ff.is_open())
        std::cout << "\nCan't open " << file_name << " file";
    ff.close();
}

int Main_Init(MDConverterSettings converter_settings)
{
    ModelConfig::SetInputPath((char*)converter_settings.get_input_path().c_str());
    ModelConfig::SetOutputPath((char*)converter_settings.get_output_path().c_str());

    std::string file_group_marker = GetFileGroupMarker(ModelConfig::InputPath);
    ReportPDR::DestroyReports();
    ReportPDR::EnableReports( true ); // enable reports
    ReportPDR::InitReportNames(file_group_marker.c_str()); // prevent initialization

    std::string out_path = ModelConfig::GetOutputPath();
    //pFF = new FusionFilter();

#if (ENABLE_OUTPUT == 1)
#if (INPUT_DATA_TYPE == 1)
    CreateFile(att_pos_file = out_path + "\\pos_att_" + file_group_marker + ".log");
    CreateFile(att_inc_file = out_path + "\\posinc_att_" + file_group_marker + ".log");
    CreateFile(unbias_mg_file = out_path + "\\mg_ub_" + file_group_marker + ".log");
    CreateFile(irl_text_file = out_path + "\\irl_data_" + file_group_marker + ".txt");
#else
    CreateFile(tpn_text_file = out_path + "\\tpn_data_" + file_group_marker + ".txt");
#endif
#endif

#if DEBUG_OUTPUT
    CreateFile(att_pos_en_file = out_path + "\\pos_att_en_" + file_group_marker + ".log");
#endif

    memset( &LoadData, 0, sizeof( LoadData ) );
    int result = !GetDataFileNames(ModelConfig::InputPath.c_str(), &LoadData);
    //assert( flag == 0 );
    
    //   ReportPDR::SetOutputPath ( out_path.c_str() );

    ////
//    pFF->initializePDR( out_path.c_str(),  MODEL_PDR_USER_HEIGHT, es_Male, file_name_marker.c_str() );

    int8_t pdr_flag;
    ReportPDR::SetOutputPath(out_path.c_str());

    aw_InitPDR(file_group_marker.c_str());
    aw_InitUserParams((int16_t)MODEL_PDR_USER_HEIGHT, (int8_t)es_Male, &pdr_flag);
    result &= (pdr_flag == 0) ? 1 : 0;
    if (pdr_flag != 0)
        std::cout << "PDR initializarion error." << std::endl;
    
#if (INPUT_DATA_TYPE == 1)
    std::string track_file;
    if (FindFile(ModelConfig::InputPath, "Track*.trk", "track", track_file))
        Track = LoadTrackData(track_file);
    else
        Track.clear();
#else
    Track.clear();
#endif

    // ! To do: get venue data from frame/or json file
    Venue venue = converter_settings.get_venue();
    geo2ff.SetFrameParams(venue.origin_lattitude, venue.origin_longitude, venue.alfa, venue.beta, venue.origin_azimuth);

    return result;
}

#if 1
//=======================================================================
// Quaretnion multiplication
static void quat_mlt(const FLOAT32 first[4], const FLOAT32 second[4], FLOAT32 result[4])
{
    double w1 = first[0];
    double x1 = first[1];
    double y1 = first[2];
    double z1 = first[3];

    double w2 = second[0];
    double x2 = second[1];
    double y2 = second[2];
    double z2 = second[3];

    result[0] = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2;
    result[1] = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2;
    result[2] = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2;
    result[3] = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2;
}
#endif


int Main_Run(MDConverterSettings &converter_settings)
{
    bool pdrEOF = ( LoadDataProcess( &LoadData ) != 0 );
    double *pAcc[3] = {LoadData.accData.data[0], LoadData.accData.data[1], LoadData.accData.data[2]};
    double *pGyr[3] = {LoadData.gyrData.data[0], LoadData.gyrData.data[1], LoadData.gyrData.data[2]};
    double *pMag[3] = {LoadData.magnData.data[0], LoadData.magnData.data[1], LoadData.magnData.data[2]};
    double *pMagBias[3] = {LoadData.magnData.data_bias[0], LoadData.magnData.data_bias[1], LoadData.magnData.data_bias[2]};
    static std::list<TrackLine>::iterator iStep = Track.begin();
    std::list<TrackLine>::iterator iStepPrev = iStep;
    bool flag_stop = false;
    TpnOutput tpn_data = { 0 };
    static TpnOutput tpn_data_prev = { 0 };

    if( pdrEOF == false )
    {
 
        int8_t cntSteps = 0, errFlag, StepsRead;
        double m_b[3];
        m_b[0] = 0.;        m_b[1] = 0.;        m_b[2] = 0.;
        aw_ProcessPDR(  pAcc, LoadData.accData.ts, LoadData.accData.cntLine,
                        pGyr, LoadData.gyrData.ts, LoadData.gyrData.cntLine,
                        pMag, LoadData.magnData.ts, LoadData.magnData.cntLine, m_b,
                        &cntSteps, &errFlag);

        tPFData step_PFData_array[10];
        if (cntSteps > 0)
        {
            aw_GetPFData(step_PFData_array, 10, &StepsRead);
            assert(cntSteps == StepsRead);
        }

#if DEBUG_OUTPUT
        std::ofstream fPosEN(att_pos_en_file, std::ios_base::app);
        for (int i = 0; (i < cntSteps); i++)
        {
            tPFData &PFData = step_PFData_array[i];
            double q[4];
            for (int j = 0; (j < PFData.SampleCount); j++)
                if (DCM2Quaternion(PFData.Cp2ne_set[j], q) && fPosEN && fPosEN.is_open())
                {
                    int floor = Track.size() ? iStep->floor : 0;
                    SaveAttPosData(fPosEN, PFData.sample_time[j] * 1e3 + 0.5, PFData.length, PFData.headingInc, floor, q);
                }
        }
#endif

#if (ENABLE_OUTPUT == 1)
        std::ofstream fPosAtt(att_pos_file, std::ios_base::app);
        std::ofstream fIncAtt(att_inc_file, std::ios_base::app);
        std::ofstream fTpnData(tpn_text_file, std::ios_base::app);
        std::ofstream fMg(unbias_mg_file, std::ios_base::app);
        std::ofstream fIrlData(irl_text_file, std::ios_base::app);
#endif

#if INPUT_DATA_TYPE == 1
        if (Track.size() > 0)
        {
            for (int i = 0; (i < cntSteps) && (iStep != Track.end()); i++)
            {
                tPFData &PFData = step_PFData_array[i];
                for (int j = 0; (j < PFData.SampleCount) && (iStep != Track.end()); j++)
                {
                    int64_t time_tag = (int64_t)(PFData.sample_time[j] * 1e3 + 0.5);
                    while (time_tag < iStep->st1)
                    {
                        if (iStep == Track.begin())
                            break;
                        else
                            iStep--;
                    }
                    
                    while (time_tag > iStep->st2)
                    {
                        ++iStep;
                        if (iStep == Track.end())
                            break;

                        if ((iStep->st1 - iStepPrev->st2) > 25)
                        {
                            // stop between steps
                            flag_stop = true;
                            break;
                        }
                        iStepPrev = iStep;

                    }

                    if ((iStep != Track.end()) && (time_tag >= iStep->st1) && (time_tag <= iStep->st2) && (! flag_stop) )
                    {
                        double heading;
                        double x, y, dx, dy;
                        static double x0, y0;
                        static int flag = 0;
                        double q[4];
                        InterpolateHeading(*iStep, heading);
                        InterpolatePos(PFData.sample_time[j] * 1e3, *iStep, x, y);
                        
                        Matrix Cp2b = GetFullAttMatrix(PFData.Cp2ne_set[j], heading);
                        if (DCM2Quaternion(Cp2b, q) )
                        {
                            double mag_data[3];
                            mag_data[0] = PFData.SyncMagnData[0][j];
                            mag_data[1] = PFData.SyncMagnData[1][j];
                            mag_data[2] = PFData.SyncMagnData[2][j];

                            if (converter_settings.is_check_mag_vector_magnitude_enabled())
                                converter_settings.add_mag_vector(mag_data[0], mag_data[1], mag_data[2]);

                            // IRL output
                            CalcTPNData(tpn_data, geo2ff, PFData.sample_time[j] * 1e3, x, y, iStep->floor, q, mag_data, PFData.pEvent, PFData.length, true);
                            tpn_data_prev = tpn_data;
#if (ENABLE_OUTPUT == 1)
                            if (fIrlData && fIrlData.is_open())
                                SaveTpnData(fIrlData, tpn_data);

                            // FPBL output
                            //if (fPosAtt && fPosAtt.is_open())
                            //    SaveAttPosData(fPosAtt, PFData.sample_time[j] * 1e3, x, y, iStep->floor, q);
#endif

                            // RTFPPL output
                            if (flag == 0)
                            {
                                dx = dy = 0;
                                flag = 1;
                            }
                            else
                            {
                                dx = x - x0;
                                dy = y - y0;
                            }
                            x0 = x;
                            y0 = y;
#if (ENABLE_OUTPUT == 1)
                            //if (fIncAtt && fIncAtt.is_open())
                            //    SaveAttIncData(fIncAtt, PFData.sample_time[j] * 1e3, dx, dy, iStep->floor, q, mag_data);
#endif
                        }
#if (ENABLE_OUTPUT == 1)
                        if (fMg && fMg.is_open())
                        {
                            double mag_data[3];
                            mag_data[0] = PFData.SyncMagnData[0][j];
                            mag_data[1] = PFData.SyncMagnData[1][j];
                            mag_data[2] = PFData.SyncMagnData[2][j];
                            //SaveSensorData(fMg, PFData.sample_time[j] * 1e3, 0, mag_data);
                        }
#endif
                    }
                    if (flag_stop)
                    {
                        // stop between steps interpolation
                        int N = (iStep->st1 - iStepPrev->st2) / 20;
                        for (int i = 1; i < N; i++)
                        {
                            tpn_data = tpn_data_prev;
                            tpn_data.timestamp = iStepPrev->st2 + i * 20;
                            tpn_data.timestamp /= ms_per_sec;
                            tpn_data.mag_meas.is_valid = false;
                            if (fIrlData && fIrlData.is_open())
                                SaveTpnData(fIrlData, tpn_data);
                        }
                        iStepPrev = iStep;
                        flag_stop = false;
                    }
                }
            }
        }

#if TO_FIND_ABNORMAL_MAGNETIC_DATA
        if ((mag_data_mean_magnitude / mag_data_count) > ABNORMAL_MAGNETIC_THRESHOLD)

#endif

#else
        for (int i = 0; (i < cntSteps); i++)
        {
            PdrData2RtfpplData(step_PFData_array[i], fTpnData);
        }
#endif

#if (ENABLE_OUTPUT == 1)
        fPosAtt.close();
        fIncAtt.close();
        fTpnData.close();
        fMg.close();
#endif

#if DEBUG_OUTPUT
        fPosEN.close();
#endif
    }
    return ( pdrEOF == false );
}


void Main_Done(MDConverterSettings converter_settings)
{
    const double mean_magnetic_magnitude_threshold = 75;// uT
    if (converter_settings.is_check_mag_vector_magnitude_enabled() &&
        (converter_settings.get_mean_magnetic_magnitude() > mean_magnetic_magnitude_threshold))
    {
        std::ofstream fCheckReport(mag_data_check_report, std::ios_base::app);
        fCheckReport << converter_settings.get_input_path();
        fCheckReport << "     " << converter_settings.get_mean_magnetic_magnitude();
        fCheckReport << std::endl;
        fCheckReport.close();
    }
}

void PdrData2RtfpplData(tPFData &PFData, std::ofstream &fTpnData)
{

        //tPFData &PFData = step_PFData_array[i];
        static double heading = TPN_INITIAL_HEADING; // to do: get initial heading from track file
        static double x = 0, y = 0;
        double headingInc = PFData.headingInc / PFData.SampleCount;
        double dx = PFData.length * cos(heading + PFData.headingInc) / PFData.SampleCount;
        double dy = PFData.length * sin(heading + PFData.headingInc) / PFData.SampleCount;


        for (int j = 0; (j < PFData.SampleCount); j++)
        {
            int64_t time_tag = (int64_t)(PFData.sample_time[j] * 1e3 + 0.5);
            if (1)
            {
                static int flag = 0;
                double q[4];
                double floor = 0; // to do: set floor from json or from track

                heading += headingInc;// !!! check sign
                x += dx;
                y += dy;


                // Conversion like in UpdateMFP3D.cpp: Rz*Cb2m*Cp2b
                Eigen::Matrix<double, 3, 3> Cp2b;
                Eigen::Matrix<double, 3, 3> Cb2ne;
                Eigen::Matrix<double, 3, 3> Cb2m;
                Eigen::Matrix<double, 3, 3> Rz = Eigen::Matrix<double, 3, 3>::Zero();

                for (int l = 0; l < 3; l++)
                    for (int k = 0; k < 3; k++)
                    {
                        Cp2b(l, k) = PFData.Cp2b.Matr[l][k];
                        Cb2ne(l, k) = PFData.Cp2ne_set[j].Matr[l][k];
                    }

                Cb2m(0, 0) = 0; Cb2m(0, 1) = 1; Cb2m(0, 2) = 0;
                Cb2m(1, 0) = 1; Cb2m(1, 1) = 0; Cb2m(1, 2) = 0;
                Cb2m(2, 0) = 0; Cb2m(2, 1) = 0; Cb2m(2, 2) = -1;
                Rz(0, 0) = cos(heading); Rz(0, 1) = -sin(heading);
                Rz(1, 0) = sin(heading); Rz(1, 1) = cos(heading);
                Rz(2, 2) = 1;

                Eigen::Matrix<double, 3, 3> Ci2l = Rz*Cb2m*Cp2b;
                Eigen::Matrix<double, 3, 3> Ci2l1 = Rz*Cb2ne;

                Matrix _Ci2l;
                for (int l = 0; l < 3; l++)
                    for (int k = 0; k < 3; k++) 
                        _Ci2l.Matr[l][k] = Ci2l(l, k);

                if (DCM2Quaternion(_Ci2l, q))
                {

                    double mag_data[3];
                    mag_data[0] = PFData.SyncMagnData[0][j];
                    mag_data[1] = PFData.SyncMagnData[1][j];
                    mag_data[2] = PFData.SyncMagnData[2][j];

#if DEBUG_OUTPUT // Debug output
                    if (j == (PFData.SampleCount - 1))
                    {
                        double m[3];
                        memset(m, 0, sizeof(m));
                        for (int l = 0; l < 3; l++)
                            for (int k = 0; k < 3; k++)
                                m[l] += _Ci2l.Matr[l][k] * PFData.SyncMagnData[k][PFData.SampleCount - 1];
                        //m[l] = Cp2b.Matr[l][k] * PFData.SyncMagnData[k][PFData.SampleCount - 1];

                        std::ofstream fOut("mag_conv.txt", std::ios_base::app);
                        fOut << (int64_t)(PFData.timeEnd * 1000);
                        fOut << ", " << mag_data[0] << ", " << mag_data[1] << ", " << mag_data[2];
                        fOut << ", " << m[0] << ", " << m[1] << ", " << m[2];
                        fOut << ", " << heading;
                        fOut << ", " << x << ", " << y;
                        fOut << ",  " << q[0] << ", " << q[1] << ", " << q[2] << ", " << q[3];
                        //fOut << ", " << heading << ", " << headingInc;
                        fOut << "\n";
                        fOut.close();
                    }
#endif

                    // TPN output, once per step
                    if (j == (PFData.SampleCount - 1))
                    {
                        TpnOutput tpn_data = { 0 };
                        CalcTPNData(tpn_data, geo2ff, PFData.sample_time[j] * 1e3, x, y, floor, q, mag_data, PFData.pEvent, PFData.length, true);
                        if (fTpnData && fTpnData.is_open())
                            SaveTpnData(fTpnData, tpn_data);
                    }

                }
            }
        } // for
}
