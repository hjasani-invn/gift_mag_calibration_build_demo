#include "Calibration_console.hpp"
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>


int main(int argc, char* argv[])
{
    Calibration::SensorDataSample *input_data = new Calibration::SensorDataSample[50000]; // warning - change it if file has more lines
    Calibration::SensorDataSample one_data_sample = {};

    Calibration::SensorCalibrationParams ClbParams = {};

    const std::string input_data_file = argv[1];
    std::cout << "input data file: " << input_data_file << std::endl;

    std::regex mag_file_mask = std::regex(".*_mag.csv");
    std::regex acc_file_mask = std::regex(".*_imu.csv");

    bool is_acc = false;
    bool is_mag = false;

    if (std::regex_match(input_data_file, mag_file_mask))
    {
        is_mag = true;
    }

    if (std::regex_match(input_data_file, acc_file_mask))
    {
        is_acc = true;
    }

    if ((!is_mag) && (!is_acc))
    {
        std::cout << "file not supported" << std::endl;
        return -1;
    }

    std::ifstream infile(input_data_file);
    std::string line;
    const char delim = ',';
    
    size_t i = 0;

    if (is_mag)
    {
        while (std::getline(infile, line))
        {
            std::stringstream buf(line);
            std::string token;
            std::vector<std::string> result;

            while (std::getline(buf, token, delim))
            {
                result.push_back(token);
            }

            std::stringstream ss;
            ss << result[0];
            ss >> one_data_sample.time;
            ss.str("");
            ss.clear();
            ss << result[1];
            ss >> one_data_sample.x;

            one_data_sample.x *= -1; // this change is required due to a bug in input data

            ss.str("");
            ss.clear();
            ss << result[2];
            ss >> one_data_sample.y;
            ss.str("");
            ss.clear();
            ss << result[3];
            ss >> one_data_sample.z;
            ss.str("");
            ss.clear();
            ss << result[4];
            ss >> one_data_sample.temperature;
            ss.str("");
            ss.clear();

            input_data[i] = one_data_sample;
            ++i;
        }
    }

    if (is_acc)
    {
        while (std::getline(infile, line))
        {
            std::stringstream buf(line);
            std::string token;
            std::vector<std::string> result;

            while (std::getline(buf, token, delim))
            {
                result.push_back(token);
            }

            std::stringstream ss;
            ss << result[0];
            ss >> one_data_sample.time;
            ss.str("");
            ss.clear();
            ss << result[1];
            ss >> one_data_sample.x;
            ss.str("");
            ss.clear();
            ss << result[2];
            ss >> one_data_sample.y;
            ss.str("");
            ss.clear();
            ss << result[3];
            ss >> one_data_sample.z;
            ss.str("");
            ss.clear();
            ss << result[7];
            ss >> one_data_sample.temperature;
            ss.str("");
            ss.clear();

            input_data[i] = one_data_sample;
            ++i;
        }
    }

    Calibration::ISensorsCalibrator* calibrator_instance = new Calibration::SensorsCalibrator();
    
    const size_t N = i;
    const Calibration::SensorDataSample* pData = input_data;
    uint32_t n = 0;
    Calibration::ReturnStatus status = Calibration::ReturnStatus::STATUS_UNKNOWN_ERROR;

    if (is_mag)
    {
        n = calibrator_instance->AddMagneticData(pData, N);
        status = calibrator_instance->EstimateMagneticCalibrationParams(ClbParams);
    }

    if (is_acc)
    {
        n = calibrator_instance->AddAccelData(pData, N);
        status = calibrator_instance->EstimateAccelCalibrationParams(ClbParams);
    }

    std::cout << "status: " << (int)status << std::endl;
    std::cout << "calib_level: " << (int)ClbParams.calibrationLevel << std::endl;
    std::cout << "accuracy: " << ClbParams.covB[0][0] << std::endl;
    std::cout << "DOP: " << ClbParams.DOP << std::endl;
    std::cout << "biases: " << ClbParams.bx << " " << ClbParams.by << " " << ClbParams.bz << std::endl;
    std::cout << "matrix: " << ClbParams.CM[0][0] << " " << ClbParams.CM[0][1] << " " << ClbParams.CM[0][2] << " "
        << ClbParams.CM[1][0] << " " << ClbParams.CM[1][1] << " " << ClbParams.CM[1][2] << " "
        << ClbParams.CM[2][0] << " " << ClbParams.CM[2][1] << " " << ClbParams.CM[2][2] << std::endl;
   

    return 0;
}