#include <iostream>
#include <iomanip>

#include <stdint.h>
#include <math.h>
#include <memory.h>

#include "MagCalibrators.hpp"
#include "rand.hpp"

namespace TrackPreProcessing
{

    MagBias Calibrator_FDMC::CalculateBias(const std::vector<TpnOutput> &irl_data)
    {
        MagBias mg_bias, mg_bias_0;
        MagBias mg_bias_1, mg_bias_2;

        // Reject Stands
        std::list<TpnOutput> irl_data_without_stands;
        TpnOutput tmp;
        long cnt = 0;
        long len = irl_data.size();
        long start_part = std::min(len / 5, long(1500));
        long finish_part = std::min(len / 5, long(1500));
        long actual_len = len - start_part - finish_part;

        if (actual_len <= 0)
        {
            return mg_bias;
        }

#ifdef APPLY_WINDOW
        window = new double[actual_len];
        create_window(window, actual_len);
#endif
        for (auto it = irl_data.begin(); it != irl_data.end(); ++it)
        {
            tmp = *it;
      //      if (tmp.pdr.is_valid && tmp.pdr.stride_length != 0)
            {

                tmp.mag_meas.mX /= 10;
                tmp.mag_meas.mY /= 10;
                tmp.mag_meas.mZ /= 10;

#if 1
                /// temporary to have identical data with matlad

                double tmp1;
                tmp1 = floor(tmp.mag_meas.mX * 1000 + 0.5);
                tmp.mag_meas.mX = (double)tmp1 / 1000;
                tmp1 = floor(tmp.mag_meas.mY * 1000 + 0.5);
                tmp.mag_meas.mY = (double)tmp1 / 1000;
                tmp1 = floor(tmp.mag_meas.mZ * 1000 + 0.5);
                tmp.mag_meas.mZ = (double)tmp1 / 1000;

                ///
                //std::cout << tmp.mag_meas.mX << "  ;  " << tmp.mag_meas.mY << "  ;  " << tmp.mag_meas.mZ << std::endl;
#endif
#if 1           // start and finish parts is not used
                if ( (cnt >= start_part) && (cnt <= (len - finish_part)) )
                    irl_data_without_stands.push_back(tmp);
                cnt++;
#else
                irl_data_without_stands.push_back(tmp);
#endif
            }
        }
        //std::cout << "====================================" << std::endl;

        if (irl_data_without_stands.size() == 0)
            return mg_bias;
        tmp = irl_data_without_stands.front();
        mg_bias_0.bias[0] = tmp.mag_meas.mX;
        mg_bias_0.bias[1] = tmp.mag_meas.mY;
        mg_bias_0.bias[2] = tmp.mag_meas.mZ;
#if 0
        mg_bias = calculateBiasWitnGlobalSearch(irl_data_without_stands, mg_bias_0);
#else
        mg_bias_1 = calculateBiasWitnGlobalSearchFromStart(irl_data_without_stands, mg_bias_0);
        mg_bias_2 = calculateBiasWitnGlobalSearchToEnd(irl_data_without_stands, mg_bias_0);
        if (mg_bias_1.calibration_level > mg_bias_2.calibration_level)
            mg_bias = mg_bias_1;
        if (mg_bias_1.calibration_level < mg_bias_2.calibration_level)
            mg_bias = mg_bias_2;
        if (mg_bias_1.calibration_level ==  mg_bias_2.calibration_level)
        {
            mg_bias.bias[0] = (mg_bias_1.bias[0] + mg_bias_2.bias[0]) / 2;
            mg_bias.bias[1] = (mg_bias_1.bias[1] + mg_bias_2.bias[1]) / 2;
            mg_bias.bias[2] = (mg_bias_1.bias[2] + mg_bias_2.bias[2]) / 2;

            mg_bias.covarianceMatrix[0][0] = (mg_bias_1.covarianceMatrix[0][0] + mg_bias_2.covarianceMatrix[0][0]) / 2;
            mg_bias.covarianceMatrix[0][1] = (mg_bias_1.covarianceMatrix[0][1] + mg_bias_2.covarianceMatrix[0][1]) / 2;
            mg_bias.covarianceMatrix[0][2] = (mg_bias_1.covarianceMatrix[0][2] + mg_bias_2.covarianceMatrix[0][2]) / 2;
            mg_bias.covarianceMatrix[1][0] = (mg_bias_1.covarianceMatrix[1][0] + mg_bias_2.covarianceMatrix[1][0]) / 2;
            mg_bias.covarianceMatrix[1][1] = (mg_bias_1.covarianceMatrix[1][1] + mg_bias_2.covarianceMatrix[1][1]) / 2;
            mg_bias.covarianceMatrix[1][2] = (mg_bias_1.covarianceMatrix[1][2] + mg_bias_2.covarianceMatrix[1][2]) / 2;
            mg_bias.covarianceMatrix[2][0] = (mg_bias_1.covarianceMatrix[2][0] + mg_bias_2.covarianceMatrix[2][0]) / 2;
            mg_bias.covarianceMatrix[2][1] = (mg_bias_1.covarianceMatrix[2][1] + mg_bias_2.covarianceMatrix[2][1]) / 2;
            mg_bias.covarianceMatrix[2][2] = (mg_bias_1.covarianceMatrix[2][2] + mg_bias_2.covarianceMatrix[2][2]) / 2;

            mg_bias.calibration_level = mg_bias_1.calibration_level;
        }
#endif
        return mg_bias;
    }
#if 0
    MagBias Calibrator_FDMC::(const std::list<TpnOutput> &irl_data, const MagBias mg_bias_0)
    {
        uint32_t irl_data_size = irl_data.size();
#if 0
        uint32_t size_of_fft = irl_data_size;
#else
        uint32_t size_of_fft = 1;

        /*
        while (size_of_fft < irl_data_size)
            size_of_fft *= 5;
        size_of_fft /= 5;
        while (size_of_fft < irl_data_size)
            size_of_fft *= 4;
        size_of_fft /= 4;
        while (size_of_fft < irl_data_size)
            size_of_fft *= 3;
        size_of_fft /= 3;
        */
        while (size_of_fft < irl_data_size)
            size_of_fft *= 2;
        size_of_fft /= 2;
        irl_data_size = size_of_fft;
#endif
#ifdef EIGENFFT

       // irl_data_mod.resize(irl_data_size);
        irl_data_mod.resize(size_of_fft);

#else
        /* This back-end is disabled currently
        // fftw plan
        fftw_in = (double*)fftw_malloc(sizeof(double) * size_of_fft);
        fftw_out = (fftw_complex*)fftw_malloc(sizeof(fftw_complex) * (size_of_fft / 2 + 1));
        fftw_p = fftw_plan_dft_r2c_1d(size_of_fft, fftw_in, fftw_out, FFTW_ESTIMATE);
        */
#endif
        TpnMagneticMeasurement * mag_meas = new TpnMagneticMeasurement[irl_data_size];
        TpnMagneticMeasurement * mag_meas_prt = mag_meas;

        //for (auto it = irl_data.begin(); it != irl_data.end(); ++it)
        auto it = irl_data.begin();
        for (int i = 0; i < (irl_data.size() - irl_data_size); i++)
        {
            ++it;
        }
        for (int i = 0; i < irl_data_size; i++)
        {
            if (it != irl_data.end())
            {
                *mag_meas_prt = it->mag_meas;
                ++it;
            }
            mag_meas_prt++;
        }

        MagBias mg_bias_randon;

        // zero iteration
        double h = sqrt(calc_h(mag_meas, irl_data_size, size_of_fft, mg_bias_0.bias) / 2);
        std::cout << std::endl
            << "start    0 iteration  "
            << std::fixed
            << std::setw(20)
            << std::right
            << std::setprecision(15)
            << mg_bias_0.bias[0] << "    "
            << mg_bias_0.bias[1] << "    "
            << mg_bias_0.bias[2] << "    "
            << h << "    "
            << std::endl;

        MagBias mg_bias_cur = calculateNewBiasWithInitBias(mag_meas, irl_data_size, size_of_fft, mg_bias_0);
        h = sqrt(calc_h(mag_meas, irl_data_size, size_of_fft, mg_bias_cur.bias) / 2);
        std::cout
            << "finish   0 iteration "
            << std::fixed
            << std::setw(20)
            << std::right
            << std::setprecision(15)
            << mg_bias_cur.bias[0] << "    "
            << mg_bias_cur.bias[1] << "    "
            << mg_bias_cur.bias[2] << "    "
            << h << "    "
            << std::right
            << std::setprecision(0)
            << (double)mg_bias_cur.calibration_level
            << std::endl << std::endl;
//        exit(0);
        const int n_iteration = 12;
        if ((mg_bias_cur.calibration_level < 3) || (h > 0.5))
        {
            double DBX = 50;
            double DBY = 50;
            double DBZ = 50;

            pf_random::rng32 rnd((uint32_t)abs(mg_bias_0.bias[0]));

            for (int i = 0; i < n_iteration; i++)
            {
                // random int bias
                double r = rnd.randnf<double>();
                mg_bias_randon.bias[0] = mg_bias_0.bias[0] + DBX*r;
                r = rnd.randnf<double>();
                mg_bias_randon.bias[1] = mg_bias_0.bias[1] + DBY*r;
                r = rnd.randnf<double>();
                mg_bias_randon.bias[2] = mg_bias_0.bias[2] + DBZ*r;

                std::cout
                    << "start    "
                    << i+1
                    << "  iteration "
                    << std::fixed
                    << std::setw(20)
                    << std::right
                    << std::setprecision(15)
                    << mg_bias_randon.bias[0]  << "    "
                    << mg_bias_randon.bias[1]  << "    "
                    << mg_bias_randon.bias[2]  << "    "
                    << h << "    "
                    << std::endl;

                MagBias mg_bias_1 = calculateNewBiasWithInitBias(mag_meas, irl_data_size, size_of_fft, mg_bias_randon);
                double h1 = sqrt(calc_h(mag_meas, irl_data_size, size_of_fft, mg_bias_1.bias) / 2);

                std::cout
                    << "finish   "
                    << i+1
                    << "  iteration "
                    << std::fixed
                    << std::setw(20)
                    << std::right
                    << std::setprecision(15)
                    << mg_bias_1.bias[0]  << "    "
                    << mg_bias_1.bias[1]  << "    "
                    << mg_bias_1.bias[2]  << "    "
                    << h1 << "    "
                    << std::setw(5)
                    << std::right
                    << std::setprecision(0)
                    << (double)mg_bias_1.calibration_level
                    << std::endl << std::endl;

                if ((mg_bias_1.calibration_level >= mg_bias_cur.calibration_level) && (h1 < h))
                {
                    mg_bias_cur = mg_bias_1;
                    h = h1;
                    if ((mg_bias_cur.calibration_level >= 3) && (h <= 0.5))
                        break;
                }

            }
            //h = sqrt(calc_h(mag_meas, irl_data_size, mg_bias_cur.bias) / 2);
            std::cout
                << std::endl
                << "final result is "
                << std::fixed
                << std::setw(20)
                << std::right
                << std::setprecision(15)
                << mg_bias_cur.bias[0] << "    "
                << mg_bias_cur.bias[1] << "    "
                << mg_bias_cur.bias[2] << "    "
                << h << std::setw(5)
                << std::right
                << std::setprecision(0)
                << (double)mg_bias_cur.calibration_level
                << std::endl;
        }

#ifdef EIGENFFT
        irl_data_mod.clear();
#else
        /* This back-end is disabled currently
        fftw_destroy_plan(fftw_p);
        fftw_free(fftw_in);
        fftw_free(fftw_out);
        */
#endif
        return mg_bias_cur;
    }
#endif
    MagBias Calibrator_FDMC::calculateBiasWitnGlobalSearchFromStart(const std::list<TpnOutput> &irl_data, const MagBias mg_bias_0)
    {
        uint32_t irl_data_size = irl_data.size();
#if 0
        uint32_t size_of_fft = irl_data_size;
#else
        uint32_t size_of_fft = 1;

        /*
        while (size_of_fft < irl_data_size)
        size_of_fft *= 5;
        size_of_fft /= 5;
        while (size_of_fft < irl_data_size)
        size_of_fft *= 4;
        size_of_fft /= 4;
        while (size_of_fft < irl_data_size)
        size_of_fft *= 3;
        size_of_fft /= 3;
        */
        while (size_of_fft < irl_data_size)
            size_of_fft *= 2;
        size_of_fft /= 2;
        irl_data_size = size_of_fft;
#endif
#ifdef EIGENFFT
        // irl_data_mod.resize(irl_data_size);
        irl_data_mod.resize(size_of_fft);
#else
        /*
        // fftw plan
        fftw_in = (double*)fftw_malloc(sizeof(double) * size_of_fft);
        fftw_out = (fftw_complex*)fftw_malloc(sizeof(fftw_complex) * (size_of_fft / 2 + 1));
        fftw_p = fftw_plan_dft_r2c_1d(size_of_fft, fftw_in, fftw_out, FFTW_ESTIMATE);
        */
#endif
        TpnMagneticMeasurement * mag_meas = new TpnMagneticMeasurement[irl_data_size];
        TpnMagneticMeasurement * mag_meas_prt = mag_meas;

        //for (auto it = irl_data.begin(); it != irl_data.end(); ++it)
        auto it = irl_data.begin();
        //for (int i = 0; i < (irl_data.size() - irl_data_size); i++)
        //    ++it;
        for (int i = 0; i < irl_data_size; i++)
        {
            if (it != irl_data.end())
            {
                *mag_meas_prt = it->mag_meas;
                ++it;
            }
            mag_meas_prt++;
        }

        MagBias mg_bias_randon;

        // zero iteration
        double h = sqrt(calc_h(mag_meas, irl_data_size, size_of_fft, mg_bias_0.bias) / 2);
        std::cout << "mag_calibration: ";
        /*
        std::cout << std::endl
            << "start    0 iteration  "
            << std::fixed
            << std::setw(20)
            << std::right
            << std::setprecision(15)
            << mg_bias_0.bias[0] << "    "
            << mg_bias_0.bias[1] << "    "
            << mg_bias_0.bias[2] << "    "
            << h << "    "
            << std::endl;
            */

        MagBias mg_bias_cur = calculateNewBiasWithInitBias(mag_meas, irl_data_size, size_of_fft, mg_bias_0);
        h = sqrt(calc_h(mag_meas, irl_data_size, size_of_fft, mg_bias_cur.bias) / 2);
        std::cout << ".";
        /*std::cout
            << "finish   0 iteration "
            << std::fixed
            << std::setw(20)
            << std::right
            << std::setprecision(15)
            << mg_bias_cur.bias[0] << "    "
            << mg_bias_cur.bias[1] << "    "
            << mg_bias_cur.bias[2] << "    "
            << h << "    "
            << std::right
            << std::setprecision(0)
            << (double)mg_bias_cur.calibration_level
            << std::endl << std::endl;
            */
        //        exit(0);
        const int n_iteration = 12;
        if ((mg_bias_cur.calibration_level < 3) || (h > 0.5))
        {
            double DBX = 50;
            double DBY = 50;
            double DBZ = 50;

            pf_random::rng32 rnd((uint32_t)abs(mg_bias_0.bias[0]));

            for (int i = 0; i < n_iteration; i++)
            {
                // random int bias
                double r = rnd.randnf<double>();
                mg_bias_randon.bias[0] = mg_bias_0.bias[0] + DBX*r;
                r = rnd.randnf<double>();
                mg_bias_randon.bias[1] = mg_bias_0.bias[1] + DBY*r;
                r = rnd.randnf<double>();
                mg_bias_randon.bias[2] = mg_bias_0.bias[2] + DBZ*r;

                /*std::cout
                    << "start    "
                    << i + 1
                    << "  iteration "
                    << std::fixed
                    << std::setw(20)
                    << std::right
                    << std::setprecision(15)
                    << mg_bias_randon.bias[0] << "    "
                    << mg_bias_randon.bias[1] << "    "
                    << mg_bias_randon.bias[2] << "    "
                    << h << "    "
                    << std::endl;*/

                MagBias mg_bias_1 = calculateNewBiasWithInitBias(mag_meas, irl_data_size, size_of_fft, mg_bias_randon);
                double h1 = sqrt(calc_h(mag_meas, irl_data_size, size_of_fft, mg_bias_1.bias) / 2);

                std::cout << ".";
                /*std::cout
                    << "finish   "
                    << i + 1
                    << "  iteration "
                    << std::fixed
                    << std::setw(20)
                    << std::right
                    << std::setprecision(15)
                    << mg_bias_1.bias[0] << "    "
                    << mg_bias_1.bias[1] << "    "
                    << mg_bias_1.bias[2] << "    "
                    << h1 << "    "
                    << std::setw(5)
                    << std::right
                    << std::setprecision(0)
                    << (double)mg_bias_1.calibration_level
                    << std::endl << std::endl;*/

                if ((mg_bias_1.calibration_level >= mg_bias_cur.calibration_level) && (h1 < h))
                {
                    mg_bias_cur = mg_bias_1;
                    h = h1;
                    if ((mg_bias_cur.calibration_level >= 3) && (h <= 0.5))
                        break;
                }

            }
            //h = sqrt(calc_h(mag_meas, irl_data_size, mg_bias_cur.bias) / 2);
            std::cout << "*";
            /*std::cout
                << std::endl
                << "final result is "
                << std::fixed
                << std::setw(20)
                << std::right
                << std::setprecision(15)
                << mg_bias_cur.bias[0] << "    "
                << mg_bias_cur.bias[1] << "    "
                << mg_bias_cur.bias[2] << "    "
                << h << std::setw(5)
                << std::right
                << std::setprecision(0)
                << (double)mg_bias_cur.calibration_level
                << std::endl;*/
        }

#ifdef EIGENFFT
        irl_data_mod.clear();
#else
        /* This back-end is disabled currently
        fftw_destroy_plan(fftw_p);
        fftw_free(fftw_in);
        fftw_free(fftw_out);
        */
#endif
        return mg_bias_cur;
    }
    MagBias Calibrator_FDMC::calculateBiasWitnGlobalSearchToEnd(const std::list<TpnOutput> &irl_data, const MagBias mg_bias_0)
    {
        uint32_t irl_data_size = irl_data.size();
#if 0
        uint32_t size_of_fft = irl_data_size;
#else
        uint32_t size_of_fft = 1;

        /*
        while (size_of_fft < irl_data_size)
        size_of_fft *= 5;
        size_of_fft /= 5;
        while (size_of_fft < irl_data_size)
        size_of_fft *= 4;
        size_of_fft /= 4;
        while (size_of_fft < irl_data_size)
        size_of_fft *= 3;
        size_of_fft /= 3;
        */
        while (size_of_fft < irl_data_size)
            size_of_fft *= 2;
        size_of_fft /= 2;
        irl_data_size = size_of_fft;
#endif
#ifdef EIGENFFT
        // irl_data_mod.resize(irl_data_size);
        irl_data_mod.resize(size_of_fft);
#else
        /* This back-end is disabled currently
        // fftw plan
        fftw_in = (double*)fftw_malloc(sizeof(double) * size_of_fft);
        fftw_out = (fftw_complex*)fftw_malloc(sizeof(fftw_complex) * (size_of_fft / 2 + 1));
        fftw_p = fftw_plan_dft_r2c_1d(size_of_fft, fftw_in, fftw_out, FFTW_ESTIMATE);
        */
#endif
        TpnMagneticMeasurement * mag_meas = new TpnMagneticMeasurement[irl_data_size];
        TpnMagneticMeasurement * mag_meas_prt = mag_meas;

        //for (auto it = irl_data.begin(); it != irl_data.end(); ++it)
        auto it = irl_data.begin();
        for (int i = 0; i < (irl_data.size() - irl_data_size); i++)
        {
            ++it;
        }
        for (int i = 0; i < irl_data_size; i++)
        {
            if (it != irl_data.end())
            {
                *mag_meas_prt = it->mag_meas;
                ++it;
            }
            mag_meas_prt++;
        }

        MagBias mg_bias_randon;

        // zero iteration
        double h = sqrt(calc_h(mag_meas, irl_data_size, size_of_fft, mg_bias_0.bias) / 2);
        std::cout << "mag_calibration: ";
        /*std::cout << std::endl
            << "start    0 iteration  "
            << std::fixed
            << std::setw(20)
            << std::right
            << std::setprecision(15)
            << mg_bias_0.bias[0] << "    "
            << mg_bias_0.bias[1] << "    "
            << mg_bias_0.bias[2] << "    "
            << h << "    "
            << std::endl;
            */

        MagBias mg_bias_cur = calculateNewBiasWithInitBias(mag_meas, irl_data_size, size_of_fft, mg_bias_0);
        h = sqrt(calc_h(mag_meas, irl_data_size, size_of_fft, mg_bias_cur.bias) / 2);
        std::cout << ".";
        /*std::cout
            << "finish   0 iteration "
            << std::fixed
            << std::setw(20)
            << std::right
            << std::setprecision(15)
            << mg_bias_cur.bias[0] << "    "
            << mg_bias_cur.bias[1] << "    "
            << mg_bias_cur.bias[2] << "    "
            << h << "    "
            << std::right
            << std::setprecision(0)
            << (double)mg_bias_cur.calibration_level
            << std::endl << std::endl;
            */
        //        exit(0);
        const int n_iteration = 12;
        if ((mg_bias_cur.calibration_level < 3) || (h > 0.5))
        {
            double DBX = 50;
            double DBY = 50;
            double DBZ = 50;

            pf_random::rng32 rnd((uint32_t)abs(mg_bias_0.bias[0]));

            for (int i = 0; i < n_iteration; i++)
            {
                // random int bias
                double r = rnd.randnf<double>();
                mg_bias_randon.bias[0] = mg_bias_0.bias[0] + DBX*r;
                r = rnd.randnf<double>();
                mg_bias_randon.bias[1] = mg_bias_0.bias[1] + DBY*r;
                r = rnd.randnf<double>();
                mg_bias_randon.bias[2] = mg_bias_0.bias[2] + DBZ*r;

                std::cout << ".";
                /*std::cout
                    << "start    "
                    << i + 1
                    << "  iteration "
                    << std::fixed
                    << std::setw(20)
                    << std::right
                    << std::setprecision(15)
                    << mg_bias_randon.bias[0] << "    "
                    << mg_bias_randon.bias[1] << "    "
                    << mg_bias_randon.bias[2] << "    "
                    << h << "    "
                    << std::endl;
                    */

                MagBias mg_bias_1 = calculateNewBiasWithInitBias(mag_meas, irl_data_size, size_of_fft, mg_bias_randon);
                double h1 = sqrt(calc_h(mag_meas, irl_data_size, size_of_fft, mg_bias_1.bias) / 2);

                /*std::cout
                    << "finish   "
                    << i + 1
                    << "  iteration "
                    << std::fixed
                    << std::setw(20)
                    << std::right
                    << std::setprecision(15)
                    << mg_bias_1.bias[0] << "    "
                    << mg_bias_1.bias[1] << "    "
                    << mg_bias_1.bias[2] << "    "
                    << h1 << "    "
                    << std::setw(5)
                    << std::right
                    << std::setprecision(0)
                    << (double)mg_bias_1.calibration_level
                    << std::endl << std::endl;
                    */

                if ((mg_bias_1.calibration_level >= mg_bias_cur.calibration_level) && (h1 < h))
                {
                    mg_bias_cur = mg_bias_1;
                    h = h1;
                    if ((mg_bias_cur.calibration_level >= 3) && (h <= 0.5))
                        break;
                }

            }
            //h = sqrt(calc_h(mag_meas, irl_data_size, mg_bias_cur.bias) / 2);
            std::cout << "*" << std::endl;
            /*std::cout
                << std::endl
                << "final result is "
                << std::fixed
                << std::setw(20)
                << std::right
                << std::setprecision(15)
                << mg_bias_cur.bias[0] << "    "
                << mg_bias_cur.bias[1] << "    "
                << mg_bias_cur.bias[2] << "    "
                << h << std::setw(5)
                << std::right
                << std::setprecision(0)
                << (double)mg_bias_cur.calibration_level
                << std::endl;
                */
        }

#ifdef EIGENFFT
        irl_data_mod.clear();
#else
        /* This back-end is disabled currently
        fftw_destroy_plan(fftw_p);
        fftw_free(fftw_in);
        fftw_free(fftw_out);
        */
#endif
        return mg_bias_cur;
    }
    MagBias Calibrator_FDMC::calculateNewBiasWithInitBias(const  TpnMagneticMeasurement * mag_meas, uint32_t irl_data_size, uint32_t size_of_fft, const MagBias mg_bias)
    {

        //% m_bias = m_bias_0;
        //% delta_m_bias = [100, -200, 300]';
        //% lamda = 50;
        //% cnt = 0;
        //% max_count = 500;

        MagBias mg_bias_cur = mg_bias;
        MagBias mg_bias_new;
        double delta_m_bias[3] = { 100, -200, 300 };
        double    lamda = 50;
        int cnt = 0;
        const int max_count = 500;

        double H[3];
        double  y, y0, y1;

        //% while ((norm(delta_m_bias) > 0.0001) && (cnt <= max_count))
        while ((norm_v(delta_m_bias, 3) > 0.0001) && (cnt <= max_count))
        {
         //   std::cout << "counter   " << cnt << std::endl;
            //std::cout << std::fixed << std::setw(20) << std::right << std::setprecision(15) << mg_bias_cur.bias[0] << "    " << mg_bias_cur.bias[1] << "    " << mg_bias_cur.bias[2] << std::endl;
            // if (cnt == 50)
            //     std::cout << cnt << std::endl;
            //% H = calc_h_matrix(m_raw, m_bias);
         //   std::cout << "bias before  " << mg_bias_cur.bias[0] << "   " << mg_bias_cur.bias[1] << "   " << mg_bias_cur.bias[2] << std::endl;
            calc_h_matrix(mag_meas, irl_data_size, size_of_fft, mg_bias_cur.bias, H, y0);

         //   std::cout << "H   " << std::fixed << std::setw(20) << std::right << std::setprecision(15) << H[0] << "    " << H[1] << "    " << H[2] << std::endl;
         //   std::cout << "y0   " << y0 << std::endl;
            //vect_to_matrix(H, 3, HH);
            //std::cout << "H   " << H[0] << "   " << H[1] << "   " << H[2] << std::endl;

            lamda = lamda * 1.1;
         //   std::cout << "lamda   " << lamda << std::endl;

            //% delta_m_bias = -H*lamda;
            delta_m_bias[0] = -H[0] * lamda;
            delta_m_bias[1] = -H[1] * lamda;
            delta_m_bias[2] = -H[2] * lamda;

            mg_bias_new.bias[0] = mg_bias_cur.bias[0] + delta_m_bias[0];
            mg_bias_new.bias[1] = mg_bias_cur.bias[1] + delta_m_bias[1];
            mg_bias_new.bias[2] = mg_bias_cur.bias[2] + delta_m_bias[2];

         //   std::cout << "bias after  " << mg_bias_new.bias[0] << "   " << mg_bias_new.bias[1] << "   " << mg_bias_new.bias[2] << std::endl;
            //y0 = calc_h(mag_meas, irl_data_size, mg_bias_cur.bias); // !!!
            y1 = calc_h(mag_meas, irl_data_size, size_of_fft, mg_bias_new.bias);

            //std::cout << cnt << "    " << y0 << "    " << y1 << std::endl;

            if (y1 < y0)
            {
                mg_bias_cur = mg_bias_new;
                //std::cout << "y1 < y0" << std::endl;
                y = y1;
            }
            else
            {
                lamda = lamda / 2;
                //std::cout << "y1 > y0" << std::endl;
                y = y0;
            }
            cnt = cnt + 1;

         //   std::cout << "norm_v(delta_m_bias, 3)  " << norm_v(delta_m_bias, 3) << std::endl;

            //y = calc_h(irl_data, mg_bias_cur.bias);
        }
        //y = calc_h(irl_data, mg_bias_cur.bias);

        // m_bias_cov = sqrt(y / 2)*norm(H, 3) ^ -2 * H*H';
//        std::cout << "H   " << H[0] << "   " << H[1] << "   " << H[2] << std::endl;

        double norm = norm_v(H, 3);
        double koef = sqrt(y / 2) / (norm * norm);
        vect_to_matrix(H, 3, koef, (double *)mg_bias_cur.covarianceMatrix);

        //     % bias class estimation
        if (cnt < 10)
        {
            memset((double *)mg_bias_cur.covarianceMatrix, 0, sizeof(mg_bias_cur.covarianceMatrix));
            mg_bias_cur.calibration_level = 0;
        }
        else
            if (cnt < 100)
            {
                mg_bias_cur.calibration_level = 3;
            }
            else
                if (cnt < 250)
                {
                    mg_bias_cur.calibration_level = 2;
                }
                else
                {
                    if (cnt < max_count)
                        mg_bias_cur.calibration_level = 1;
                    else
                        mg_bias_cur.calibration_level = 0;
                }
        double    cut_off_threshold = 2;
        if (sqrt(y / 2) > cut_off_threshold)
            mg_bias_cur.calibration_level = 0;

        return mg_bias_cur;
    }
    double Calibrator_FDMC::calc_h(const  TpnMagneticMeasurement * mag_meas, uint32_t irl_data_size, uint32_t size_of_fft, const double bias[])
    {
        //Fs = 20; %Hz - sample rate
        //    for i = size(m_raw, 1) :-1 : 1
        //        m_bf = m_raw(i, :) - m_bias;
        //m_mfp_module(i, 1) = sqrt(m_bf(1)*m_bf(1) + m_bf(2)*m_bf(2) + m_bf(3)*m_bf(3));
        //end

        //        std::cout << std::fixed << std::setw(12) << std::right << std::setprecision(7) << bias[0] << "    " << bias[1] << "    " << bias[2] << std::endl << std::endl;
        double mod, x, y, z;
        double b0 = bias[0];
        double b1 = bias[1];
        double b2 = bias[2];

#ifndef EIGENFFT
        /* This back-end is disabled currently
        double *fftw_in_prt = fftw_in;
        */
#endif
#ifdef REMOVE_ZERO_FREQ
        double mod_sum = 0;
#endif
        for (int i = 0; i < irl_data_size; i++)
        {
            x = mag_meas->mX - b0;
            y = mag_meas->mY - b1;
            z = mag_meas->mZ - b2;
            //            std::cout << std::fixed << std::setw(12) << std::right << std::setprecision(7) << x << "    " << y << "    " << z  << std::endl;
            mag_meas++;
            mod = (x*x + y*y + z*z);
            //            std::cout << std::fixed << std::setw(12) << std::right << std::setprecision(7) << mod << "    ";
            mod = sqrt(mod);
#ifdef REMOVE_ZERO_FREQ
            mod_sum += mod;
#endif
//            std::cout << std::fixed << std::setw(20) << std::right << std::setprecision(15) << mod << std::endl;
#ifdef EIGENFFT
            irl_data_mod[i] = mod;
#else
            /* This back-end is disabled currently
            *fftw_in_prt = mod;
            fftw_in_prt++;
            */
#endif
        }
#ifdef REMOVE_ZERO_FREQ
        double mod_aver = mod_sum / irl_data_size;
        // remove average
#ifndef EIGENFFT
        /* This back-end is disabled currently
        fftw_in_prt = fftw_in;
        */
#endif
        for (int i = 0; i < irl_data_size; i++)
        {
#ifdef EIGENFFT
            irl_data_mod[i] -= mod_aver;
#else
            /* This back-end is disabled currently
            *fftw_in_prt -= mod_aver;
            fftw_in_prt++;
            */
#endif
        }
#endif

#ifdef APPLY_WINDOW
        // apply window
#ifndef EIGENFFT
        fftw_in_prt = fftw_in;
#endif
        for (int i = 0; i < irl_data_size; i++)
        {
#ifdef EIGENFFT
            irl_data_mod[i] *= window[i];
#else
            /* This back-end is disabled currently
            *fftw_in_prt *= window[i];
            fftw_in_prt++;
            */
#endif
    }
#endif
        // add zeros to length will be power(2,N)
        for (int i = irl_data_size; i < size_of_fft; i++)
        {
#ifdef EIGENFFT
            irl_data_mod[i] = 0;
#else
            /* This back-end is disabled currently
            *fftw_in_prt = 0;
            fftw_in_prt++;
            */
#endif
        }

#ifndef EIGENFFT
        /* This back-end is disabled currently
        // signal energy in time domain
        double time_energy_sum = 0;
        fftw_in_prt = fftw_in;
        for (int i = 0; i < size_of_fft; i++)
        {
            time_energy_sum += (*fftw_in_prt) * (*fftw_in_prt);
            fftw_in_prt++;
        }
        */
#endif
        //y = spectral_criterion_local(m_mfp_module, Fs);
        double Fs = F_SAMPLE; //%Hz - sample rate
        double F1 = F_LOW; //
        double F2 = F_HIGH; //
        int N1 = (int)(F1 / Fs * size_of_fft) + 1;
        int N2 = (int)(F2 / Fs * size_of_fft);
        if (N2 >= size_of_fft / 2)
            N2 = size_of_fft / 2 - 1;
        double freq_energy_sum = 0;
#ifdef EIGENFFT
        std::vector<std::complex<double> > freqs;
        Eigen::FFT<double> fft;
        fft.fwd(freqs, irl_data_mod);

        // signal energy in frequency domain
        double re = freqs[0].real();
        double im = freqs[0].imag();
#if 0
        freq_energy_sum = (re*re + im*im);
#else  // without zero freuency
        freq_energy_sum = 0;
#endif

        /*
        for (int i = N1; i <= N2; i++)
        {
            float re = freqs[i].real();
            float im = freqs[i].imag();
            mod = sqrt(re*re + im*im) / size;
            if (i != 0)
                mod *= 2;
            freq_energy_sum += mod*mod;
        }
        */
        freq_energy_sum = 0;
        for (int i = N1; i <= N2; i++)
        {
            re = freqs[i].real();
            im = freqs[i].imag();
            freq_energy_sum += 2 * (re*re + im*im);
        }
        freq_energy_sum /= size_of_fft;
        //std::cout << freq_energy_sum << std::endl;
        // norming
        freq_energy_sum /= (irl_data_size / 2);
#else
        /* This back-end is disabled currently
        // pure fftw
        {
            fftw_execute(fftw_p);

            //for (int i = 0; i < N / 2 + 1; i++)
            //    std::cout << out[i][0] << "    " << out[i][1] << std::endl;

            // signal energy in frequency domain
            double re = fftw_out[0][0];
            double im = fftw_out[0][1];
#if 0
            freq_energy_sum = (re*re + im*im);
#else  // without zero freuency
            freq_energy_sum = 0;
#endif
            for (int i = 1; i <= size_of_fft / 2; i++)
            {
                re = fftw_out[i][0];
                im = fftw_out[i][1];
                freq_energy_sum += 2 * (re*re + im*im);
            }
            freq_energy_sum /= size_of_fft;

            // signal energy in needed frequency band
            int N1 = (int)(F1 / Fs * size_of_fft) + 1;
            int N2 = (int)(F2 / Fs * size_of_fft);
            freq_energy_sum = 0;
            for (int i = N1; i <= N2; i++)
            {
                re = fftw_out[i][0];
                im = fftw_out[i][1];
                freq_energy_sum += 2 * (re*re + im*im);
            }
            freq_energy_sum /= size_of_fft;
            //std::cout << freq_energy_sum << std::endl;
            // norming
            freq_energy_sum /= (irl_data_size / 2);
            //std::cout << freq_energy_sum << std::endl;
            ////
            /*
            double freq_sum = 0;
            for (int i = N1; i <= N2; i++)
            {
                double re = fftw_out[i][0];
                double im = fftw_out[i][1];
                mod = sqrt(re*re + im*im) / irl_data_size;
                if (i != 0)
                    mod *= 2;
                //               std::cout << std::fixed << std::setw(20) << std::right << std::setprecision(15) << out[i][0] << "    " << out[i][1] << "    " << mod << std::endl;
                freq_sum += mod*mod;
            }
            //std::cout << freq_sum << std::endl;
            return freq_sum;
            */
        }
        */
#endif
       // std::cout << "sum  " << std::fixed << std::setw(20) << std::right << std::setprecision(15) << freq_sum << std::endl;
        return freq_energy_sum;
    }
    void Calibrator_FDMC::calc_h_matrix(const  TpnMagneticMeasurement * mag_meas, uint32_t irl_data_size, uint32_t size_of_fft, const double bias[], double H[], double &h)
    {
        double dx = 1;
        double  dy = 1;
        double  dz = 1;
        double d_bias_x[3] = { 1, 0.0, 0.0 };
        double d_bias_y[3] = {0.0, 1, 0.0};
        double d_bias_z[3] = { 0.0, 0.0, 1 };

        for (int i = 0; i < 3; i++)
        {
            d_bias_x[i] += bias[i];
            d_bias_y[i] += bias[i];
            d_bias_z[i] += bias[i];
        }

        h = calc_h(mag_meas, irl_data_size, size_of_fft, bias);
        double hx = calc_h(mag_meas, irl_data_size, size_of_fft, d_bias_x);
        double hy = calc_h(mag_meas, irl_data_size, size_of_fft, d_bias_y);
        double hz = calc_h(mag_meas, irl_data_size, size_of_fft, d_bias_z);

        //std::cout << std::fixed << std::setw(20) << std::right << std::setprecision(15) << bias[0] << "    " << bias[1] << "    " << bias[2] << "    " << h << std::endl;
        //std::cout << std::fixed << std::setw(20) << std::right << std::setprecision(15) << d_bias_x[0] << "    " << d_bias_x[1] << "    " << d_bias_x[2] << "    " << hx << std::endl;
        //std::cout << std::fixed << std::setw(20) << std::right << std::setprecision(15) << d_bias_y[0] << "    " << d_bias_y[1] << "    " << d_bias_y[2] << "    " << hy << std::endl;
        //std::cout << std::fixed << std::setw(20) << std::right << std::setprecision(15) << d_bias_z[0] << "    " << d_bias_z[1] << "    " << d_bias_z[2] << "    " << hz << std::endl;

        //std::cout << "====" << std::endl;


        double dhx = hx - h;
        double dhy = hy - h;
        double dhz = hz - h;

        H[0] = dhx / dx;
        H[1] = dhy / dy;
        H[2] = dhz / dz;
    }

    double Calibrator_FDMC::norm_v(double v[], int size)
    {
        double norm = 0;
        for (int i = 0; i < size; i++)
            norm += v[i]*v[i];
        return sqrt(norm);
    }

    void Calibrator_FDMC::vect_to_matrix(double v[], int size, double koef, double m[])
    {
        for (int i = 0; i < size; i++)
        {
            for (int j = 0; j < size; j++)
            {
                m[j + i * size] = koef  * v[i] * v[j];
            }
        }
    }

    void Calibrator_FDMC::create_window(double *window, long length)
    {
        const double pi = 3.14159265358979323846;
#ifdef HAMMING_WINDOW
        // Hamming window
        for (int i = 0; i < length; i++)
        {
            window[i] = 0.54 - 0.46 * cos( (2 * pi * i) / (length - 1) );
        }
#else
#ifdef BLACKMAN_WINDOW
        // Blackman window
        for (int i = 0; i < length; i++)
        {
            window[i] = 0.42 - 0.5 * cos((2 * pi * i) / (length - 1)) + 0.08 * cos((4 * pi * i) / (length - 1));
        }
#else
#ifdef BLACKMAN_HARRIS_WINDOW
        // Blackman-Hassir window
        for (int i = 0; i < length; i++)
        {
            window[i] = 0.35875 - 0.48829 * cos((2 * pi * i) / (length - 1)) + 0.14128 * cos((4 * pi * i) / (length - 1)) - 0.01168 * cos((6 * pi * i) / (length - 1));
        }
#else
        // rectangle window
        for (int i = 0; i < length; i++)
        {
            window[i] = 1;
    }
#endif
#endif
#endif
    }

}
