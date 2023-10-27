#ifndef MAG_CALIBRATORS
#define MAG_CALIBRATORS

#include<fstream>
#include<math.h>
#include <vector>
#include <list>

#define EIGENFFT
#ifdef EIGENFFT
//#define EIGEN_FFTW_DEFAULT  // use fftw as backend
#include <unsupported/Eigen/FFT>
#else
/* currently this back end is not used
#include <fftw3.h>
*/
#endif
#define F_SAMPLE  20   //%Hz - sample rate
#define F_LOW     0.6
#define F_HIGH    3 //3

//#define REMOVE_ZERO_FREQ

#define APPLY_WINDOW
#ifdef APPLY_WINDOW
//#define HAMMING_WINDOW
#ifndef HAMMING_WINDOW
//#define BLACKMAN_WINDOW
#ifndef BLACKMAN_WINDOW
#define BLACKMAN_HARRIS_WINDOW
#endif
#endif
#endif


#include "TpnData.hpp"
#include "TrackValidator.hpp"

namespace TrackPreProcessing
{

    struct MagBias
    {
        double bias[3];                     ///< mag bias error covariance matrix [mT^2], column order
        double covarianceMatrix[3][3];      ///< mag bias error covariance matrix [mT^2], column order
        uint8_t calibration_level;          ///< calibration level
        MagBias() : calibration_level(0)
        {
            for (int i = 0; i < 3; i++)
            {
                bias[i] = covarianceMatrix[i][0] = covarianceMatrix[i][1] = covarianceMatrix[i][2] = 0;
            };
        };
    };

    class Calibrator_FDMC
    {
    public:
        Calibrator_FDMC(){};
        MagBias CalculateBias(const std::vector<TpnOutput> &irl_data);

    private:
//        MagBias calculateBiasWitnGlobalSearch(const std::list<TpnOutput> &irl_data, const MagBias mg_bias);
        MagBias calculateBiasWitnGlobalSearchFromStart(const std::list<TpnOutput> &irl_data, const MagBias mg_bias);
        MagBias calculateBiasWitnGlobalSearchToEnd(const std::list<TpnOutput> &irl_data, const MagBias mg_bias);
        MagBias calculateNewBiasWithInitBias(const  TpnMagneticMeasurement * mag_meas, uint32_t irl_data_size, uint32_t size_of_fft, const MagBias mg_bias);
        double calc_h(const  TpnMagneticMeasurement * mag_meas, uint32_t irl_data_size, uint32_t size_of_fft, const double bias[]);
        void calc_h_matrix(const  TpnMagneticMeasurement * mag_meas, uint32_t irl_data_size, uint32_t size_of_fft, const double bias[], double H[], double &h);
        double norm_v(double v[], int size);
        void vect_to_matrix(double v[], int size, double koef, double m[]);
        void create_window(double *win, long len);

#ifdef EIGENFFT
        std::vector<double> irl_data_mod;
#else
        fftw_plan fftw_p;
        fftw_complex *fftw_out;
        double *fftw_in;
#endif
#ifdef APPLY_WINDOW
        double *window;
#endif
    };

}
#endif
