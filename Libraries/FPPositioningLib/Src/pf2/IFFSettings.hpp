#ifndef IFF_SETTINGS_HPP
#define IFF_SETTINGS_HPP
#include <stdint.h>
#include <queue>
#include <eigen/Geometry>
#include "../magnetic/MFP.h"
#include "Particle.hpp"
#include "../Map_Matching/MapMatching.hpp"

template <typename T = PF::Float> struct Prediction
{
    Prediction() : 
        number (0), t(0), x(T(0)), y (T(0)), floor(T(0)),
        pos_cov(Eigen::Matrix<T, 3, 3>::Zero()),
        qi2l(Eigen::Quaternion<T>(T(0), T(0), T(0), T(0))), // 
        C_avg(Eigen::Matrix<T, 3, 3>::Zero()),
        mis_std(T(0)),
        is_motion(true), is_transit(false), quat_valid(false)
    {};

    uint64_t number;
    int64_t t;
    T x, y, floor;
    Eigen::Matrix<T, 3, 3> pos_cov;
    Eigen::Quaternion<T> qi2l;
    Eigen::Matrix<T, 3, 3> C_avg;
    T mis_std;
    bool is_motion;

    bool is_transit;
    bool quat_valid;
};

struct PositionWithUncertainties
{
    double x;               ///< local x [m]
    double y;               ///< local y [m]
    double z;               ///< floor number [-]
    double heading;         ///< frames missaligment [rad]
    double cov_xy[2][2];    ///< local coordinates covarinace matrix [m^2]
    double std_z;           ///< floor number std [-]
    double std_h;           ///< frames missaligment std [rad]
    int64_t timestamp;      ///< timestamp [ms]
    bool valid;
};


template<typename T = PF::Float> struct MagMeasurements
{
    MagMeasurements() : m( Eigen::Matrix<T, 3, 1>::Zero() ), timestamp( 0 ), valid( false ) {};
    Eigen::Matrix<T, 3, 1> m;
    int64_t timestamp;
    bool valid;
};

/**
* This interface is used for generalization of obtaining input data for PF
*/
template<typename T = PF::Float> class IFFData
{
    public:
        virtual ~IFFData() {};
        virtual const Prediction<T>& getPrediction() const = 0;
        virtual const MagMeasurements<T>& getMagMeas() const = 0;
        virtual const PositionWithUncertainties& getExternalPosition() const = 0;
        //virtual const tMFP* & getMFP() const = 0;
        virtual tMFP * const & getMFP() const = 0;
        virtual MapMatching * const & getMapMatching() const = 0;
};


#endif//IFF_SETTINGS_HPP
