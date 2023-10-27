#ifndef MAG_RECALIBRATORS
#define MAG_RECALIBRATORS

#include<fstream>
#include<math.h>
#include <vector>
#include <list>

#include "MagCalibrators.hpp"
#include "MfpBuilder.hpp"
#include "tpn_converter.hpp"
#include <eigen/Geometry>

enum RecalibrationWeightMethod {
	UNWEIGHTED,							// w = 1
	WEIGHTED_STD_MAG_XYZ_STD_POS_XYZ,	// wa // w  = 1 / sqrt(pow(mag_record.x.s1, 2) + pow(mag_record.y.s1, 2) + pow(mag_record.z.s1, 2)) * 1 / sqrt(pow(irl_data_item.position.sigma_east, 2) + pow(irl_data_item.position.sigma_north, 2) + pow(irl_data_item.position.sigma_altitude, 2));
	WEIGHTED_STD_MAG_XYZ_STD_POS_XY,	// wb // w  = 1 / sqrt(pow(mag_record.x.s1, 2) + pow(mag_record.y.s1, 2) + pow(mag_record.z.s1, 2)) * 1 / sqrt(pow(irl_data_item.position.sigma_east, 2) + pow(irl_data_item.position.sigma_north, 2));
	WEIGHTED_STD_PER_I_MAG_XYZ_POS_XYZ,	// wc // wi = (1 / fabs(mag_record.i.s1)) * (1 / fabs(irl_data_item.position.sigma_i));
	WEIGHTED_STD_PER_I_MAG_XYZ_POS_XY,	// wd // wi = (1 / fabs(mag_record.i.s1)) * (1 / fabs(irl_data_item.position.sigma_i)); /* except for wz don't use position.sigma
	WEIGHTED_STD_MAG_XYZ,				// we // w  = 1 / sqrt(pow(mag_record.x.s1, 2) + pow(mag_record.y.s1, 2) + pow(mag_record.z.s1, 2)) ;
};

namespace TrackPreProcessing
{

    class ReCalibrator
    {
    public:
		ReCalibrator(RecalibrationWeightMethod weightMethod) : weightMethod(weightMethod) {};
		MagBias CalculateBias(const std::vector<TpnOutput> &irl_data, Fpbl::MfpBuilder::LocalDB* pDbmfp, const BaseVenueType venue);

    private:
		RecalibrationWeightMethod weightMethod;
    };

}
#endif
