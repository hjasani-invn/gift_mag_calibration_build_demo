#include "MagReCalibrators.hpp"
#include <algorithm> 

namespace TrackPreProcessing
{

	MagBias ReCalibrator::CalculateBias(const std::vector<TpnOutput> &irl_data, Fpbl::MfpBuilder::LocalDB* pDbmfp, const BaseVenueType venue)
    {
		const double uT_to_mG = 10.0;

		// stats
		double weighted_sum0 = 0, weighted_sum1 = 0, weighted_sum2 = 0;
		double weighted_sum_square0 = 0, weighted_sum_square1 = 0, weighted_sum_square2 = 0;
		double w = 0;
		double w0 = 0, w1 = 0, w2 = 0;
		// double sumw = 0;
		double sumw0 = 0, sumw1 = 0, sumw2 = 0;

		// estimate bias for each epoch
		TpnConverter converter(venue);
		std::vector<double> biases0, biases1, biases2;
		for (TpnOutput irl_data_item : irl_data)
		{
			if (irl_data_item.position.sigma_east < 2 && irl_data_item.position.sigma_north < 2 // exclude when std of position is high
				&& irl_data_item.pdr.stride_length > 0 // exclude dwells
				&& irl_data_item.mag_meas.is_valid // exclude invalid mag measurements
				)
			{
				FfPosition ff_position = converter.ConvertPositionData(irl_data_item.timestamp, irl_data_item.position);
				MagneticMeasurement udf_mag = converter.ConvertMagneticData(irl_data_item.timestamp, irl_data_item.mag_meas);
				MffAttitude mff_attitude = converter.ConvertAttitudeData(irl_data_item.timestamp, irl_data_item.attitude);

				Eigen::Matrix<double, 3, 1> Z;
				TpnMagneticMeasurement mag_meas = irl_data_item.mag_meas;
				Z(0) = mag_meas.mX / uT_to_mG;
				Z(1) = mag_meas.mY / uT_to_mG;
				Z(2) = mag_meas.mZ / uT_to_mG;

				Eigen::Matrix<double, 3, 1> B;
				Fpbl::MfpBuilder::DBRecord mag_record;
				Fpbl::CoordinatesInGrid fpbl_position = { ff_position.x, ff_position.y, ff_position.floor_number };
				if (pDbmfp->getRecord(fpbl_position, &mag_record) == Fpbl::ReturnStatus::STATUS_SUCCESS
					&& mag_record.x.s1 < 15 && mag_record.y.s1 < 15 && mag_record.z.s1 < 15 // exclude when std of cell is high (CHANGED 5 to 15 for experiment)
					)
				{
					B(0) = mag_record.x.mu1;
					B(1) = mag_record.y.mu1;
					B(2) = mag_record.z.mu1;
				}
				else
				{
					continue;
				}

				Eigen::Matrix<double, 3, 3> C = Eigen::Matrix<double, 3, 3>::Zero();
				C = Eigen::Quaternion<double>(mff_attitude.quaternion[0], mff_attitude.quaternion[1], mff_attitude.quaternion[2], mff_attitude.quaternion[3]).matrix();

				Eigen::Matrix<double, 3, 1> b;
				b = Z - C.inverse() * B;

				biases0.push_back(b(0));
				biases1.push_back(b(1));
				biases2.push_back(b(2));

				switch (weightMethod)
				{
					case UNWEIGHTED:
						w = 1.0;
						w0 = w1 = w2 = w;
						break;

					case WEIGHTED_STD_MAG_XYZ_STD_POS_XYZ:
						w = 1 / sqrt(pow(mag_record.x.s1, 2) + pow(mag_record.y.s1, 2) + pow(mag_record.z.s1, 2)) * 1 / sqrt(pow(irl_data_item.position.sigma_east, 2) + pow(irl_data_item.position.sigma_north, 2) + pow(irl_data_item.position.sigma_altitude, 2));
						w0 = w1 = w2 = w;
						break;

					case WEIGHTED_STD_MAG_XYZ_STD_POS_XY:
						w = 1 / sqrt(pow(mag_record.x.s1, 2) + pow(mag_record.y.s1, 2) + pow(mag_record.z.s1, 2)) * 1 / sqrt(pow(irl_data_item.position.sigma_east, 2) + pow(irl_data_item.position.sigma_north, 2));
						w0 = w1 = w2 = w;
						break;

					case WEIGHTED_STD_PER_I_MAG_XYZ_POS_XYZ:
						w0 = (1 / fabs(mag_record.x.s1)) * (1 / fabs(irl_data_item.position.sigma_east));
						w1 = (1 / fabs(mag_record.y.s1)) * (1 / fabs(irl_data_item.position.sigma_north));
						w2 = (1 / fabs(mag_record.z.s1)) * (1 / fabs(irl_data_item.position.sigma_altitude));
						break;

					case WEIGHTED_STD_PER_I_MAG_XYZ_POS_XY:
						w0 = (1 / fabs(mag_record.x.s1)) * (1 / fabs(irl_data_item.position.sigma_east));
						w1 = (1 / fabs(mag_record.y.s1)) * (1 / fabs(irl_data_item.position.sigma_north));
						w2 = (1 / fabs(mag_record.z.s1));
						break;

					case WEIGHTED_STD_MAG_XYZ:
						w = 1 / sqrt(pow(mag_record.x.s1, 2) + pow(mag_record.y.s1, 2) + pow(mag_record.z.s1, 2));
						w0 = w1 = w2 = w;
						break;
						
				}

				weighted_sum0 += w0*b(0);
				weighted_sum1 += w1*b(1);
				weighted_sum2 += w2*b(2);

				sumw0 += w0;
				sumw1 += w1;
				sumw2 += w2;

				weighted_sum_square0 += w0*pow(b(0), 2);
				weighted_sum_square1 += w1*pow(b(1), 2);
				weighted_sum_square2 += w2*pow(b(2), 2);
			}
		}

		// calculate stats
		assert(biases0.size() == biases1.size());
		assert(biases0.size() == biases2.size());
		assert(biases1.size() == biases2.size());

		int N = biases0.size();
		// mean
		double weighted_mean0 = sumw0 == 0 ? 0 : weighted_sum0 / sumw0;
		double weighted_mean1 = sumw1 == 0 ? 0 : weighted_sum1 / sumw1;
		double weighted_mean2 = sumw2 == 0 ? 0 : weighted_sum2 / sumw2;
		// variance
		double weighted_var0 = sumw0 == 0 ? 1e6 : weighted_sum_square0 / sumw0 - pow(weighted_mean0, 2);
		double weighted_var1 = sumw1 == 0 ? 1e6 : weighted_sum_square1 / sumw1 - pow(weighted_mean1, 2);
		double weighted_var2 = sumw2 == 0 ? 1e6 : weighted_sum_square2 / sumw2 - pow(weighted_mean2, 2);
		// standard deviation
		double weighted_std0 = sqrt(weighted_var0);
		double weighted_std1 = sqrt(weighted_var1);
		double weighted_std2 = sqrt(weighted_var2);

		// create mag bias struct and fill it
		MagBias traj_bias;
		traj_bias.bias[0] = weighted_mean0;
		traj_bias.bias[1] = weighted_mean1;
		traj_bias.bias[2] = weighted_mean2;

		traj_bias.covarianceMatrix[0][0] = weighted_std0;
		traj_bias.covarianceMatrix[1][1] = weighted_std1;
		traj_bias.covarianceMatrix[2][2] = weighted_std2;

		traj_bias.calibration_level = 1;
		if (weighted_std0 < 2.0 && weighted_std1 < 2.0 && weighted_std2 < 2.0) 
			traj_bias.calibration_level = 5;
		else if (weighted_std0 < 4.0 && weighted_std1 < 4.0 && weighted_std2 < 4.0)
			traj_bias.calibration_level = 4;
		else if (weighted_std0 < 6.0 && weighted_std1 < 6.0 && weighted_std2 < 6.0)
			traj_bias.calibration_level = 3;
		else if (weighted_std0 < 8.0 && weighted_std1 < 8.0 && weighted_std2 < 8.0)
			traj_bias.calibration_level = 2;
		else if (weighted_std0 < 10.0 && weighted_std1 < 10.0 && weighted_std2 < 10.0)
			traj_bias.calibration_level = 1;
		else
			traj_bias.calibration_level = 0;

		return traj_bias;
    }


}
