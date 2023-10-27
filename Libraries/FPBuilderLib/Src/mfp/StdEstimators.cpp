#include <map>
#include <list>
#include <math.h>
#include <algorithm>
#include <iostream>
#include <string>
#include <armadillo>

#include "stat_func.hpp"
#include "StdEstimators.hpp"
#include "StdEstimatorCreator.hpp"

using namespace Fpbl;

IStdEstimator* StdEstimatorCreator::create(eStdEstimatorType stdEstimatorType)
{
    if (ptrEstimator != NULL)
    {
        delete ptrEstimator;
        ptrEstimator = NULL;
    }

    if (stdEstimatorType == eRobustStdEstimator)
        ptrEstimator = new RobustStdEstimator();
    else if (stdEstimatorType == eQuantileStdEstimator)
        ptrEstimator = new QuantileStdEstimator();
    else if (stdEstimatorType == eCombineStdEstimator)
        ptrEstimator = new CombineStdEstimator();
    else
    {
        ptrEstimator = new RobustStdEstimator();
    }
    return ptrEstimator;
}

IStdEstimator* StdEstimatorCreator::create_mag_quality_estimator()
{
	if (ptrQualityEstimator != NULL)
	{
		delete ptrQualityEstimator;
		ptrQualityEstimator = NULL;
	}

	ptrQualityEstimator = new MagQualityEstimator();
	
	return ptrQualityEstimator;
}


double RobustStdEstimator::estimate(const std::vector<SimpleMeas> &processed_cell, double &mean, double &width, bool &flag_alternative)
{
    flag_alternative = false;
    double s2 = robustEstimateVar(processed_cell, mean);
    double std = sqrt(s2);
    return std;
}

double QuantileStdEstimator::estimate(const std::vector<SimpleMeas> &processed_cell, double &mean, double &width, bool &flag_alternative)
{
    static const double level = 0.1;  // level must be >= 0.01 and < 0.5
    std::vector<double> vec(0);
    flag_alternative = false;

    for (auto it = processed_cell.cbegin(); it != processed_cell.cend(); ++it)
    {
        vec.push_back(it->v);
    }
    double left = quantile(vec, level);
    double right = quantile(vec, (1 - level));
    double weight = right - left;
    weight *= (1 + 2*level);
    return weight;
}

double CombineStdEstimator::estimate(const std::vector<SimpleMeas> &processed_cell, double &mean, double &width, bool &flag_alternative)
{
    static const double level = 0.1;  // level must be >= 0.01 and < 0.5
    std::vector<double> vec(0);
    flag_alternative = false;

    double s2 = robustEstimateVar(processed_cell, mean);
    double std = sqrt(s2);

    for (auto it = processed_cell.cbegin(); it != processed_cell.cend(); ++it)
    {
        vec.push_back(it->v);
    }
    size_t N = vec.size();

    double kurt = kurtosis(vec);

    double left = quantile(vec, level);
    double right = quantile(vec, (1 - level));
    width = right - left;
#if 0
    weight /= (1 - 2 * level);
    double t = std::min(3., kurt);
    double w = std::max(0., (t - 1.8) / (3 - 1.8));
    double result = w * std + (1 - w) * weight;
#else
    static const double kurt_threshold = 2.5;
    static const double ratio_width_std = 2.0;
    double result = 0;

    double true_mean = 0;
    for (auto it = vec.cbegin(); it != vec.cend(); ++it)
    {
        true_mean += *it;
    }
    true_mean /= N;
    double true_std = 0;
    for (auto it = vec.cbegin(); it != vec.cend(); ++it)
    {
        true_std += (*it - true_mean)*(*it - true_mean);
    }
    true_std /= N;
    true_std = sqrt(true_std);


    //width *= (1. + 2. * level);
    //std::cout << kurt;
    //std::cout << " , " << std;
    //std::cout << " , " << true_std;
    //std::cout << " , " << width;
    //std::cout << std::endl;
    if (kurt > kurt_threshold)
    {
        result = std;
    }
    else
    {
        double sigma_q = width / (2 * 1.282);
        //if ((width > ratio_width_std * std) /* && (width > 40)*/)
        if ((sigma_q > ratio_width_std * std) /* && (width > 40)*/)
        {
            //std::cout << kurt;
            //std::cout << " , " << std;
            //std::cout << " , " << true_std;
            //std::cout << " , " << width;
            //std::cout << " , " << mean;
            //std::cout << std::endl;
            flag_alternative = true;
            mean = (right + left) / 2;//true_mean;
            //mean = true_mean;
            //std::cout << "  true_mean: " << mean;
            result = sigma_q;
            if (result > 59)
                result = 59;
        }
        else
            result = std;
    }
#endif
    return result;
}

double MagQualityEstimator::estimate(const std::vector<SimpleMeas> &processed_cell, double &mean, double &width, bool &flag_alternative)
{
	static const double level = 0.1;  // level must be >= 0.01 and < 0.5
	std::vector<double> vec(0);
	flag_alternative = false;

	double s2 = robustEstimateVar(processed_cell, mean);
	double std = sqrt(s2);

	for (auto it = processed_cell.cbegin(); it != processed_cell.cend(); ++it)
	{
		vec.push_back(it->v);
	}
	size_t N = vec.size();

	double kurt = kurtosis(vec);

	double left = quantile(vec, level);
	double right = quantile(vec, (1 - level));
	width = right - left;
	width /= (1 - 2 * level);
	double t = std::min(3., kurt);
	double w = std::max(0., (t - 1.8) / (3 - 1.8));
	double result = w * std + (1 - w) * width;

	return result;
}