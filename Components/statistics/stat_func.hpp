#ifndef STAT_FUNC_HPP
#define  STAT_FUNC_HPP

#include <vector>
#include "IStdEstimator.hpp"

template<class RandAccessIter>
double median(RandAccessIter begin, RandAccessIter end);
double robustEstimateVar(const std::vector<SimpleMeas> &processed_cell, const double &mean);
double quantile(std::vector<double> &vec, double level /* 0.01 - 0.99 */);
double kurtosis(std::vector<double> &vec);

#endif
