#include <math.h>
#include <algorithm>
#include "stat_func.hpp"

double kurtosis(std::vector<double> &vec)  /*  throw( std::string )*/
{
    std::vector<double> centre_vec(0);
    double x = 0;
    double x2 = 0;
    double mean = 0;
    double s1 = 0;
    double s2 = 0;
    double kurt;
    size_t N = vec.size();

    for (auto it = vec.cbegin(); it != vec.cend(); ++it)
    {
        mean += *it;
    }
    mean /= N;

    for (auto it = vec.cbegin(); it != vec.cend(); ++it)
    {
        x = *it - mean;
        centre_vec.push_back(x);
    }
    for (auto it = centre_vec.cbegin(); it != centre_vec.cend(); ++it)
    {
        x = *it;
        x2 = x * x;
        s1 += (x2*x2);
        s2 += x2;
    }
    kurt = (s1 / (s2*s2)) * N;

    return kurt;
}


double quantile(std::vector<double> &vec, double level /* 0.01 - 0.99 */)  /*  throw( std::string )*/
{
    auto begin = vec.begin();
    auto end = vec.end();
    std::size_t size = end - begin;
    std::size_t index = static_cast<std::size_t>(size / (1 / level));
    auto target = begin + index;
    std::nth_element(begin, target, end);
    return *target;
}

template<class RandAccessIter>
double median(RandAccessIter begin, RandAccessIter end)
{
    std::size_t size = end - begin;
    std::size_t middleIdx = size / 2;
    RandAccessIter target = begin + middleIdx;
    std::nth_element(begin, target, end);

    if (size % 2 != 0) //Odd number of elements
    {
        return *target;
    }
    else             //Even number of elements
    {
        double a = *target;
        RandAccessIter targetNeighbor = target - 1;
        std::nth_element(begin, targetNeighbor, end);
        return (a + *targetNeighbor) / 2.0;
    }
}

double robustEstimateVar(const std::vector<SimpleMeas> &processed_cell, const double &mean)
{
    static const double DENUMERATOR_VALUE_MIN = 1e-6;

    double numerator = 0.;
    double denumerator = 0.;
    double result = 0.;

    for (auto it = processed_cell.cbegin(); it != processed_cell.cend(); ++it)
    {
        double d = it->v - mean;
        double w = it->w;
        numerator += w * d * d;
        denumerator += w;
    }

    if (denumerator < DENUMERATOR_VALUE_MIN)
    {
        std::vector<double> vec(0);

        for (auto it = processed_cell.cbegin(); it != processed_cell.cend(); ++it)
        {
            vec.push_back(std::abs(it->v - mean));
        }
        result = median(vec.begin(), vec.end()) / 0.6745;
        result *= result;
    }
    else
    {
        result = numerator / denumerator;
    }
    return result;
}
