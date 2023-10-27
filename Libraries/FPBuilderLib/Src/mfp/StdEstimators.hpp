#ifndef STDESTIMATORS_HPP
#define  STDESTIMATORS_HPP
#include "IStdEstimator.hpp"

namespace Fpbl
{

    class RobustStdEstimator : public IStdEstimator
    {
    public:
        virtual double estimate(const std::vector<SimpleMeas> &processed_cell, double &mean, double &width, bool &flag_alternative);

    };
    class QuantileStdEstimator : public IStdEstimator
    {
    public:
        virtual double estimate(const std::vector<SimpleMeas> &processed_cell, double &mean, double &width, bool &flag_alternative);

    };
    class CombineStdEstimator : public IStdEstimator
    {
    public:
        virtual double estimate(const std::vector<SimpleMeas> &processed_cell, double &mean, double &width, bool &flag_alternative);

    };
	class MagQualityEstimator : public IStdEstimator
	{
	public:
		virtual double estimate(const std::vector<SimpleMeas> &processed_cell, double &mean, double &width, bool &flag_alternative);

	};
}


#endif //  STDESTIMATORS_HPP
