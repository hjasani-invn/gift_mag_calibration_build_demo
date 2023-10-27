#ifndef  STDESTIMATORCREATOR_HPP
#define  STDESTIMATORCREATOR_HPP
#include "IStdEstimator.hpp"

namespace Fpbl
{
    class StdEstimatorCreator
    {
    public:
        virtual ~StdEstimatorCreator() {};
        IStdEstimator*  create(eStdEstimatorType stdEstimatorType);
		IStdEstimator*  create_mag_quality_estimator();
    private:
        IStdEstimator* ptrEstimator = NULL;
		IStdEstimator* ptrQualityEstimator = NULL;
    };
}

#endif //  STDESTIMATORCREATOR_HPP
