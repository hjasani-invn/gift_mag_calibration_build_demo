#ifndef ISTDESTIMATOR_HPP
#define  ISTDESTIMATOR_HPP

struct SimpleMeas
{
    double w;
    double v;
    double s2;
};

namespace Fpbl
{
    enum eStdEstimatorType
    {
        eUnknownStdEstimator,
        eRobustStdEstimator,
        eQuantileStdEstimator,
        eCombineStdEstimator
    };
   
    class IStdEstimator
    {
    public:
        virtual ~IStdEstimator() {};
        virtual double estimate(const std::vector<SimpleMeas> &processed_cell, double &mean, double &width, bool &flag_alternative) = 0;
    };
}

#endif //  ISTDESTIMATOR_HPP
