/**
* \copyright       Copyright (C) InvenSense
* \brief           RBPF initialzer (augumented state of PF)
* \ingroup         PF
* \file            InitializerNormal.hpp
* \author          M.Frolov
* \date            23.03.2016
*/

#ifndef PF_RBF_INITIALIZER_HPP
#define PF_RBF_INITIALIZER_HPP

#include <ostream>
#include <string>
#include "IPF.hpp"


namespace PF
{

///Initialize RBF part of state vector
class RBFInitializer
{
public:
    /// default constructor, assumes bias is unknown
    RBFInitializer() : bias(0, 0, 0), t(0), sig(1000), name("rbf")
    {
    }

    /**
    * sets new magnetic bias
    * \param[in] mgBias 3-components vector [x,y,z], [uT]
    */
    void updateBias(Eigen::Matrix<Float, 3, 1> mgBias)
    {
        bias = mgBias;
    }

    /**
    * increments bias uncertainty
    * \param[in] mgSig  uncertainty increment
    */
    void incrementSig(Float mgSig)
    {
        if (sig < 1.0) sig = mgSig;
        else if (sig < 1000) sig *= mgSig;

        //else
        //sig+=mgSig;
        //sig*=mgSig;
    }

    /**
    * sets new bias uncertainty
    * \param[in] mgSig  bias std [uT]
    */
    void updateSig(Float mgSig)
    {
        sig = mgSig;
    }

    /**
    * \return default RBF state
    */
    RBF<Float>  getDefault() const
    {
        RBF<Float> result(100);
        //result.S = 1E+10 * result.P;
        result.S = result.P;
        return result;
    }

    /** updates current time and internal state (increase uncertainty)
    * \param[in] bias age [ms]
    */
    void updateTime(int64_t time)
    {
        Float k = 1. / (60 * 60 * 24 * 1000) * 50; //50uT / ( ms in 24h)
        Float s = sig + k * time;
        sig = std::min(Float(10000), s);
        t = time;
    }

    /// \return current timestamp
    int64_t getTime()
    {
        return t;
    }

    /// \return RBF object
    RBF<Float> getRBF() const
    {
        Float s = std::max(Float(1.0), sig);
        s = std::min(Float(1000.0), s);
        RBF<Float> result(s);
        result.b = bias;
        result.S = 1E+10 * result.P;
        return result;
    }

    void print(std::ostream &os) const
    {
        RBF<Float> rbf = getRBF();
        os << "Initializer: " << name << " "
            << t << " "
            << sig << " "
            << std::endl
            << rbf.b << std::endl
            << rbf.P << std::endl
            << rbf.S << std::endl;
    }

private:
    Eigen::Matrix<Float, 3, 1> bias;
    int64_t t;
    Float sig;
    std::string name;
};

}

#endif