/**
* \copyright       Copyright (C) SPIRIT Navigation, LLC., 2014
* \brief           PF state vector defenition.
* \ingroup         PF
* \file            Particle.hpp
* \author          M.Frolov
* \date            28.11.2014
*/

#ifndef PARTICLE_H
#define PARTICLE_H

#include <cmath>
#include <eigen/Core>

namespace PF
{
    template<class T> struct Position
    {
        Position(): x(0), y(0), level(0) {};
        T x, y, level;

        friend Position &operator+=(Position &L, const Position &R )
        {
            L.x += R.x;
            L.y += R.y;
            L.level += R.level;
            return L;
        };

        friend Position operator-(const Position &L, const Position &R )
        {
            Position result;
            result.x = L.x - R.x;
            result.y = L.y - R.y;
            result.level = L.level - R.level;
            return result;
        };

        friend Position operator*(const Position &L, const Position &R )
        {
            Position result;
            result.x = L.x * R.x;
            result.y = L.y * R.y;
            result.level = L.level * R.level;
            return result;
        };



        friend Position operator*(const Position &L, const T &w )
        {
            Position result;
            result.x = L.x*w;
            result.y = L.y*w;
            result.level = L.level*w;
            return result;
        };

        //friend T norm(const Position &L, const Position &R )
        //{
        //    T result;
        //    result = sqrt( (L.x - R.x)*(L.x - R.x) + (L.y - R.y)*(L.y - R.y) + (L.level - R.level)*(L.level - R.level) );
        //    return result;
        //}

        friend T norm2D(const Position &L, const Position &R )
        {
            T result;
            result = sqrt( (L.x - R.x)*(L.x - R.x) + (L.y - R.y)*(L.y - R.y) );
            return result;
        }


    };

    template<class T > struct RBF
    {
        RBF(T sig)
        {
            b.setZero();
            z.setZero();
            P = sig*sig*Eigen::Matrix<T,3,3>::Identity();
            S = P;
            C = 0.01*P;
            b1 = b;
            P1 = P;
            alpha.setOnes();
            alpha *= 10;
            beta = 100 * alpha;
        };
        Eigen::Matrix<T,3,1> b;
        Eigen::Matrix<T,3,1> z;
        Eigen::Matrix<T,3,3> P;
        Eigen::Matrix<T,3,3> S;
        Eigen::Matrix<T,3,3> C;

    
        Eigen::Matrix<T, 3, 1> b1;
        Eigen::Matrix<T, 3, 3> P1;
        Eigen::Matrix<T, 3, 1> alpha;
        Eigen::Matrix<T, 3, 1> beta;


        friend RBF<T> operator-(const RBF<T> &L, const RBF<T> &R )
        {
            RBF<T> result;
            result.b = L.b-R.b;
            result.z = L.z-R.z;
            result.P = L.P-R.P;
            result.S = L.S-R.S;
            result.C = L.C-R.C;

            result.b1 = L.b1 - R.b1;
            result.P1 = L.P1 - R.P1;
            result.alpha = L.alpha - R.alpha;
            result.beta = L.beta - R.beta;

            return result;
        }
        RBF<T> cwiseProduct(const RBF<T> &R) const
        {
            RBF<T> result;
            result.b = b.cwiseProduct(R.b);
            result.z = z.cwiseProduct(R.z);
            result.P = P.cwiseProduct(R.P);
            result.S = S.cwiseProduct(R.S);
            result.C = C.cwiseProduct(R.C);


            result.b1 = b1.cwiseProduct(R.b1);
            result.P1 = P1.cwiseProduct(R.P1);
            result.alpha = alpha.cwiseProduct(R.alpha);
            result.beta = beta.cwiseProduct(R.beta);
        }
    };

    template<class T> struct ParticleT
    {
        ParticleT() : pos(), h(0), w(0), rbf(100), ph(0), kl(1), lkh(0), portal_type(false) {};
        ParticleT(T s) : pos(), h(0), w(0), rbf(s), ph(0), kl(1), lkh(0), portal_type(false) {};
        Position<T> pos;    // position
        T h;                // heading
        T w;                // weigth
        RBF<T> rbf;
        T ph;
        T kl;
        bool   lkh;
        bool   portal_type;

        friend ParticleT<T> operator-(const ParticleT<T> &L, const ParticleT<T> &R )
        {
            ParticleT<T> result;
            result.pos = L.pos - R.pos;
            result.rbf = L.rbf - R.rbf;
            result.h = angle_diff(L.h, R.h);
            result.kl = L.kl - R.kl;

            return result;
        };

        ParticleT<T> cwiseProduct(const ParticleT<T> &R) const
        {
            ParticleT<T> result;
            result.pos = this->pos * R.pos;
            result.rbf = this->rbb.cwiseProduct( R.rbf );
            result.h = this->h*R.h;
            result.kl = this->kl*R.kl;
            return result;
        }

    };

    template <class T> T angle_diff(T a1, T a2)
    {
        T dA = fmod( a1 - a2, 2*M_PI );
        if (dA > M_PI) dA -= 2*M_PI;
        if (dA <-M_PI) dA += 2*M_PI;

        return dA;
    }

    typedef double Float;

    typedef ParticleT<Float> Particle;

}

#endif //PARTICLE_H
