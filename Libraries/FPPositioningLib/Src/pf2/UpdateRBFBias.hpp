/**
* \copyright       Copyright (C) SPIRIT Navigation, LLC., 2014
* \brief           Magnetic bias estimator
* \ingroup         PF
* \file            UpdateRBFBias.hpp
* \author          M.Frolov
* \date            28.11.2014
*/

#ifndef UPDATER_RBF_BIAS_H
#define UPDATER_RBF_BIAS_H
#define _USE_MATH_DEFINES

#include "IPF.hpp"
#include "IFFSettings.hpp"
#include "../magnetic/MFP.h"
#include <eigen/LU>
#include <cmath>
#include <iostream>
#include "UKF.hpp"

namespace PF
{
    /**
    * This class implements Rao-Blackwellized magnetic bias estimator.
    * Particle filter state vector separates into two parts:\n
    * 1) variables that involved in non-linear operations:
    * \f$\{x,y,floor,h\}\f$\n
    * 2) variables that involved in linear operations:
    * \f$\{b_{x},b_{y},b_{z}\}\f$ - magnetic bias components\n
    * It is assumed that magnetic bias is a constant, the propagation model looks as follows:
    * \f$\mathbf{b_{i}}=\mathbf{b_{i-1}}+\mathbf{I}N(0,\sigma)\f$, \f$\mathbf{I}\f$ - [3x3] identity matrix\n
    * Magnetic vector stored in a correspondig MFP-map cell \f$\mathbf{B}_{map}\f$
    * and estimated bias are used to propagate the measurement vector wich is observed by mag sensor:
    * \f$\mathbf{z}= \mathbf{b} + \mathbf{C}(roll,pitch,h)\mathbf{B}_{map} + \mathbf{C}(roll,pitch,h)\mathbf{I}N(0,\sigma_{map})\f$\n
    * Where orientation matrix \f$\mathbf{C}(roll,pitch,h)\f$ can be decomposed as
    * \f$\mathbf{C}(roll,pitch,h)=\mathbf{C}_{m2p}(roll,pitch)\mathbf{R}_{z}(h)\f$\n
    * This module runs in two stages:\n
    * 1)Prediction stage - executes after PF prediction and before PF update stages:\n
    * \f$\mathbf{b}_{i,i-1}=\mathbf{b}_{i-1,i-1}\f$ - state vector prediction\n
    * \f$\mathbf{P}_{i,i-1}=\mathbf{P}_{i-1,i-1}+\mathbf{\Sigma}_{b}\f$ - state covariance matrix prediction,
    * \f$\mathbf{\Sigma}_{b}\f$ - state vector noise covariance matrix\n
    * \f$\mathbf{S}_{i}=\mathbf{P}_{i,i-1}+\mathbf{C}\mathbf{\Sigma}_{map}\mathbf{C}^{T}\f$ - measurement covariance matrix prediction,
    * \f$\mathbf{\Sigma}_{map}\f$ - mfp-map noise covariance matrix\n
    * \f$\mathbf{z}_{i}=\mathbf{b}_{i,i-1}+\mathbf{C}\mathbf{B}_{map}\f$ - measurement prediction\n
    * 2)Update stage - executes after PF update stage\n
    * \f$\mathbf{K}_{i}=\mathbf{P}_{i,i-1}\mathbf{I}\mathbf{S}_{i}^{-1}\f$ - calculate the kalman-gain\n
    * \f$\mathbf{b}_{i,i}=\mathbf{b}_{i,i-1}+\mathbf{K}_{i}(\mathbf{m}_{i}-\mathbf{z}_{i})\f$ - update bias vector,
    * \f$\mathbf{m}_{i}\f$ - current measurement vector\n
    * \f$\mathbf{P}_{i,i}=\mathbf{P}_{i,i-1}-\mathbf{K}_{i}\mathbf{I}\mathbf{P}_{i,i-1}\f$ - update state covariance matrix\n
    */
    class UpdateRBFBias : public RBFModel
    {
        public:
            UpdateRBFBias()
            {
                initialize( 0 );
            }

            UpdateRBFBias( IFFData<> *pIFFData )
            {
                initialize( pIFFData );
            }

            virtual ~UpdateRBFBias()
            {
                delete [] pMfpParticles3D;
                delete [] pVec;
            };

            void initialize( IFFData<> *pIFFData )
            {

                isEnabled = false;
                pIFF = pIFFData;
                pMfpParticles3D = 0;
                pVec = 0;
                mfpSize = 0;
                name = "UpdaterRBFBias";
                D_min = 10;
                s2_m = 1;// 2.5 * 2.5;// 1 * 1; // 1[uT^2]
                s2_psi   = pow( 2 * M_PI / 180, 2 ); // std = 2[deg]
                s2_tetta = pow( 2 * M_PI / 180, 2 );
                s2_phi   = pow( 2 * M_PI / 180, 2 );
                Q_add = 0;
                h_step = 0;
                start_stop = false;
                kf_enabled = false;
                kf_enabled_when_no_motion = true;
                cnt = 0;
                kS = 1.0;
            }

            void setIFF( IFFData<> *pIFFData )
            {
                pIFF = pIFFData;
            }

            void setKf( bool enable )
            {
                kf_enabled = enable;
            }

            void setKfWhenNoMotion(bool enable)
            {
                kf_enabled_when_no_motion = enable;
            }

            virtual bool enabled()
            {
                return isEnabled;
            }

            void  setUpdate( bool flag )
            {
                isEnabled = flag;
            }

            void setName( const std::string &objName )
            {
                name = objName;
            }

            void setD( const Float sig )
            {
                D_min = sig;
            }

            void setQadd( const Float sig )
            {
                Q_add = sig * sig;
            }

            void setHstep( const Float h )
            {
                h_step = h;
            }


            void print( std::ostream &os ) const
            {
                bool attitude( false ), is_motion( false );
                double m_x( 0 ), m_y( 0 ), m_z( 0 );
                Eigen::Matrix<Float, 3, 3> Ci2l = Eigen::Matrix<Float, 3, 3>::Zero(); //instrumental to local
                void * ptr( 0 );

                if ( pIFF != 0 )
                {
                    ptr = pIFF->getMFP();
                    attitude = pIFF->getPrediction().quat_valid;
                    is_motion = pIFF->getPrediction().is_motion;
                    m_x = pIFF->getMagMeas().m( 0 );
                    m_y = pIFF->getMagMeas().m( 1 );
                    m_z = pIFF->getMagMeas().m( 2 );
                    Ci2l = pIFF->getPrediction().qi2l.toRotationMatrix();
                }

                os << name << "<UpdRBFBias>" << ": " << (isEnabled ? 1 : 0) << " " << ptr << " " << mfpSize << " "
                    << attitude << " " << is_motion << " " << kf_enabled << " " << kf_enabled_when_no_motion << " "
                    << m_x << " " << m_y << " " << m_z << " " << std::endl;
                //os << Ci2l << std::endl;
            }

        private:
            virtual void propagate( IFilter *pFilter )
            {
                bool attitude( false ), is_motion( false ), has_map( false );
                Eigen::Matrix<Float, 3, 3> Ci2l = Eigen::Matrix<Float, 3, 3>::Zero(); //instrumental to local

                if ( pIFF != 0 )
                {
                    has_map = ( pIFF->getMFP() != 0 );
                    attitude = pIFF->getPrediction().quat_valid;
                    is_motion = pIFF->getPrediction().is_motion;
                    is_motion &= pIFF->getMagMeas().valid; //!TODO WARNING MAKE THIS CORRECT
                    Ci2l = pIFF->getPrediction().qi2l.toRotationMatrix();
                }

                bool ok = has_map && is_motion && attitude == true;
                bool ok_stop = has_map && !is_motion && attitude == true;
                bool ok_mag = has_map && attitude == true;

                if ( enabled() && ok_stop && kf_enabled && kf_enabled_when_no_motion)
                {
                    Particle *state_est = pFilter->getState();
                    Particle *state_prop = pFilter->getPrediction();
                    int N = pFilter->getParcticlesCount();
                    Eigen::Matrix<Float, 3, 3> Rz = Eigen::Matrix<Float, 3, 3>::Zero();
                    Eigen::Matrix<Float, 3, 3> H, C, Cinv;
                    Eigen::Matrix<Float, 3, 1> m, b;

                    Rz( 2, 2 ) = 1;

                    resizeMfpArrays( N );

                    for ( int i = 0; i < N; ++i )
                    {
                        pMfpParticles3D[i].X = state_prop[i].pos.x * 100;
                        pMfpParticles3D[i].Y = state_prop[i].pos.y * 100;
                        pMfpParticles3D[i].Floor = ( int )floor( state_prop[i].pos.level + 0.5 );
                    }

                    ( pIFF->getMFP() )->GetMgVector( pMfpParticles3D, pVec, N );

                    auto sigmas = []( const Eigen::Matrix<Float, 5, 1> &x_a, const Eigen::Matrix<Float, 5, 5> &P_a, const double & c )
                    {
                        Eigen::Matrix<Float, 5, 5> A ( P_a.llt().matrixL() );
                        A = c * A;
                        Eigen::Matrix<Float, 5, 5> Y;

                        for ( int i = 0; i < x_a.rows(); ++i )
                        {
                            Y.block<5, 1>( 0, i ) = x_a;
                        }

                        Eigen::Matrix<Float, 5, 11> X;
                        X.block<5, 1>( 0, 0 ) = x_a;
                        X.block<5, 5>( 0, 1 ) = Y + A;
                        X.block<5, 5>( 0, 6 ) = Y - A;

                        return X;
                    };

                    auto hmeas = [&]( const double & h, const Eigen::Matrix<Float, 3, 1>  &v )
                    {
                        Eigen::Matrix<Float, 3, 1> y;

                        //std::cout << "ref h " << h << std::endl;
                        //std::cout << "ref v " << v << std::endl;


                        Rz( 0, 0 ) = cos( h ); Rz( 0, 1 ) = -sin( h );
                        Rz( 1, 0 ) = sin( h ); Rz( 1, 1 ) = cos( h );

                        C = ( Rz * Ci2l ).transpose();
                        //std::cout << "ref C " << C << std::endl;

                        y = C * m + b + v; // mecause m is from map
                        //std::cout << "ref y " << y << std::endl;

                        return y;
                    };

                    for ( int i = 0; i < N; ++i )
                    {
                        PF::Float h = state_prop[i].h;
                        Rz( 0, 0 ) = cos( h ); Rz( 0, 1 ) = -sin( h );
						Rz(1, 0) = -Rz(0, 1); Rz(1, 1) = Rz(0, 0);
                        //Rz( 1, 0 ) = sin( h ); Rz( 1, 1 ) = cos( h );

                        //state_prop[i].rbf = state_est[i].rbf;
                        b = state_prop[i].rbf.b;

                        m( 0 ) = pVec[i].x.mu1;
                        m( 1 ) = pVec[i].y.mu1;
                        m( 2 ) = pVec[i].z.mu1;
                        Cinv = Rz * Ci2l;

                        C = Cinv.transpose(); //C = C.inverse();


                        double P = pow( 4 * M_PI / 180, 2 );
                        double Q = pow( 10 * M_PI / 180, 2 );

                        if ( start_stop == false )
                        {
                            state_prop[i].ph = P;
                        }

                        Eigen::Matrix<Float, 3, 1> z;
                        z = pIFF->getMagMeas().m;
                        //z( 0 ) = m_x;
                        //z( 1 ) = m_y;
                        //z( 2 ) = m_z;
                        b = state_prop[i].rbf.b;
                        Eigen::Matrix<Float, 1, 1> h0;
                        h0( 0 ) = h;
                        Eigen::Matrix<Float, 1, 1> P0;
                        P0( 0 ) = state_prop[i].ph;

                        //ukf( h, state_prop[i].ph, z, Q, state_prop[i].rbf.S );
                        auto ukf2 = [&]( PF::Float & h, PF::Float & P, Eigen::Matrix<Float, 3, 1> &z, const PF::Float & Q, const Eigen::Matrix<Float, 3, 3> &R )
                        {
                            int L = 1;

                            PF::Float alpha = 0.5;   //default, tunable
                            PF::Float ki = 0;        //default, tunable
                            PF::Float beta = 2;      //default, tunable
                            PF::Float lambda = alpha * alpha * ( L + ki ) - L;

                            PF::Float gamma = sqrt( L + lambda );

                            //Kalman filtering and neural networks p233
                            Eigen::Matrix<Float, 3, 1> Xi, Xi_star, Xik, Wm, Wc;
                            Wm( 0 ) = lambda / ( L + lambda );
                            Wc( 0 ) = lambda / ( L + lambda ) + 1 - alpha * alpha + beta;
                            Wm( 1 ) = Wm( 2 ) = Wc( 1 ) = Wc( 2 ) = 0.5 / ( L + lambda );


                            Xi( 0 ) = h;
                            //Xi(1) = h + gamma*sqrt(P);
                            //Xi(2) = h - gamma*sqrt(P);
                            Xi( 1 ) = h + gamma * sqrt( Q );
                            Xi( 2 ) = h - gamma * sqrt( Q );

                            Xi_star = Xi;
                            //std::cout << "ref Xp " << Xi_star << std::endl;

                            PF::Float x_minus = 0;

                            for ( int i = 0; i <= 2 * L; ++i ) x_minus += Wm( i ) * Xi( i );

                            PF::Float P_minus = Q;

                            for ( int i = 0; i <= 2 * L; ++i ) P_minus += Wc( i ) * ( Xi( i ) - x_minus ) * ( Xi( i ) - x_minus );

                            //std::cout << "ref xp " << x_minus << std::endl;
                            //std::cout << "ref Px " << P_minus << std::endl;


                            Xik( 0 ) = Xi_star( 0 );
                            //Xik(1) = Xi_star(0) + gamma*sqrt(P_minus);
                            //Xik(2) = Xi_star(0) - gamma*sqrt(P_minus);
                            Xik( 1 ) = Xi_star( 0 ) + gamma * sqrt( Q );
                            Xik( 2 ) = Xi_star( 0 ) - gamma * sqrt( Q );

                            //std::cout << "ref Xik " << Xik << std::endl;

                            const Eigen::Matrix<Float, 3, 1>  v = Eigen::Matrix<Float, 3, 1>::Zero();
                            Eigen::Matrix<Float, 3, 3>  Yi_minus;

                            for ( int i = 0; i <= 2 * L; ++i )
                            {
                                //std::cout << "ref i " << i << std::endl;
                                Yi_minus.block<3, 1>( 0, i ) = hmeas( Xik( i ), v );
                            }

                            //std::cout << "ref Yp " << Yi_minus << std::endl;

                            Eigen::Matrix<Float, 3, 1>  y_minus = Eigen::Matrix<Float, 3, 1>::Zero();

                            for ( int i = 0; i <= 2 * L; ++i )
                                y_minus += Wm( i ) * Yi_minus.block<3, 1>( 0, i );

                            //std::cout << "ref yp " << y_minus << std::endl;

                            Eigen::Matrix<Float, 3, 3> Pyy = R;

                            for ( int i = 0; i <= 2 * L; ++i )
                                Pyy += Wc( i ) * ( Yi_minus.block<3, 1>( 0, i ) - y_minus ) * ( Yi_minus.block<3, 1>( 0, i ) - y_minus ).transpose();

                            //std::cout << "ref Py " << Pyy << std::endl;



                            Eigen::Matrix<Float, 1, 3> Pxy = Eigen::Matrix<Float, 1, 3>::Zero();

                            for ( int i = 0; i <= 2 * L; ++i )
                                Pxy += Wc( i ) * ( Xik( i ) - x_minus ) * ( Yi_minus.block<3, 1>( 0, i ) - y_minus ).transpose();

                            //std::cout << "ref Pxy " << Pxy << std::endl;


                            Eigen::Matrix<Float, 1, 3> K;

                            K = Pxy * Pyy.inverse();

                            h = x_minus + K * ( z - y_minus );

                            P = P_minus - K * Pyy * K.transpose();

                            return h;
                        };
                        ukf2( h, state_prop[i].ph, z, Q, state_prop[i].rbf.S );
                        state_prop[i].h = h;
                    }

                    if ( start_stop == false )
                    {
                        start_stop = true;
                    }
                }


                if ( enabled() && ( ( ok_mag && kf_enabled ) || ok ) )
                {
                    Particle *state_est = pFilter->getState();
                    Particle *state_prop = pFilter->getPrediction();
                    int N = pFilter->getParcticlesCount();
                    Eigen::Matrix<Float, 3, 3> I = Eigen::Matrix<Float, 3, 3>::Identity();
                    Eigen::Matrix<Float, 3, 3> D = Eigen::Matrix<Float, 3, 3>::Zero();
                    Eigen::Matrix<Float, 3, 3> Rz = Eigen::Matrix<Float, 3, 3>::Zero();
                    Eigen::Matrix<Float, 3, 3> Sm = s2_m * Eigen::Matrix<Float, 3, 3>::Identity(); //measurement covariance matrix
                    Eigen::Matrix<Float, 3, 3> C, Cinv, CDCt;
                    Eigen::Matrix<Float, 3, 1> m;
                    static const Float B = 0.025;//0.25;

                    start_stop = false;

                    Rz( 2, 2 ) = 1;

                    resizeMfpArrays( N );

                    //Eigen::Matrix<Float, 3, 11> ww = Eigen::Matrix<Float, 3, 11>::Ones();
                    //Eigen::Matrix<Float, 3, 1> yy = Eigen::Matrix<Float, 3, 1>::Zero();

                    //auto yyy = ww.colwise().sum();
                    //std::cout << ww.rowwise().sum() << std::endl;


                    for ( int i = 0; i < N; ++i )
                    {
                        pMfpParticles3D[i].X = state_prop[i].pos.x * 100;
                        pMfpParticles3D[i].Y = state_prop[i].pos.y * 100;
                        pMfpParticles3D[i].Floor = ( int )floor( state_prop[i].pos.level + 0.5 );
                    }

                    ( pIFF->getMFP() )->GetMgVector( pMfpParticles3D, pVec, N );

                    Eigen::Matrix<Float, 3, 1> y = pIFF->getMagMeas().m;

                    for ( int i = 0; i < N; ++i )
                    {
                        double h = state_prop[i].h;
                        Rz( 0, 0 ) = cos( h ); Rz( 0, 1 ) = -sin( h );
						Rz(1, 0) = -Rz(0, 1); Rz(1, 1) = Rz(0, 0);
                        //Rz( 1, 0 ) = sin( h ); Rz( 1, 1 ) = cos( h );

                        state_prop[i].rbf.b = state_est[i].rbf.b;
                        state_prop[i].rbf.P = state_prop[i].rbf.P + B * B * I;
                        m( 0 ) = pVec[i].x.mu1;
                        m( 1 ) = pVec[i].y.mu1;
                        m( 2 ) = pVec[i].z.mu1;
                        Cinv = Rz * Ci2l;

                        C = Cinv.transpose(); //C = C.inverse();



                        Float sx = std::max( ( Float )pVec[i].x.s1, D_min );
                        Float sy = std::max( ( Float )pVec[i].y.s1, D_min );
                        Float sz = std::max( ( Float )pVec[i].z.s1, D_min );

                        Float f = std::min( ( ( pVec[i].x.s1 * pVec[i].x.s1 + pVec[i].y.s1 * pVec[i].y.s1 + pVec[i].z.s1 * pVec[i].z.s1 ) / ( 3 * D_min * D_min ) ), PF::Float( 1. ) );
                        f = std::max( f, PF::Float( 0.2 ) );

                        D.setZero();




                        D( 0, 0 ) = sx * sx;// + Q_add;
                        D( 1, 1 ) = sy * sy;// + Q_add;
                        D( 2, 2 ) = sz * sz;
                        CDCt = C * D * Cinv;


                        PF::Float kk = state_prop[i].rbf.C.trace() / ( CDCt ).trace();

                        if ( kk > 1 ) kk = 1;
                        else if ( kk < f ) kk = f;

                        //kk = 1;
                        state_prop[i].rbf.S = state_prop[i].rbf.P + kk * CDCt + Sm;
                        state_prop[i].rbf.z = state_prop[i].rbf.b + C * m;

                        kf_state st;
                        st.b = state_prop[i].rbf.b1;
                        st.P = state_prop[i].rbf.P1;
                        st.Q = B * B * Eigen::Matrix<Float, 3, 3>::Identity();
                        //st.H = Eigen::Matrix<Float, 3, 3>::Identity();

                        st.Sig = Eigen::Matrix<Float, 3, 3>::Zero();
                        st.alpha = state_prop[i].rbf.alpha;
                        st.beta = state_prop[i].rbf.beta;

                        Eigen::Matrix<Float, 3, 1> y1 = y - C * m;
                        bv_akf( st, y1 );
                        state_prop[i].rbf.b1 = st.b;
                        state_prop[i].rbf.P1 = st.P;

                        {
                            //KFP
                            Eigen::Matrix<Float, 3, 1> xs, ys;
                            xs( 0 ) = state_prop[i].rbf.S( 0, 0 );
                            xs( 1 ) = state_prop[i].rbf.S( 1, 1 );
                            xs( 2 ) = state_prop[i].rbf.S( 2, 2 );
                            ys( 0 ) = st.Sig( 0, 0 );
                            ys( 1 ) = st.Sig( 1, 1 );
                            ys( 2 ) = st.Sig( 2, 2 );
                            const double SS_2 = 200;
                            Eigen::Matrix<Float, 3, 3> Pxs = SS_2 * Eigen::Matrix<Float, 3, 3>::Identity();
                            Eigen::Matrix<Float, 3, 3> K, Sxs, Qxs = Pxs;

                            Pxs += Qxs;
                            Sxs = Pxs + st.PSig;
                            K = Pxs * Sxs.inverse();

                            xs = xs + K * ( ys - xs );


                            double k2 = xs.sum() / ( state_prop[i].rbf.P + CDCt + Sm ).trace();

                            state_prop[i].rbf.S = state_prop[i].rbf.P + k2 * CDCt + Sm;
                        }

                    }
                }
            }

            virtual void update( IFilter *pFilter )
            {
                bool attitude( false ), is_motion( false ), has_map( false );
                Eigen::Matrix<Float, 3, 3> Ci2l = Eigen::Matrix<Float, 3, 3>::Zero(); //instrumental to local

                if ( pIFF != 0 )
                {
                    has_map = ( pIFF->getMFP() != 0 );
                    attitude = pIFF->getPrediction().quat_valid;
                    is_motion = pIFF->getPrediction().is_motion;
                    is_motion &= pIFF->getMagMeas().valid;
                    Ci2l = pIFF->getPrediction().qi2l.toRotationMatrix();
                }

                bool ok = has_map && (is_motion || cnt++ < 0 ) && attitude == true;

                if( enabled() && ok )
                {
                    int N = pFilter->getParcticlesCount();
                    resizeMfpArrays( N );
                    Particle *state_est = pFilter->getState();
                    Particle *state_prop = pFilter->getPrediction();
                    Eigen::Matrix<Float, 3, 1> m = pIFF->getMagMeas().m;

                    for( int i = 0; i < N; ++i )
                    {
                        Eigen::Matrix<Float, 3, 3> K, P = state_prop[i].rbf.P;
                        Eigen::Matrix<Float, 3, 3> S = state_prop[i].rbf.S;
                        Eigen::Matrix<Float, 3, 1> z = state_prop[i].rbf.z;
                        //Eigen::Matrix<Float,3,1> d_z = m-z;
                        //Float w = 1/(sqrt(pow(2*M_PI,3))*S.norm())*exp(-1./2*d_z.transpose()*S.inverse()*d_z);
                        //state_est[i].w *=w;
                        K =  P * S.inverse();
                        //if (S.maxCoeff() < 1000*1000)
                        {
                            state_prop[i].rbf.b = state_prop[i].rbf.b + K * ( m - z );
                            state_prop[i].rbf.P = P - K * P;

                            //state_prop[i].rbf.b = state_prop[i].rbf.b1;
                            //state_prop[i].rbf.P = state_prop[i].rbf.P1;

                        }
                    }
                }
            }

            void resizeMfpArrays( int new_size )
            {
                if( new_size != mfpSize )
                {
                    delete [] pMfpParticles3D;
                    delete [] pVec;

                    pMfpParticles3D = new tMFP_Particle3D[new_size];
                    pVec     = new tMFP_CellDescrApprox[new_size];
                    mfpSize = new_size;
                }
            }
            struct kf_state
            {
                Eigen::Matrix<Float, 3, 1> b;
                Eigen::Matrix<Float, 3, 3> P;
                Eigen::Matrix<Float, 3, 3> Q;
                Eigen::Matrix<Float, 3, 3> H;
                Eigen::Matrix<Float, 3, 1> Cu;
                Eigen::Matrix<Float, 3, 3> V;
                Float v;

                Eigen::Matrix<Float, 3, 3> Sig;
                Eigen::Matrix<Float, 3, 1> alpha;
                Eigen::Matrix<Float, 3, 1> beta;
                Eigen::Matrix<Float, 3, 3> PSig;


            };

            void bv_akf( kf_state & state, const Eigen::Matrix < Float, 3, 1> &y )
            {
                const int N = 3;
                const Float rho = 1 - exp( -4.0 );
                Eigen::Matrix<Float, 3, 1> beta_ = rho * state.beta;
                Eigen::Matrix<Float, 3, 1> xk, x_ = state.b;
                Eigen::Matrix<Float, 3, 3> Pk, P_ = state.P + state.Q;

                Eigen::Matrix<Float, 3, 1> alpha_k = 0.5 * Eigen::Matrix<Float, 3, 1>::Ones() + rho * state.alpha;
                Eigen::Matrix<Float, 3, 1> beta_k = beta_;
                Eigen::Matrix<Float, 3, 3> Sig_k = Eigen::Matrix<Float, 3, 3>::Zero();
                Sig_k( 0, 0 ) = beta_k( 0 ) / alpha_k( 0 );
                Sig_k( 1, 1 ) = beta_k( 1 ) / alpha_k( 1 );
                Sig_k( 2, 2 ) = beta_k( 2 ) / alpha_k( 2 );

                Eigen::Matrix<Float, 3, 3>  K;
                Eigen::Matrix < Float, 3, 1> y_xk;

                for ( int i = 0; i < N; ++i )
                {
                    K = P_ * ( ( P_ + Sig_k ).inverse() );
                    xk = x_ + K * ( y - x_ );
                    Pk = P_ - K * P_;
                    y_xk = ( y - xk );
                    beta_k = beta_ + 0.5 * y_xk.cwiseProduct( y_xk );
                    beta_k( 0 ) += 0.5 * Pk( 0, 0 );
                    beta_k( 1 ) += 0.5 * Pk( 1, 1 );
                    beta_k( 2 ) += 0.5 * Pk( 2, 2 );
                    Sig_k( 0, 0 ) = beta_k( 0 ) / alpha_k( 0 );
                    Sig_k( 1, 1 ) = beta_k( 1 ) / alpha_k( 1 );
                    Sig_k( 2, 2 ) = beta_k( 2 ) / alpha_k( 2 );
                }

                state.b = xk;
                state.P = Pk;
                state.Sig = Sig_k;
                state.alpha = alpha_k;
                state.beta = beta_k;
                state.PSig.setZero();

                for ( int i = 0; i < 3; ++i )
                {
                    Float num = beta_k( i ) * beta_k( i );
                    Float a1 = std::max( alpha_k( i ) - 1, PF::Float( 1. ) );
                    Float a2 = std::max( alpha_k( i ) - 2, PF::Float( 1. ) );
                    state.PSig( i, i ) = num / ( a1 * a1 * a2 );
                }
            }

            IFFData<> *pIFF;
            bool  isEnabled;
            tMFP_Particle3D *pMfpParticles3D;
            tMFP_CellDescrApprox *pVec;
            int mfpSize;
            std::string name;
            Float D_min;
            Float s2_m; //measurement noise
            Float s2_psi, s2_tetta, s2_phi; //angle variance
            Float Q_add; //angle discrette compensation
            Float h_step;
            bool start_stop;
            bool kf_enabled;
            bool kf_enabled_when_no_motion;
            int cnt;
            double kS;

    };
}

#endif
