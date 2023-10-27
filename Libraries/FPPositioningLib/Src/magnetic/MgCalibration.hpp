/**
* \copyright       Copyright (C) SPIRIT Navigation, LLC., 2014
* \brief           Provides magnetic calibration during special procedure
* \ingroup         MFP 
* \file            MgCalibration.hpp
* \author          M.Frolov
* \date            23.07.2015
*/

#ifndef MAGNETIC_CALIBRATION_H
#define MAGNETIC_CALIBRATION_H

#include <eigen/Core>

namespace mg_calibration
{
    using namespace Eigen;
    enum Status
    {
        eUncalibrated,      //bias unknown
        eRoughBias,         //bias with var(bias) < 5uT
        eFineBias,          //bias with var(bias) < 1uT
        eFineAll,           //bias and soft iron corrections
        eError              //calibration error is occured
    };

    enum CalibrationType
    {
        eCalibrateBias,          //bias with var(bias) < 1uT
        eCalibrateAll            //bias and soft iron corrections
    };

    struct CalibrationParameters
    {
        // M = softiron*scale*(Mraw-bias)
        Vector4d hardiron;
        Vector3d scale;
        Matrix3d softiron;

        Matrix4d hardironCov;

        Matrix3d scaleCov;         //unimplemented yet
        Matrix3d softironCov;      //unimplemented yet

        Status status;
    };



    struct StateKF
    {
        StateKF()
        {
            X.setZero();
            P.setIdentity();
            Q.setIdentity();
            R = 5;
            init = false;
        }

        Vector4d X;
        Matrix4d P;
        Matrix4d Q;
        double R;
        bool init;
    };

    class MgCalibration
    {
        public:
            MgCalibration(CalibrationType type)
            {
                calibrationType = type;
                kf.P = 1 * kf.P;
                kf.Q = 1e-19 * kf.Q;

                ekf.X(3) = 40;
                ekf.P = 50 * ekf.P;
                ekf.Q = 0.0001 * ekf.Q;

                lastdata.setZero();
            }

            double process( const Vector3d &rawdata )
            {
                Status status(eUncalibrated);
                double progress;
                ++cnt;
                if ((rawdata-lastdata).norm()>7.0)
                {
                	iterationEKF(ekf, rawdata );
                	iterationKF(kf, rawdata);
                	lastdata = rawdata;
                }
                status = checkConsistance(kf, ekf);
                progress = estmateProgress(ekf);
                result.hardiron = ekf.X;
                result.status = status;
                return progress;
            }

            CalibrationParameters getCalibrationParams() const
            {

                return result;
            }


        private:
            CalibrationParameters result;
            StateKF stateKF;
            StateKF stateEKF;
            CalibrationType calibrationType;
            Vector3d lastdata;
            void iterationKF( StateKF &kf, const Vector3d &rawdata )
            {
                //I = diag( ones( 4, 1 ) );
                static const Matrix4d I = Matrix4d::Identity();
                Eigen::Matrix<double, 1, 4> H;

                //P = P + Q;
                kf.P += kf.Q;
                // H = [x^2+y^2+z^2; 2*x; 2*y; 2*z];
                H( 0 ) = pow( rawdata( 0 ), 2 ) + pow( rawdata( 1 ), 2 ) + pow( rawdata( 2 ), 2 );
                H( 1 ) = 2 * rawdata( 0 );
                H( 2 ) = 2 * rawdata( 1 );
                H( 3 ) = 2 * rawdata( 2 );

                //S = H * P * H'+R;
                double S = H * kf.P * H.transpose() + kf.R;
                //K = P*H'*inv( S );
                Vector4d K = kf.P * H.transpose() / S;
                //y = H * X;
                double y = H * kf.X;
                //X = X + K * ( Z - y );
                kf.X = kf.X + K * ( 1.0 - y );
                //P = ( I - K * H ) * P;
                kf.P = ( I - K * H ) * kf.P;
                //R = R + 0.05 * ( 0.2 + y ^ 2 - R );
                kf.R += 0.05 * ( 0.2 + y * y - kf.R );
            }


            void iterationEKF( StateKF &ekf, const Vector3d rawdata )
            {
                //    I = diag(ones(length(X),1));
                //
                //%     syms x0 y0 z0 r x y z;
                //%     hc = ( 1/(r^2)*((x-x0)^2 + (y-y0)^2 + (z-z0)^2) );
                //%     hm = matlabFunction(hc, 'vars', [x y z x0 y0 z0 r]);
                //%     Hm = matlabFunction(jacobian(hc,[x0, y0, z0, r]), 'vars', [x y z x0 y0 z0 r]);
                //%
                //%     hf = @(x,y,z,X)hm(x,y,z, X(1),X(2),X(3),X(4));
                //%     Hf = @(x,y,z,X)Hm(x,y,z, X(1),X(2),X(3),X(4));
                //
                //    P = P + Q;
                //
                //    H = Hf(Y(1),Y(2),Y(3), X);
                //
                //    S = H*P*H' + R;
                //    K = P*H'*inv(S);
                //    h = hf(Y(1),Y(2),Y(3), X);
                //    X = X + K*(Z-h);
                //
                //    P = (I - K*H)*P;
                //    R = R + 0.05*(0.2 + (Z-h)^2 - R);

                double x = rawdata( 0 );
                double y = rawdata( 1 );
                double z = rawdata( 2 );

                if (ekf.init == false)
                {
                    ekf.X( 0 ) = x;
                    ekf.X( 1 ) = y;
                    ekf.X( 2 ) = z;
                    ekf.init = true;
                }
                double x0 = ekf.X( 0 );
                double y0 = ekf.X( 1 );
                double z0 = ekf.X( 2 );
                double r  = ekf.X( 3 );

                double dx = x - x0;
                double dy = y - y0;
                double dz = z - z0;
                double r2 = r * r;

                static const Matrix4d I = Matrix4d::Identity();
                Eigen::Matrix<double, 1, 4> J = ekfJ( ekf.X, rawdata );

                ekf.P += ekf.Q;

                double S = J * ekf.P * J.transpose() + ekf.R;
                Vector4d K = ekf.P * J.transpose() / S;

                double h = 1 / r2 * ( dx * dx + dy * dy + dz * dz );

                ekf.X = ekf.X + K * ( 1.0 - h );
                ekf.P = ( I - K * J ) * ekf.P;
                ekf.R += 0.05 * ( 0.2 + ( 1.0 - h ) * ( 1.0 - h ) - ekf.R );
            }

            Eigen::Matrix<double, 1, 4> ekfJ( const Vector4d &X, const Vector3d &data )
            {
                Eigen::Matrix<double, 1, 4> J;
                double x = data( 0 );
                double y = data( 1 );
                double z = data( 2 );
                double x0 = X( 0 );
                double y0 = X( 1 );
                double z0 = X( 2 );
                double r  = X( 3 );

                double dx = x - x0;
                double dy = y - y0;
                double dz = z - z0;
                double r2 = r * r;
                //J = [ -(2*x - 2*x0)/r^2, -(2*y - 2*y0)/r^2, -(2*z - 2*z0)/r^2, -(2*((x - x0)^2 + (y - y0)^2 + (z - z0)^2))/r^3];
                J( 0 ) = -2 * dx / r2;
                J( 1 ) = -2 * dy / r2;
                J( 2 ) = -2 * dz / r2;
                J( 3 ) = -2 * ( dx * dx + dy * dy + dz * dz ) / ( r2 * r );

                return J;
            }

            // convert conic params  a(x^2 + y^2 + z^2) + bx + cy + dz = 1;
            // to geometric [x0, y0, z0, R]
            Vector4d conic2geometric(const Vector4d &conicParams) const
            {
                Vector4d result;
                double a = conicParams(0);
                double b = conicParams(1);
                double c = conicParams(2);
                double d = conicParams(3);
                result(0) = - b / a;
                result(1) = - c / a;
                result(2) = - d / a;
                result(3) = ( sqrt ( ( a + b*b + c*c + d*d ) / ( a*a ) ) ); 

                return result;
            }

            StateKF kf, ekf;
            unsigned int cnt;
            Status checkConsistance( const StateKF &kf, const StateKF &ekf )
            {
                Status result(eUncalibrated);
                Vector4d kf_geometric = conic2geometric(kf.X);
                Vector4d delta = kf_geometric - ekf.X;

                if (delta.norm() < 0.5 && ekf.P.maxCoeff() < varFine())
                {
                    result = eFineBias;
                }
                else if (delta.norm() < 5 && ekf.P.maxCoeff() < varRough())
                {
                    result = eRoughBias;
                }
                return result;
            }

            double estmateProgress(const StateKF &ekf)
            {
                double progress(0);
                double var = ekf.P.diagonal().norm();

                progress = 1. + log(varFine())/log(86.) - log(var)/log(86.) ;
                progress = std::max(0., progress);
                progress = std::min(1., progress);
                return progress;
            }

            double varFine() const
            {
                return 1.0;
            }

            double varRough() const
            {
                return 5.0;
            }

    };
}
#endif //MAGNETIC_CALIBRATION_H
