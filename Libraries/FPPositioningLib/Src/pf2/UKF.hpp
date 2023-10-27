#ifndef UKF_HPP
#define UKF_HPP
#include "eigen/LU"
template <class T, int L> static inline  Eigen::Matrix < T, L, 2 * L + 1 > sigmas(
    const Eigen::Matrix<T, L, 1> &x_a,
    const Eigen::Matrix<T, L, L> &P_a,
    const T & c
)
{
    Eigen::Matrix<T, L, L> A( P_a.llt().matrixL() );
    A = c * A;
    Eigen::Matrix<T, L, L> Y;

    for ( int i = 0; i < x_a.rows(); ++i )
    {
        Y.block<L, 1>( 0, i ) = x_a;
    }

    Eigen::Matrix < T, L, 2 * L + 1 > X;
    X.block<L, 1>( 0, 0 ) = x_a;
    X.block<L, L>( 0, 1 ) = Y + A;
    X.block<L, L>( 0, L + 1 ) = Y - A;

    return X;
};


template<bool N>
struct init_augmented // true
{
    template<typename T, typename T2 >
    static inline void init( T &t, T2 &t2 )
    {
        t = t2;
    }
};

template<>
struct init_augmented<false>
{
    template<typename T, typename T2 >
    static inline void init( T &t, T2 &t2 )
    {
    }
};

template<class T, class TF, class TH, int Nx, int Ny, int Nv, int Nn> inline void ukf_t(
    TF &F,                                  //transition func
    Eigen::Matrix<T, Nx, 1> &x,             //state vector
    Eigen::Matrix<T, Nx, Nx> &P,            //state cov matrix
    const Eigen::Matrix<T, Nx, Nx> &Q,      //state noise
    TH &H,                                  //observation func
    const Eigen::Matrix<T, Ny, 1> &z,       //observation
    const Eigen::Matrix<T, Ny, Ny> &R,       //observation noise
    Eigen::Matrix<T, Nx, 1> &x_p,             //state vector
    Eigen::Matrix<T, Nx, Nx> &P_p,            //state cov matrix
    Eigen::Matrix<T, Ny, 1> &y_p,             //state vector
    Eigen::Matrix<T, Ny, Ny> &P_y            //state cov matrix
)
{
    const int L = Nx + Nv + Nn;     //num of augmented states
    const int NS = 2 * L + 1;       //num of sigma points
    //const T alpha = 0.9;            //default, tunable
    //const T ki = 0;                 //default, tunable
    //const T beta = 0.05;               //default, tunable
    const T alpha = 0.5;            //default, tunable
    const T ki = 0;                 //default, tunable
    const T beta = 2;               //default, tunable
    const T lambda = alpha * alpha * (L + ki) - L;
    const T c2 = L + lambda;
    const T c = sqrt( c2 );

    Eigen::Matrix<T, 1, NS> Wm;
    Eigen::Matrix<T, Nx, NS> Wmx;
    Eigen::Matrix<T, Ny, NS> Wmy;
    Wm( 0 ) = lambda / c2;

    for ( int i = 1; i < NS; ++i )
    {
        Wm( i ) = 0.5 / c2;
    }

    for ( int i = 0; i < Ny; ++i )
    {
        Wmy.block<1, NS>( i, 0 ) = Wm;
    }

    for ( int i = 0; i < Nx; ++i )
    {
        Wmx.block<1, NS>( i, 0 ) = Wm;
    }


    Eigen::Matrix<T, 1, NS> Wc = Wm;
    Wc( 0 ) += ( 1. - alpha * alpha + beta );


    Eigen::Matrix<T, L, 1> xa = Eigen::Matrix<T, L, 1>::Zero();
    xa.block<Nx, 1>( 0, 0 ) = x;

    Eigen::Matrix<T, L, L> Pa = Eigen::Matrix<T, L, L>::Zero();
    Pa.block<Nx, Nx>( 0, 0 ) = P;
    if (Nx == Nv)
        init_augmented<Nx == Nv>::init( Pa.block<Nv, Nv>( Nx, Nx ), Q );
    if (Ny == Nn)
        init_augmented<Ny == Nn>::init(Pa.block<Ny, Ny>(Nx + Nv, Nx + Nv), R);


    //time update
    Eigen::Matrix<T, L, NS> Xa = sigmas<T, L>( xa, Pa, c );
    //Eigen::Matrix<T, Nx, NS> Xp = F(Xa);
    //Eigen::Matrix<T, Nx, 1> xp = Wm.cwiseProduct(Xp).rowwise().sum();
    Xa.block<Nx, NS>( 0, 0 ) = F( Xa );

    //std::cout << "new Xp " << Xa.block<Nx, NS>(0, 0)  << std::endl;

    Eigen::Matrix<T, Nx, 1> xp = Wmx.cwiseProduct( Xa.block<Nx, NS>( 0, 0 ) ).rowwise().sum();
    Eigen::Matrix<T, Nx, Nx> Px = Eigen::Matrix<T, Nx, Nx>::Zero();

    init_augmented<Nv == 0>::init(Px, Q);
    for ( int i = 0; i < NS; ++i )
    {
        Px += Wc( i ) * ( Xa.block<Nx, 1>( 0, i ) - xp ) * ( Xa.block<Nx, 1>( 0, i ) - xp ).transpose();
    }

    x_p = xp;
    P_p = Px;
    //std::cout << "new xp " << xp << std::endl;
    //std::cout << "new Px " << Px << std::endl;

    if (Nv == 0)
    {
        init_augmented<Nv == 0>::init(Xa.block<Nx, NS>(0, 0), sigmas<T, Nx>(Xa.block<Nx, 1>(0, 0), Px, c));
        //Xa.block<Nx, NS>(0, 0) = sigmas<T,Nx>(Xa.block<Nx, 1>(0, 0), Q, c);
    }
    //std::cout << "new Xik " << Xa << std::endl;

    Eigen::Matrix<T, Ny, NS> Yp = H( Xa );
    //std::cout << "new Yp " << Yp << std::endl;
    Eigen::Matrix<T, Ny, 1> yp = Wmy.cwiseProduct(Yp).rowwise().sum();

    //meas update
    Eigen::Matrix<T, Ny, Ny> Pyy = Eigen::Matrix<T, Ny, Ny>::Zero();

    init_augmented<Nn == 0>::init(Pyy, R);
    for (int i = 0; i < NS; ++i)
    {
        Pyy += Wc( i ) * ( Yp.block<Ny, 1>( 0, i ) - yp ) * ( Yp.block<Ny, 1>( 0, i ) - yp ).transpose();
    }
    //Pyy += 0.01*R;
    P_y = Pyy;
    y_p = yp;
    //std::cout << "new yp " << yp << std::endl;
    //std::cout << "new Py " << Pyy << std::endl;

    Eigen::Matrix<T, Nx, Ny> Pxy = Eigen::Matrix<T, Nx, Ny>::Zero();

    for ( int i = 0; i < NS; ++i )
    {
        Pxy += Wc( i ) * ( Xa.block<Nx, 1>( 0, i ) - xp ) * ( Yp.block<Ny, 1>( 0, i ) - yp ).transpose();
    }
    //std::cout << "new Pxy " << Pxy << std::endl;

    Eigen::Matrix<T, Nx, Ny> K = Pxy * Pyy.inverse();
    x = xp + K * ( z - yp );
    P = Px - K * Pyy * K.transpose();

}


template<class T, class TF, class TH, int Nx, int Ny, int Nv, int Nn > inline void ukf_tt(
    TF &F,                                  //transition func
    Eigen::Matrix<T, Nx, 1> &x,             //state vector
    Eigen::Matrix<T, Nx, Nx> &P,            //state cov matrix
    const Eigen::Matrix<T, Nx, Nx> &Q,      //state noise
    TH &H,                                  //observation func
    const Eigen::Matrix<T, Ny, 1> &z,       //observation
    const Eigen::Matrix<T, Ny, Ny> &R,       //observation noise
    Eigen::Matrix<T, Nx, 1> &x_p,             //state vector
    Eigen::Matrix<T, Nx, Nx> &P_p,            //state cov matrix
    Eigen::Matrix<T, Ny, 1> &y_p,             //state vector
    Eigen::Matrix<T, Ny, Ny> &P_y            //state cov matrix
    )
{
    const int L = Nx + Nv + Nn;     //num of augmented states
    const int NS = 2 * L + 1;       //num of sigma points
    const T alpha = 0.9;            //default, tunable
    const T ki = 0;                 //default, tunable
    const T beta = 0.05;               //default, tunable
    //const T alpha = 0.5;            //default, tunable
    //const T ki = 0;                 //default, tunable
    //const T beta = 2;               //default, tunable
    const T lambda = alpha * alpha * (L + ki) - L;
    const T c2 = L + lambda;
    const T c = sqrt(c2);

    Eigen::Matrix<T, 1, NS> Wm;
    Eigen::Matrix<T, Nx, NS> Wmx;
    Eigen::Matrix<T, Ny, NS> Wmy;
    Wm(0) = lambda / c2;

    for (int i = 1; i < NS; ++i)
    {
        Wm(i) = 0.5 / c2;
    }

    for (int i = 0; i < Ny; ++i)
    {
        Wmy.block<1, NS>(i, 0) = Wm;
    }

    for (int i = 0; i < Nx; ++i)
    {
        Wmx.block<1, NS>(i, 0) = Wm;
    }


    Eigen::Matrix<T, 1, NS> Wc = Wm;
    Wc(0) += (1. - alpha * alpha + beta);


    Eigen::Matrix<T, L, 1> xa = Eigen::Matrix<T, L, 1>::Zero();
    xa.block<Nx, 1>(0, 0) = x;

    Eigen::Matrix<T, L, L> Pa = Eigen::Matrix<T, L, L>::Zero();
    Pa.block<Nx, Nx>(0, 0) = P;
    if (Nx == Nv)
        init_augmented<Nx == Nv>::init(Pa.block<Nv, Nv>(Nx, Nx), Q);
    if (Ny == Nn)
        init_augmented<Ny == Nn>::init(Pa.block<Ny, Ny>(Nx + Nv, Nx + Nv), R);


    //time update
    Eigen::Matrix<T, L, NS> Xa = sigmas<T, L>(xa, Pa, c);
    //Eigen::Matrix<T, Nx, NS> Xp = F(Xa);
    //Eigen::Matrix<T, Nx, 1> xp = Wm.cwiseProduct(Xp).rowwise().sum();
    Xa.block<Nx, NS>(0, 0) = F(Xa);

    //std::cout << "new Xp " << Xa.block<Nx, NS>(0, 0)  << std::endl;

    Eigen::Matrix<T, Nx, 1> xp = Wmx.cwiseProduct(Xa.block<Nx, NS>(0, 0)).rowwise().sum();
    Eigen::Matrix<T, Nx, Nx> Px = Eigen::Matrix<T, Nx, Nx>::Zero();

    init_augmented<Nv == 0>::init(Px, Q);
    for (int i = 0; i < NS; ++i)
    {
        Px += Wc(i) * (Xa.block<Nx, 1>(0, i) - xp) * (Xa.block<Nx, 1>(0, i) - xp).transpose();
    }

    x_p = xp;
    P_p = Px;
    //std::cout << "new xp " << xp << std::endl;
    //std::cout << "new Px " << Px << std::endl;

    if (Nv == 0)
    {
        init_augmented<Nv == 0>::init(Xa.block<Nx, NS>(0, 0), sigmas<T, Nx>(Xa.block<Nx, 1>(0, 0), Px, c));
        //Xa.block<Nx, NS>(0, 0) = sigmas<T,Nx>(Xa.block<Nx, 1>(0, 0), Q, c);
    }
    //std::cout << "new Xik " << Xa << std::endl;

    Eigen::Matrix<T, Ny, NS> Yp = H(Xa);
    //std::cout << "new Yp " << Yp << std::endl;
    Eigen::Matrix<T, Ny, 1> yp = Wmy.cwiseProduct(Yp).rowwise().sum();

    //meas update
    Eigen::Matrix<T, Ny, Ny> Pyy = Eigen::Matrix<T, Ny, Ny>::Zero();

    init_augmented<Nn == 0>::init(Pyy, R);
    for (int i = 0; i < NS; ++i)
    {
        Pyy += Wc(i) * (Yp.block<Ny, 1>(0, i) - yp) * (Yp.block<Ny, 1>(0, i) - yp).transpose();
    }
    //Pyy += 0.01*R;
    P_y = Pyy;
    y_p = yp;
    //std::cout << "new yp " << yp << std::endl;
    //std::cout << "new Py " << Pyy << std::endl;

    Eigen::Matrix<T, Nx, Ny> Pxy = Eigen::Matrix<T, Nx, Ny>::Zero();

    for (int i = 0; i < NS; ++i)
    {
        Pxy += Wc(i) * (Xa.block<Nx, 1>(0, i) - xp) * (Yp.block<Ny, 1>(0, i) - yp).transpose();
    }
    //std::cout << "new Pxy " << Pxy << std::endl;

    Eigen::Matrix<T, Nx, Ny> K = Pxy * Pyy.inverse();
    x = xp + K * (z - yp);
    P = Px - K * Pyy * K.transpose();

};

template <class T, int N> class Foo
{
    T a;
    T foo() { return N; }
};



template <class T, class TF, class TH, int Nx=2, int Ny=1, int Nv=2, int Nn=1, T alpha=0.9, T ki=0.0, T beta=0.05> void ukf_2x1(
    TF &F,                                  //transition func
    Eigen::Matrix<T, Nx, 1> &x,             //state vector
    Eigen::Matrix<T, Nx, Nx> &P,            //state cov matrix
    const Eigen::Matrix<T, Nx, Nx> &Q,      //state noise
    TH &H,                                  //observation func
    const Eigen::Matrix<T, Ny, 1> &z,       //observation
    const Eigen::Matrix<T, Ny, Ny> &R,       //observation noise
    Eigen::Matrix<T, Nx, 1> &x_p,             //state vector
    Eigen::Matrix<T, Nx, Nx> &P_p,            //state cov matrix
    Eigen::Matrix<T, Ny, 1> &y_p,             //state vector
    Eigen::Matrix<T, Ny, Ny> &P_y            //state cov matrix
    )
{
    return ukf_tt<T, TF, TH, Nx, Ny, Nv, Nn, 0.9, 0.0, 0.05>(
        F,       //transition func
        x,       //state vector
        P,       //state cov matrix
        Q,       //state noise
        H,       //observation func
        z,       //observation
        R,       //observation noise
        x_p,     //state vector
        P_p,     //state cov matrix
        y_p,     //state vector
        P_y      //state cov matrix
        );
}
//using ukf_2x1 = ukf_tt< T, TF, TH, 2, 1,  2, 1, T(0.9), T(0), T(0.05)>;

#endif