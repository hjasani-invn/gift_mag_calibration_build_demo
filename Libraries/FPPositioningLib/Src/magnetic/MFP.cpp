/**
* \copyright       Copyright (C) SPIRIT Navigation, LLC., 2014
* \brief           Magnetic fingerprint class. Provides storage, processing and data conversion
* \defgroup        MFP 
* \file            MFP.cpp
* \author          L.Purto M.Frolov
* \date            28.11.2014
*/

#ifdef _MSC_VER
#pragma warning( push )
#pragma warning( pop )
#pragma warning( disable : 4996 )
#pragma warning( disable : 4244 )
#endif
#define _USE_MATH_DEFINES

#define Sqr(x) ( (x)*(x) )

#include "MFP.h"
#include <math.h>
#ifndef __APPLE__
#include <malloc.h>
#endif
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <map>
#include <vector>
#include <limits>
#include <algorithm>
#include <stdint.h>


#ifndef _WIN32
#   include <endian.h>
#   if __BYTE_ORDER != __LITTLE_ENDIAN
#       error works only on __LITTE_ENDIAN arch
#   endif
#endif

#ifdef _WIN32
#include <crtdbg.h>
#else
#define  _ASSERT(x)
#endif

template<class T> static T normpdf_fast( T x, T sig )
{
    union
    {
        double d;
        int64_t x;
    } u = { x };

    double val = -( x * x ) / ( 2 * sig * sig );
    u.x = ( 1512775 * val + 1072632447 );
    u.x <<= 32;
    //    u.x = u.x << 32;
    val = u.d;

    return 1 / ( sqrt( 2 * M_PI ) * sig ) * val;
}

template<class T> static T normpdf( T x, T sig )
{
    double val = -( x * x ) / ( 2 * sig * sig );

    val = exp( val );
    return 1 / ( sqrt( 2 * M_PI ) * sig ) * val;
}
/*
tMFP::tMFP(double MinX_cm, double MaxX_cm, double MinY_cm, double MaxY_cm, double CellSize_cm)
{
	initialize(MinX_cm, MaxX_cm, MinY_cm, MaxY_cm, CellSize_cm, 0, 0);
}
*/

tMFP::tMFP(double MinX_cm, double MaxX_cm, double MinY_cm, double MaxY_cm, double CellSize_cm, int MinFloor, int MaxFloor) :
        mMinX (0), mMaxX (0), mMinY (0), mMaxY (0), //Physical coordinates in a local system that bound the area of a map
        mCellSize (0), mCellSizeReciprocal (0),    //Physical size of each cell. The cell is squared so that size X is equal to size Y
        mMinFloor (0), mMaxFloor (0),
        mpMap (NULL),
        mpMapApprox (NULL),
        outOfMap  (false),
        mpMapReliability (0),         //Array of estimates of completeness (reliability) for each cell
        mMapSizeX (0),                    //Number of bitmap cells in X-dimension
        mMapSizeY (0),                    //Number of bitmap cells in Y-dimension
        mMagnitudeCorrection_K (0),    //Parameter K from expression: Corrected magnitude = K * Magnitude from sensor + B
        mMagnitudeCorrection_B (0),    //Parameter B from expression: Corrected magnitude = K * Magnitude from sensor + B
        mAbsoluteValueReliability (0), //The closer to one, the better is reliability of absolute value of magnetic sensor
        //AbsoluteValueReliability = 1 means that relative change of the sensor value is ignored
        //and particle estimation is based on current value only
        //AbsoluteValueReliability = 0 means that absolute value of the sensor value is ignored
        //and particle estimation is based on difference between current and previous values
        mHistoryCount (0)
{
    initialized = initialize(MinX_cm, MaxX_cm, MinY_cm, MaxY_cm, CellSize_cm, MinFloor, MaxFloor);
}

tMFP::~tMFP()
{
        delete [] mpMap;
        delete [] mpMapReliability;
        delete [] mpMapApprox;
}

bool tMFP::initialize( double MinX_cm, double MaxX_cm, double MinY_cm, double MaxY_cm, double CellSize_cm, int MinFloor, int MaxFloor )
{
    vecRandomCells.reserve(512);

    mMinX = MinX_cm;
    mMaxX = MaxX_cm;
    mMinY = MinY_cm;
    mMaxY = MaxY_cm;
    mMinFloor = MinFloor;
    mMaxFloor = MaxFloor;

    mCellSize = CellSize_cm;
    mCellSizeReciprocal = 1. / mCellSize;

    mMapSizeX = ( int ) ceil( ( mMaxX - mMinX ) / mCellSize );
    mMapSizeY = ( int ) ceil( ( mMaxY - mMinY ) / mCellSize );

    unsigned long cell_count = GetMapCellsCount();
    mpMap = 0;
    mpMapReliability = 0;
    mpMapApprox = new (std::nothrow) tMFP_CellDescrApprox[cell_count];
    
    bool init_status = (nullptr != mpMapApprox);

    mFormat = eMFP_FORMAT_G_MIX;
    
    mMagnitudeCorrection_K = 1;
    mMagnitudeCorrection_B = 0;
    mHistoryCount = 0;
    memset( &distrOutOfMap, 0, sizeof( distrOutOfMap ) );
    outOfMap = false;

    return init_status;
}


bool tMFP::status()
{
    return initialized;
};

void *tMFP::GetMfpMap()
{
	return (void *)mpMapApprox;
}

static double GetProbabilityForCellDescr( tMFP_CellDescr *pCellDescr, double Magnitude )
{
    if( Magnitude >= MFP_HISTOGRAM_LEN )
        return 0;
    else
        return ( double ) pCellDescr->aHistogram[( int ) Magnitude] / MFP_HISTOGRAM_SUM;
}

static double CalcGauss( double X, double Mu, double Sigma )
{
    return 1. / ( Sigma * 2.5066282746310005 ) * exp( - ( X - Mu ) * ( X - Mu ) / ( 2 * Sigma * Sigma ) );  //sqrt (2*pi)=2.5066282746310005
}
#ifdef _MSC_VER
#   pragma warning( push )
#   pragma warning( disable : 4100 )
#endif
static double GetProbabilityForCellDescr_Gauss( tMFP_CellDescr *pCellDescr, double Magnitude, double GaussSigmaExt )
{
    double Result = CalcGauss( Magnitude, pCellDescr->Mu1, pCellDescr->Sigma1 ) * pCellDescr->Scale1 +
                    CalcGauss( Magnitude, pCellDescr->Mu2, pCellDescr->Sigma2 ) * ( 1 - pCellDescr->Scale1 );
    return Result;
}
#ifdef _MSC_VER
#   pragma warning( pop )
#endif
void tMFP::GetMapCoords( double UserX, double UserY, int &MapX, int &MapY ) const
{
    //UserX -= mCellSize / 2;
    //UserY -= mCellSize / 2;

    if( UserX < mMinX )
        UserX = mMinX;

    if( UserX > mMaxX )
        UserX = mMaxX;

    if( UserY < mMinY )
        UserY = mMinY;

    if( UserY > mMaxY )
        UserY = mMaxY;

    MapX = ( int ) floor( ( UserX - mMinX - mCellSize/2 ) / mCellSize + 0.5);
    MapY = ( int ) floor( ( UserY - mMinY - mCellSize/2 ) / mCellSize + 0.5);

    if( MapX >= mMapSizeX )
        MapX = mMapSizeX - 1;

    if( MapY >= mMapSizeY )
        MapY = mMapSizeY - 1;
}

void tMFP::CheckParticles( tMFP_Particle *pParticles, double *pEstimatedMetrics, int ParticleCount,
                           double *pLastMagnitudes, int MagnitudeCount, bool IsLastIndexLastInTime, double WeightThreshold )
{
    int ParticleNumber;
    int MapX, MapY;

    for( ParticleNumber = 0; ParticleNumber < ParticleCount; ParticleNumber++ )
    {
        GetMapCoords( pParticles[ParticleNumber].X, pParticles[ParticleNumber].Y, MapX, MapY );
        int Floor =  pParticles[ParticleNumber].Floor;

        if( CheckFloor( Floor ) == false )
        {
            pEstimatedMetrics[ParticleNumber] = 0.;
            continue;
        }

        unsigned int idx = ConvertCoordsToIndex( MapX, MapY, Floor );

        if( mFormat == eMFP_FORMAT_HIST )
        {
            _ASSERT(mpMap);
            pEstimatedMetrics[ParticleNumber] = GetProbabilityForCellDescr( &mpMap[idx],
                                                IsLastIndexLastInTime ? pLastMagnitudes[MagnitudeCount - 1] : pLastMagnitudes[0] );
        }
        else if( mFormat == eMFP_FORMAT_G_MIX )
        {
            double mu = sqrt( mpMapApprox[idx].x.mu1 * mpMapApprox[idx].x.mu1 +
                              mpMapApprox[idx].y.mu1 * mpMapApprox[idx].y.mu1 +
                              mpMapApprox[idx].z.mu1 * mpMapApprox[idx].z.mu1 );
            double s  = sqrt( mpMapApprox[idx].x.s1 * mpMapApprox[idx].x.s1 +
                              mpMapApprox[idx].y.s1 * mpMapApprox[idx].y.s1 +
                              mpMapApprox[idx].z.s1 * mpMapApprox[idx].z.s1 );
            double m = IsLastIndexLastInTime ? pLastMagnitudes[MagnitudeCount - 1] : pLastMagnitudes[0];

            pEstimatedMetrics[ParticleNumber] = normpdf( m - mu, 20. );

            if( pParticles[ParticleNumber].X > mMaxX || pParticles[ParticleNumber].X < mMinX
                    || pParticles[ParticleNumber].Y > mMaxY || pParticles[ParticleNumber].Y < mMinY )
            {
                if( outOfMap )
                {
                    mu = sqrt( distrOutOfMap.x.mu1 * distrOutOfMap.x.mu1 +
                               distrOutOfMap.y.mu1 * distrOutOfMap.y.mu1 +
                               distrOutOfMap.z.mu1 * distrOutOfMap.z.mu1 );
                    s  = sqrt( distrOutOfMap.x.s1 * distrOutOfMap.x.s1 +
                               distrOutOfMap.y.s1 * distrOutOfMap.y.s1 +
                               distrOutOfMap.z.s1 * distrOutOfMap.z.s1 );

                    pEstimatedMetrics[ParticleNumber] = normpdf( m - mu, s );;
                }
                else
                {
                   // pEstimatedMetrics[ParticleNumber] = ( 1. / MFP_HISTOGRAM_LEN );
                }
            }
        }
        else
        {
            pEstimatedMetrics[ParticleNumber] = 1.0;
        }

        //pEstimatedMetrics[ParticleNumber]=(pEstimatedMetrics[ParticleNumber]+0.1)/1.1;
        //pEstimatedMetrics[ParticleNumber] = (pEstimatedMetrics[ParticleNumber] > 0.) ? 1 : 0.1;
        //if (pEstimatedMetrics[ParticleNumber]   < 0.2)  pEstimatedMetrics[ParticleNumber] = 0.2;
        //if( pEstimatedMetrics[ParticleNumber] < WeightThreshold )
        //    pEstimatedMetrics[ParticleNumber] = WeightThreshold;

        //    else
        //      pEstimatedMetrics[ParticleNumber] = pEstimatedMetrics[ParticleNumber];

    }
}

void tMFP::CheckParticlesGauss( tMFP_Particle *pParticles, double *pEstimatedMetrics, int ParticleCount,
                                double *pLastMagnitudes, int MagnitudeCount, bool IsLastIndexLastInTime,
                                double GaussSigmaExt )
{
    int ParticleNumber;
    int MapX, MapY;
    tMFP_CellDescr CellDescr;

    for( ParticleNumber = 0; ParticleNumber < ParticleCount; ParticleNumber++ )
    {
        int Floor =  pParticles[ParticleNumber].Floor;

        if( CheckFloor( Floor ) == false )
        {
            pEstimatedMetrics[ParticleNumber] = 0.;
            continue;
        }
        _ASSERT(mpMap);

        GetMapCoords( pParticles[ParticleNumber].X, pParticles[ParticleNumber].Y, MapX, MapY );
        CellDescr = mpMap[ConvertCoordsToIndex( MapX, MapY, Floor )];
        pEstimatedMetrics[ParticleNumber] = GetProbabilityForCellDescr_Gauss( &CellDescr,
                                            IsLastIndexLastInTime ? pLastMagnitudes[MagnitudeCount - 1] : pLastMagnitudes[0], GaussSigmaExt );
    }
}



typedef struct tagLoggedDescr
{
    double X;
    double Y;
    int RecNumber;
    double CellReliability;
    tMFP_CellDescr CellDescr;
    tagLoggedDescr *pNext;
} tLoggedDescr;
#ifdef _MSC_VER
#   pragma warning( disable : 4505 )
#endif

#ifdef _MSC_VER
#   pragma warning( push )
#   pragma warning( disable : 4101 )
#endif

void tMFP::LimitMagneticMap()
{
	int N = GetMapCellsCount();
	double x, y;
    tMFP_Particle3D cell = {};
	for (int idx = 0; idx<N; ++idx)
	{
        const float k_mag_sigma_limit_in_cell = 20.;
        if ((mpMapApprox[idx].x.s1 < k_mag_sigma_limit_in_cell) &&
            (mpMapApprox[idx].y.s1 < k_mag_sigma_limit_in_cell) &&
            (mpMapApprox[idx].z.s1 < k_mag_sigma_limit_in_cell))
		{
			cell.Floor = GetCellCenterPosition(idx, x, y);
			cell.X = x;
			cell.Y = y;
			vecRandomCells.push_back(cell);
		}
	}
    for (int idx = 0; idx < N; ++idx)
    {
        const float k_mag_sigma_limit_in_cell = 20.;
        if ( (int)mpMapApprox[idx].x.reserved[0] != 0 )
        {
            cell.Floor = GetCellCenterPosition(idx, x, y);
            cell.X = x;
            cell.Y = y;
            vecPortalCells.push_back(cell);
            //std::cout << "vecPortalCells:   " << x / 100. << "    ,    " << y / 100. << "    ,    " 
            //    << cell.Floor << "    ,    " << (int)mpMapApprox[idx].x.reserved[0] << std::endl;
        }
    }
}

void tMFP::Hist2GMix( )
{
    _ASSERT(mpMap);

    for( unsigned int i = 0; i < GetMapCellsCount(); ++i )
    {
        float m_x( 0 ), m_y( 0 ), m_z( 0 );
        float s_x( 0 ), s_y( 0 ), s_z( 0 );

        for( int k = 0; k < MFP_HISTOGRAM_LEN_2; ++k )
        {
            // estimate E(X)
            m_x += ( k - MFP_HISTOGRAM_LEN ) * mpMap[i].aHistogramX[k];
            m_y += ( k - MFP_HISTOGRAM_LEN ) * mpMap[i].aHistogramY[k];
            m_z += ( k - MFP_HISTOGRAM_LEN ) * mpMap[i].aHistogramZ[k];
        }

        for( int k = 0; k < MFP_HISTOGRAM_LEN_2; ++k )
        {
            // estimate sigma2
            s_x += pow( m_x - ( k - MFP_HISTOGRAM_LEN ), 2 ) * mpMap[i].aHistogramX[k];
            s_y += pow( m_y - ( k - MFP_HISTOGRAM_LEN ), 2 ) * mpMap[i].aHistogramY[k];
            s_z += pow( m_z - ( k - MFP_HISTOGRAM_LEN ), 2 ) * mpMap[i].aHistogramZ[k];
        }

        s_x = sqrt( s_x );
        s_y = sqrt( s_y );
        s_z = sqrt( s_z );



        memset( &mpMapApprox[i], 0, sizeof( *mpMapApprox ) );

        mpMapApprox[i].x.mu1 = m_x;
        mpMapApprox[i].x.s1  = s_x;
        mpMapApprox[i].x.w1  = 1.0f;

        mpMapApprox[i].y.mu1 = m_y;
        mpMapApprox[i].y.s1  = s_y;
        mpMapApprox[i].y.w1  = 1.0f;

        mpMapApprox[i].z.mu1 = m_z;
        mpMapApprox[i].z.s1  = s_z;
        mpMapApprox[i].z.w1  = 1.0f;

    }
}



#if MFP_DIRECTION_SUPPORT
bool tMFP::GetExpectedVector( double PosX_cm, double PosY_cm, int Floor,
                              double &ExpectedX, double &ExpectedY, double &ExpectedZ, double &DirectionToXAxis )
{
    int MapX, MapY;
    tMFP_CellDescr CellDescr;
    memset( &CellDescr, 0, sizeof( CellDescr ) );
    GetMapCoords( PosX_cm, PosY_cm, MapX, MapY );

    _ASSERT(mpMap);

    if( CheckFloor( Floor ) == true )
    {
        CellDescr = mpMap[ConvertCoordsToIndex( MapX, MapY, Floor )];
    }

    ExpectedX = CellDescr.AverX;
    ExpectedY = CellDescr.AverY;
    ExpectedZ = CellDescr.AverZ;
    DirectionToXAxis = atan2( ExpectedY, ExpectedX ) - M_PI / 2;
    return ( ExpectedX || ExpectedY || ExpectedZ );
}

bool tMFP::GetExpectedVector( double PosX_cm, double PosY_cm, int Floor, double AssumedDirectionToXAxis,
                              double &ExpectedX, double &ExpectedY, double &ExpectedZ, double &CorrectionToAssumedDir )
{
    double MyAssumedDirectionToXAxis;
    GetExpectedVector( PosX_cm, PosY_cm, Floor, ExpectedX, ExpectedY, ExpectedZ, MyAssumedDirectionToXAxis );

    CorrectionToAssumedDir = -MyAssumedDirectionToXAxis - AssumedDirectionToXAxis * M_PI / 180;
    return ( ExpectedX || ExpectedY || ExpectedZ );
}

void tMFP::CheckCompassParticles2D( tMFP_CompassParticle2D *pParticles, double *pEstimatedMetrics, int ParticleCount )
{
    int ParticleNumber;
    int MapX, MapY;
    int Disp;
    tMFP_CellDescr CellDescr;

    _ASSERT(mpMap);

    for( ParticleNumber = 0; ParticleNumber < ParticleCount; ParticleNumber++ )
    {
        int Angle;
        GetMapCoords( pParticles[ParticleNumber].X, pParticles[ParticleNumber].Y, MapX, MapY );
        int Floor = pParticles[ParticleNumber].Floor;

        if( CheckFloor( Floor ) == false )
        {
            pEstimatedMetrics[ParticleNumber] = 0.;
            continue;
        }

        CellDescr = mpMap[ConvertCoordsToIndex( MapX, MapY, Floor )];
        Angle = ( int )( ( ( pParticles[ParticleNumber].NorthToXAngle + M_PI / 2 ) / ( 2 * M_PI ) * MFP_DIR_HORHISTOGRAM_LEN ) );
        Angle &= ( MFP_DIR_HORHISTOGRAM_LEN - 1 );
        pEstimatedMetrics[ParticleNumber] = 0;
#define COMPASS_THRESHOLD 0.01
#define COMPASS_DELTAHEADING 2

        for( Disp = -COMPASS_DELTAHEADING; Disp <= COMPASS_DELTAHEADING; Disp++ )
        {
            int ITemp = ( Angle + Disp ) & ( MFP_DIR_HORHISTOGRAM_LEN - 1 );
            double DTemp = ( double ) CellDescr.aHorHistogram[ITemp] / MFP_HISTOGRAM_SUM;

            if( pEstimatedMetrics[ParticleNumber] < DTemp )
                pEstimatedMetrics[ParticleNumber] = DTemp;
        }

        if( pEstimatedMetrics[ParticleNumber] > COMPASS_THRESHOLD )
            pEstimatedMetrics[ParticleNumber] = 1;


        for( Disp = -COMPASS_DELTAHEADING * 2; Disp <= COMPASS_DELTAHEADING * 2; Disp++ )
        {
            int ITemp = ( Angle + Disp ) & ( MFP_DIR_HORHISTOGRAM_LEN - 1 );
            double DTemp = ( double ) CellDescr.aHorHistogram[ITemp] / MFP_HISTOGRAM_SUM;

            if( ( pEstimatedMetrics[ParticleNumber] <= 0.1 ) && ( pEstimatedMetrics[ParticleNumber] < DTemp ) )
                pEstimatedMetrics[ParticleNumber] = 0.1;
        }
    }
}

#endif

void tMFP::GaussianSmooth( double sigA,  double sigX, double sigY, double sigZ )
{
    _ASSERT(mpMap);

    for( unsigned int i = 0; i < GetMapCellsCount(); ++i )
    {
        tMFP_CellDescr *pCell = &mpMap[i];

        smoothHistGauss( pCell->aHistogram,  sigA, MFP_HISTOGRAM_LEN, 0 );
        smoothHistGauss( pCell->aHistogramX, sigX, MFP_HISTOGRAM_LEN_2, MFP_HISTOGRAM_LEN );
        smoothHistGauss( pCell->aHistogramY, sigY, MFP_HISTOGRAM_LEN_2, MFP_HISTOGRAM_LEN );
        smoothHistGauss( pCell->aHistogramZ, sigZ, MFP_HISTOGRAM_LEN_2, MFP_HISTOGRAM_LEN );

    }
}

void tMFP::smoothHistGauss( float *pHist, double sig, int length, int idx0 )
{
    double sig2 = sig * sig;
    double norm = 0;
    idx0;

    float *sHist = new float[length];

    for( int i = 0; i < length; ++i ) norm += pHist[i];



    for( int i = 0; i < length; ++i )
    {
        double p_sum = 0;

        for( int j = 0; j < length; ++j )
        {
            double d = i - j;
            double p = 1 / ( sqrt( 2 * M_PI ) * sig ) * exp( - ( d * d ) / ( 2 * sig2 ) );

            p_sum += p * pHist[j];
        }

        sHist[i] = ( float )( p_sum / norm );
    }

    for( int i = 0; i < length; ++i ) pHist[i] = sHist[i];

    norm = 0;

    for( int i = 0; i < length; ++i ) norm += sHist[i];


    delete[] sHist;
}

void tMFP::GetMgVector( tMFP_Particle3D *pParticles, tMFP_CellDescrApprox *vec, int ParticleCount ) const
{
    int MapX, MapY;
    bool is_out = false;

    for( int i = 0; i < ParticleCount; ++i )
    {
        memset(&vec[i],0, sizeof(vec[0]));
        vec[i].x.s1=1000;
        vec[i].y.s1=1000;
        vec[i].z.s1=1000;

        GetMapCoords( pParticles[i].X, pParticles[i].Y, MapX, MapY );
        int Floor =  pParticles[i].Floor;

        bool is_out = false;
        if( CheckPosition( pParticles[i].X, pParticles[i].Y ) == false ||
            CheckFloor( Floor ) == false )
        {
            is_out = true;
        }

        if (!is_out && mFormat == eMFP_FORMAT_G_MIX )
        {
            int idx = ConvertCoordsToIndex( MapX, MapY, Floor );

            vec[i] = mpMapApprox[idx];
        }
    }
}

PortalType tMFP::getPortalType(double x, double y, int floor)
{
    PortalType portalType = k_NoPortal;
    int MapX, MapY;
    tMFP_CellDescrApprox vec;
    
    bool is_out = false;
    if (CheckPosition(x * 100, y * 100) == true && CheckFloor(floor) == true)
    {
        GetMapCoords(x * 100, y * 100, MapX, MapY);
        int idx = ConvertCoordsToIndex(MapX, MapY, floor);
        vec = mpMapApprox[idx];
        portalType = (PortalType)((int)vec.x.reserved[0]);
    }
    return portalType;
}


void tMFP::CheckParticles3D( tMFP_Particle3D *pParticles, double *pEstimatedMetrics, int ParticleCount, bool mfp_map_matching_enabled )
{
    int MapX, MapY;

    for( int i = 0; i < ParticleCount; ++i )
    {
        GetMapCoords( pParticles[i].X, pParticles[i].Y, MapX, MapY );
        int Floor = (int)floor(pParticles[i].Floor + 0.5);

        if( CheckFloor( Floor ) == false )
        {
            pEstimatedMetrics[i] = 0.;
            continue;
        }

        int idx = ConvertCoordsToIndex( MapX, MapY, Floor );

        double p_x = 0;
        double p_y = 0;
        double p_z = 0;

		pEstimatedMetrics[i] = 1.0; // metric is only compared to zero, so any positive value is the same

		double s_x = mpMapApprox[idx].x.s1;
		double s_y = mpMapApprox[idx].y.s1;
		double s_z = mpMapApprox[idx].z.s1;

		if (mfp_map_matching_enabled)
		{
			if ((s_x >= 60.0) || (s_y >= 60.0) || (s_z >= 60.0))
			{
				pEstimatedMetrics[i] = 0.; // using MFP map matching to kill particles inside untraversable areas
			}
		}

		// making weights of particles zero if outside of map, UNLESS we allow outside navigation ( outOfMap == true )
        if( pParticles[i].X > mMaxX || pParticles[i].X < mMinX
                || pParticles[i].Y > mMaxY || pParticles[i].Y < mMinY )
        {
            if( outOfMap )  // if out of map navigation is allowed
            {
				pEstimatedMetrics[i] = 1.;
            }
            else
            {
                pEstimatedMetrics[i] = 0.;
            }
        }
    }

}



void tMFP::UpdateParticles3D( tMFP_Particle3D *pParticles, double *pWeights, int ParticleCount )
{
    int MapX, MapY;
    tMFP_CellDescr *CellDescr;
    std::map<tMFP_CellDescr*, std::vector<int> > cell_map;

    _ASSERT( mpMap );


    for( int i = 0; i < ParticleCount; ++i )
    {
        if( pParticles[i].X > mMaxX || pParticles[i].X < mMinX
                || pParticles[i].Y > mMaxY || pParticles[i].Y < mMinY
                || ( CheckFloor( pParticles[i].Floor ) == false ) )
        {
            continue;
        }


        int Floor = pParticles[i].Floor;
        GetMapCoords( pParticles[i].X, pParticles[i].Y, MapX, MapY );
        CellDescr = &mpMap[ConvertCoordsToIndex( MapX, MapY, Floor )];

        std::map<tMFP_CellDescr*, std::vector<int> >::iterator cell_it;
        cell_it = cell_map.find( CellDescr );

        if( cell_it != cell_map.end() )
        {
            ( *cell_it ).second.push_back( i );
        }
        else
        {
            std::vector<int> v;
            v.push_back( i );
            cell_map.insert( std::pair<tMFP_CellDescr*, std::vector<int> > ( CellDescr, v ) );
        }
    }

    std::map<tMFP_CellDescr*, std::vector<int> >::iterator c_it;

    for( c_it = cell_map.begin(); c_it != cell_map.end(); ++c_it )
    {
        float p_sum = 0;
        float K = 10;

        std::vector<int>::iterator v_it;

        for( v_it = ( *c_it ).second.begin(); v_it != ( *c_it ).second.end(); ++v_it )
        {
            p_sum += ( float )pWeights[( *v_it )];
        }

        float alpha = 1.f / K;

        if( alpha * p_sum > 0.9 )
        {
            alpha = 0.9f / p_sum;
        }


        for( int i = 0; i < MFP_HISTOGRAM_LEN_2; ++i )
        {
            ( *c_it ).first->aHistogramX[i] *= ( 1 - alpha * p_sum );
            ( *c_it ).first->aHistogramY[i] *= ( 1 - alpha * p_sum );
            ( *c_it ).first->aHistogramZ[i] *= ( 1 - alpha * p_sum );

            if( i < MFP_HISTOGRAM_LEN )
            {
                ( *c_it ).first->aHistogramZ[i] *= ( 1 - alpha * p_sum );
            }
        }

        for( v_it = ( *c_it ).second.begin(); v_it != ( *c_it ).second.end(); ++v_it )
        {
            int i = ( *v_it );

            int idxX = ( int )floor( pParticles[i].v[0] + 0.5 + MFP_HISTOGRAM_LEN );
            int idxY = ( int )floor( pParticles[i].v[1] + 0.5 + MFP_HISTOGRAM_LEN );
            int idxZ = ( int )floor( pParticles[i].v[2] + 0.5 + MFP_HISTOGRAM_LEN );

            float magA = ( float )sqrt( pParticles[i].v[0] * pParticles[i].v[0] + pParticles[i].v[1] * pParticles[i].v[1] +
                                        pParticles[i].v[2] * pParticles[i].v[2] );
            int idxA = ( int )floor( magA + 0.5 );

            idxX = std::max( idxX, 0 );
            idxX = std::min( idxX, MFP_HISTOGRAM_LEN * 2 );

            idxY = std::max( idxY, 0 );
            idxY = std::min( idxY, MFP_HISTOGRAM_LEN * 2 );

            idxZ = std::max( idxZ, 0 );
            idxZ = std::min( idxZ, MFP_HISTOGRAM_LEN * 2 );

            idxA = std::max( idxA, 0 );
            idxA = std::min( idxA, MFP_HISTOGRAM_LEN - 1 );



            ( *c_it ).first->aHistogramX[idxX] += alpha * ( float )pWeights[i];
            ( *c_it ).first->aHistogramY[idxY] += alpha * ( float )pWeights[i];
            ( *c_it ).first->aHistogramZ[idxZ] += alpha * ( float )pWeights[i];


            ( *c_it ).first->aHistogram[idxA] += alpha * ( float )pWeights[i];

        }
    }
}

void tMFP::InitializeUniform( )
{

    float v = 0.1f;

    if (mpMap == 0)
    {
        mpMap = ( tMFP_CellDescr* ) malloc( GetMapSize() );
        memset( mpMap, 0, GetMapSize() );
    }
    _ASSERT(mpMap);


    for( unsigned int i = 0; i < GetMapCellsCount(); ++i )
    {
        for( int z = 0; z < MFP_HISTOGRAM_LEN_2; ++z )
        {
            mpMap[i].aHistogramX[z] = v;
            mpMap[i].aHistogramY[z] = v;
            mpMap[i].aHistogramZ[z] = v;
        }

        for( int z = 0; z < MFP_HISTOGRAM_LEN; ++z )
        {
            mpMap[i].aHistogram[z] = v;
        }

    }

    mFormat = eMFP_FORMAT_HIST;
}

void tMFP::NormalizeMap( )
{
    float sumX, sumY, sumZ, sumA;
    _ASSERT(mpMap);

    for( unsigned int i = 0; i < GetMapCellsCount(); ++i )
    {
        sumX = sumY = sumZ = sumA = 0;

        for( int m = 0; m < MFP_HISTOGRAM_LEN_2; ++m )
        {
            sumX += mpMap[i].aHistogramX[m];
            sumY += mpMap[i].aHistogramY[m];
            sumZ += mpMap[i].aHistogramZ[m];
        }

        for( int m = 0; m < MFP_HISTOGRAM_LEN; ++m )
        {
            sumA += mpMap[i].aHistogram[m];
        }

        for( int m = 0; m < MFP_HISTOGRAM_LEN_2; ++m )
        {
            mpMap[i].aHistogramX[m] /= sumX;
            mpMap[i].aHistogramY[m] /= sumY;
            mpMap[i].aHistogramZ[m] /= sumZ;
        }

        for( int m = 0; m < MFP_HISTOGRAM_LEN; ++m )
        {
            mpMap[i].aHistogram[m] /= sumA;
        }


    }
}



std::ostream &operator<< ( std::ostream &os, tMFP_Particle &p )
{
    os << "X=" << p.X << " Y=" << p.Y << "\n";
    return os;
}

// The function calculates magnetic vector for specified location (x,y) by mfp-db data
double const MFP_MEAS_COV = 25; // assume sigma = 5
double const MFP_UNKNOWN_COV = 200 * 200; //

void tMFP::GetMagneticVector( double X, double Y, int Floor, double *mfp_vector, double *mfp_cov ) const
{
    int MapX, MapY, i, idx;

    GetMapCoords( X, Y, MapX, MapY );

    if( ( mFormat == eMFP_FORMAT_HIST ) && ( CheckFloor( Floor ) != false ) )
    {
        double hist_norm[3] = {0., 0., 0.};
        _ASSERT(mpMap);
        tMFP_CellDescr CellDescr = mpMap[ConvertCoordsToIndex( MapX, MapY , Floor )];

        for( i = 0; i < 3; i++ )
            mfp_vector[i] = 0;

        for( idx = 0; idx < MFP_HISTOGRAM_LEN_2; idx++ )
        {
            mfp_vector[0] += ( idx - MFP_HISTOGRAM_LEN ) * CellDescr.aHistogramX[idx];
            mfp_vector[1] += ( idx - MFP_HISTOGRAM_LEN ) * CellDescr.aHistogramY[idx];
            mfp_vector[2] += ( idx - MFP_HISTOGRAM_LEN ) * CellDescr.aHistogramZ[idx];
            hist_norm[0] += CellDescr.aHistogramX[idx];
            hist_norm[1] += CellDescr.aHistogramY[idx];
            hist_norm[2] += CellDescr.aHistogramZ[idx];
        }

        // covariance calculation  // to do !!!
        for( i = 0; i < 3; i++ )
        {
            mfp_vector[i] /= hist_norm[i];
            mfp_cov[i] = MFP_MEAS_COV;
        }
    }
    else if( ( mFormat == eMFP_FORMAT_G_MIX )  && ( CheckFloor( Floor ) != false ) )
    {
        tMFP_CellDescrApprox CellDescr = mpMapApprox[ConvertCoordsToIndex( MapX, MapY, Floor )];
        mfp_vector[0] = CellDescr.x.mu1;
        mfp_cov[0] = CellDescr.x.s1 * CellDescr.x.s1;
        mfp_vector[1] = CellDescr.y.mu1;
        mfp_cov[1] = CellDescr.y.s1 * CellDescr.y.s1;
        mfp_vector[2] = CellDescr.z.mu1;
        mfp_cov[2] = CellDescr.z.s1 * CellDescr.z.s1;
    }
    else
    {
        for( i = 0; i < 3; i++ )
        {
            mfp_vector[i] = 0;
            mfp_cov[i] = MFP_UNKNOWN_COV;
        }
    }
}

void tMFP::setOutOfMapDistrib( tMFP_CellDescrApprox distrib, bool enabled )
{
    outOfMap = enabled;
    distrOutOfMap = distrib;
}
