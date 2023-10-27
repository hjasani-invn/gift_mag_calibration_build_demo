/**
* \copyright       Copyright (C) SPIRIT Navigation, LLC., 2014
* \brief           Magnetic fingerprint class. Provides storage, processing and data conversion
* \defgroup        MFP 
* @{
* \file            MFP.h
* \author          L.Purto M.Frolov
* \date            28.11.2014
*/
#ifndef __MFP_H
#define __MFP_H

#define MFP_DIRECTION_SUPPORT 1 /**< enables direction support */

#include <ostream>
#include <vector>
//#include <crtdbg.h>
#include <iostream>

#include "fpeDataTypes.h"

/**< coordinates description */
typedef struct
{
    /** Coordinate in plane [m] */
    ///@{
    double X, Y;
    ///@}
    int Floor; /**< discrete floor number */
} tMFP_Particle;

/**< extended coordinates description, obsolete */
typedef struct
{
    /** Coordinate in plane [m] */
    ///@{
    double X, Y;        //New position
    ///@}
    int Floor;           /**< discrete floor number */
    double NorthToXAngle; /**< magnetic vector direction in the plane */
} tMFP_CompassParticle2D;

/**< PF hypothesis */
typedef struct
{
    /** Coordinate in plane [m] */
    ///@{
    double X, Y;        //New position
    ///@}
    int Floor; /**< discrete floor number */
    double v[3]; /** magnetic field vector */
    double s[3]; /** standart deviation */
} tMFP_Particle3D;



std::ostream &operator<<( std::ostream &os, tMFP_Particle &p );


#define MFP_HISTORY_SIZE 100 /**< history length in samples, obsolete*/

#define MFP_MAX_MAGNITUDE 100 /**< mag fileld limit [uT] */

#define MFP_HISTOGRAM_LEN MFP_MAX_MAGNITUDE /**< magnitude histogram length */
#define MFP_HISTOGRAM_LEN_2 ((MFP_HISTOGRAM_LEN)*2+1) /**< mag vector componets histogram length */

#define MFP_HISTOGRAM_SUM 1000 /**< empirical measurements count threshold */


//For direction support:
#define MFP_DIR_HORHISTOGRAM_LEN 32 /**< direction histogram length in plane, obsolete */
#define MFP_DIR_VERTHISTOGRAM_LEN 16 /**< vertical histogram length, obsolete */

/** fingerprint cell description, histogram model */
typedef struct
{
    float aHistogram[MFP_HISTOGRAM_LEN];  /**< filed magnitude histogram */

    /** Gaussion mixture params */
    ///@{
    float Mu1;      
    float Sigma1;
    float Scale1;
    float Mu2;
    float Sigma2;
    ///@}
    //float Scale2; Scale2=1-Scale1
    float GaussApproximationError; /**< Gaussian approximation error */

#if MFP_DIRECTION_SUPPORT
    int aHorHistogram[MFP_DIR_HORHISTOGRAM_LEN]; /**< plane direction histogram */
    int aVertHistogram[MFP_DIR_VERTHISTOGRAM_LEN]; /**< vertical direction histogram */

    /** mag field componets histigram */
    ///@{
    float aHistogramX[MFP_HISTOGRAM_LEN_2];
    float aHistogramY[MFP_HISTOGRAM_LEN_2];
    float aHistogramZ[MFP_HISTOGRAM_LEN_2];
    ///@}

    /** componets mean values */
    ///@{
    int AverX, AverY, AverZ;
    ///@}
#endif
} tMFP_CellDescr;

/** fingerprint cell description, GM model */
typedef struct
{
    /** gaussian mix model parameters */
    typedef struct
    {
        float mu1;  /**< gaussian #1 mean */
        float s1;   /**< gaussian #1 std */
        float w1;   /**< gaussian #1 weight */
        float mu2;  /**< gaussian #2 mean */
        float s2;   /**< gaussian #2 std */
        float w2;   /**< gaussian #2 weight */
        float reserved[2]; /**< reserved fileds */
    } GaussMix;

    /** field componets */
    ///@{
    GaussMix x;
    GaussMix y;
    GaussMix z;
    ///@}
}
tMFP_CellDescrApprox;

/** defines MFP models */
enum eMFP_FORMAT
{
    eMFP_FORMAT_HIST = 0,  /**< histogram */
    eMFP_FORMAT_G_MIX = 1, /**< gaussian mix */
    eMFP_FORMAT_UNK = 2    /**< error */
};

/**
* Main MFP class. Stores fingerprint and provides all operations with mag data
*/
class tMFP
{
    public:
        /** 
        * Crates single-floor MFP object and allocates memory for a specific area
        * \param[in] MinX_cm,MaxX_cm,MinY_cm,MaxY_cm defines map area in [sm]
        * \param[in] CellSize_cm defines square cell discrete [sm]
        */
 //       tMFP( double MinX_cm, double MaxX_cm, double MinY_cm, double MaxY_cm, double CellSize_cm );
        /** 
        * Crates multi-floor MFP object and allocates memory for a specific area
        * \param[in] MinX_cm,MaxX_cm,MinY_cm,MaxY_cm defines a map area in [sm]
        * \param[in] CellSize_cm defines square cell discrete [sm]
        * \param[in] MinFloor,MaxFloor defines a floors range
        */
        tMFP( double MinX_cm, double MaxX_cm, double MinY_cm, double MaxY_cm, double CellSize_cm, int MinFloor, int MaxFloor );

        /** destructor */
        ~tMFP();

        /**
        * \return true id the object was initialized properly, false otherwice; 
        */
        bool status();

        /**
        * \return poiner to MFP map
        */
        void *GetMfpMap();

        /**
        * \disable cells with sigma more than threshold
        */
        void LimitMagneticMap();

        /**
        * \return mfp map cell size in cm
        */
        double GetMapCellSize() const
        {
            return this->mCellSize;
        };

        /**
        * Weights position hypothesis with saturation, according existing fingerprint and measured magnitude
        * \param[in]  pParticles pointer to the hypothesis array
        * \param[out] pEstimatedMetrics pointer to the weights array
        * \param[in]  ParticleCount hypothesis array length
        * \param[in]  pLastMagnitudes pointer to the measurements array
        * \param[in]  IsLastIndexLastInTime use only first element in the measurements array
        * \param[in]  WeightThreshol weighs low saturation level
        */
        void CheckParticles( tMFP_Particle *pParticles, double *pEstimatedMetrics, int ParticleCount,
                             double *pLastMagnitudes, int MagnitudeCount, bool IsLastIndexLastInTime, double WeightThreshol );

        /**
        * Weights position hypothesis according existing fingerprint and present measurements
        * \param[in]  pParticles pointer to the hypothesis array
        * \param[out] pEstimatedMetrics pointer to the weights array
        * \param[in]  ParticleCount hypothesis array length
        * \param[in]  mfp_map_matching_enabled flag that allows map matching (killing particles which enter unsurveyed areas)
        */
        void CheckParticles3D( tMFP_Particle3D *pParticles, double *pEstimatedMetrics, int ParticleCount, bool mfp_map_matching_enabled);

        /**
        * Returns fingerprint data for the selected positions
        * \param[in]  pParticles pointer to positions array
        * \param[out] vec pointer to the fingerprint data
        * \param[in]  ParticleCount array length
        */
        void GetMgVector( tMFP_Particle3D *pParticles, tMFP_CellDescrApprox *vec, int ParticleCount ) const;

        /**
        * Returns portal type for the selected positions
        * \param[in]  x coordinate
        * \param[in]  y coordinate
        * \param[in]  floor
        * \return portal type
        */
        PortalType getPortalType(double x, double y, int floor);

        /**
        * Updates the fingerprint data according current hypothesis distribution
        * \param[in]  pParticles pointer to the hypothesis array
        * \param[in]  pWeights pointer to the hypothesis weights
        * \param[in]  ParticleCount array length
        */
        void UpdateParticles3D( tMFP_Particle3D *pParticles, double *pWeights, int ParticleCount );

        /** Initialises fingerprint with uniform distribution */
        void InitializeUniform( );

        /** Normalises fingerprint */
        void NormalizeMap( );

        /** Returns number of well surveyed cells*/
        size_t getRandomPosCount() const
        {
            return vecRandomCells.size();
        }
        /**
        * Returns description of the random  well surveyed cells
        * \param[in] idx random value
        * \return cell data
        */
        tMFP_Particle3D getRandomPos(size_t idx) const
        {
            size_t idx_max = vecRandomCells.size();
            size_t i = idx % idx_max;
            return vecRandomCells[i];
        }

        /**
        * Weights position hypothesis according existing fingerprint and measured magnitude
        * \param[in]  pParticles pointer to the hypothesis array
        * \param[out] pEstimatedMetrics pointer to the weights array
        * \param[in]  ParticleCount hypothesis array length
        * \param[in]  pLastMagnitudes pointer to the measurements array
        * \param[in]  IsLastIndexLastInTime use only first element in the measurements array
        */
        void CheckParticles( tMFP_Particle *pParticles, double *pEstimatedMetrics, int ParticleCount,
                             double *pLastMagnitudes, int MagnitudeCount, bool IsLastIndexLastInTime )
        {
            CheckParticles( pParticles, pEstimatedMetrics, ParticleCount, pLastMagnitudes,
                            MagnitudeCount, IsLastIndexLastInTime, 0.0 );
        }

        /**
        * Weights position hypothesis according existing fingerprint and measured magnitude
        * \param[in]  pParticles pointer to the hypothesis array
        * \param[out] pEstimatedMetrics pointer to the weights array
        * \param[in]  ParticleCount hypothesis array length
        * \param[in]  pLastMagnitudes pointer to the measurements array
        * \param[in]  IsLastIndexLastInTime use only first element in the measurements array
        * \param[in]  GaussSigmaExt unused
        */
        void CheckParticlesGauss( tMFP_Particle *pParticles, double *pEstimatedMetrics, int ParticleCount,
                                  double *pLastMagnitudes, int MagnitudeCount, bool IsLastIndexLastInTime,
                                  double GaussSigmaExt );

        /** \return fingerprint size in [bytes] */
        size_t GetMapSize()
        {
            //return GetMapCellsCount() * sizeof( tMFP_CellDescr ) ;
            size_t cellDescrApprox = sizeof(tMFP_CellDescrApprox);
            return GetMapCellsCount() * cellDescrApprox;
        }

        /** \return total cells count */
        unsigned long GetMapCellsCount() const
        {
            
            return mMapSizeX * mMapSizeY * ( mMaxFloor - mMinFloor + 1 );
            
        }

        /**
        * Smooth histogram with Gaussian kernel
        * \param[in] sigA  magnitude smoothing kernel width
        * \param[in] sigX,sigY,sigZ  vector componets smoothing kernel width
        */
        void GaussianSmooth( double sigA,  double sigX, double sigY, double sigZ );

        /** 
        * load data from raw logs, each log corresponds to a separate floor number
        * \param[in] papFileNames  null terminated array of filenames
        * \param[in] aFloors  array of floor numbers
        */
        //void LoadLog( char **papFileNames, int *aFloors );

        /**
        * Save fingerprint in a specific binary file
        * \param[in] pFileName filename to save fingerprint
        */
        void StoreMagneticMap( const char *pFileName );
        /**
        * Save fingerprint in a specific binary file and format
        * \param[in] pFileName filename to save fingerprint
        * \param[in] format describes fingerprint representation
        */
        void StoreMagneticMap( const char *pFileName, eMFP_FORMAT format );

        /**
        * Loads fingerprint in from binary file in default format
        * \param[in] pFileName binary filename 
        */
        //bool LoadMagneticMap( const char *pFileName );
        /**
        * Loads fingerprint in from binary file in specific format
        * \param[in] pFileName binary filename 
        * \param[in] format describes fingerprint representation
        */
        bool LoadMagneticMap( const char *pFileName, eMFP_FORMAT format );

        /**
        * Return magnetic field gaussian distribution for a specific location
        * \param[in] X,Y coordinates in a plane [sm] 
        * \param[in] Floor discrete floor number
        * \param[out] mfp_vector three components array of mean values
        * \param[out] mfp_cov three components array of standart deviation
        */
        void GetMagneticVector( double X, double Y, int Floor, double *mfp_vector, double *mfp_cov ) const;

#if MFP_DIRECTION_SUPPORT
        /**
        * Return magnetic field and direction  for a specific position
        * \param[in] PosX_cm,PosY_cm coordinates in a plane [sm] 
        * \param[in] Floor discrete floor number
        * \param[out] ExpectedX,ExpectedY,ExpectedZ field vertor in a specific position
        * \param[out] DirectionToXAxis field direction in a plane [rad]
        * \return succes status
        */
        bool GetExpectedVector( double PosX_cm, double PosY_cm, int Floor,
                                double &ExpectedX, double &ExpectedY, double &ExpectedZ, double &DirectionToXAxis );
        /**
        * Return magnetic field and direction dufference for a specific position and expected direction
        * \param[in] PosX_cm,PosY_cm coordinates in a plane [sm] 
        * \param[in] Floor discrete floor number
        * \param[in] AssumedDirectionToXAxis expected field direction in a plane [deg]
        * \param[out] ExpectedX,ExpectedY,ExpectedZ field vertor in a specific position
        * \param[out] CorrectionToAssumedDir difference between  [rad]
        * \return succes status
        */
        bool GetExpectedVector( double PosX_cm, double PosY_cm, int Floor, double AssumedDirectionToXAxis,
                                double &ExpectedX, double &ExpectedY, double &ExpectedZ, double &CorrectionToAssumedDir );
        /**
        * Weights position hypothesis according field direction in a plane
        * \param[in]  pParticles pointer to the hypothesis array
        * \param[out] pEstimatedMetrics pointer to the weights array
        * \param[in]  ParticleCount array length
        */
        void CheckCompassParticles2D( tMFP_CompassParticle2D *pParticles, double *pEstimatedMetrics, int ParticleCount );
#endif

        /**
        * Save fingerprint reliability as temperature map  binary and text format
        * \param[in] pFileNameBin  binary map filename
        * \param[in] pFileNameBin  text file (MM script)
        */
        void Debug_StoreMagneticMapReliability( const char *pFileNameBin, const char *pFileNameScript );

#if MFP_DIRECTION_SUPPORT
        /**
        * Save field direction in text format (MMScript)
        * \param[in] pFileNameHor  plane direction script
        * \param[in] pFileNameBin  vertical direction script
        */
        void Debug_StoreMagneticDirection( const char *pFileNameHor, const char *pFileNameVert );
#endif

        /**
        * Sets field distribution outside the map
        * \param[in] distrib field distribution
        * \param[in] enabled  enables distribution outside the map
        */
        void setOutOfMapDistrib( tMFP_CellDescrApprox distrib, bool enabled );

        bool CheckFloor(int Floor) const
        {
            return (Floor >= mMinFloor && Floor <= mMaxFloor);
        }
        bool CheckPosition(int X, int Y) const
        {
            return (X >= mMinX && X <= mMaxX && Y >= mMinY && Y <= mMaxY);
        }

        std::vector<tMFP_Particle3D> getPortalCells()
        {
            return vecPortalCells;
        }

    protected:
        double mMinX, mMaxX, mMinY, mMaxY; //Physical coordinates in a local system that bound the area of a map
        double mCellSize, mCellSizeReciprocal;    //Physical size of each cell. The cell is squared so that size X is equal to size Y

        int mMinFloor, mMaxFloor;

        tMFP_CellDescr *mpMap;
        tMFP_CellDescrApprox *mpMapApprox;
        tMFP_CellDescrApprox distrOutOfMap;
        bool outOfMap;

        std::vector<tMFP_Particle3D> vecRandomCells;
        std::vector<tMFP_Particle3D> vecPortalCells;

        double *mpMapReliability;         //Array of estimates of completeness (reliability) for each cell

        int mMapSizeX;                    //Number of bitmap cells in X-dimension

        int mMapSizeY;                    //Number of bitmap cells in Y-dimension

        double mMagnitudeCorrection_K;    //Parameter K from expression: Corrected magnitude = K * Magnitude from sensor + B

        double mMagnitudeCorrection_B;    //Parameter B from expression: Corrected magnitude = K * Magnitude from sensor + B

        double mAbsoluteValueReliability; //The closer to one, the better is reliability of absolute value of magnetic sensor
        //AbsoluteValueReliability = 1 means that relative change of the sensor value is ignored
        //and particle estimation is based on current value only
        //AbsoluteValueReliability = 0 means that absolute value of the sensor value is ignored
        //and particle estimation is based on difference between current and previous values
        /*
        struct
        {
            double MagnitudesFromSensor;

            double PosX;

            double PosY;
        } maHistory[MFP_HISTORY_SIZE];
        */
        int mHistoryCount;

        void GetMapCoords( double UserX, double UserY, int &MapX, int &MapY ) const;

        int ConvertCoordsToIndex( int MapX, int MapY, int Floor ) const
        {
            return ( Floor - mMinFloor ) * ( mMapSizeX * mMapSizeY ) + MapY * mMapSizeX + MapX;
        }

        void smoothHistGauss( float *pHist, double sig, int length, int idx0 );
        eMFP_FORMAT mFormat;

        bool LoadMagneticMapRaw( const char *pFileName, void *pData, size_t size );
        void StoreMagneticMapRaw( const char *pFileName, void *pData, size_t size );
        void Hist2GMix( );

        int GetCellCenterPosition( int idx, double &X, double &Y )
        {
            int idx_floor = idx % ( mMapSizeX * mMapSizeY );
            int idx_y = idx_floor / mMapSizeX;
            int idx_x = idx_floor % mMapSizeX;

            X = idx_x * mCellSize - mMinX + mCellSize / 2;
            Y = idx_y * mCellSize - mMinY + mCellSize / 2;

            return idx / ( mMapSizeX * mMapSizeY );
        }

        /**
        Initialize the object with specified parameters
        * \param[in] MinX_cm - bottom left corner x coordinate, cm
        * \param[in] MaxX_cm - top rightcorner x coordinate, cm
        * \param[in] MinY_cm - bottom left corner x coordinate, cm
        * \param[in] MaxY_cm - top rightcorner x coordinate, cm
        * \param[in] CellSize_cm - cell size, cm
        * \param[in] MinFloor - lowess floor number
        * \param[in] MaxFloor - upper floor number
        * \return succes status
        */
        bool initialize( double MinX_cm, double MaxX_cm, double MinY_cm, double MaxY_cm, double CellSize_cm, int MinFloor, int MaxFloor );
        
        bool initialized; //initialization status
};


#endif //__MFP_H
///@}
