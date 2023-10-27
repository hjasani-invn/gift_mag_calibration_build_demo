#include <iostream>
#include <stdint.h>


#define OUT_PITCH_ROLL_ONLY     0 // testing output

#define OUT_QUATERNION_DISPERSION_ONLY    0  // 1 - out dispersion only; 0 - out covariance matrix


enum TrackLineType { eLine, eFloor, eUnused };


struct TrackLine
{
public:
    /**
    * Constructor by default
    */
    TrackLine()    { this->clear(); };

    /**
    * Copy constructor
    * \param[in] TrackLine - GNSS location structure instance
    */
    TrackLine(const TrackLine &L)    { *this = L; };

    /**
    * The method clears fields of class
    */
    void clear()
    {
        this->x1 = this->x2 = 0;
        this->y1 = this->y2 = 0;
        this->st1 = this->st2 = 0;
        this->floor = 0;
        this->pitch = this->roll = 0;
        int8_t att_flag = 0;
    }

    int64_t st1;
    double x1, y1;
    int64_t st2;
    double x2, y2;

    int16_t floor;
    double pitch;
    double roll;
    int8_t att_flag;
};


void SaveAttPosData(std::ofstream &os, int64_t sys_t, double lattitude, double longitude, int16_t floor, double q[4]);
void SaveAttIncData(std::ofstream &os, int64_t sys_t, double dx, double dy, int16_t floor, double q[4], double mg[3]);

void InterpolateHeading(TrackLine step, double &heading);
void InterpolatePos(int64_t st, TrackLine step, double &x, double &y);

std::list<TrackLine> LoadTrackData(std::string TrackDataFileName);
Matrix GetFullAttMatrix(const Matrix &Cp2b, double heading);
bool DCM2Quaternion(const Matrix &dcm, double q[4]);
void SaveSensorData(std::ofstream &os, int64_t time_tag, int64_t call_counter, double sensor_data[3]);