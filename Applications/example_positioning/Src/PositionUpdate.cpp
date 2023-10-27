#include "PositionUpdate.hpp"

PositionUpdate::PositionUpdate( const std::string &output_log_file )
{
    //outputstream = std::ofstream(output_log_file);
    outputstream.open( output_log_file );

    if ( outputstream.fail() )
    {
        throw 1;
    }
}

PositionUpdate::~PositionUpdate()
{
    outputstream.close();
}

/**
* Debug position callback.
* This method is called when new position estimation in PF frame is avaliable
* This method is also used in PC model to visualize particle cloud
* \param[in] X,Y coordinates in a local frame [m]
* \param[in] Floor floor level
* \param[in] H heading [rad]
* \param[in] Sig estimated position deviation
* \param[in] t timestamp [ms]
* \param[in] state pointer to the particles array
* \param[in] N particles count
*/
void PositionUpdate::update( double X, double Y, double Floor, double H, double Sig, double t, const Fppe::Particle *state, int N )
{
    /*
    int width = 15;

    outputstream << std::fixed;
    outputstream.width( width );
    outputstream.precision( 4 );
    outputstream << X;
    outputstream.width( width );
    outputstream.precision( 4 );
    outputstream << Y;
    outputstream.width( width );
    outputstream.precision( 4 );
    outputstream << Floor;
    outputstream.width( width );
    outputstream.precision( 4 );
    outputstream << H;
    outputstream.width( width );
    outputstream.precision( 4 );
    outputstream << Sig;
    width = 25;
    outputstream.width( width );
    outputstream.precision( 4 );
    outputstream << t;
    outputstream << std::endl;
    */
}

/**
* Position callback.
* This method is called when new position estimation in GEO frame is avaliable
* \param[in]  position navigation solution
*/
void PositionUpdate::update( const Fppe::Position &position )
{
    int width = 15;

    outputstream << std::fixed;
    outputstream.width( width );
    outputstream << position.timestamp;
    outputstream.width( width );
    outputstream.precision( 8 );
    outputstream << position.lattitude;
    outputstream.width( width );
    outputstream.precision( 8 );
    outputstream << position.longitude;
    outputstream.width( width );
    outputstream.precision( 4 );
    outputstream << position.azimuth;
    outputstream.width( width );
    outputstream.precision( 4 );
    outputstream << position.altitude;
    outputstream.width( width );
    outputstream << position.floor_number;
    outputstream.width( width );
    outputstream.precision( 4 );
    outputstream << position.covariance_lat_lon[0][0];
    outputstream.width( width );
    outputstream.precision( 4 );
    outputstream << position.covariance_lat_lon[0][1];
    outputstream.width( width );
    outputstream.precision( 4 );
    outputstream << position.covariance_lat_lon[1][0];
    outputstream.width( width );
    outputstream.precision( 4 );
    outputstream << position.covariance_lat_lon[1][1];
    outputstream.width( width );
    outputstream.precision( 4 );
    outputstream << position.azimuth_std;
    outputstream.width( width );
    outputstream.precision( 4 );
    outputstream << position.floor_std;
    outputstream.width( width );
    outputstream << position.is_valid;
    outputstream << std::endl;


}

