#include "PositionUpdate.hpp"
#include <QThread>

PositionUpdate::PositionUpdate( const std::string &output_log_file, OutPut_for_image *output_struct, const std::string &output_log_name_dbg )
{
    //outputstream = std::ofstream(output_log_file);
    outputstream.open( output_log_file );
    outputstream << "<?xml version=\"1.0\" encoding=\"utf-8\" ?>" << std::endl
                 << "<kml xmlns=\"http://www.opengis.net/kml/2.2\">" << std::endl
                 << "<Document><Folder><name>" << output_log_file << "</name>" << std::endl
                 << "<Schema name=\"" << output_log_file << "\" id=\"" << output_log_file << "\">" << std::endl
                 << "\t<SimpleField name=\"Name\" type=\"string\"></SimpleField>" << std::endl
                 << "\t<SimpleField name=\"Description\" type=\"string\"></SimpleField>" << std::endl
                 << "\t<SimpleField name=\"number\" type=\"string\"></SimpleField>" << std::endl
                 << "\t<SimpleField name=\"latitude\" type=\"string\"></SimpleField>" << std::endl
                 << "\t<SimpleField name=\"longitude\" type=\"string\"></SimpleField>" << std::endl
                 << "\t<SimpleField name=\"timestamp\" type=\"string\"></SimpleField>" << std::endl
                 << "\t<SimpleField name=\"cov_lat\" type=\"string\"></SimpleField>" << std::endl
                 << "\t<SimpleField name=\"cov_lon\" type=\"string\"></SimpleField>" << std::endl
                 << "\t<SimpleField name=\"cov_lat_lon\" type=\"string\"></SimpleField>" << std::endl
                 << "</Schema>" << std::endl;

    outputstream << "<Style id = \"default0\">" << std::endl
                 << "<LineStyle>" << std::endl
                 << "<color>ff00aaff</color>" << std::endl
                 << "<width>2.3</width>" << std::endl
                 << "</LineStyle>" << std::endl
                 << "</Style>" << std::endl;

    if ( outputstream.fail() )
    {
        throw 1;
    }

    if ( output_log_name_dbg.size() > 0 )
    {
        outputstream_dbg.open( output_log_name_dbg );
    }

    fname = output_log_file;
    counter = 0;
    this->output_struct = output_struct;
    mTrack = NULL;
}

PositionUpdate::~PositionUpdate()
{
    outputstream << "</Folder></Document></kml>" << std::endl;
    outputstream.close();
    outputstream_dbg.close();
}

/**
* this prototype is used in PC model to visualize particle cloud
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
    int width = 15;
#if 0
    std::cout << std::fixed;
    std::cout.width( width );
    std::cout.precision( 4 );
    std::cout << X;
    std::cout.width( width );
    std::cout.precision( 4 );
    std::cout << Y;
    std::cout.width( width );
    std::cout.precision( 4 );
    std::cout << Floor;
    std::cout.width( width );
    std::cout.precision( 4 );
    std::cout << floor(Floor + 0.5);
    std::cout << std::endl;
#endif


    outputstream_dbg << std::fixed;
    outputstream_dbg.width( width );
    outputstream_dbg.precision( 4 );
    outputstream_dbg << X;
    outputstream_dbg.width( width );
    outputstream_dbg.precision( 4 );
    outputstream_dbg << Y;
    outputstream_dbg.width( width );
    outputstream_dbg.precision( 4 );
    outputstream_dbg << Floor;
    outputstream_dbg.width( width );
    outputstream_dbg.precision( 4 );
    outputstream_dbg << H;
    outputstream_dbg.width( width );
    outputstream_dbg.precision( 4 );
    outputstream_dbg << Sig;
    width = 25;
    outputstream_dbg.width( width );
    outputstream_dbg.precision( 4 );
    outputstream_dbg << t;
    //outputstream_dbg << std::endl;

    outputstream_dbg.width(width);
    outputstream_dbg.precision(4);
    outputstream_dbg << N;
    outputstream_dbg << std::endl;
    outputstream_dbg.flush();

    if (mTrack != NULL)
    {
        mTrack->mtx.lock();

        delete[] this->output_struct->OutParticleArray;
        this->output_struct->OutParticleArray = NULL;

        if ( N > 0 )
        {
            this->output_struct->OutParticleArray = new Fppe::Particle[N];

            for (int i = 0; i < N; ++i)
            {
                this->output_struct->OutParticleArray[i] = state[i];
            }
        }

        this->output_struct->time = t;
        this->output_struct->OutPos[0] = X;
        this->output_struct->OutPos[1] = Y;
        this->output_struct->OutPos[2] = Floor;
        this->output_struct->initialized = true;
        this->output_struct->Particle_count = N;
        //    if (mTrack != NULL)
        //    {
        //        mTrack->mtx.lock();
        emit mTrack->sceneChanged();
        mTrack->mtx.unlock();
   //     QThread::msleep(200);

    }
}

/**
* This method is called when new position estimation avaliable
* \param[in]  position navigation solution
*/
void PositionUpdate::update( const Fppe::Position &position )
{
#if 1
    int width = 15;

    if (counter > 0)
    {
        outputstream << std::fixed
            << "\t<Placemark>" << std::endl
            << "\t\t<LineString>" << std::endl
            << "\t\t\t<tessellate>1</tessellate>" << std::endl
            << "\t\t\t<altitudeMode>clampToGround</altitudeMode>" << std::endl
            << "\t\t\t<coordinates>" << std::endl
            << "\t\t\t\t" << prev_pos.longitude << "," << prev_pos.lattitude << ",0"/* << prev_pos.altitude*/ << std::endl
            << "\t\t\t\t" << position.longitude << "," << position.lattitude << ",0"/* << position.altitude*/ << std::endl
            << "\t\t\t</coordinates>" << std::endl
            << "\t\t</LineString>" << std::endl
            << "\t</Placemark>" << std::endl;
    }
    prev_pos = position;

    outputstream << std::fixed
        << "\t<Placemark>" << std::endl
        << "\t\t<ExtendedData><SchemaData schemaUrl=\"#" << fname <<"\">" << std::endl;

    outputstream << "\t\t\t<SimpleData name=\"number\">";
    outputstream.precision(8);
    outputstream << counter++ << "</SimpleData>" << std::endl;

    outputstream << "\t\t\t<SimpleData name=\"latitude\">";
    outputstream.precision(8);
    outputstream << position.lattitude << "</SimpleData>" << std::endl;

    outputstream << "\t\t\t<SimpleData name=\"longitude\">";
    outputstream.precision(8);
    outputstream << position.longitude << "</SimpleData>" << std::endl;

    outputstream << "\t\t\t<SimpleData name=\"timestamp\">";
    outputstream.precision(8);
    outputstream << position.timestamp << "</SimpleData>" << std::endl;

    outputstream << "\t\t\t<SimpleData name=\"cov_lat\">";
    outputstream.precision(8);
    outputstream << position.covariance_lat_lon[0][0] << "</SimpleData>" << std::endl;

    outputstream << "\t\t\t<SimpleData name=\"cov_lon\">";
    outputstream.precision(8);
    outputstream << position.covariance_lat_lon[1][1] << "</SimpleData>" << std::endl;

    outputstream << "\t\t\t<SimpleData name=\"cov_lat_lon\">";
    outputstream.precision(8);
    outputstream << position.covariance_lat_lon[1][0] << "</SimpleData>" << std::endl;


    outputstream << "\t\t</SchemaData></ExtendedData>" << std::endl;
    outputstream << "\t\t\t<Point><coordinates>";
    outputstream.precision(8);
    outputstream << position.longitude << ",";
    outputstream.precision(8);
    outputstream << position.lattitude << "</coordinates></Point>" << std::endl;

    outputstream << "\t</Placemark>" << std::endl;
#endif
}

void PositionUpdate::setTrack(Track *track)
{
    mTrack = track;
}
