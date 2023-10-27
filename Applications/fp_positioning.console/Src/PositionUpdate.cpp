#include "PositionUpdate.hpp"
#include <iomanip>
#include <list>
#include <algorithm>
#include <chrono>
#include <sstream>

static const std::string kml_magenta = "ffff55ff"; // magenta
static const std::string kml_light_green = "ff00ff55";  // light green
static const std::string kml_cyan = "ffffff55";  // cyan
static const std::string kml_yellow = "ff00ffff";  // yellow
static const std::string kml_green = "ff00aa00";  // green
static const std::string kml_pure_green = "ff00ff00";  // pure_green
static const std::string kml_red = "ffff0000";  // red
static const std::string kml_orange = "ff00aaff";  // orange 
static const std::string kml_blue = "ff5555ff";  // blue

static const std::string kml_white_pushpin = "<href>http://maps.google.com/mapfiles/kml/pushpin/wht-pushpin.png</href>";
static const std::string kml_placemark_circle = "<href>http://maps.google.com/mapfiles/kml/shapes/placemark_circle.png</href>";

static std::string GetRandomKmlCollor()
{
    static std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
    static auto int_ms = std::chrono::duration_cast<std::chrono::milliseconds>(t1.time_since_epoch());
    static std::chrono::duration<long, std::micro> int_usec = int_ms;
    std::srand(int_usec.count()); // update pseudo-random seed
    uint32_t saturation = 0xff;
    uint32_t red = static_cast <uint8_t> (rand());
    uint32_t green = static_cast <uint8_t> (rand());
    uint32_t blue = static_cast <uint8_t> (rand());
    uint32_t collor = (saturation << 24) | (red << 16) | (green << 8) | (blue);
    std::stringstream sstream;
    sstream << std::hex << collor;
    //sstream << "ff";
    //sstream << std::hex << int(red);
    //sstream << std::hex << int(green);
    //sstream << std::hex << int(blue);

    return sstream.str();
}

PositionUpdate::PositionUpdate(FilterType filter_type, const std::string &output_log_file, const std::string &output_log_name_dbg, const std::string &output_particleslog_file_dbg)
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
                 << "\t<SimpleField name=\"timestamp\" type=\"string\"></SimpleField>" << std::endl
                 << "\t<SimpleField name=\"latitude\" type=\"string\"></SimpleField>" << std::endl
                 << "\t<SimpleField name=\"longitude\" type=\"string\"></SimpleField>" << std::endl
                 << "\t<SimpleField name=\"cov_lat\" type=\"string\"></SimpleField>" << std::endl
                 << "\t<SimpleField name=\"cov_lon\" type=\"string\"></SimpleField>" << std::endl
                 << "\t<SimpleField name=\"cov_lat_lon\" type=\"string\"></SimpleField>" << std::endl
                 << "\t<SimpleField name=\"floor\" type=\"string\"></SimpleField>" << std::endl
                 << "\t<SimpleField name=\"floor_std\" type=\"string\"></SimpleField>" << std::endl
                 << "\t<SimpleField name=\"altitude\" type=\"string\"></SimpleField>" << std::endl
                 << "</Schema>" << std::endl;

    std::string kml_pin = kml_placemark_circle;

#if ENABLE_RANDOM_COLORS
    std::string line_collor = GetRandomKmlCollor();
#else 
    std::string line_collor = kml_pure_green;
#endif

#if ENABLE_COLORED_FLOORS
    std::vector<std::string> floor_collors = { kml_cyan, kml_light_green, kml_yellow, kml_orange, kml_red };
    //std::vector<std::string> floor_collors = { kml_yellow, kml_orange, kml_magenta };
    // std::vector<std::string> floor_collors = { kml_red, kml_light_green, kml_magenta };
#else
    std::vector<std::string> floor_collors = { line_collor };
#endif

    for (int i = 0; i < floor_collors.size(); i++)
    {
        pin_styles.push_back("floor_" + std::to_string(i + 1));

        outputstream << "<Style id = \"" << pin_styles.back() << "\">" << std::endl;
		outputstream << "\t<LabelStyle>" << std::endl;
		outputstream << "\t\t<scale>0</scale>" << std::endl;
		outputstream << "\t</LabelStyle>" << std::endl;
        outputstream << "\t<IconStyle>" << std::endl;
        outputstream << "\t\t<color>" << floor_collors[i] << "</color>" << std::endl;
        outputstream << "\t\t<scale>0.4</scale>" << std::endl;
        outputstream << "\t\t<Icon>" << std::endl;
        outputstream << "\t\t" << kml_pin << std::endl;
        outputstream << "\t\t</Icon>" << std::endl;
        outputstream << "\t\t<hotSpot x = \"20\" y = \"2\" xunits = \"pixels\" yunits = \"pixels\"/>" << std::endl;
        outputstream << "\t</IconStyle>" << std::endl;
        outputstream << "\t<LineStyle>" << std::endl;
        outputstream << "\t\t<color>" << floor_collors[i] << "</color>" << std::endl;
        outputstream << "\t</LineStyle>" << std::endl;  
        outputstream << "</Style>" << std::endl;
    }

    if ( outputstream.fail() )
    {
        throw 1;
    }

    if ( output_log_name_dbg.size() > 0 )
    {
        outputstream_dbg.open( output_log_name_dbg );
    }

    this->filter_type = filter_type;
    if (output_particleslog_file_dbg.size() > 0 && !outputstream_particles.is_open())
    {
        outputstream_particles.open(output_particleslog_file_dbg);
    }

    fname = output_log_file;
    counter = 0;
}

PositionUpdate::~PositionUpdate()
{
    outputstream << "</Folder></Document></kml>" << std::endl;
    outputstream.close();
    outputstream_dbg.close();
    outputstream_particles.close();
}

static void LocalPositionTextOutput(std::ofstream &outstream, double X, double Y, double Floor, double H, double Sig, double t)
{
    int width = 15;
    outstream << std::fixed;
    outstream << std::setw(width) << std::setprecision(4) << X;
    outstream << std::setw(width) << std::setprecision(4) << Y;
    width = 12;
    outstream << std::setw(width) << std::setprecision(2) << Floor;
    outstream << std::setw(width) << std::setprecision(3) << H;
    outstream << std::setw(width) << std::setprecision(3) << Sig;
    outstream << std::setw(width) << std::setprecision(0) << t;
}

static void GeoPositionTextOutput(std::ofstream &outstream, const Fppe::Position &position)
{
    int width = 20;
    outstream << std::fixed;
    
    outstream << std::setw(width) << std::setprecision(4) << position.timestamp;
    
    width = 15;
    outstream << std::setw(width) << std::setprecision(8) << position.lattitude;
    outstream << std::setw(width) << std::setprecision(8) << position.longitude;
    
    width = 10;
    outstream << std::setw(width) << std::setprecision(3) << position.azimuth;
    outstream << std::setw(width) << std::setprecision(3) << position.altitude;
    
    width = 7;
    outstream << std::setw(width) << position.floor_number;
    
    width = 10;
    outstream << std::setw(width) << std::setprecision(2) << position.covariance_lat_lon[0][0];
    outstream << std::setw(width) << std::setprecision(2) << position.covariance_lat_lon[0][1];
    outstream << std::setw(width) << std::setprecision(2) << position.covariance_lat_lon[1][0];
    outstream << std::setw(width) << std::setprecision(2) << position.covariance_lat_lon[1][1];
    
    outstream << std::setw(width) << std::setprecision(3) << position.azimuth_std;
    outstream << std::setw(width) << std::setprecision(3) << position.floor_std;
    
    width = 15;
    outstream << std::setw(width) << position.is_valid;
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
    // debug text output
    LocalPositionTextOutput(outputstream_dbg, X, Y, Floor, H, Sig, t);

    if (outputstream_particles.is_open() && (filter_type == FilterType::FROM_MIXED_FILTER /*|| filter_type == FilterType::FROM_MAGNETIC_FILTER*/ ) )
    for (int i = 0; i < N; i++)
    {
#if DISABLE_ZERO_WEIGHTED_PARTICLES
        if (state[i].w>0)
        {
            outputstream_particles << (int)filter_type << " ; " << t << " ; " << state[i].x << " ; " << state[i].y << " ; " << state[i].z << " ; " << state[i].w << std::endl;
        }
#else 
        outputstream_particles << (int)filter_type << " ; " << t << " ; " << state[i].x << " ; " << state[i].y << " ; " << state[i].z << " ; " << state[i].w << std::endl;
#endif

    }
    // std::cout << "=============" << std::endl;
}

/**
* This method is called when new position estimation avaliable
* \param[in]  position navigation solution
*/
void PositionUpdate::update( const Fppe::Position &position )
{
    // debug text output
    GeoPositionTextOutput(outputstream_dbg, position);
    outputstream_dbg.flush();
    outputstream_dbg << std::endl;


    // kml output

    int idx = std::abs(position.floor_number - 1) % pin_styles.size();

    if ((counter % 120) == 0)
    {
        outputstream << "</Folder>" << std::endl;
        outputstream << "<Folder><name>" << "m" + std::to_string(2 * uint32_t(counter / 120)) << "</name>" << std::endl;
    }


    if (counter > 0)
    {
        outputstream << std::fixed
            << "\t<Placemark>" << std::endl
            << "\t\t<styleUrl>#" << pin_styles[idx] << "</styleUrl>" << std::endl
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

	outputstream.precision(3);
    outputstream << std::fixed
	    << "\t<Placemark>" << std::endl
		<< "\t<name>" << position.timestamp/1000.0 << "</name>" << std::endl
        << "\t\t<styleUrl>#" << pin_styles[idx] << "</styleUrl>" << std::endl
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

    outputstream << "\t\t\t<SimpleData name=\"floor\">";
    outputstream.precision(8);
    outputstream << position.floor_number << "</SimpleData>" << std::endl;

    outputstream << "\t\t\t<SimpleData name=\"floor_std\">";
    outputstream.precision(8);
    outputstream << position.floor_std << "</SimpleData>" << std::endl;

    outputstream << "\t\t\t<SimpleData name=\"altitude\">";
    outputstream.precision(8);
    outputstream << position.altitude << "</SimpleData>" << std::endl;


    outputstream << "\t\t</SchemaData></ExtendedData>" << std::endl;
    outputstream << "\t\t\t<Point><coordinates>";
    outputstream.precision(8);
    outputstream << position.longitude << ",";
    outputstream.precision(8);
    outputstream << position.lattitude << "</coordinates></Point>" << std::endl;

    outputstream << "\t</Placemark>" << std::endl;
    outputstream.flush();

}

//-----------------------------------------------------------------------------
// Extended proximity update
ExtendedProximityUpdate::ExtendedProximityUpdate(FilterType filter_type, const std::string &output_log_file, const std::string &output_log_name_dbg)
{
    this->filter_type = filter_type;
    counter = 0;

    fname = output_log_file;
    if (output_log_file.size() > 0)
    {
        outputstream.open(output_log_file);
    }

    fname_dbg = output_log_name_dbg;
    if (output_log_name_dbg.size() > 0)
    {
        outputstream_dbg.open(output_log_name_dbg);
    }

    if (outputstream.fail() && outputstream_dbg.fail())
    {
        throw 1;
    }
}

ExtendedProximityUpdate::~ExtendedProximityUpdate()
{
    outputstream.close();
    outputstream_dbg.close();
}


void ExtendedProximityUpdate::update(double t, const Fppe::ProximityBeaconData &proximity_beacon_data)
{
    if (outputstream_dbg.is_open())
    {
        auto  uuid2str = [](const uint8_t uuid[16])
        {
            std::string str = "00000000-0000-0000-0000-000000000000";

            return str;
        };
        {   // debug logging in console
            outputstream_dbg << std::setprecision(0) << t;
            outputstream_dbg << ", " << int(proximity_beacon_data.beacon_type);
            outputstream_dbg << ", " << uuid2str(proximity_beacon_data.uuid);
            outputstream_dbg << ", " << proximity_beacon_data.major;
            outputstream_dbg << ", " << proximity_beacon_data.minor;
            outputstream_dbg << ", " << std::setprecision(8) << proximity_beacon_data.lattitude;
            outputstream_dbg << ", " << std::setprecision(8) << proximity_beacon_data.longitude;
            outputstream_dbg << ", " << proximity_beacon_data.floor;
            outputstream_dbg << ", " << std::setprecision(3) << proximity_beacon_data.elevation;
            outputstream_dbg << ", " << std::setprecision(3) << proximity_beacon_data.azimuth;
            outputstream_dbg << ", " << int(proximity_beacon_data.txPower);
            outputstream_dbg << ", " << int(proximity_beacon_data.rxPower);
            outputstream_dbg << ", " << proximity_beacon_data.distance;
            outputstream_dbg << ", " << proximity_beacon_data.is_valid ? 1 : 0;;
            outputstream_dbg << std::endl;
        }
    }
}

void ExtendedProximityUpdate::update(double t, const std::initializer_list< Fppe::ProximityBeaconData> &proximity_beacon_data)
{
    /* TBD */
}