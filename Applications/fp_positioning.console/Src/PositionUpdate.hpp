
#ifndef POSITION_UPDATE_HPP
#define POSITION_UPDATE_HPP

#include <fstream>

#include "Fppe.hpp"

#define ENABLE_COLORED_FLOORS 0
#define ENABLE_RANDOM_COLORS 1
#define DISABLE_ZERO_WEIGHTED_PARTICLES 1
/**
* particles type
0 - none / general, 1 - from mixed filter, 2 - from magnetic filter, 3 - from starting filter,
10 - mixed position, 11 - mag position, 12 - wifi position, 13 - ble posiytion, 14 - proximity position
*/
enum class FilterType
{
    NONE_GENERAL = 0,
    FROM_MIXED_FILTER = 1,
    FROM_MAGNETIC_FILTER = 2,
    FROM_STARTING_FILTER = 3,
    MIXED_POSITION = 10,
    MAG_POSITION = 11,
    WIFI_POSITION = 12,
    BLE_POSIYTION = 13,
    PROXIMITY_POSITION = 14
};

// callback class for writing the position into file
class PositionUpdate : public Fppe::IPositionUpdate
{
    public:
        PositionUpdate(FilterType filter_type, const std::string &output_log_name, const std::string &output_log_name_dbg = "", const std::string &output_particleslog_file_dbg = "");

        virtual ~PositionUpdate();

        virtual void update( double X, double Y, double Floor, double H, double Sig, double t, const Fppe::Particle *state, int N );

        virtual void update( const Fppe::Position &position );

    private:
        std::ofstream outputstream;	/// stream for log file
        std::ofstream outputstream_dbg;
        std::ofstream outputstream_particles;
        FilterType filter_type;
        std::string fname;
        uint32_t counter;
        Fppe::Position prev_pos;
        std::vector<std::string> pin_styles;
};


// callback class for writing the position into file
class ExtendedProximityUpdate : public Fppe::IExtendedProximityUpdate
{
public:
    ExtendedProximityUpdate(FilterType filter_type, const std::string &output_log_name, const std::string &output_log_name_dbg = "");

    virtual ~ExtendedProximityUpdate();

    virtual void update(double t, const Fppe::ProximityBeaconData &proximity_beacon_data);

    /**
    * This method is called when new proximity position estimation avaliable
    * \param[in]  list of proximity beacons data
    */
    virtual void update(double t, const std::initializer_list< Fppe::ProximityBeaconData> &proximity_beacon_data);


private:
    std::ofstream outputstream;/// stream for log file
    std::ofstream outputstream_dbg;
    FilterType filter_type;
    std::string fname, fname_dbg;
    uint32_t counter;
    Fppe::Position prev_pos;
    std::vector<std::string> pin_styles;
};

#endif //POSITION_UPDATE_HPP
