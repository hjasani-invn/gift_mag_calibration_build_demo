#ifndef VENUDE_DETECTION_UPDATE_HPP
#define VENUDE_DETECTION_UPDATE_HPP

#include <fstream>
#include "Fppe.hpp"

// callback class for writing the position into file
class VenueDetectionUpdate : public Fppe::IVenueDetectionUpdate
{
    public:
		VenueDetectionUpdate(const std::string &output_log_name_dbg = "");

		virtual ~VenueDetectionUpdate();

		virtual void update(bool pos_inside_venue, double t);
		
    private:
        std::ofstream outputstream_dbg;
        std::string fname;
        uint32_t counter;
};

#endif //VENUDE_DETECTION_UPDATE_HPP