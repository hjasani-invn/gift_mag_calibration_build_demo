#include "VenueDetectionUpdate.hpp"

VenueDetectionUpdate::VenueDetectionUpdate(const std::string &output_log_name_dbg)
{

	if (output_log_name_dbg.size() > 0)
	{
		outputstream_dbg.open(output_log_name_dbg);
	}

	counter = 0;
}

VenueDetectionUpdate::~VenueDetectionUpdate()
{
    outputstream_dbg.close();
}

void VenueDetectionUpdate::update(bool pos_inside_venue, double t)
{
    int width = 15;

    outputstream_dbg << std::fixed;
    outputstream_dbg << t;
    outputstream_dbg.width(width);
    outputstream_dbg.precision(4);
    outputstream_dbg << pos_inside_venue << std::endl;
    outputstream_dbg.flush();
}
