
#ifndef OCCUPANCYOFPOSITION_HPP
#define OCCUPANCYOFPOSITION_HPP

#include <stdint.h>
namespace FPVerificator
{
    struct OccupancyOfPosition
    {
        float X;
        float Y;
        int32_t number_of_measurements;
        OccupancyOfPosition() :X(0), Y(0), number_of_measurements(0){}
        /*bool operator<(const OccupancyOfPosition& pos) const
        {
        return (this->X < pos.X);
        }
        */
    };

    inline bool compare_occupancy_of_positions_by_x(const OccupancyOfPosition& pos1, const OccupancyOfPosition& pos2)
    {
        return (pos1.X < pos2.X);
    }

    inline bool compare_occupancy_of_positions_by_number_of_measurements(const OccupancyOfPosition& pos1, const OccupancyOfPosition& pos2)
    {
        return (pos1.number_of_measurements < pos2.number_of_measurements);
    }
}
#endif //OCCUPANCYOFPOSITION_HPP
