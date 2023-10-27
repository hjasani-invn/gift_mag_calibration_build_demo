
#ifndef POSITION_UPDATE_HPP
#define POSITION_UPDATE

#include <fstream>

#include "Fppe.hpp"
#include "track.h"


class PositionUpdate : public Fppe::IPositionUpdate
{
    public:
        PositionUpdate( const std::string &output_log_name, OutPut_for_image *output_struct, const std::string &output_log_name_dbg = "" );

        virtual ~PositionUpdate();

        void setTrack(Track *track);

        virtual void update( double X, double Y, double Floor, double H, double Sig, double t, const Fppe::Particle *state, int N );

        virtual void update( const Fppe::Position &position );

    private:
        std::ofstream outputstream;
        std::ofstream outputstream_dbg;
        std::string fname;
        OutPut_for_image *output_struct;
        Track *mTrack;
        uint32_t counter;
        Fppe::Position prev_pos;
};

#endif //POSITION_UPDATE
