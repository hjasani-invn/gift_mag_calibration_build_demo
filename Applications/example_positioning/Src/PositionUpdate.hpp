
#ifndef POSITION_UPDATE_HPP
#define POSITION_UPDATE

#include <fstream>

#include "Fppe.hpp"

// callback class for writing the position into file
class PositionUpdate : public Fppe::IPositionUpdate
{
    public:
        PositionUpdate( const std::string &output_log_name );

        virtual ~PositionUpdate();

        virtual void update( double X, double Y, double Floor, double H, double Sig, double t, const Fppe::Particle *state, int N );

        virtual void update( const Fppe::Position &position );

    private:
		std::ofstream outputstream;	/// stream for log file

};

#endif //POSITION_UPDATE