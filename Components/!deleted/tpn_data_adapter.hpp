/*****************************************************************************/
/**
* @copyright   Copyright © InvenSense, Inc. 2016
* @project     RTFPPL
* @file        tpn_data_adapter.hpp
* @author      D Churikov
* @date        7 Apr 2016
* @brief       TPN-RTFPPL data adapter
*/
/*****************************************************************************/


#ifndef TPN_DATA_ADAPTER_HPP
#define TPN_DATA_ADAPTER_HPP

#include "Fppe.hpp"
#include <ostream>

class TpnDataConverter
{
    public:
        //tpn_data_adapter_t();
        TpnDataConverter( Fppe::FPEngine *rt_fppe = NULL );
        ~TpnDataConverter();

        void set_pdr_data( const Fppe::TpnOutput &pdr_data );

        bool convert_pdr_data( const Fppe::TpnOutput &pdr_data, Fppe::CoordinatesIncrement *fppe_data, Fppe::MagneticVector *fppe_mag_vector );
        bool process_pdr_data( const Fppe::TpnOutput &pdr_data );

        static void convert_attitude_data(const Fppe::TpnAttitude &tpn_att, Fppe::Attitude *att);
        static void convert_magnetic_data(const double &time_tag, const Fppe::MagneticVector &tpn_mag_vector, Fppe::MagneticVector *fppe_mag_vector);
        static void convert_position_data(const Fppe::TpnPosition &tpn_pos0, const Fppe::TpnPosition &tpn_pos1, Fppe::CoordinatesIncrement *pos_inc);


    // debug functions
        void set_tpn_converter_out_stream(std::ostream &debug_out);
        void out_tpn_converter_data(Fppe::CoordinatesIncrement inc_data, Fppe::MagneticVector mag_vector);


    private:
        Fppe::FPEngine *rt_fppe;        ///<
        Fppe::TpnOutput last_pdr_data;   ///<

    //debug data
        std::ostream *dbg_rtfppl_stream;         ///< debug output stream
};


#endif
