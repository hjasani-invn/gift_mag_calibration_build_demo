/*****************************************************************************/
/**
* @copyright   Copyright © InvenSense, Inc. 2016
* @file        tpn_converter.hpp
* @author      D Churikov
* @date        Jun 20 2016
* @brief       TPN/IRL data converter
*/
/*****************************************************************************/


#ifndef TPN_CONVERTER_HPP
#define TPN_CONVERTER_HPP

#include "LocalData.hpp"
#include "TpnData.hpp"
#include "MagData.hpp"
#include "Venue.h"
#include "CoordinateConverter.h"
#include "FloorConverter.h"
#include <fstream>

#ifndef float64_t
#define float64_t   double;
#endif

class TpnConverter
{
    public:
        TpnConverter();
        explicit TpnConverter(const BaseVenueType &venue);

        // set venue frame parameters
        bool SetVenueFrameParams(const BaseVenueType &venue); // short form
        bool SetVenueFrameParams( float64_t lat0, float64_t lon0, float64_t lat_scale, float64_t lon_scale, float64_t azimut );

        MffAttitude ConvertAttitudeData( double timestamp, const TpnAttitude &tpn_att ); // converts attitude only
        MagneticMeasurement ConvertMagneticData( double timestamp, const TpnMagneticMeasurement &tpn_mag_meas ); // converts magnetic only
        FfPosition ConvertPositionData( double timestamp, const TpnPosition &tpn_pos ); // converts position only


        bool SetFloorsParams(
            uint16_t floors_count,        /**< total floor number in venue */
            double   floor_height,        /**< floor height [m] */
            int16_t  floors_shift,        /**< floor shift*/
            bool     floor_zero_enable
            );

        bool GetFloorsParams(
            uint16_t *floors_count,      /**< total floor number in venue */
            double   *floor_height,        /**< floor height [m] */
            int16_t  *floors_shift,        /**< floor shift*/
            bool     *zero_floor_enable
            ) const;

        bool SetCurrentLogicalFloor(int16_t logical_floor);

        bool SetCurrentRealFloor(int16_t real_floor);

        bool GetCurrentLogicalFloor(int16_t *logical_floor) const;

        bool GetCurrentRealFloor(int16_t *real_floor) const;
        
        int16_t GetCurrentRealFloor(int16_t logical_floor) const
        {
            return floor_converter.GetCurrentRealFloor(logical_floor);
        }


    public: // debug functions
        void SetTpnConverterOutStream( std::ostream &debug_out );
        void OutTpnConverterData( FfPosition &pos, MffAttitude &att, MagneticMeasurement &mag_meas );
        std::ostream ** getLogDescriptor();

    private:
        void OutPositionData( const FfPosition &pos );
        void OutAttitudeData( const MffAttitude &att );
        void OutMagneticData( const MagneticMeasurement &mag_meas );

        GeoLocConverter2D geo2local;
        GeoLocConverter2D ned2local;
        
        FloorConverter    floor_converter;

        //debug data
        std::ostream *dbg_rtfppl_stream;         ///< debug output stream
        std::ofstream stub_dbg_rtfppl_stream;     ///< debug output stream

};

#endif
