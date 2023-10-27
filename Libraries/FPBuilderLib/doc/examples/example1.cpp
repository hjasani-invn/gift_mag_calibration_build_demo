#include "Fpbl.hpp"


Fpbl::MagneticGrid magnetic_grid;
Fpbl::WiFiGrid wifi_grid;
Fpbl::Grid venue_grid_mfp;
Fpbl::Grid venue_grid_wifi;

 //now supported only CELL_SQUARE
venue_grid_mfp.type = Fpbl::CellType::CELL_SQUARE;
venue_grid_mfp.size = 1.0; // cell size [m]
//define rectangle area [[0,0,0],[64,32,0]]
venue_grid_mfp.min.x = 0.0;
venue_grid_mfp.min.y = 0.0;
venue_grid_mfp.min.floor = 0;
venue_grid_mfp.max.x = 64.0;
venue_grid_mfp.max.y = 32.0;
venue_grid_mfp.max.floor = 0;

venue_grid_wifi.type = venue_grid_mfp.type;
venue_grid_wifi.size = 5.0; // cell size [m]
//wifi area same as for mfp
venue_grid_wifi.min = venue_grid_mfp.min;
venue_grid_wifi.max = venue_grid_mfp.max;

foreach( path: dataset )
{
    Fpbl::ReturnStatus status;
    positions = path.positions;
    attitudes = path.attitudes;
    mag_data = path.mag_data;
    wifi_data = path.wifi_data;
    status = grid_builder.processDevicePosition( pos.timestamp, pos.position );

    Fpbl::GridBuilder grid_builder; // new builder instance is created for each track

    foreach( pos : positions )
    {
        status = grid_builder.processDevicePosition( pos.timestamp, pos.position );
        if (status != ReturnStatus::STATUS_SUCCESS)
        {
            //handle error processing position data
            //...
        }
    }

    foreach( att : attitudes )
    {
        status = grid_builder.processDeviceAttitude( att.timestamp, att.attitude );
        if (status != ReturnStatus::STATUS_SUCCESS)
        {
            //handle error processing attitude data
            //...
        }
    }

    foreach( mag : mag_data )
    {
        status = grid_builder.processMFPMeasurement( mag.timestamp, mag.mag_vector );
        if (status != ReturnStatus::STATUS_SUCCESS)
        {
            //handle error processing magnetic  data
            //...
        }
    }

    foreach( wifi : wifi_data )
    {
        status = grid_builder.processMFPMeasurement( wifi.timestamp, wifi.wifi_scan );
        if (status != ReturnStatus::STATUS_SUCCESS)
        {
            //handle error processing wifi data
            //...
        }
    }

    //updates magnetic grid
    status = grid_builder.updateGridMFP( venue.id, venue_grid_mfp, &magnetic_grid );
    if (status != ReturnStatus::STATUS_SUCCESS)
    {
        //handle error building magnetic grid
        //...
    }

    //updates wifi grid
    Fpbl::ReturnStatus status_wifi = grid_builder.updateGridWiFi( venue.id, venue_grid_wifi, &wifi_grid ); 
    if (status != ReturnStatus::STATUS_SUCCESS)
    {
        //handle error building wifi grid
        //...
    }
}
