#include "Fpbl.hpp"
#include "WiFiBuilder.hpp"

Fpbl::WiFiBuilder wifi_builder;
Fpbl::WiFiBuilder::LocalDB db();

Fpbl::ReturnStatus result = wifi_builder.buildFingerprint( wifi_grid, &db );
if (status != ReturnStatus::STATUS_SUCCESS)
{
    //handle error processing attitude data
    //...
}

std::string output_file_name = "fingerprint.wifi3";
//TODO save fingerprint to file
