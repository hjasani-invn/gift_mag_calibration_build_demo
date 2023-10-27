#include "MocFpbl.hpp"

using namespace Fpbl;

GridBuilder::GridBuilder()
{
    mGridBuilder = new MocFpbl();
}

GridBuilder::~GridBuilder(){
    delete mGridBuilder;
}

VersionNumber GridBuilder::getVersionNumber() const{
    return mGridBuilder->getVersionNumber();
}

ReturnStatus GridBuilder::updateGridWiFi(const Fpbl::venue_id &venueId, const Fpbl::Grid &grid, Fpbl::WiFiGrid * wifiGrid){
    return mGridBuilder->updateGridWiFi(venueId, grid, wifiGrid);
}