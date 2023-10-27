#ifndef IWIFI_BUILDER_HPP
#define IWIFI_BUILDER_HPP
#include "WiFiBuilder.hpp"

namespace Fpbl
{
class IWiFiBuilder {
public:
	IWiFiBuilder() { ; }
        IWiFiBuilder(Fpbl::FPGrid wifi_fp) { ; }
	virtual ~IWiFiBuilder() { ; }
  virtual ReturnStatus buildFingerprint(const WiFiGrid &wifiGrid, WiFiBuilder::LocalDB *wifiFp) = 0;
};
}

#endif //IWIFI_BUILDER_HPP
