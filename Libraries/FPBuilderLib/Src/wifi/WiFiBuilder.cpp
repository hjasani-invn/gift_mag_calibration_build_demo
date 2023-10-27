#include "WiFiMapCreator.hpp"
#include "WiFiBuilder.hpp"

using namespace Fpbl;

WiFiBuilder::WiFiBuilder()
{
	mWiFiBuilder = new WiFiMapCreator();
}

WiFiBuilder::WiFiBuilder(FPGrid wifi_fp)
{
	mWiFiBuilder = new WiFiMapCreator(wifi_fp);
}

WiFiBuilder::~WiFiBuilder()
{
	delete mWiFiBuilder;
}

ReturnStatus WiFiBuilder::buildFingerprint(const WiFiGrid &wifiGrid, LocalDB *wifiFp)
{
	ReturnStatus status = mWiFiBuilder->buildFingerprint(wifiGrid, wifiFp);
	return status;
}
