#include <iostream>
#include "fpbl.hpp"
#include "WiFiBuilder.hpp"

int main(int argc, char *argv[])
{
	std::cout << "test" << std::endl;
    Fpbl::GridBuilder *pB = new Fpbl::GridBuilder();
	Fpbl::VersionNumber version = pB->getVersionNumber();
	std::cout << "StaticBuilder version:" << std::endl;
	std::cout << "\tbuild type:\t" << static_cast<int>(version.releaseId) << std::endl;
	std::cout << "\tbuild number:\t" << static_cast<int>(version.major)
		<< "." << static_cast<int>(version.minor)
		<< "." << static_cast<int>(version.build) << std::endl;

	Fpbl::venue_id v_id = {0};
	Fpbl::Grid grid = {Fpbl::CellType::CELL_SQUARE, 5.0};
	Fpbl::WiFiGrid wifiGrid;
	Fpbl::ReturnStatus status = pB->updateGridWiFi( v_id, grid, &wifiGrid);
	delete pB;
	Fpbl::WiFiBuilder *wifiBulder = new Fpbl::WiFiBuilder();
	Fpbl::WiFiBuilder::LocalDB wifiDb;
	wifiBulder->buildFingerprint(wifiGrid, &wifiDb);

	delete wifiBulder;
	std::cin.get();
	return 0;
}