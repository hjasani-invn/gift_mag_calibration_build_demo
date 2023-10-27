#include "BleMapCreator.hpp"
#include "BleBuilder.hpp"

using namespace Fpbl;

BleBuilder::BleBuilder()
{
	mBleBuilder = new BleMapCreator();
}

BleBuilder::BleBuilder(FPGrid ble_fp)
{
    mBleBuilder = new BleMapCreator(ble_fp);
}

BleBuilder::~BleBuilder()
{
	delete mBleBuilder;
}

ReturnStatus BleBuilder::buildFingerprint(const BleGrid &bleGrid, LocalDB *BleFp)
{
    ReturnStatus status = mBleBuilder->buildFingerprint(bleGrid, BleFp);
	return status;
}
