#ifndef IBLE_BUILDER_HPP
#define IBLE_BUILDER_HPP
#include "BleBuilder.hpp"

namespace Fpbl
{
class IBleBuilder {
public:
	IBleBuilder() { ; }
	IBleBuilder(FPGrid ble_fp) { ; }
	virtual ~IBleBuilder() { ; }
  virtual ReturnStatus buildFingerprint(const BleGrid &bleGrid, BleBuilder::LocalDB *bleFp) = 0;
};
}

#endif //IBle_BUILDER_HPP
