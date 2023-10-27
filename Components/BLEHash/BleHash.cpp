
#include <stdint.h>

#include "BleHash.h"

uint64_t getBleHash(const uint16_t major, const uint16_t minor, const uint8_t uuid[16])
{
    return ((uint64_t)major << 16) + minor;
}

uint16_t getBleMajor(const uint64_t ble_hash)
{
    return static_cast<uint16_t>((ble_hash >> 16) & 0xFFFF);
}

uint16_t getBleMinor(const uint64_t ble_hash)
{
    return static_cast<uint16_t>(ble_hash & 0xFFFF);
}
