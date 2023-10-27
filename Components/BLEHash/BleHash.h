#ifndef BLE_HASH_H
#define BLE_HASH_H
#include <stdint.h>
uint64_t getBleHash(const uint16_t major, const uint16_t minor, const uint8_t uuid[16]);
uint16_t getBleMajor(const uint64_t ble_hash);
uint16_t getBleMinor(const uint64_t ble_hash);
#endif  // BLE_HASH_H
