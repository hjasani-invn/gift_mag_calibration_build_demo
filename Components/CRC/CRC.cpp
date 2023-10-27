#include "CRC.h"

uint32_t crc32(const void *buff, const size_t size)
{
    static const uint32_t INIT_XOR = 0xFFFFFFFFUL;
    static const uint32_t FINAL_XOR = 0xFFFFFFFFUL;
    static const uint32_t CRC_POLY = 0xEDB88320UL;

    uint32_t crcreg = INIT_XOR;

    for (size_t j = 0; j < size; ++j)
    {
        uint8_t b = ((uint8_t*)buff)[j];

        for (int i = 0; i < 8; ++i)
        {
            if ((crcreg ^ b) & 1)
            {
                crcreg = (crcreg >> 1) ^ CRC_POLY;
            }
            else
            {
                crcreg = (crcreg >> 1);
            }

            b >>= 1;
        }
    }

    crcreg = (crcreg ^ FINAL_XOR);
    return crcreg;
}
