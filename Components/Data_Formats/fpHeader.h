#ifndef FP_HEADER
#define FP_HEADER

#include <time.h>
#include "LibraryInfo.h"

/** General FP header information*/
typedef struct FPHeaderBaseType
{
    uint32_t fp_signature;               /// Start signature
    uint32_t fp_type;                    /// FP type and format signature(ASCII codes "mfp4", "wfp4", "bfp4", "bbp4" are avaliable)
    uint32_t sz_header;                  /// Header size in bites
    uint64_t sz_data;                    /// FP size in bytes
    uint32_t crc;                        /// CRC: algorithm - TBD
    VersionNumberBase fp_buider_version; /// Version of builder
    uint32_t fp_build_number;            /// FP build number
    struct tm fp_build_time;             /// Date and time of fp build (yyyy - mm - dd - hh - mm - ss)
}FPHeaderBaseType;

#endif //FP_HEADER
