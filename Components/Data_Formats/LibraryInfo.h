/**
* \file LibraryInfo.h
* \brief Version data types. C implementation
* \author Mikhail Frolov, D.Churikov
* \version 0.2
* \copyright TDK-InvenSense, all rights reserved
* \date Oct 23, 2017
*/

#ifndef LIBRAY_INFO_H
#define LIBRAY_INFO_H

#include <stdint.h>

/** build type */
typedef enum VersionNumberReleaseId
{
    VERSION_ALPHA = 0, ///< alpha version
    VERSION_BETA = 1,  ///< beta version
    VERSION_RELEASE_CANDIDATE = 2, ///< pre release
    VERSION_RELEASE = 3           ///< release
}VersionNumberReleaseId;

/** library version info */
typedef struct VersionNumberBase
{
    uint8_t major;  /**< major version number */
    uint8_t minor;  /**< minor version number */
    uint32_t build; /**< plain build number */
    VersionNumberReleaseId releaseId; /**< build type */
}VersionNumberBase;

#endif //LIBRAY_INFO_H

