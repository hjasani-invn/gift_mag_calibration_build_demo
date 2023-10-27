#pragma once
#include "SensorsCalibrator.h"
#include <eigen/LU>
#include <eigen/Geometry>
#include <eigen/Core>
#include <eigen/Dense>
#include "dirent.h"
#include <regex>

#ifdef _WIN32
#define  SLASH "\\"
#define  ENDL "\n"
#else
#include <limits.h>
#define  SLASH "/"
#define  ENDL "\r\n"
#endif
