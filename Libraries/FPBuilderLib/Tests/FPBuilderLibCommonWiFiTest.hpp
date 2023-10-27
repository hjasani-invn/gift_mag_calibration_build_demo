
#ifndef _FPBuilderLibCommonBleTest_H_
#define _COORDINATE_CONVERTER_H_


#include <string>
#include <fstream>
#include <iomanip>
#include <direct.h>

#include "GridBuilderPrivate.hpp"


void devicePositionGeneration(Fpbl::GridBuilderPrivate *pStaticBuilder, std::fstream  &log);
void wifiGeneration(Fpbl::GridBuilderPrivate *pStaticBuilder, std::fstream  &log);
void bleGeneration(Fpbl::GridBuilderPrivate *pStaticBuilder, std::fstream  &log);
void magGeneration(Fpbl::GridBuilderPrivate *pStaticBuilder, std::fstream  &log);



#endif
