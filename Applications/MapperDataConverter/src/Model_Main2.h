#ifndef __MODELMAIN_H
#define __MODELMAIN_H

#include "MDConverterSettings.h"

#define INPUT_DATA_TYPE     1 // mapper data: trk-file, from_rt logs (calibrated data)
//#define INPUT_DATA_TYPE     0 // navigator datta

#define ENABLE_OUTPUT       1 // enable / disable output of converted data

#define M_PI    3.1415926535897932384626433832795
#define TPN_INITIAL_HEADING     (0/180.*M_PI)

int Main_Init(MDConverterSettings converter_settings);
int Main_Run(MDConverterSettings &converter_settings);
void Main_Done(MDConverterSettings converter_settings);

#endif