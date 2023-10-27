// FPBuilderLibCommonTest.cpp : Defines the entry point for the console application.
//

//#include "stdafx.h"
//#include <windows.h>]

#include <string>
#include <fstream>
#include <iomanip>
#include <direct.h>

#include "GridBuilderPrivate.hpp"

#include "FPBuilderLibCommonWiFiTest.hpp""



int main(int argc, char* argv[])
{

    Fpbl::GridBuilderPrivate *pStaticBuilder = new Fpbl::GridBuilderPrivate();

                 char current_work_dir[1000];
#ifdef _WIN32
                 //GetCurrentDirectory(1000, current_work_dir);
                 _getcwd(current_work_dir, sizeof(current_work_dir));
#else
                 getcwd(current_work_dir, sizeof(current_work_dir));
#endif
                 const std::string folderPath(current_work_dir);
                 const std::string logFimeNamePos = folderPath + "\\posdatagrid.log";
                 std::fstream  posdatagrid_log(logFimeNamePos.c_str(), std::ios::out);
                 const std::string logFimeNameWiFi = folderPath + "\\wifidatagrid.log";
                 std::fstream  wifidatagrid_log(logFimeNameWiFi.c_str(), std::ios::out);
                 const std::string logFimeNameBle = folderPath + "\\Bledatagrid.log";
				 std::fstream  bledatagrid_log(logFimeNameBle.c_str(), std::ios::out);
                 const std::string logFimeNameMag = folderPath + "\\magdatagrid.log";
				 std::fstream  magdatagrid_log(logFimeNameMag.c_str(), std::ios::out);

				 
                 devicePositionGeneration(pStaticBuilder, posdatagrid_log);

                 wifiGeneration(pStaticBuilder, wifidatagrid_log);

                 bleGeneration(pStaticBuilder, bledatagrid_log);

                 magGeneration(pStaticBuilder, magdatagrid_log);
				 
				 devicePositionGeneration(pStaticBuilder, std::fstream());

    return 0;
}
