

#include <memory.h>
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <iomanip>

#include "model_config.h"
#include "Model_Main2.h"
#include "CmdReader.hpp"
#include "MDConverterSettings.h"
#include "SettingsParser.hpp"


//#define MAX_LENGTH_STR 1000

static void GetAppFileNameAndWorkingFolder(char *argv_0, std::string &working_path, std::string &app_file_name);
static void PrintHelp(std::string app_file_name);

//=======================================================================
/// Main function of PC-model
///
/// @param[in] argc - count of commad line parameters
/// @param[in] argv - commad line parameters
/// @return: error status
//=======================================================================
int main(int argc, char** argv)
{
  std::string app_file_name;
  std::string working_path;
  MDConverterSettings converter_settings;

  std::cout << std::endl << "Mapper/Navigator track converter" << std::endl;

  GetAppFileNameAndWorkingFolder(argv[0], working_path, app_file_name);

  // command line parsing
  bool success = true;
  std::string str;
  
  
  if (getCmdOption((const char**)argv, (const char**)(argv + argc), "--?") || getCmdOption((const char**)argv, (const char**)(argv + argc), "/?"))
  {
      PrintHelp(app_file_name);
      return 0;
  }

  if (success &= setOptionFromCmd((const int)argc, (const char **)argv, "--settings", &str))
  {// load json
      std::cout << str;
      if (!parseMDConverterSettings(str, converter_settings))
      {
          std::cout << "Settings parsing error." << std::endl;
          return 0;
      }
  }
  else
  {
      std::cout << "Settings file was not specified." << std::endl;
      std::cout << "Default settings will be used in conversion." << std::endl;
      // todo: default settings output
  }

  if (setOptionFromCmd((const int)argc, (const char **)argv, "--input_path", &str))
  {
      converter_settings.set_input_path(str);
  }
  std::cout << "\nInput path: " << converter_settings.get_input_path() << std::endl;

  if (setOptionFromCmd((const int)argc, (const char **)argv, "--output_path", &str))
  {
      converter_settings.set_output_path(str);
  }
  std::cout << "Output path: " << converter_settings.get_output_path() << std::endl;
  
  
  
  //ModelConfig::LoadConfig("model.ini");

  if (!Main_Init(converter_settings))    return 0;

  int run = 1;
  while (run)
  {
      run = Main_Run(converter_settings);
  }

  Main_Done(converter_settings);



  //getchar();

  return 1;
}

//=======================================================================
/// The function generate file name and working path from specified argument
/// of command line
/// @param[in] argv_0 - command line argument argv[0]
/// @param[out] working_path - application working path
/// @param[out] app_file_name - applicationfile name
//=======================================================================
static void GetAppFileNameAndWorkingFolder(char *argv_0, std::string &working_path, std::string &app_file_name)
{
    char *p_app_file_name = std::max(strrchr(argv_0, '\\'), strrchr(argv_0, '/'));
    if (p_app_file_name)
    {
        app_file_name = p_app_file_name + 1;
        working_path = working_path.substr(0, p_app_file_name + 1 - argv_0);
    }
    else
    {
        app_file_name = argv_0;
        working_path = ".\\";
    }

}

//=======================================================================
/// Help info output
/// @param[in] app_file_name - applicationfile name
//=======================================================================
void PrintHelp(std::string app_file_name)
 {
     std::cout << std::endl << app_file_name << " [[--option option_value] ...]";
     std::cout << std::endl << " Options:";
     std::cout << std::endl << "   --input_path path        - spesifies path to input data";
     std::cout << std::endl << "   --output_path path       - spesifies path to conversion results output";
     std::cout << std::endl << "   --settings json_file     - spesifies json file with converter settings";
     std::cout << std::endl << "   --? or /?                - help output";
     std::cout << std::endl << " Example:";
     std::cout << std::endl << "   " << app_file_name;
     std::cout << " --input_path path1 --output_path path2 --settings settings.json" << std::endl;
 }