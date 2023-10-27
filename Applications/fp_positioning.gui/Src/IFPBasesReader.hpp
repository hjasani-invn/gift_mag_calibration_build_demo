#ifndef IFPBASESREADER_HPP
#define IFPBASESREADER_HPP

#include <iostream>
#include <fstream>
#include <string>
#include <stdio.h>
#include <vector>
#include <regex>
#include <cstdint>
#include <sstream>
// WARNING: dirent.h is not provided with visual studio
#include "dirent.h"

#include "Fppe.hpp"

namespace FPPositionConsole
{
    class IFPBasesReader
    {
        public:
            virtual ~IFPBasesReader() {};

            virtual Fppe::ReturnStatus setFPBasesFolder( const std::string &input_data_folder ) = 0;
            /*
            virtual Fppe::ReturnStatus getFPMagBaseName(const std::string &magnetic_base_name, const std::string &file_mask) = 0;

            virtual Fppe::ReturnStatus getFPWiFiBaseName(const std::string &wifi_base_name, const std::string &file_mask) = 0;

            virtual Fppe::ReturnStatus getFPtBleBaseName(const std::string &ble_base_name, const std::string &file_mask) = 0;
            */
            virtual Fppe::ReturnStatus getBaseName( std::string &base_name, const std::string &file_mask ) = 0;

    };
}

#endif // IFPBASESREADER_HPP