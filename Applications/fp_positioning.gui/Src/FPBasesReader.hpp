#ifndef FPBASESREADER_HPP
#define FPBASESREADER_HPP

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

#include "IFPBasesReader.hpp"


namespace FPPositionConsole
{
    class FPBasesReader: public IFPBasesReader
    {
        public:
            FPBasesReader();

            virtual ~FPBasesReader();

            virtual Fppe::ReturnStatus setFPBasesFolder( const std::string &input_data_folder );
            /*
            virtual Fppe::ReturnStatus getFPMagBaseName(const std::string &magnetic_base_name, const std::string &file_mask);

            virtual Fppe::ReturnStatus getFPWiFiBaseName(const std::string &wifi_base_name, const std::string &file_mask);

            virtual Fppe::ReturnStatus getFPtBleBaseName(const std::string &ble_base_name, const std::string &file_mask);
            */
            virtual Fppe::ReturnStatus getBaseName( std::string &base_name, const std::string &file_mask );

        private:
            std::string input_data_folder;
            std::vector<std::string> fileslist;



    };
}

#endif // FPBASESREADER_HPP