//#include <conio.h>
#include <iomanip>
#include <iostream>

#include "FPBasesReader.hpp"


namespace FPPositionConsole
{
    FPBasesReader::FPBasesReader()
    {
    }

    FPBasesReader::~FPBasesReader()
    {
        fileslist.clear();
    }

    Fppe::ReturnStatus FPBasesReader::setFPBasesFolder( const std::string &input_data_folder )
    {
        DIR *dir_pointer;
        struct dirent *entry;

        this->input_data_folder = std::string( input_data_folder );

#ifdef _WIN32
		if (this->input_data_folder.back() != '\\')
		{
			this->input_data_folder = this->input_data_folder + "\\";
		}
#else
		if ( this->input_data_folder.back() != '/' )
		{
			this->input_data_folder = this->input_data_folder + "/";
		}
#endif

        dir_pointer = opendir( this->input_data_folder.c_str() );

        if ( dir_pointer == NULL )
        {
            return Fppe::ReturnStatus::STATUS_UNKNOWN_ERROR;
        }

        std::string fullfilename;

        while ( ( entry = readdir( dir_pointer ) ) )
        {
            if ( entry->d_type != DT_DIR )
            {
                fileslist.push_back( entry->d_name );
            }
        }

        //	std::sort(fileslist.begin(), fileslist.end());
        return Fppe::ReturnStatus::STATUS_SUCCESS;
    }

    Fppe::ReturnStatus FPBasesReader::getBaseName( std::string &base_name, const std::string &file_mask )
    {
        std::string input_data_file = "";

        for ( auto it = fileslist.begin(); it != fileslist.end(); ++it )
        {
            if ( std::regex_match( *it, std::regex( file_mask ) ) )
            {
                //std::cout << *it << std::endl;
                input_data_file = *it;
                break;
            }
        }
		if (input_data_file == "")
		{
			return Fppe::ReturnStatus::STATUS_UNKNOWN_ERROR;
		}

        base_name = input_data_folder + input_data_file;
        /*
        inputDataStream.open(fullfilename);

        if (inputDataStream.fail())
        {
        	inputDataStream.close();
        	return Fppe::ReturnStatus::STATUS_UNKNOWN_ERROR;
        }
        else
        */
        return Fppe::ReturnStatus::STATUS_SUCCESS;
    }





} // namespace FPBuilderConsole