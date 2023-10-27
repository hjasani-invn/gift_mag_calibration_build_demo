#include "DatasetsListProcessor.hpp"
#include "stringMac_to_intMac.h"
#include "jsoncons/json.hpp"

#include <cctype>
// WARNING: dirent.h is not provided with visual studio
#include "dirent.h"

using jsoncons::json;
using jsoncons::json_array;
using jsoncons::json_deserializer;

void FindDatasetsRecursive(std::string folder_name, std::regex irl_file_mask, std::vector<std::string> &dataset_list)
{
    DIR *dir_pointer = opendir(folder_name.c_str());
    struct dirent *entry;

    while ((dir_pointer != 0) && (entry = readdir(dir_pointer)))
    {
        if (entry->d_type == DT_REG)
        {
            std::string file_name(entry->d_name);
            if (std::regex_match(file_name, irl_file_mask))
            {
                dataset_list.push_back(folder_name);
            }
        }
        else if (entry->d_type == DT_DIR)
        {
            if (std::string(entry->d_name).at(0) != '.')
            {
                //std::string sub_folder_name = folder_name + SLASH + std::string(entry->d_name);
                std::string sub_folder_name = folder_name + "/" + std::string(entry->d_name);
                FindDatasetsRecursive(sub_folder_name, irl_file_mask, dataset_list);
            }
        }
    }
    closedir(dir_pointer);
}

void parseDatasetsList(std::string data_sets_list_file, std::string list_name, std::vector<std::vector<std::string>> &list)
{
    json data_sets_list = json::parse_file(data_sets_list_file);
    {
        try
        {
            if (data_sets_list.has_member(list_name))
            {
                json data_list = data_sets_list[list_name];
                for (size_t i = 0; i < data_list.size(); i++)
                {
                    //std::cout << data_list[i]<< std::endl;
                    std::vector<std::string> item_of_list = data_list[i].as_vector<std::string>();
                    list.push_back(item_of_list);
                }
            }
        }
        catch (const std::exception& e)
        {
            std::cerr << e.what() << std::endl;
        }
    }
}
