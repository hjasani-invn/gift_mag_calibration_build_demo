#ifndef DATASETSLISTPARSER_HPP
#define DATASETSLISTPARSER_HPP

#include <vector>
#include <utility>
#include <stdint.h>
#include <string>
#include <regex>

void FindDatasetsRecursive(std::string folder_name, std::regex irl_file_mask, std::vector<std::string> &dataset_list);

void parseDatasetsList(std::string data_sets_list_file, std::string list_name, std::vector<std::vector<std::string>> &list);

#endif //DATASETSLISTPARSER_HPP
