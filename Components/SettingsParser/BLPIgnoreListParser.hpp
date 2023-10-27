#ifndef BLPIGNORELISTPARSER_HPP
#define BLPIGNORELISTPARSER_HPP

#include "jsoncons/json.hpp"
#include "SettingsParser.hpp"

void parseBLEIgnoreList(std::string blp_ignore_list_file, std::vector <uint64_t>  &ignore_list);

#endif //BLPIGNORELISTPARSER_HPP
