#ifndef WIFIIGNORELISTPARSER_HPP
#define WIFIIGNORELISTPARSER_HPP

#include <vector>
#include <utility>
#include <stdint.h>
#include <string>

void parseWiFiIgnoreList(std::string wifi_ignore_list_file, std::vector <std::pair <uint64_t, uint64_t>>  &list);
void parseWiFiWhiteList(std::string wifi_ignore_list_file, std::vector <std::pair <uint64_t, uint64_t>>  &list);

#endif //WIFIIGNORELISTPARSER_HPP
