#include "WiFiIgnoreListParser.hpp"
#include "stringMac_to_intMac.h"
#include "jsoncons/json.hpp"

#include <cctype>

using jsoncons::json;
using jsoncons::json_deserializer;


static bool parseListItem(std::string item_string, std::vector <std::pair <uint64_t, uint64_t>>  &list) //uint64_t &mac_template, uint64_t &mac_mask)
{
    std::string mac_template_string, mac_mask_string;
    for (auto symbol : item_string)
    {
        if (std::isxdigit(symbol))
        {
            mac_template_string += symbol;
            mac_mask_string += '0';
        }
        else if ((symbol == 'x') || (symbol == 'X') || (symbol == '*'))
        {
            mac_template_string += '0';
            mac_mask_string += 'F';
        }
        else if ( (symbol == ':') && ((mac_template_string.size()%3) == 2) && ((mac_mask_string.size() % 3) == 2)) // check for right ':' location in string
        {
            mac_template_string += symbol;
            mac_mask_string += symbol;
        }
        else if (std::isspace(symbol)) { continue; }
        else { break; }
    }
   

    // supporting formats: 66:55:44:33:22:11, 77:66:55:44:33:22:11, 88:77:66:55:44:33:22:11.
    bool check1 = (mac_template_string.size() == (12+5)) || (mac_template_string.size() == (14 + 6)) || (mac_template_string.size() == (16 + 7));
    bool check2 = (mac_mask_string.size() == (12 + 5)) || (mac_mask_string.size() == (14 + 6)) || (mac_mask_string.size() == (16 + 7));
    bool result = check1 && check2;
    
    if (result)
    { 
		uint64_t mac_template = stringMac_to_intMac(mac_template_string);
		uint64_t mac_mask = ~stringMac_to_intMac(mac_mask_string); // inverse bits to convert mask to conventional format
		list.push_back(std::make_pair(mac_template, mac_mask));

		if (mac_template_string.size() == (12 + 5))
		{
			// adding second element to output list in this case (5Ghz mac)
			uint64_t mac_template_2 = stringMac_to_intMac("02:" + mac_template_string);
			uint64_t mac_mask2 = ~stringMac_to_intMac(mac_mask_string); // inverse bits to convert mask to conventional format
			list.push_back(std::make_pair(mac_template_2, mac_mask2));
		}
    }

    return result;
}

void parseWiFiRestrictionList(std::string ignore_list_file, std::string list_name, std::vector <std::pair <uint64_t, uint64_t>>  &list)
{
    

    json wifi_ignore = json::parse_file(ignore_list_file);
    {
        try
        {
            if (wifi_ignore.has_member(list_name))
            {
                json wifi_ignore_list = wifi_ignore[list_name];
                for (size_t i = 0; i < wifi_ignore_list.size(); i++)
                {
                    std::string item_string = wifi_ignore_list[i].as<std::string>();
                    uint64_t mac_template = 0, mac_mask = 0;

					std::vector <std::pair <uint64_t, uint64_t>>  item_list;

					if ( parseListItem(item_string, item_list) )	// now a list of pairs is returned (mac_template, mac_mask), it can have length = 1 or length = 2
                    {
                        //list.push_back(std::make_pair(mac_template, mac_mask));
						for (auto element : item_list)
							list.push_back(element);
                    }

                    else
                    {
                        std::cout << list_name << ": item " << item_string <<" has been ignored" << std::endl;
                    }
                }
            }
        }
        catch (const std::exception& e)
        {
            std::cerr << e.what() << std::endl;
        }
    }
}

void parseWiFiIgnoreList(std::string wifi_ignore_list_file, std::vector <std::pair <uint64_t, uint64_t>>  &list)
{
    parseWiFiRestrictionList(wifi_ignore_list_file, "wifi_ignore_list", list);
    parseWiFiRestrictionList(wifi_ignore_list_file, "wifi_ap_black_list", list);
}

void parseWiFiWhiteList(std::string wifi_ignore_list_file, std::vector <std::pair <uint64_t, uint64_t>>  &list)
{
    parseWiFiRestrictionList(wifi_ignore_list_file, "wifi_ap_white_list", list);
    parseWiFiRestrictionList(wifi_ignore_list_file, "wifi_white_list", list);
}
