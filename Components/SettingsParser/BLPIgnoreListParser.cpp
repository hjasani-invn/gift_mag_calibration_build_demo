#include "BLPIgnoreListParser.hpp"

using jsoncons::json;
using jsoncons::json_deserializer;

void parseBLEIgnoreList(std::string blp_ignore_list_file, std::vector <uint64_t>  &ignore_list)
{
    // JSON file parsing
    json blp_ignored = json::parse_file(blp_ignore_list_file);
    {
        try
        {
            if (blp_ignored.has_member("blp_ignore_list"))
            {
                json blp_ignore_list = blp_ignored["blp_ignore_list"];
                for (int i = 0; i < blp_ignore_list.size(); i++)
                {
                    json blp_ignore = blp_ignore_list[i];
                    uint64_t hash_ignore = blp_ignore["hash"].as<uint64_t>();
                    ignore_list.push_back(hash_ignore);
                }
            }
        }
        catch (const std::exception& e)
        {
            std::cerr << e.what() << std::endl;
        }
    }
}
