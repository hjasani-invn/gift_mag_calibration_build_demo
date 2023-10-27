#ifndef CMD_READER_HPP
#define CMD_READER_HPP
#include <string>
#include <sstream>
#include <algorithm>

const char *getCmdOption(const char **begin, const char **end, const std::string &option)
{
    const char **itr = std::find(begin, end, option);

    if (itr != end && ++itr != end)
    {
        return *itr;
    }

    return NULL;
}
template<typename T>
bool setOptionFromCmd(const int argc, const char **argv, const std::string &key, T* output)
{
    const char *v = getCmdOption(argv, argv + argc, key);
    if (v != NULL)
    {
        std::stringstream ss;
        ss << v;
        ss >> *output;
    }
    return v != NULL;
}

#endif