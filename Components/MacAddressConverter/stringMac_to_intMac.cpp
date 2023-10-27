#include <vector>
#include <sstream>


#include "stringMac_to_intMac.h"

uint64_t stringMac_to_intMac(std::string const &s)
{
	uint64_t mac = 0;
	uint64_t a[8];

	int cnt = std::count(s.begin(), s.end(), ':');


	const char delim = ':';

	std::stringstream buf(s);
	std::string token;
	std::vector<std::string> result;

	while (std::getline(buf, token, delim))
	{
		result.push_back(token);
	}

	uint16_t size = result.size();

	if (size != 8 && size != 7 && size != 6)
		return 0;


	for (int i = 0; i < 8; i++)
	{
		a[i] = 0;
	}

	mac = 0;

	for (int i = 0; i < size; i++)
	{
		std::stringstream ss;
		ss << result[size - 1 - i];
		ss >> std::hex >> a[i];
		ss.str("");
		ss.clear();

		mac |= a[i] << i * 8;
	}

	return mac;
}
