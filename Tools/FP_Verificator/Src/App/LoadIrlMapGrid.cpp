// C includes
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>

// Cpp includes
#include <iostream>
//#include <iomanip>
#include <fstream>
#include <sstream>

#include <string>
#include <stdint.h>
bool load_irl_map_grid_file(const std::string &fname, char** pBuffer, uint32_t &fileSize)
{
 #if 0

    // reading binary file in C-style
 
    // getting file size
    struct stat fileBuf;
    stat(fname, &fileBuf);
    size_t fileSize = fileBuf.st_size;

    FILE *pF = fopen(fname, "rb");
    if (pF == NULL)
    {
        printf("%s %s %s", "file ", fname, "cannot be opened");
        exit(0);
    }

    
    // allocating buffer memory for file
    char *pBuffer = (char *)malloc(fileSize);
    
    fread(pBuffer, fileSize, 1, pF);
    fclose(pF);
#else

    // reading binary file in Cpp-style

    std::ifstream is;
    is.open(fname, std::ios::binary);
    if (is.fail())
    {
        is.close();
        std::cout << "file " << fname << "cannot be opened" << std::endl;
        exit(0);
    }

    // get length of file:
    is.seekg(0, std::ios::end);
    fileSize = is.tellg();
    is.seekg(0, std::ios::beg);

    // allocate memory:
    *pBuffer = new char[fileSize];
    // read data as a block:
    is.read(*pBuffer, fileSize);
    is.close();
#endif

    return true;
}
