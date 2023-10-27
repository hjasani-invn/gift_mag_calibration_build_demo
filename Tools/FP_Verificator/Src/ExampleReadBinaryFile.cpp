// Example of reading binary files

// C includes
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>

// Cpp includes
#include <iostream>
//#include <iomanip>
#include <fstream>
#include <string>

//using namespace std;

int main(const int argc, const char** argv)
{
#if 1

    // reading binary file in C - style
    char* fileName = "filename.bin";

    // getting file size
    struct stat fileBuf;
    stat(fileName, &fileBuf);
    size_t fileSize = fileBuf.st_size;

    FILE *pF = fopen(fileName, "rb");
    if (pF == NULL)
    {
        printf("%s %s %s", "file ", fileName, "cannot be opened");
        exit(0);
    }

    
    // allocating buffer memory for file
    char *pBuffer = (char *)malloc(fileSize);
    
    fread(pBuffer, fileSize, 1, pF);
    fclose(pF);
#else

    // reading binary file in C++ - style
    char* fileName = "filename.bin";

    std::ifstream is;
    is.open(fileName, std::ios::binary);
    if (is.fail())
    {
        is.close();
        std::cout << "file " << fileName <<"cannot be opened" << std::endl;
        exit(0);
    }

    // get length of file:
    is.seekg(0, std::ios::end);
    size_t fileSize = is.tellg();
    is.seekg(0, std::ios::beg);

    // allocate memory:
    char *pBuffer = new char[fileSize];
    // read data as a block:
    is.read(pBuffer, fileSize);
    is.close();

#endif
}