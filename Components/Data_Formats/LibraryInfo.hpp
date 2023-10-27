/**
* \file LibraryInfo.hpp
* \brief Version data types. C++ implementation
* \author Mikhail Frolov, D.Churikov
* \version 0.2
* \copyright TDK-InvenSense, all rights reserved
* \date Oct 23, 2017
*/

#ifndef LIBRAY_INFO_HPP
#define LIBRAY_INFO_HPP

#include"LibraryInfo.h"

#include <stdint.h>
#include <fstream>
#include <cstring>


/** library build info */
struct VersionNumber : public VersionNumberBase
{
    VersionNumber() : VersionNumber(0, 0, 0, VersionNumberReleaseId::VERSION_ALPHA) {};

    VersionNumber(uint8_t _major_, uint8_t _minor_, uint32_t _build_, VersionNumberReleaseId _releaseId_)
    {
        major = _major_, minor = _minor_, build = _build_, releaseId = _releaseId_;
    };

    VersionNumber(const VersionNumber& vn)
    {
        *this = vn;
    };

    explicit VersionNumber(const VersionNumberBase& vn)
    {
        *(VersionNumberBase*)this = vn;
    };

    virtual ~VersionNumber() {};

    VersionNumber &operator=(const VersionNumber& vn)
    {
        this->major = vn.major;
        this->minor = vn.minor;
        this->build = vn.build;
        this->releaseId = vn.releaseId;
        return *this;
    }

    void write(std::ofstream &fs)
    {
#define write_param(fs, param) (fs.write(reinterpret_cast<char*>(&param), sizeof(param)))
        write_param(fs, major);
        write_param(fs, minor);
        write_param(fs, build);
        int32_t tm_int = releaseId;
        write_param(fs, tm_int);
#undef write_param
    }

    const void* parse(const void *pBuf)
    {
        pBuf = parse_param(pBuf, sizeof(major), &major);
        pBuf = parse_param(pBuf, sizeof(minor), &minor);
        pBuf = parse_param(pBuf, sizeof(build), &build);
        int32_t tm_int(0);
        pBuf = parse_param(pBuf, sizeof(tm_int), &tm_int);
        releaseId = static_cast<VersionNumberReleaseId>(tm_int);

        return pBuf;
    }

    void print(std::ofstream &fs)
    {
        fs << major;
        fs << "." << minor;
        fs << "." << build;
        fs << "." << static_cast<int>(releaseId);
    };

private:
    const void* parse_param(const void* pBuf, size_t sz, void* pParam)
    {
        std::memcpy(pParam, pBuf, sz);
        return static_cast<const void*>(static_cast<const uint8_t*>(pBuf)+sz);
    }
};

#endif //LIBRAY_INFO_HPP

