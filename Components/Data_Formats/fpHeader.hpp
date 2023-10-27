#ifndef FP_HEADER_HPP
#define FP_HEADER_HPP

#include <ctime>

#include"fpHeader.h"
#include"fpVenue.hpp"
#include"CRC.h"

#include <stdint.h>
#include <cstring>
#include <iomanip>


class FPHeader : protected FPHeaderBaseType
{
public:

    FPHeader()
    {
        fp_signature = 0, fp_type = 0, sz_data = 0, crc = 0;
        fp_buider_version = VersionNumber(0, 0, 0, VersionNumberReleaseId::VERSION_ALPHA);
        fp_build_number = 0, fp_build_time = { 0 };
        sz_header = getHeaderSizeInBytes();
    };

    FPHeaderBaseType getInstance()
    {
        return static_cast<FPHeaderBaseType>(*this);
    }
    
    void setDataSize(uint64_t sz_data)
    {
        this->sz_data = sz_data;
    }
    
    uint64_t getDataSize()
    {
        return this->sz_data;
    }

    void setCRC(const void * pData, const size_t szData)
    {
        if ((pData != 0) && (szData != 0))
        {
            this->crc = crc32(pData, szData);
        }
        else
        {
            this->crc = 0;
        }
    }

    bool checkCRC(const void * pData, const size_t szData)
    {
        bool result = true;
        if ((pData != 0) && (szData != 0) && (this->crc != 0))
        {
            uint32_t data_crc = crc32(pData, szData);
            result = data_crc == this->crc;
        }

        return result;
    }

    void setBuiderVersion(VersionNumber version)
    {
        this->fp_buider_version = version;
    }
    
    VersionNumber getBuilderVersion()
    {
        return VersionNumber(this->fp_buider_version);
    }

    void setFpBuidNumber(uint32_t fp_build_number)
    {
        this->fp_build_number = fp_build_number;
    }

    void setFpBuildTime(std::tm fp_build_time)
    {
        this->fp_build_time = fp_build_time;
    }

    uint32_t getHeaderSizeInBytes() const
    {
        uint32_t sz = 0;
        sz += sizeof(fp_signature);
        sz += sizeof(fp_type);
        sz += sizeof(sz_header);
        sz += sizeof(sz_data);
        sz += sizeof(crc);

        sz += sizeof(fp_buider_version.major);
        sz += sizeof(fp_buider_version.minor);
        sz += sizeof(fp_buider_version.build);
        sz += sizeof(fp_buider_version.releaseId);

        sz += sizeof(fp_build_number);

        sz += sizeof(fp_build_time.tm_year);
        sz += sizeof(fp_build_time.tm_yday);
        sz += sizeof(fp_build_time.tm_mon);
        sz += sizeof(fp_build_time.tm_mday);
        sz += sizeof(fp_build_time.tm_wday);
        sz += sizeof(fp_build_time.tm_hour);
        sz += sizeof(fp_build_time.tm_min);
        sz += sizeof(fp_build_time.tm_sec);
        sz += sizeof(fp_build_time.tm_isdst);

        return sz;
    }

    void write(std::ofstream &fs)
    {
        // TO DO: change macro to private method
#define write_param(fs, param) (fs.write(reinterpret_cast<char*>(&param), sizeof(param)))
        write_param(fs, fp_signature);
        write_param(fs, fp_type);
        write_param(fs, sz_header);
        write_param(fs, sz_data);
        write_param(fs, crc);

        VersionNumber(fp_buider_version).write(fs);

        write_param(fs, fp_build_number);
#if 1
        write_param(fs, fp_build_time.tm_year);
        write_param(fs, fp_build_time.tm_yday);
        write_param(fs, fp_build_time.tm_mon);
        write_param(fs, fp_build_time.tm_mday);
        write_param(fs, fp_build_time.tm_wday);
        write_param(fs, fp_build_time.tm_hour);
        write_param(fs, fp_build_time.tm_min);
        write_param(fs, fp_build_time.tm_sec);
        write_param(fs, fp_build_time.tm_isdst);
#else
        int t = 0;
        write_param(fs, t);
        write_param(fs, t);
        write_param(fs, t);
        write_param(fs, t);
        write_param(fs, t);
        write_param(fs, t);
        write_param(fs, t);
        write_param(fs, t);
        write_param(fs, t);
#endif
        fs.flush();
#undef write_param
    }

    const void*  parse(const void * pBuf)
    {
        
        pBuf = parse_param(pBuf, sizeof(uint32_t), &fp_signature);
        pBuf = parse_param(pBuf, sizeof(uint32_t), &fp_type);
        pBuf = parse_param(pBuf, sizeof(uint32_t), &sz_header);
        pBuf = parse_param(pBuf, sizeof(uint64_t), &sz_data);
        pBuf = parse_param(pBuf, sizeof(uint32_t), &crc);

        VersionNumber version;
        pBuf = version.parse(pBuf);
        fp_buider_version = (VersionNumberBase)version;

        pBuf = parse_param(pBuf, sizeof(uint32_t), &fp_build_number);
        pBuf = parse_param(pBuf, sizeof(int32_t), &fp_build_time.tm_year);
        pBuf = parse_param(pBuf, sizeof(int32_t), &fp_build_time.tm_yday);
        pBuf = parse_param(pBuf, sizeof(int32_t), &fp_build_time.tm_mon);
        pBuf = parse_param(pBuf, sizeof(int32_t), &fp_build_time.tm_mday);
        pBuf = parse_param(pBuf, sizeof(int32_t), &fp_build_time.tm_wday);
        pBuf = parse_param(pBuf, sizeof(int32_t), &fp_build_time.tm_hour);
        pBuf = parse_param(pBuf, sizeof(int32_t), &fp_build_time.tm_min);
        pBuf = parse_param(pBuf, sizeof(int32_t), &fp_build_time.tm_sec);
        pBuf = parse_param(pBuf, sizeof(int32_t), &fp_build_time.tm_isdst);

        return pBuf;
    }

    void  print(std::ostream  &os) const
    {
        
        os << "0x" << std::hex << fp_signature << std::dec;
        os << ", " << "0x" << std::hex << fp_type << std::dec;
        os << ", " << sz_header;
        os << ", " << sz_data;
        os << ", " << "0x" << std::hex << crc << std::dec;
        
        os << ", " << int(fp_buider_version.major);
        os << "." << int(fp_buider_version.minor);
        os << "." << int(fp_buider_version.build);
        os << "." << int(fp_buider_version.releaseId);

        os << ", " << int(fp_build_number);

        os << ", " << int(fp_build_time.tm_mday);
        os << "." << int(fp_build_time.tm_mon + 1);
        os << "." << int(fp_build_time.tm_year + 1900);
        os << ", " << int(fp_build_time.tm_hour);
        os << ":" << int(fp_build_time.tm_min);
        os << ":" << int(fp_build_time.tm_sec);
        os << ", " << int(fp_build_time.tm_yday);
        os << ", " << int(fp_build_time.tm_wday);
        os << ", " << int(fp_build_time.tm_isdst);
    }

private:
    const void* parse_param(const void* pBuf, size_t sz, void* pParam)
    {
        std::memcpy(pParam, pBuf, sz);
        return static_cast<const void*>(static_cast<const uint8_t*>(pBuf) + sz);
    }
};

class fp3Header : public FPHeader
{
    const uint32_t fp3_signature = 0x0;//0x42504446; // denotes "FPDB"
    const uint32_t mfp3_type = 0x3370666D;      // denotes mfp3
    const uint32_t wfp3_type = 0x33706677;      // denotes wfp3
    const uint32_t bfp3_type = 0x33706646;      // denotes bfp3
    const uint32_t blp3_type = 0x33706C46;      // denotes blp3

public:
    fp3Header() {};

    fp3Header initializeMfp3()
    {
        fp_signature = fp3_signature;
        fp_type = mfp3_type;
        sz_header = getHeaderSizeInBytes();
        return *this;
    }

    fp3Header initializeWfp3()
    {
        fp_signature = fp3_signature;
        fp_type = wfp3_type;
        sz_header = getHeaderSizeInBytes();
        return *this;
    }

    fp3Header initializeBfp3()
    {
        fp_signature = fp3_signature;
        fp_type = bfp3_type;
        sz_header = getHeaderSizeInBytes();
        return *this;
    }

    fp3Header initializeBlp3()
    {
        fp_signature = fp3_signature;
        fp_type = blp3_type;
        sz_header = getHeaderSizeInBytes();
        return *this;
    }

    uint32_t getHeaderSizeInBytes()
    {
        return static_cast<FPHeader*>(this)->getHeaderSizeInBytes();
    }
};

/** magnetic fingerprint header information*/
class Mfp4Header : public FPHeader, private MFPVenue
{
public:
    Mfp4Header() {};

    void initialize()
    {
        fp_signature = mfp4_signature;
        fp_type = mfp4_type;
        sz_header = getHeaderSizeInBytes();
    }

    bool check(const size_t szBuf)
    {
        bool result = true;
        result = result && (fp_signature == mfp4_signature);
        result = result && (fp_type == mfp4_type);
        result = result && (sz_header == getHeaderSizeInBytes());
        result = result && (szBuf == (getHeaderSizeInBytes() + getDataSize())); // DB size consistancy check
        // to do: check db dimension consistancy
        //MFPVenue *pVenue = this;

        return result;
    }

    uint32_t getHeaderSizeInBytes()
    {
        //return static_cast<FPHeader*>(this)->getHeaderSizeInBytes() + static_cast<MFPVenue*>(this)->getHeaderSizeInBytes();
        uint32_t sz1 = static_cast<FPHeader*>(this)->getHeaderSizeInBytes();
        uint32_t sz2 = static_cast<MFPVenue*>(this)->getHeaderSizeInBytes();
        return sz1 + sz2;
    }

    void setVenue(const MFPVenue &venue)
    {
        MFPVenue* pVenue = this;
        *pVenue = venue;
    }

    MFPVenue getVenue()
    {
        MFPVenue * pVenue = this;
        return (*pVenue);
    }

    void write(std::ofstream &fs)
    {
        FPHeader* pHeader = this;
        pHeader->write(fs);
        MFPVenue* pVenue = this;
        pVenue->write(fs);
    }

    void parse(const void* pBuf)
    {
        FPHeader* pHeader = this;
        pBuf = pHeader->parse(pBuf);
        MFPVenue* pVenue = this;
        pVenue->parse(pBuf);
    }

private:
    const uint32_t mfp4_signature = 0x42504446; // denotes "FPDB"
    const uint32_t mfp4_type = 0x3470666D;      // denotes mfp4

};

/** WiFi fingerprint header information*/
class Wfp4Header : public FPHeader, protected WFPVenue
{
public:
    Wfp4Header() {};

    void initialize()
    {
        fp_signature = wfp4_signature;
        fp_type = wfp4_type;
        sz_header = getHeaderSizeInBytes();
    }

    uint32_t getHeaderSizeInBytes()
    {
        return static_cast<FPHeader*>(this)->getHeaderSizeInBytes() + static_cast<WFPVenue*>(this)->getHeaderSizeInBytes();
    }

    bool check()
    {
        bool result = true;
        result = result && (fp_signature == wfp4_signature);
        result = result && (fp_type = wfp4_type);
        result = result && (sz_header = getHeaderSizeInBytes());

        return result;
    }

    void setVenue(const WFPVenue &venue)
    {
        *static_cast<WFPVenue*>(this) = venue;
    }

    WFPVenue getVenue()
    {
        return *static_cast<WFPVenue*>(this);
    }

    void write(std::ofstream &fs)
    {
        FPHeader* pHeader = this;
        pHeader->write(fs);
        WFPVenue* pVenue = this;
        pVenue->write(fs);
    }

    void parse(const void* pBuf)
    {
        FPHeader* pHeader = this;
        pBuf = pHeader->parse(pBuf);
        WFPVenue* pVenue = this;
        pVenue->parse(pBuf);
    }

private:
    const uint32_t wfp4_signature = 0x42504446; // denotes "FPDB"
    const uint32_t wfp4_type = 0x34706677; // denotes wfp4
};

/** BLE fingerprint header information*/
class Bfp4Header : public FPHeader, protected BFPVenue
{
public:
    Bfp4Header()    {};

    uint32_t getHeaderSizeInBytes()
    {
        return static_cast<FPHeader*>(this)->getHeaderSizeInBytes() + static_cast<BFPVenue*>(this)->getHeaderSizeInBytes();
    }


    void initialize()
    {
        fp_signature = bfp4_signature;
        fp_type = bfp4_type;
        sz_header = getHeaderSizeInBytes();
    }

    bool check()
    {
        bool result = true;
        result = result && (fp_signature == bfp4_signature);
        result = result && (fp_type = bfp4_type);
        result = result && (sz_header = getHeaderSizeInBytes());

        return result;
    }

    void setVenue(const BFPVenue &venue)
    {
        *static_cast<BFPVenue*>(this) = static_cast<const BFPVenue>(venue);
    }

    BFPVenue getVenue()
    {
        BFPVenue Venue = *static_cast<BFPVenue*>(this);
        return BFPVenue(Venue);
    }

    void write(std::ofstream &fs)
    {
        FPHeader* pHeader = this;
        pHeader->write(fs);

//        fs.flush();
//        exit(0);

        BFPVenue* pVenue = this;
        pVenue->write(fs);

//        fs.flush();
//        exit(0);

    }

    void parse(const void* pBuf)
    {
        FPHeader* pHeader = this;
        pBuf = pHeader->parse(pBuf);
        BFPVenue* pVenue = this;
        pVenue->parse(pBuf);
    }


private:
    const uint32_t bfp4_signature = 0x42504446; // denotes "FPDB"
    const uint32_t bfp4_type = 0x34706662; // denotes wfp4

};

/** BLE proximity header information*/
class Blp4Header : public FPHeader, protected BProxVenue
{
public:
    Blp4Header() {};

    void initialize()
    {
        fp_signature = format_signature;
        fp_type = format_type;
        sz_header = getHeaderSizeInBytes();
    }

    uint32_t getHeaderSizeInBytes()
    {
        return static_cast<FPHeader*>(this)->getHeaderSizeInBytes() + static_cast<BProxVenue*>(this)->getHeaderSizeInBytes();
    }

    bool check()
    {
        bool result = true;
        result = result && (fp_signature == format_signature);
        result = result && (fp_type = format_type);
        result = result && (sz_header = getHeaderSizeInBytes());

        return result;
    }

    void setVenue(const BProxVenue &venue)
    {
        *static_cast<BProxVenue*>(this) = venue;
    }

    BProxVenue getVenue()
    {
        return *static_cast<BProxVenue*>(this);
    }

    void write(std::ofstream &fs)
    {
        FPHeader* pHeader = this;
        pHeader->write(fs);
        BProxVenue* pVenue = this;
        pVenue->write(fs);
    }

    void parse(const void* pBuf)
    {
        FPHeader* pHeader = this;
        pBuf = pHeader->parse(pBuf);
        BProxVenue* pVenue = this;
        pVenue->parse(pBuf);
    }

private:
    const uint32_t format_signature = 0x42445850; // denotes "PXDB"
    const uint32_t format_type = 0x34706C62; // denotes blp4
};

#endif //FP_HEADER_HPP
