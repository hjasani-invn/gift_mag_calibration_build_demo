#ifndef FP_VENUE_DATA_HPP
#define FP_VENUE_DATA_HPP

#include "Venue.h"
#include "LibraryInfo.hpp"
#include <iostream>

#include <stdint.h>

const double k_default_magnetic_cell_size = 1; // default magnetic cell size
const double k_default_wifi_cell_size = 5; // default magnetic cell size
const double k_default_ble_cell_size = 5; // default magnetic cell size
const int k_default_fp_cell_type = 0;

struct BaseVenue : public BaseVenueType
{
    BaseVenue() // default constructor
    {
        id = 0;
        origin_lattitude = 0;
        origin_longitude = 0;
        origin_altitude = 0;
        origin_azimuth = 0;
        floors_count = 0;
        floor_height = 0;
        size_x = 0;
        size_y = 0;
        alfa = 0;
        beta = 0;
        venue_type = VenueType::kDefaultVenue;
        floor_shift = 1;
        floor_zero_enable = false;
    };

    BaseVenue(const Venue& venue) // default constructor
    {
        *this = venue;
    }

    BaseVenue &operator=(const Venue& venue)
    {
        this->id = venue.id;

        this->origin_lattitude = venue.origin_lattitude;
        this->origin_longitude = venue.origin_longitude;
        this->origin_altitude = venue.origin_altitude;
        this->origin_azimuth = venue.origin_azimuth;
        this->floors_count = venue.floors_count;
        this->floor_height = venue.floor_height;
        this->size_x = venue.size_x;
        this->size_y = venue.size_y;

        this->alfa = venue.alfa;
        this->beta = venue.beta;

        this->venue_type = VenueType::kDefaultVenue; // default value for additional member
        this->floor_shift = 1;
        this->floor_zero_enable = false;
        return *this;
    }

    void combine_floor_shift_and_floor_zero_enable()
    {
        floor_shift &= 0x7FFF;  // high bit to zero
        if (floor_zero_enable)
            floor_shift |= 0x8000;
            
    }

    void divide_floor_shift_and_floor_zero_enable()
    {
        
        floor_zero_enable = ((floor_shift & 0x8000) != 0) ? true : false;
        floor_shift = ((floor_shift & 0x4000) != 0) ? floor_shift |= 0x8000 : floor_shift &= 0x7FFF;
        /*floor_zero_enable = false;
        if ((floor_shift & 0x8000) != 0)
            floor_zero_enable = true;
        if ((floor_shift & 0x4000) != 0)
            floor_shift |= 0x8000;
        */
    }

    uint32_t getHeaderSizeInBytes()
    {
        uint32_t sz = 0;
        sz += sizeof(id);
        sz += sizeof(origin_lattitude);
        sz += sizeof(origin_longitude);
        sz += sizeof(origin_altitude);
        sz += sizeof(origin_azimuth);
        sz += sizeof(floors_count);
        sz += sizeof(floor_height);
        sz += sizeof(size_x);
        sz += sizeof(size_y);
        sz += sizeof(alfa);
        sz += sizeof(beta);

        return sz;
    }

    void write(std::ofstream &fs)
    {
        combine_floor_shift_and_floor_zero_enable();

        fs.write(reinterpret_cast<char*>(&id), sizeof(id));
        fs.write(reinterpret_cast<char*>(&origin_lattitude), sizeof(origin_lattitude));
        fs.write(reinterpret_cast<char*>(&origin_longitude), sizeof(origin_longitude));
        fs.write(reinterpret_cast<char*>(&origin_altitude), sizeof(origin_altitude));
        fs.write(reinterpret_cast<char*>(&origin_azimuth), sizeof(origin_azimuth));
        fs.write(reinterpret_cast<char*>(&floors_count), sizeof(floors_count));
        fs.write(reinterpret_cast<char*>(&floor_height), sizeof(floor_height));
        fs.write(reinterpret_cast<char*>(&size_x), sizeof(size_x));
        fs.write(reinterpret_cast<char*>(&size_y), sizeof(size_y));
        fs.write(reinterpret_cast<char*>(&alfa), sizeof(alfa));
        fs.write(reinterpret_cast<char*>(&beta), sizeof(beta));
    }

protected:
    const void* parse_param(const void* pBuf, size_t sz, void* pParam)
    {
        std::memcpy(pParam, pBuf, sz);
        return static_cast<const void*>(static_cast<const uint8_t*>(pBuf)+sz);
    }

};

struct FingerprintVenue : public virtual BaseVenue
{
    double cell_size;               /**< fingerprint cell size, m*/
    int8_t celltype;                /**< cell type*/

    FingerprintVenue() : BaseVenue(), cell_size(1), celltype(0) {};
};

/** MFP venue structure*/
struct MFPVenue : public FingerprintVenue
{
    double magX;                    /**< average venue magnetic vector X [uT] */
    double magY;                    /**< average venue magnetic vector Y [uT] */
    double magZ;                    /**< average venue magnetic vector Z [uT] */
    
    MFPVenue() : FingerprintVenue(), magX(0), magY(0), magZ(0) {};

    MFPVenue(const Venue& venue)
    {
        *this = venue;
    }

    MFPVenue &operator=(const Venue& venue)
    {
        BaseVenue::operator=(venue);

        this->cell_size = k_default_magnetic_cell_size;
        this->celltype = k_default_fp_cell_type;

        this->magX = venue.magX;
        this->magY = venue.magY;
        this->magZ = venue.magZ;

        return *this;
    }

    uint32_t getHeaderSizeInBytes()
    {
        uint32_t sz = 0;

        sz = static_cast<BaseVenue*>(this)->getHeaderSizeInBytes();

        sz += sizeof(cell_size);
        sz += sizeof(floor_shift);
        sz += sizeof(celltype);
        sz += sizeof(venue_type);

        sz += sizeof(magX);
        sz += sizeof(magY);
        sz += sizeof(magZ);
        return sz;
    }

    void write(std::ofstream &fs)
    {

        static_cast<BaseVenue*>(this)->write(fs);

        fs.write(reinterpret_cast<char*>(&cell_size), sizeof(cell_size));
        fs.write(reinterpret_cast<char*>(&floor_shift), sizeof(floor_shift));
        fs.write(reinterpret_cast<char*>(&celltype), sizeof(celltype));
        int32_t tm_int = venue_type;
        fs.write(reinterpret_cast<char*>(&tm_int), sizeof(tm_int));

        fs.write(reinterpret_cast<char*>(&magX), sizeof(magX));
        fs.write(reinterpret_cast<char*>(&magY), sizeof(magY));
        fs.write(reinterpret_cast<char*>(&magZ), sizeof(magZ));
    }

    const void *parse(const void * pBuf)
    {
        //static_cast<BaseVenue*>(this)->parse(pBuf);
        pBuf = parse_param(pBuf, sizeof(venue_id), &id);
        pBuf = parse_param(pBuf, sizeof(double), &origin_lattitude);
        pBuf = parse_param(pBuf, sizeof(double), &origin_longitude);
        pBuf = parse_param(pBuf, sizeof(double), &origin_altitude);
        pBuf = parse_param(pBuf, sizeof(double), &origin_azimuth);
        pBuf = parse_param(pBuf, sizeof(uint16_t), &floors_count);
        pBuf = parse_param(pBuf, sizeof(double), &floor_height);
        pBuf = parse_param(pBuf, sizeof(double), &size_x);
        pBuf = parse_param(pBuf, sizeof(double), &size_y);

        pBuf = parse_param(pBuf, sizeof(double), &alfa);
        pBuf = parse_param(pBuf, sizeof(double), &beta);

        pBuf = parse_param(pBuf, sizeof(double), &cell_size);
        pBuf = parse_param(pBuf, sizeof(int16_t), &floor_shift);
        pBuf = parse_param(pBuf, sizeof(int8_t), &celltype);

        int32_t tm_int(0); 
        pBuf = parse_param(pBuf, sizeof(tm_int), &tm_int);
        venue_type = static_cast<VenueType>(tm_int);
        pBuf = parse_param(pBuf, sizeof(double), &magX);
        pBuf = parse_param(pBuf, sizeof(double), &magY);
        pBuf = parse_param(pBuf, sizeof(double), &magZ);

        divide_floor_shift_and_floor_zero_enable();

        return pBuf;
    }
};

/** Wifi fingerprint venue default parameters*/
const double k_default_wifi_min_prob_metric = 0.005;
const int k_default_wifi_min_ap_count = 5;

/** WiFi venue structure*/
struct WFPVenue : public FingerprintVenue
{
    double minProbMetric;           /**< min probability metric in navigation */
    int32_t APCount;                /**< total AP count in WFP */
    int8_t minAPCount;              /**< min AP count in navigation */

    WFPVenue() : FingerprintVenue(), minProbMetric(0.), APCount(0), minAPCount(0) {};

    uint32_t getHeaderSizeInBytes()
    {
        uint32_t sz = 0;

        sz = static_cast<BaseVenue*>(this)->getHeaderSizeInBytes();

        sz += sizeof(cell_size);
        sz += sizeof(floor_shift);
        sz += sizeof(celltype);
        sz += sizeof(venue_type);

        sz += sizeof(minProbMetric);
        sz += sizeof(APCount);
        sz += sizeof(minAPCount);
        return sz;
    }

    void write(std::ofstream &fs)
    {
        static_cast<BaseVenue*>(this)->write(fs);

        fs.write(reinterpret_cast<char*>(&cell_size), sizeof(cell_size));
        fs.write(reinterpret_cast<char*>(&floor_shift), sizeof(floor_shift));
        fs.write(reinterpret_cast<char*>(&celltype), sizeof(celltype));
        fs.write(reinterpret_cast<char*>(&venue_type), sizeof(venue_type));

        fs.write(reinterpret_cast<char*>(&minProbMetric), sizeof(minProbMetric));
        fs.write(reinterpret_cast<char*>(&APCount), sizeof(APCount));
        fs.write(reinterpret_cast<char*>(&minAPCount), sizeof(minAPCount));
    }

    const void *parse(const void * pBuf)
    {
        //static_cast<BaseVenue*>(this)->parse(pBuf);
        pBuf = parse_param(pBuf, sizeof(venue_id), &id);
        pBuf = parse_param(pBuf, sizeof(double), &origin_lattitude);
        pBuf = parse_param(pBuf, sizeof(double), &origin_longitude);
        pBuf = parse_param(pBuf, sizeof(double), &origin_altitude);
        pBuf = parse_param(pBuf, sizeof(double), &origin_azimuth);
        pBuf = parse_param(pBuf, sizeof(uint16_t), &floors_count);
        pBuf = parse_param(pBuf, sizeof(double), &floor_height);
        pBuf = parse_param(pBuf, sizeof(double), &size_x);
        pBuf = parse_param(pBuf, sizeof(double), &size_y);

        pBuf = parse_param(pBuf, sizeof(double), &alfa);
        pBuf = parse_param(pBuf, sizeof(double), &beta);

        pBuf = parse_param(pBuf, sizeof(double), &cell_size);
        pBuf = parse_param(pBuf, sizeof(int16_t), &floor_shift);
        pBuf = parse_param(pBuf, sizeof(int8_t), &celltype);

        int32_t tm_int(0);
        pBuf = parse_param(pBuf, sizeof(tm_int), &tm_int);
        venue_type = static_cast<VenueType>(tm_int);
        pBuf = parse_param(pBuf, sizeof(double), &minProbMetric);
        pBuf = parse_param(pBuf, sizeof(int32_t), &APCount);
        pBuf = parse_param(pBuf, sizeof(int8_t), &minAPCount);

        divide_floor_shift_and_floor_zero_enable();

        return pBuf;
    }
};

/** BLE fingerprint venue default parameters*/
const double k_default_ble_min_prob_metric = 0.005;
const int k_default_ble_min_ap_count = 3;

/** BLE venue structure*/
struct BFPVenue : public FingerprintVenue
{
    double minProbMetric;           /**< min probability metric in navigation */
    int32_t APCount;                /**< total AP count in WFP */
    int8_t minAPCount;              /**< min AP count in navigation */

    BFPVenue() : minProbMetric(0.), APCount(0), minAPCount(0) {};

    BFPVenue(const Venue& venue)
    {
        *this = venue;
    }

    BFPVenue &operator=(const Venue& venue)
    {
        BaseVenue::operator=(venue);

        this->cell_size = k_default_ble_cell_size;
        this->celltype = 0;

        this->minProbMetric = 0;
        this->APCount = 0;
        this->minAPCount = 0;

        return *this;
    }

    uint32_t getHeaderSizeInBytes()
    {
        uint32_t sz = 0;
        sz = static_cast<BaseVenue*>(this)->getHeaderSizeInBytes();
        sz += sizeof(cell_size);
        sz += sizeof(floor_shift);
        sz += sizeof(celltype);
        sz += sizeof(venue_type);

        sz += sizeof(minProbMetric);
        sz += sizeof(APCount);
        sz += sizeof(minAPCount);
        return sz;
    }

    void write(std::ofstream &fs)
    {
        static_cast<BaseVenue*>(this)->write(fs);

        fs.write(reinterpret_cast<char*>(&cell_size), sizeof(cell_size));
        fs.write(reinterpret_cast<char*>(&floor_shift), sizeof(floor_shift));
        fs.write(reinterpret_cast<char*>(&celltype), sizeof(celltype));
        fs.write(reinterpret_cast<char*>(&venue_type), sizeof(venue_type));

        fs.write(reinterpret_cast<char*>(&minProbMetric), sizeof(minProbMetric));
        fs.write(reinterpret_cast<char*>(&APCount), sizeof(APCount));
        fs.write(reinterpret_cast<char*>(&minAPCount), sizeof(minAPCount));
    }

    const void *parse(const void * pBuf)
    {
        //static_cast<BaseVenue*>(this)->parse(pBuf);
        pBuf = parse_param(pBuf, sizeof(venue_id), &id);
        pBuf = parse_param(pBuf, sizeof(double), &origin_lattitude);
        pBuf = parse_param(pBuf, sizeof(double), &origin_longitude);
        pBuf = parse_param(pBuf, sizeof(double), &origin_altitude);
        pBuf = parse_param(pBuf, sizeof(double), &origin_azimuth);
        pBuf = parse_param(pBuf, sizeof(uint16_t), &floors_count);
        pBuf = parse_param(pBuf, sizeof(double), &floor_height);
        pBuf = parse_param(pBuf, sizeof(double), &size_x);
        pBuf = parse_param(pBuf, sizeof(double), &size_y);

        pBuf = parse_param(pBuf, sizeof(double), &alfa);
        pBuf = parse_param(pBuf, sizeof(double), &beta);

        pBuf = parse_param(pBuf, sizeof(double), &cell_size);
        pBuf = parse_param(pBuf, sizeof(int16_t), &floor_shift);
        pBuf = parse_param(pBuf, sizeof(int8_t), &celltype);

        int32_t tm_int(0);
        pBuf = parse_param(pBuf, sizeof(tm_int), &tm_int);
        venue_type = static_cast<VenueType>(tm_int);
        pBuf = parse_param(pBuf, sizeof(double), &minProbMetric);
        pBuf = parse_param(pBuf, sizeof(int32_t), &APCount);
        pBuf = parse_param(pBuf, sizeof(int8_t), &minAPCount);

        divide_floor_shift_and_floor_zero_enable();

        return pBuf;
    }
};

/** BLE proximity venue default parameters*/
const int16_t k_default_cutoff_threshold = -10;   /**< Default Cut off threshold for BLE meassurement*/
const uint16_t k_default_filter_length = 2;      /**< Default BLE measurement filter lenght*/
const uint16_t k_default_repeat_number = 2;      /**< Default BLE measurement repetedness*/
const int16_t k_default_cutoff_threshold_start = -30; // -15dBm ~matches to default proximity initializer uncertainty (6m)
const int16_t k_default_filter_length_in_start = 1;
const int16_t k_default_repeat_number_in_start = 1;

/** BLE proximity venue structure*/
struct BProxVenue : public virtual BaseVenue
{
    int16_t cutoff_threshold;           /**< Cut off threshold for BLE meassurement*/
    uint16_t filter_length;             /**< BLE measurement filter lenght*/
    uint16_t repeat_number;             /**< BLE measurement repetedness*/

    BProxVenue() : BaseVenue(), cutoff_threshold(k_default_cutoff_threshold), filter_length(k_default_filter_length), repeat_number(k_default_repeat_number) {};

    BProxVenue (const BaseVenue venue)
    {
        BaseVenueType::operator=(venue);
        BProxVenue::cutoff_threshold = k_default_cutoff_threshold;
        BProxVenue::filter_length = k_default_filter_length;
        BProxVenue::repeat_number = k_default_repeat_number;
    }

    uint32_t getHeaderSizeInBytes() const
    {
        uint32_t sz = 0;
        //sz = static_cast<const BProxVenue *const>(this)->getHeaderSizeInBytes();
        //sz = static_cast<BaseVenue*>(this)->getHeaderSizeInBytes();
        sz += sizeof(id);
        sz += sizeof(origin_lattitude);
        sz += sizeof(origin_longitude);
        sz += sizeof(origin_altitude);
        sz += sizeof(origin_azimuth);
        sz += sizeof(floors_count);
        sz += sizeof(floor_height);
        sz += sizeof(size_x);
        sz += sizeof(size_y);
        sz += sizeof(alfa);
        sz += sizeof(beta);

        sz += sizeof(floor_shift);
        sz += sizeof(venue_type);

        sz += sizeof(cutoff_threshold);
        sz += sizeof(filter_length);
        sz += sizeof(repeat_number);
        return sz;
    }

    void write(std::ofstream &fs)
    {
        static_cast<BaseVenue*>(this)->write(fs);

        fs.write(reinterpret_cast<char*>(&floor_shift), sizeof(floor_shift));
        fs.write(reinterpret_cast<char*>(&venue_type), sizeof(venue_type));

        fs.write(reinterpret_cast<char*>(&cutoff_threshold), sizeof(cutoff_threshold));
        fs.write(reinterpret_cast<char*>(&filter_length), sizeof(filter_length));
        fs.write(reinterpret_cast<char*>(&repeat_number), sizeof(repeat_number));
    }

    const void *parse(const void * pBuf)
    {
        //static_cast<BaseVenue*>(this)->parse(pBuf);
        pBuf = parse_param(pBuf, sizeof(venue_id), &id);
        pBuf = parse_param(pBuf, sizeof(double), &origin_lattitude);
        pBuf = parse_param(pBuf, sizeof(double), &origin_longitude);
        pBuf = parse_param(pBuf, sizeof(double), &origin_altitude);
        pBuf = parse_param(pBuf, sizeof(double), &origin_azimuth);
        pBuf = parse_param(pBuf, sizeof(uint16_t), &floors_count);
        pBuf = parse_param(pBuf, sizeof(double), &floor_height);
        pBuf = parse_param(pBuf, sizeof(double), &size_x);
        pBuf = parse_param(pBuf, sizeof(double), &size_y);

        pBuf = parse_param(pBuf, sizeof(double), &alfa);
        pBuf = parse_param(pBuf, sizeof(double), &beta);

        pBuf = parse_param(pBuf, sizeof(int16_t), &floor_shift);

        int32_t tm_int(0);
        pBuf = parse_param(pBuf, sizeof(tm_int), &tm_int);
        venue_type = static_cast<VenueType>(tm_int);
        pBuf = parse_param(pBuf, sizeof(int16_t), &cutoff_threshold);
        pBuf = parse_param(pBuf, sizeof(uint16_t), &filter_length);
        pBuf = parse_param(pBuf, sizeof(uint16_t), &repeat_number);

        divide_floor_shift_and_floor_zero_enable();

        return pBuf;
    }
};

#endif //FP_VENUE_DATA_HPP
