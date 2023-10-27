/*****************************************************************************
*    Copyright (c) 2016 Invensense Inc.
******************************************************************************/
/**
*   @project                MapperDataConverter
*   @brief                  Settings parsing from json file
*   @file                   MDConverterSettings.h
*   @author                 V. Pentyukhov, D.Churikov
*   @date                   17.08.2016
*/
/*****************************************************************************/
#ifndef SETTINGSPARSER_HPP
#define SETTINGSPARSER_HPP

#include "MDConverterSettings.h"
#include "Venue.h"
#include "jsoncons/json.hpp"

using jsoncons::json;
using jsoncons::json_deserializer;

static  void parseVenueParams(json venueParams, Venue &venue);

bool parseMDConverterSettings(
    std::string settings_file, // input JSON file
    MDConverterSettings &conv_settings
    )
{
    bool result = true;
    json args;

    try
    {
        args = json::parse_file(settings_file);
    }
    catch (const std::exception& e)
    {
        std::cout << "Error during """ << settings_file << """ file parsing." << std::endl;
        std::cerr << e.what() << std::endl;
        return false;
    }

    // JSON file parsing
    {
        try
        {
            {
                json venueParams = args["venue"];
                Venue venue;
                parseVenueParams(venueParams, venue);
                conv_settings.set_venue(venue);
            }
        }
        catch (const std::exception& e)
        {
            std::cout << "Error during venue params parsing." << std::endl;
            std::cerr << e.what() << std::endl;
            result = false;
        }
    }
    
    return result;
}

static	void parseVenueParams(json venueParams, Venue &venue)
{
    venue.id = venueParams["id"].as<int>();
    venue.origin_lattitude = venueParams["origin_lattitude"].as<double>();
    venue.origin_longitude = venueParams["origin_longitude"].as<double>();
    venue.origin_altitude = venueParams["origin_altitude"].as<double>();
    venue.origin_azimuth = venueParams["origin_azimuth"].as<double>();
    venue.floors_count = venueParams["floors_count"].as<int>();
    venue.size_x = venueParams["size_x"].as<double>();
    venue.size_y = venueParams["size_y"].as<double>();
    venue.alfa = venueParams["alfa"].as<double>();
    venue.beta = venueParams["beta"].as<double>();
}


#endif //SETTINGSPARSER_HPP