/*****************************************************************************
*    Copyright (c) 2016 Invensense Inc.
******************************************************************************/
/**
*   @project                MapperDataConverter
*   @brief                  Converter settings class definition
*   @file                   MDConverterSettings.h
*   @author                 D.Churikov
*   @date                   17.08.2016
*/
/*****************************************************************************/
#ifndef __MD_CONVERTER_SETTINGS
#define __MD_CONVERTER_SETTINGS

#include "Venue.h"

#include <iostream>

class MDConverterSettings
{
public:
    MDConverterSettings() 
    {
        SetDefaultSettings();
    };
    ~MDConverterSettings() {};

    void SetDefaultSettings()
    {
        input_path = "./";
        output_path = "./";

        venue = { 0 };

        f_check_mag_vector = false;
        mean_mag_vector_magnitude = 0.;
        mag_vector_count = 0;
    };

    void set_input_path(std::string _input_path)
    {
        input_path = _input_path;
    };
    
    std::string get_input_path()
    {
        return input_path;
    };

    void set_output_path(std::string _output_path)
    {
        output_path = _output_path;
    };

    std::string get_output_path()
    {
        return output_path;
    };

    void set_venue(Venue _venue)
    {
        venue = _venue;
    };

    Venue get_venue()
    {
        return venue;
    };

    // debug methods
    bool is_check_mag_vector_magnitude_enabled()
    {
        return f_check_mag_vector;
    }

    void add_mag_vector(double mx, double my, double mz)
    {
        mean_mag_vector_magnitude += sqrt(mx*mx + my*my + mz*mz);
        mag_vector_count++;
    }

    double get_mean_magnetic_magnitude()
    {
        if (mag_vector_count > 0)
            return mean_mag_vector_magnitude / mag_vector_count;
        else
            return 0;
    }

private :
    std::string input_path;
    std::string output_path;

    Venue venue;
    


    bool f_check_mag_vector;
    double mean_mag_vector_magnitude;
    double mag_vector_count;
    
};

#endif