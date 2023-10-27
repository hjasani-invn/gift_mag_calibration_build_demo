/*****************************************************************************
 *    Copyright (c) 2013 Spirit Corp.
******************************************************************************/
/**
 *   @project                Indoor Demo PC Model
 *   @brief                  PC model config
 *   @file                   model_config.cpp
 *   @author                 D. Churikov
 *   @date                   07.11.2013
 *   @version                1.0
 */
/*****************************************************************************/
//AHTUNG C/C++ mixed code
#define _CRT_SECURE_NO_WARNINGS

#include "model_config.h"
#include <string.h>
#include <stdio.h>
#include <io.h>
#include <fstream>
#include <sstream>

#define MAX_LENGTH_STR  1000

std::string ModelConfig::InputPath = DEFAULT_INPUT_PATH;
std::string ModelConfig::WorkingPath = DEFAULT_WORKING_PATH;
std::string ModelConfig::OutputPath = DEFAULT_OUTPUT_PATH;

char ModelConfig::fAutoPath = 0;

PF::tPersoneParams ModelConfig::PersoneParams = {0.5, 0.00001, 175.};
bool ModelConfig::PersonalyzerEnabled = false;

double ModelConfig::MagneticBias[3] = {0., 0., 0.};
double ModelConfig::MagneticBiasCov[3] = {0., 0., 0.};
long long ModelConfig::MagneticBiasTime = 0;
bool ModelConfig::bFixMagneticBias = true;
bool ModelConfig::bMagneticBiasValid = false;

//=======================================================================
void ModelConfig::DefaultConfig()
{
    SetInputPath( DEFAULT_INPUT_PATH );
    SetWorkingPath( DEFAULT_WORKING_PATH );
    SetOutputPath( DEFAULT_OUTPUT_PATH );
    fAutoPath = 0;

    // persone parameters
    PersoneParams.asl_factor = 0.5;
    PersoneParams.p_asl_factor = 0.00001;
    PersoneParams.PersoneHeight = 175;
    PersonalyzerEnabled = false;

    memset( MagneticBias, 0, sizeof( MagneticBias ) );
    memset( MagneticBiasCov, 0, sizeof( MagneticBiasCov ) );
    MagneticBiasTime = 0;
    bFixMagneticBias = true;
}

//=======================================================================
bool ModelConfig::SetInputPath( char * str )
{
    if( ( str ) && strlen( str ) )
    {
        InputPath = str;
        return true;
    }
    else
        return false;
}

//=======================================================================
bool ModelConfig::SetWorkingPath( char * str )
{
    if( ( str ) && strlen( str ) )
    {
        WorkingPath = str;
        return true;
    }
    else
        return false;
}

//=======================================================================
bool ModelConfig::SetOutputPath( char * str )
{
    if( ( str ) && strlen( str ) )
    {
        OutputPath =  str;
        return true;
    }
    else
        return false;
}

//=======================================================================
std::string ModelConfig::GetOutputPath()
{
    char c_str[1000];

    if( fAutoPath )
    {
        GenerateAutoPath( c_str );
    }
    else
    {
        strcpy( c_str, OutputPath.c_str() );
    }

    return c_str;
}

//=======================================================================
void ModelConfig::SetPersonaParams( PF::tPersoneParams *newPersoneParams )
{
    PersoneParams = *newPersoneParams;
}

//=======================================================================
PF::tPersoneParams ModelConfig::GetPersonaParams()
{
    return PersoneParams;
}

//=======================================================================
void ModelConfig::SetPersonalyzerEnabled( bool newPersonalyzerEnabled )
{
    PersonalyzerEnabled = newPersonalyzerEnabled;
}

//=======================================================================
bool  ModelConfig::GetPersonalyzerEnabled()
{
    return PersonalyzerEnabled;
}

//=======================================================================
bool ModelConfig::LoadConfig( char *cfg_file )
{
    FILE *fin;
    char str [1000];
    int MagBiasCount = 0;
    bool settings_file_found = false;
    bool magnetic_bias_read = false;
    double data_from_settings[7];

    // looking for settings file in input path
    std::string dir = ModelConfig::InputPath;
    std::string settings_file_name = "";
    std::string filter = dir;
    filter += "settings_*.*";
    _finddata64i32_t file_info;

    if( ( _findfirst64i32( filter.c_str(), &file_info ) != -1 ) )
    {
        settings_file_name = dir;
        settings_file_name += file_info.name;
        settings_file_found = true;
    }

    // if settings are found, the file is parsed
    if( settings_file_found )
    {
        std::ifstream infile( settings_file_name );
        std::string line;

        while( std::getline( infile, line ) )
        {
            std::string name = line.substr( 0, line.find( ":" ) );

            if( name == "setMagneticBias" )
            {
                std::string bias_data = line.substr( line.find( ":" ) + 1 );
                size_t pos = 0;
                std::string token;

                int i = 0;
                std::string::size_type sz;

                while( ( pos = bias_data.find( "," ) ) != std::string::npos )
                {
                    token = bias_data.substr( 0, pos );
                    data_from_settings[i] = std::stod( token, &sz );
                    i++;
                    bias_data.erase( 0, pos + 1 );
                }

                data_from_settings[6] = std::stod( bias_data, &sz );
                magnetic_bias_read = true;
            }
        }
    }

    if( ( fin = fopen( cfg_file, "rt" ) ) == 0 )
    {
        //printf("\nDefault model config used\n");
        return 0;
    }

    // read and parse five lines
    for( int i = 0; i < 1000; i++ )
    {
        if( fgets( str, 1000, fin ) == 0 )
            break;

        if( 0 == strncmp( "InputPath=", str, strlen( "InputPath=" ) ) )
            ModelConfig::SetInputPath( str + strlen( "InputPath=" ) );
        else if( 0 == strncmp( "OutputPath=", str, strlen( "OutputPath=" ) ) )
            ModelConfig::SetOutputPath( str + strlen( "OutputPath=" ) );
        else if( 0 == strncmp( "asl_factor=", str, strlen( "asl_factor=" ) ) )
        {
            double tm_dbl;

            if( sscanf( str + strlen( "asl_factor=" ), "%lf", &tm_dbl ) > 0 )
                PersoneParams.asl_factor = tm_dbl;
        }
        else if( 0 == strncmp( "p_asl_factor=", str, strlen( "p_asl_factor=" ) ) )
        {
            double tm_dbl;

            if( sscanf( str + strlen( "p_asl_factor=" ), "%lf", &tm_dbl ) > 0 )
                PersoneParams.p_asl_factor = tm_dbl;
        }
        else if( 0 == strncmp( "PersonalyzerEnabled=", str, strlen( "PersonalyzerEnabled=" ) ) )
        {
            int tm_int;

            if( sscanf( str + strlen( "PersonalyzerEnabled=" ), "%d", &tm_int ) > 0 )
                PersonalyzerEnabled = ( tm_int != 0 ) ? true : false;
        }

#if (MODEL_MAGNETIC_CALIBRATION_ENABLED != 0)
        else if( 0 == strncmp( "bFixMagneticBias=", str, strlen( "bFixMagneticBias=" ) ) )
        {
            int tm_int;

            if( sscanf( str + strlen( "bFixMagneticBias=" ), "%d", &tm_int ) > 0 )
                bFixMagneticBias = ( bool )tm_int;
        }
        else if( 0 == strncmp( "MagneticBiasX=", str, strlen( "MagneticBiasX=" ) ) )
        {
            double tm_dbl;

            if( sscanf( str + strlen( "MagneticBiasX=" ), "%lf", &tm_dbl ) > 0 )
            {
                MagneticBias[0] = tm_dbl;
                MagBiasCount++;
            }
        }
        else if( 0 == strncmp( "MagneticBiasY=", str, strlen( "MagneticBiasY=" ) ) )
        {
            double tm_dbl;

            if( sscanf( str + strlen( "MagneticBiasY=" ), "%lf", &tm_dbl ) > 0 )
            {
                MagneticBias[1] = tm_dbl;
                MagBiasCount++;
            }
        }
        else if( 0 == strncmp( "MagneticBiasZ=", str, strlen( "MagneticBiasZ=" ) ) )
        {
            double tm_dbl;

            if( sscanf( str + strlen( "MagneticBiasZ=" ), "%lf", &tm_dbl ) > 0 )
            {
                MagneticBias[2] = tm_dbl;
                MagBiasCount++;
            }
        }
        else if( 0 == strncmp( "MagneticBiasCovX=", str, strlen( "MagneticBiasCovX=" ) ) )
        {
            double tm_dbl;

            if( sscanf( str + strlen( "MagneticBiasCovX=" ), "%lf", &tm_dbl ) > 0 )
            {
                MagneticBiasCov[0] = tm_dbl;
                MagBiasCount++;
            }
        }
        else if( 0 == strncmp( "MagneticBiasCovY=", str, strlen( "MagneticBiasCovY=" ) ) )
        {
            double tm_dbl;

            if( sscanf( str + strlen( "MagneticBiasCovY=" ), "%lf", &tm_dbl ) > 0 )
            {
                MagneticBiasCov[1] = tm_dbl;
                MagBiasCount++;
            }
        }
        else if( 0 == strncmp( "MagneticBiasCovZ=", str, strlen( "MagneticBiasCovZ=" ) ) )
        {
            double tm_dbl;

            if( sscanf( str + strlen( "MagneticBiasCovZ=" ), "%lf", &tm_dbl ) > 0 )
            {
                MagneticBiasCov[2] = tm_dbl;
                MagBiasCount++;
            }
        }
        else if( 0 == strncmp( "MagneticBiasT=", str, strlen( "MagneticBiasT=" ) ) )
        {
            double tm_dbl;

            if( sscanf( str + strlen( "MagneticBiasT=" ), "%lf", &tm_dbl ) > 0 )
            {
                MagneticBiasTime = tm_dbl;
            }
        }

        // WARNING: biases are rewritten here if "settings_***.log" file is present
        if( ( settings_file_found ) && ( magnetic_bias_read ) )
        {
            MagneticBiasTime = ( long )data_from_settings[0];
            MagneticBias[0] = data_from_settings[1];
            MagneticBias[1] = data_from_settings[2];
            MagneticBias[2] = data_from_settings[3];
            MagneticBiasCov[0] = data_from_settings[4];
            MagneticBiasCov[1] = data_from_settings[5];
            MagneticBiasCov[2] = data_from_settings[6];
        }

#endif // MODEL_MAGNETIC_CALIBRATION_ENABLED != 0
    }

    fclose( fin );

    bMagneticBiasValid = ( MagBiasCount == 6 );

    return 1;
}

//=======================================================================
bool ModelConfig::SaveConfig( char *cfg_file )
{
    FILE *fout;
    //char str [1000];

    if( ( fout = fopen( cfg_file, "wt" ) ) == 0 )
    {
        //printf("\nDefault model config used\n");
        return 0;
    }

    // personalyzer
    fprintf( fout, "asl_factor=%lf\r\n", PersoneParams.asl_factor );
    fprintf( fout, "p_asl_factor=%g\r\n", PersoneParams.p_asl_factor );
    fprintf( fout, "PersonalyzerEnabled=%d\r\n", PersonalyzerEnabled );

    // magnetic sensor
#if (MODEL_MAGNETIC_CALIBRATION_ENABLED != 0)
    fprintf( fout, "bFixMagneticBias=%d\r\n", bFixMagneticBias );
    fprintf( fout, "MagneticBiasT=%.0lf\r\n", ( double )MagneticBiasTime );
    fprintf( fout, "MagneticBiasX=%lf\r\n", MagneticBias[0] );
    fprintf( fout, "MagneticBiasY=%lf\r\n", MagneticBias[1] );
    fprintf( fout, "MagneticBiasZ=%lf\r\n", MagneticBias[2] );
    fprintf( fout, "MagneticBiasCovX=%lf\r\n", MagneticBiasCov[0] );
    fprintf( fout, "MagneticBiasCovY=%lf\r\n", MagneticBiasCov[1] );
    fprintf( fout, "MagneticBiasCovZ=%lf\r\n", MagneticBiasCov[2] );
#else
    fprintf( fout, "bFixMagneticBias=%d\r\n", 1 );
#endif //(MODEL_MAGNETIC_CALIBRATION_ENABLED != 0)
    fclose( fout );
    return 1;
}

//=====================================================================================
/// The function generates the new path located in specified Output_path
/// and with name generated from Input_path
///
/// @param[in] in_path - path to sensor data file
/// @param[out] out_path - destination path for the report files
//=====================================================================================
void ModelConfig::GenerateAutoPath( char *cur_out_path )
{
    char* str, *str1, *str2;
    char* remain;
    char *next_token = NULL;
    char buffer[MAX_LENGTH_STR];

    strcpy_s( buffer, MAX_LENGTH_STR, InputPath.c_str() );
    str1 = buffer;
    str2 = "";
    remain = buffer;
    str = strtok_s( remain, "\\", &next_token );
    remain = next_token;

    while( str != NULL )
    {
        str2 = str1;
        str1 = str;
        str = strtok_s( remain, "\\", &next_token );
        remain = next_token;
    }

    strcpy_s( cur_out_path, MAX_LENGTH_STR, OutputPath.c_str() );
    strcat_s( cur_out_path, MAX_LENGTH_STR, "\\" );
    strcat_s( cur_out_path, MAX_LENGTH_STR, str2 );
    strcat_s( cur_out_path, MAX_LENGTH_STR, "_" );
    strcat_s( cur_out_path, MAX_LENGTH_STR, str1 );
    strcat_s( cur_out_path, MAX_LENGTH_STR, "\\" );
}

//=====================================================================================
bool ModelConfig::GetMagneticBias( int64_t *pTime, double *pBias )
{
    pBias[0] = MagneticBias[0];
    pBias[1] = MagneticBias[1];
    pBias[2] = MagneticBias[2];
    *pTime = MagneticBiasTime;
    return bMagneticBiasValid;
}

void ModelConfig::SetMagneticBias( int64_t Time, double *pBias )
{
    MagneticBias[0] = pBias[0];
    MagneticBias[1] = pBias[1];
    MagneticBias[2] = pBias[2];
    MagneticBiasTime = Time;
    bMagneticBiasValid = true;
    return ;
}

//=====================================================================================
void ModelConfig::GetMagneticBiasCov( double *pBiasCov )
{
    pBiasCov[0] = MagneticBiasCov[0];
    pBiasCov[1] = MagneticBiasCov[1];
    pBiasCov[2] = MagneticBiasCov[2];
}

void ModelConfig::SetMagneticBiasCov( double *pBiasCov )
{
    MagneticBiasCov[0] = pBiasCov[0];
    MagneticBiasCov[1] = pBiasCov[1];
    MagneticBiasCov[2] = pBiasCov[2];
    return ;
}

//=====================================================================================
bool ModelConfig::IsFixMagneticBias()
{
    return bFixMagneticBias;
}

void ModelConfig::SetFixMagneticBias( bool flag )
{
    bFixMagneticBias = flag;
}
