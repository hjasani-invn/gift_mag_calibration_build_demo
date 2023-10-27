/*****************************************************************************
 *    Copyright (c) 2013 Spirit Corp.
******************************************************************************/
/**
 *   @project                HIOL
 *   @brief                  Personalization module, header file
 *   @file                   Personalization.cpp
 *   @author                 Y. Kotik, D. Churikov
 *   @date                   26.11.2013
 *   @version                1.0
 */
/*****************************************************************************/
#ifndef PERSONALIZATION_H
#define PERSONALIZATION_H

#ifdef _MSC_VER
 #if  _MSC_VER < 1600
  #include "stdint.h"
 #else 
  #include <stdint.h>
 #endif
#else
 #include <stdint.h>
#endif


#ifdef __ANDROID__
#   include <android/log.h>
#	ifdef LOGI
#		undef LOGI
#	endif
#   define LOGI(...) ((void)__android_log_print(ANDROID_LOG_INFO, "FusionFilter", __VA_ARGS__))
#else
#   define LOGI(...)
#endif



namespace PF
{
  ///--------------------------------------------------------------------------
  /// Persone parameters structure
  struct tPersoneParams
  {
    double asl_factor;      // conversion factor
    double p_asl_factor;    // asl_factor covariance
    float PersoneHeight;
  };

  ///--------------------------------------------------------------------------
  /// Personalizer class
  class Personalizer
  {
  public:
    Personalizer(); // constructor
    ~Personalizer() // destructor
    {};

    // enable/disable
    bool enabled()                { return isEnabled; }
    void  setEnabled( bool flag ) { isEnabled = flag; LOGI("Person Enabled %d",isEnabled);}
    bool is_sleep() { return isSleep;};
    void setSleep(bool value) { isSleep = value;};

    // initialization/reset
    bool initialize(float PersoneHeight);
    void reset();
    
    // update personal parameters
    void update(bool inTracking, double pos_x, double pos_y, float p_event, double acc_span);

    // get persone parameters
    tPersoneParams getPersoneParams();
    double getStepLength( double acc_span, float p_event);
    double getPersoneHeight();

    // Personalizer context
    void getContext( double* pASLFactor, double* pP_ASLFactor, bool *pisEnabled);
    void setContext( double* pASLFactor, double* pP_ASLFactor, bool *pisEnabled);

  private:
    void reset_params();
    double getQ();
    double getR();

  private:
    bool isEnabled;       // enable flag for exteranl control
    bool isSleep;       // speep (enable) flag for internal control
    double asl_factor;     // persone parameter: conversion factor of acceleration magnitude to step length
    double p_asl_factor;   // asl_factor covariance
    int16_t count_down;
    double acc_span1;
    float p_event0;
    float p_event1;
    double pos_x1, pos_y1; // one steps ago
    double pos_x0, pos_y0; // two steps ago
    double pf_step_length1;
    float PersoneHeight;  // information data: persone hight
  };


  double GetPersoneHeight(double ASLFactor);
};

#endif
