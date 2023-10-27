/**
* \copyright       Copyright (C) TDK-Invensense, 2018
* \brief           Nonlinear filter to reduce BLE levels fluctuations
* \defgroup        ble_proximity
* \file            ble_nonlinear_filter.cpp
* \ingroup         ble_proximity
* \author          V Pentukhov
* \date            07.09.2018
*/

#include <string>
#include <map>
#include <list>
#include <iostream>
#include "Fppe.hpp"
#include "wifi_data.hpp"
#include "CoordinateConverter.h"
#include "ble_nonlinear_filter.hpp"


// for x < 0
double  BleNonlinearFilter::process(int64_t timestamp, double x)
     {

         if ((timestamp - this->prev_timestamp) > timestamp_interval)
         {
             counter = 0;
         }

        counter++;
        if (counter == 1)
        {
            x_s = x;            
        }
        else      
        {
            if (x > (y_pref + x_s * decline_coef))
          {
               x_s = x;
          }
          else
          {
               x_s = y_pref + x_s * decline_coef;
          }       
           // std::cout << "counter = " << counter << "  x =  " << x << "  y_pref =  " << x_s << std::endl;

        }
        y_pref = x_s;
        //std::cout << "counter = " << counter << "  x =  " << x << "  y_pref =  " << y_pref << std::endl;
        timestamp = this->prev_timestamp;

        return y_pref;
     }

BSSID  BleNonlinearFilter::get_hash()
{
    return ble_hash;
}
