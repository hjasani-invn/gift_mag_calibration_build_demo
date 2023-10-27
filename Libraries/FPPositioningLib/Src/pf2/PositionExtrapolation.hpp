/**
* \copyright       Copyright (C) SPIRIT Navigation, LLC., 2014
* \brief           Position Extrapolator class. Serves to compensate delay in PF position.
* \defgroup        Extrapolator 
* \file            PositionExtrapolation.hpp
* \author          Y. Kotik
* \date            28.11.2014
*/

#include "PF.hpp"
#include <cmath>
#ifdef _MSC_VER
#if  _MSC_VER < 1600
#include "stdint.h"
#else 
#include <stdint.h>
#endif
#else
#include <stdint.h>
#endif

namespace PF
{
	class PositionExtrapolator
	{
	public:
		PositionExtrapolator()
		{
			isEnabled = true;
			isInitialized = false;
			step_length = 0.5;
			step_start = 0;
			step_end = 0;
			LastExtrapolationTime = -1;
			LastVirtStepTime = -1;
			PF::Particle pf_pos;
			PF::Particle pf_std;
		}; 
		//~PositionExtrapolator() {}; 

		// enable/disable extrapolation
		bool enabled()                { return isEnabled; }
		void setEnabled( bool flag ) { isEnabled = flag; }
		bool initialized()			{ return isInitialized; }
		void reset() { isInitialized = false; }
		int64_t getLastExtrapolationTime() { return LastExtrapolationTime; }
		double getLastVirtStepTime() { return LastVirtStepTime; }
		void setLastVirtStepTime( double time ) {LastVirtStepTime = time;}
	
		// set necessary data
		void setData( const double *new_step_length, 
						const double *new_step_start, 
						const double *new_step_end, 
						PF::Particle &pf1_pos,
						PF::Particle &pf1_std )
		{
			if ( *new_step_length > 0 )
			{
				step_length = *new_step_length;
			}
			step_start = *new_step_start;
			step_end = *new_step_end;
			pf_pos = pf1_pos;
			pf_std = pf1_std;
			isInitialized = true;
		}

		void CalculateDelayCompensation( const int64_t *tsAcc,
										double *x,
										double *y,
										bool *valid,
										tPFData* virt_step_PFData_array,
										int8_t VirtStepsRead)
		{
			// pf1_std.h - choose threshold
			double heading = pf_pos.h;
			*x = pf_pos.pos.x;
			*y = pf_pos.pos.y;
			*valid = true;
			LastExtrapolationTime = *tsAcc;

			if ( isEnabled )
			{
				if (( isInitialized ) && ( pf_std.h < 0.7 ))
				{
					*valid = true;
										
					for ( int i = 0; i < VirtStepsRead; i++ )
					{
						heading += virt_step_PFData_array[i].headingInc;

						if ( virt_step_PFData_array[i].pEvent > 0 )
						{
							*x += step_length * cos(heading);
			 				*y += step_length * sin(heading);
						}
					}
				}
				else
				{
					*x = pf_pos.pos.x;
					*y = pf_pos.pos.y;
					*valid = false;
				}
			}
			else 
			{
				*x = pf_pos.pos.x;
				*y = pf_pos.pos.y;
				*valid = false; 
			}
		}

	private:
		bool isEnabled;
		bool isInitialized;
		int64_t LastExtrapolationTime; // last time we extrapolated position
		double LastVirtStepTime;
		double step_length;
		double step_start;
		double step_end;
		PF::Particle pf_pos;
		PF::Particle pf_std;
	};

};
