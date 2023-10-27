/*****************************************************************************
 *    Copyright (c) 2013 Spirit Corp.
******************************************************************************/
/**
 *   @project                PDR project
 *   @brief                  initialization User parameters, 
 *                           definition names of data files
 *   @file                   Init.cpp
 *   @author                 M. Zhokhova
 *   @date                   09.07.2013
 *   @version                1.0
 */
/*****************************************************************************/

#include "Init.h"
#include "PDR.h"

//=========================================================================
/// Initialization general User parameters
///
/// @param[in]  userHeight - growth of user [cm]
/// @param[in]  userSex    - sex of user 
/// @param[out] UserParams - pointer to user parameters structure
//=========================================================================
void InitUserParams(int16_t userHeight, eSex userSex, tUserParams* UserParams)
{
	UserParams->height = userHeight;
	UserParams->sex = userSex;
	UserParams->K = 0.008*userHeight - 0.89;
	UserParams->AttUpdateFlag = 1; // flag: enable/disable update of PhoneToUser attitude matrix in user motion

	UserParams->SensorParams.acc_scale_factor = 1;
	UserParams->SensorParams.acc_mean_g = MEAN_G;
	UserParams->SensorParams.IsUpdate = 0;
	
	CreateMatrix(&UserParams->Ci2p, 3, 3);
	UserParams->HeadingCompensation = 0;

	UserParams->Ci2p.Matr[0][0] = 0.; UserParams->Ci2p.Matr[0][1] = 1.; UserParams->Ci2p.Matr[0][2] = 0.; 
	UserParams->Ci2p.Matr[1][0] = 1.; UserParams->Ci2p.Matr[1][1] = 0.; UserParams->Ci2p.Matr[1][2] = 0.;
	UserParams->Ci2p.Matr[2][0] = 0.; UserParams->Ci2p.Matr[2][1] = 0.; UserParams->Ci2p.Matr[2][2] = -1.;
}
