/***************************************************************************//**
 *   @file     platform_support.h
 *   @brief:   support functions and declarations for selected platform
 *   @details: This is a platform specific file that supports functionality
 *             required from application generic file. This file should be
 *             modified according to platform that you are working with.
********************************************************************************
 * Copyright (c) 2019 - 2021 Analog Devices, Inc.
 * All rights reserved.
 *
 * This software is proprietary to Analog Devices, Inc. and its licensors.
 * By using this software you agree to the terms of the associated
 * Analog Devices Software License Agreement.
*******************************************************************************/

#ifndef PLATFORM_SUPPORT_H_
#define PLATFORM_SUPPORT_H_


/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <stdio.h>

/******************************************************************************/
/********************** Macros and Constants Definitions **********************/
/******************************************************************************/

/******************************************************************************/
/********************** Variables and User defined data types *****************/
/******************************************************************************/

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/

char getchar_noblock(void);

#endif /* PLATFORM_SUPPORT_H_ */
