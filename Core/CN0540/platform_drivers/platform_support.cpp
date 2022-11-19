/***************************************************************************//**
 *   @file     platform_support.cpp
 *   @brief:   support functions and declarations for particular platform
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

/******************************************************************************/
/************************ Includes Files **************************************/
/******************************************************************************/
#include <mbed.h>

// Platform support needs to be C-compatible to work with other drivers
#ifdef __cplusplus
extern "C"
{
#endif

#include "platform_support.h"

/******************************************************************************/
/********************** Variables and User defined data types *****************/
/******************************************************************************/

/******************************************************************************/
/************************ Variable Declarations *******************************/
/******************************************************************************/

// Configure and instantiate UnbufferedSerial object to access the stdin.
// The default mbed baud rate is 9600, unless is it overriden in the
// mbed_app.json file, or by creating another UnbufferedSerial object using
// the same pins.
static UnbufferedSerial port(USBTX, USBRX);

/******************************************************************************/
/************************ Functions Definitions *******************************/
/******************************************************************************/

/**
  * @brief  getchar, but does not block if nothing waiting to be read
  * @param  None
  * @return character if available, NULL otherwise
  */
char getchar_noblock(void)
{
	char rx_char = '\0';

	// Return the character read from the serial port
	if (port.readable() > 0) {
		port.read(&rx_char, 1);
	}

	return rx_char;
}


#ifdef __cplusplus // Closing extern c
}
#endif