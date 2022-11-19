/***************************************************************************//**
 *   @file     i2c_extra.h
 *   @brief:   Header containing extra types required for I2C interface
********************************************************************************
 * Copyright (c) 2021 Analog Devices, Inc.
 * All rights reserved.
 *
 * This software is proprietary to Analog Devices, Inc. and its licensors.
 * By using this software you agree to the terms of the associated
 * Analog Devices Software License Agreement.
*******************************************************************************/

#ifndef I2C_EXTRA_H
#define I2C_EXTRA_H

// Platform support needs to be C-compatible to work with other drivers
#ifdef __cplusplus
extern "C"
{
#endif

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

/*
 * Note: The structure members are not strongly typed, as this file is included
 *       in application specific '.c' files. The mbed code structure does not
 *       allow inclusion of mbed driver files (e.g. mbed.h) into '.c' files.
 *       All the members are hence typecasted to mbed specific type during
 *       i2c init and read/write operations.
 **/

/**
* @struct mbed_i2c_init_param
* @brief Structure holding the I2C init parameters for mbed platform.
*/
typedef struct mbed_i2c_init_param {
	uint8_t i2c_sda_pin;  	// I2C SDA pin (PinName)
	uint8_t i2c_scl_pin;  	// I2C SCL pin (PinName)
} mbed_i2c_init_param;

/**
* @struct mbed_i2c_desc
* @brief I2C specific descriptor for the mbed platform.
*/
typedef struct mbed_i2c_desc {
	void *i2c_port;  		// I2C port instance (mbed::I2C)
} mbed_i2c_desc;


/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/


#ifdef __cplusplus // Closing extern c
}
#endif

#endif /* I2C_EXTRA_H */
