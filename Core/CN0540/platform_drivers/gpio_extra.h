/***************************************************************************//**
 *   @file     gpio_extra.h
 *   @brief:   Header containing extra types required for GPIO interface
********************************************************************************
 * Copyright (c) 2021 Analog Devices, Inc.
 * All rights reserved.
 *
 * This software is proprietary to Analog Devices, Inc. and its licensors.
 * By using this software you agree to the terms of the associated
 * Analog Devices Software License Agreement.
*******************************************************************************/

#ifndef GPIO_EXTRA_H
#define GPIO_EXTRA_H

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
 *       gpio init and read/write operations.
 **/

/**
* @struct mbed_gpio_init_param
* @brief Structure holding the GPIO init parameters for mbed platform.
*/
typedef struct mbed_gpio_init_param {
	uint8_t pin_mode; 		// GPIO pin mode (PinMode)
} mbed_gpio_init_param;

/**
* @struct mbed_gpio_desc
* @brief GPIO specific descriptor for the mbed platform.
*/
typedef struct mbed_gpio_desc {
	uint8_t direction;
	void *gpio_pin;     	// GPIO pin instance (DigitalIn/DigitalOut)
	uint8_t pin_mode;
} mbed_gpio_desc;


/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/


#ifdef __cplusplus // Closing extern c
}
#endif

#endif /* GPIO_EXTRA_H */
