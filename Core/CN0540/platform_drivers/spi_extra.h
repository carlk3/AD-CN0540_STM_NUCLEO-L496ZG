/***************************************************************************//**
 *   @file     spi_extra.h
 *   @brief:   Header containing extra types required for SPI interface
********************************************************************************
 * Copyright (c) 2021 Analog Devices, Inc.
 * All rights reserved.
 *
 * This software is proprietary to Analog Devices, Inc. and its licensors.
 * By using this software you agree to the terms of the associated
 * Analog Devices Software License Agreement.
*******************************************************************************/

#ifndef SPI_EXTRA_H
#define SPI_EXTRA_H

// Platform support needs to be C-compatible to work with other drivers
#ifdef __cplusplus
extern "C"
{
#endif

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <stdio.h>
#include <stdbool.h>

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
 *       spi init and read/write operations.
 **/

/**
 * @struct mbed_spi_init_param
 * @brief Structure holding the SPI init parameters for mbed platform.
 */
typedef struct mbed_spi_init_param {
	uint8_t spi_miso_pin;		// SPI MISO pin (PinName)
	uint8_t spi_mosi_pin;  		// SPI MOSI pin (PinName)
	uint8_t spi_clk_pin;  		// SPI CLK pin (PinName)
	bool use_sw_csb;			// Software/Hardware control of CSB pin
} mbed_spi_init_param;

/**
 * @struct mbed_spi_desc
 * @brief SPI specific descriptor for the mbed platform.
 */
typedef struct mbed_spi_desc {
	void *spi_port; 			// SPI port instance (mbed::SPI)
	void *csb_gpio;  			// SPI chip select gpio instance (DigitalOut)
	bool use_sw_csb; 			// Software/Hardware control of CSB pin
} mbed_spi_desc;

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/


#ifdef __cplusplus // Closing extern c
}
#endif

#endif /* SPI_EXTRA_H */
