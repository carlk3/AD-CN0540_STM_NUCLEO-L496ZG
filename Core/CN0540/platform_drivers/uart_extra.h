/***************************************************************************//**
 *   @file     uart_extra.h
 *   @brief:   Header containing extra types required for UART interface
********************************************************************************
 * Copyright (c) 2021 Analog Devices, Inc.
 * All rights reserved.
 *
 * This software is proprietary to Analog Devices, Inc. and its licensors.
 * By using this software you agree to the terms of the associated
 * Analog Devices Software License Agreement.
*******************************************************************************/

#ifndef UART_EXTRA_H
#define UART_EXTRA_H

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
 *       uart init and read/write operations.
 **/

/**
 * @struct mbed_uart_init_param
 * @brief Structure holding the UART init parameters for mbed platform.
 */
typedef struct {
	bool virtual_com_enable;	/* Flag that enables the selection between
								 * Virtual COM Port Or standard UART link */
	uint8_t uart_tx_pin;		/* UART Transmit Pin (only for UART comm) */
	uint8_t uart_rx_pin;  		/* UART Receive Pin (only for UART comm) */
	uint16_t vendor_id;			/* USB VCOM Vendor ID (only for USB Virtual comm) */
	uint16_t product_id;		/* USB VCOM Product ID (only for USB Virtual comm) */
	char *serial_number;		/* USB VCOM serial number (only for USB Virtual comm) */
} mbed_uart_init_param;

/**
 * @struct mbed_uart_desc
 * @brief UART specific descriptor for the mbed platform.
 */
typedef struct {
	void *uart_port; 			/* UART port instance */
	bool virtual_com_enable; 	/* Flag that enables the selection between
								 * Virtual COM Port Or standard UART link */
} mbed_uart_desc;

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/


#ifdef __cplusplus // Closing extern c
}
#endif

#endif /* UART_EXTRA_H */
