/***************************************************************************//**
 *   @file     irq_extra.h
 *   @brief:   Header containing extra types required for IRQ drivers
********************************************************************************
 * Copyright (c) 2020-2021 Analog Devices, Inc.
 * All rights reserved.
 *
 * This software is proprietary to Analog Devices, Inc. and its licensors.
 * By using this software you agree to the terms of the associated
 * Analog Devices Software License Agreement.
*******************************************************************************/

#ifndef IRQ_EXTRA_H
#define IRQ_EXTRA_H


// Platform support needs to be C-compatible to work with other drivers
#ifdef __cplusplus
extern "C"
{
#endif

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/

#include <stdbool.h>

/******************************************************************************/
/********************** Macros and Constants Definitions **********************/
/******************************************************************************/

/******************************************************************************/
/*************************** Types Declarations *******************************/
/******************************************************************************/

/**
 * @enum irq_id
 * @brief Interrupts IDs supported by the mbed irq driver
 */
enum irq_id {
	/** External interrupt ID1 */
	EXTERNAL_INT_ID1,
	/** External interrupt ID2 */
	EXTERNAL_INT_ID2,
	/** External interrupt ID3 */
	EXTERNAL_INT_ID3,
	/** External interrupt ID4 */
	EXTERNAL_INT_ID4,
	/** External interrupt ID5 */
	EXTERNAL_INT_ID5,
	/** UART Rx interrupt ID1 */
	UART_RX_INT_ID1,
	/** Ticker interrupt ID */
	TICKER_INT_ID,
	/* Number of available interrupts */
	NB_INTERRUPTS
};

/*
 * External IRQ events
 * */
typedef enum {
	EXT_IRQ_NONE,
	EXT_IRQ_RISE,
	EXT_IRQ_FALL
} ext_irq_event;

/**
 * @struct mbed_irq_init_param
 * @brief Structure holding the extra parameters for Interrupt Request.
 */
typedef struct {
	uint32_t int_mode;				// Interrupt mode (falling/rising etc)
	uint32_t ext_int_pin; 			// External Interrupt pin
	uint32_t ticker_period_usec;	// Time period in usec for ticker event
	/* Application created peripheral object to attach/generate interrupt for (e.g. UART) */
	void *int_obj_type;
} mbed_irq_init_param;

/**
 * @struct mbed_irq_desc
 * @brief Structure holding the platform descriptor for Interrupt Request.
 */
typedef struct {
	uint32_t int_mode;				// Interrupt mode (falling/rising etc)
	uint32_t ext_int_pin;			// External Interrupt pin
	uint32_t ticker_period_usec; 	// Time period in usec for ticker event
	void *int_obj;					// Interrupt specific object (e.g. InterruptIn, Ticker)
} mbed_irq_desc;

#ifdef __cplusplus // Closing extern c
}
#endif

#endif // IRQ_EXTRA_H_
