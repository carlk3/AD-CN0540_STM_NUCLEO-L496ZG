/***************************************************************************//**
 * @file  irq.cpp
 * @brief Implementation of Interrupt Mbed platform driver interfaces
********************************************************************************
 * Copyright (c) 2020-2021 Analog Devices, Inc.
 * All rights reserved.
 *
 * This software is proprietary to Analog Devices, Inc. and its licensors.
 * By using this software you agree to the terms of the associated
 * Analog Devices Software License Agreement.
*******************************************************************************/

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/

#include <stdio.h>
#include <mbed.h>

// Platform support needs to be C-compatible to work with other drivers
#ifdef __cplusplus
extern "C"
{
#endif

#include "error.h"
#include "irq.h"
#include "irq_extra.h"
#include "uart_extra.h"

/******************************************************************************/
/*************************** Types Declarations *******************************/
/******************************************************************************/

using namespace std::chrono;

/**
* @struct mbed_irq_callback_desc
* @brief Structure holding the callback functions for mbed irqs
* @note  The callback functions are mapped with 'irq_id' structure
*/
typedef struct {
	struct callback_desc callback_ext_int_id1;
	struct callback_desc callback_ext_int_id2;
	struct callback_desc callback_ext_int_id3;
	struct callback_desc callback_ext_int_id4;
	struct callback_desc callback_ext_int_id5;
	struct callback_desc callback_uart_rx_id1;
	struct callback_desc callback_ticker_id;
} mbed_irq_callback_desc;

/* Mbed callback function pointer typedef */
typedef void(*mbed_callback_func)(void);

/* Mbed irq callback structure variable */
static mbed_irq_callback_desc mbed_irq_callbacks;

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/

/******************************************************************************/
/************************ Functions Definitions *******************************/
/******************************************************************************/

/**
 * @brief	Mbed callback function for external interrupt ID1 event
 * @return	none
 */
static void mbed_ext_int_id1_callback(void)
{
	if (mbed_irq_callbacks.callback_ext_int_id1.callback) {
		mbed_irq_callbacks.callback_ext_int_id1.callback(
			mbed_irq_callbacks.callback_ext_int_id1.ctx, EXTERNAL_INT_ID1, NULL);
	}
}


/**
 * @brief	Mbed callback function for external interrupt ID2 event
 * @return	none
 */
static void mbed_ext_int_id2_callback(void)
{
	if (mbed_irq_callbacks.callback_ext_int_id2.callback) {
		mbed_irq_callbacks.callback_ext_int_id2.callback(
			mbed_irq_callbacks.callback_ext_int_id2.ctx, EXTERNAL_INT_ID2, NULL);
	}
}


/**
 * @brief	Mbed callback function for external interrupt ID3 event
 * @return	none
 */
static void mbed_ext_int_id3_callback(void)
{
	if (mbed_irq_callbacks.callback_ext_int_id3.callback) {
		mbed_irq_callbacks.callback_ext_int_id3.callback(
			mbed_irq_callbacks.callback_ext_int_id3.ctx, EXTERNAL_INT_ID3, NULL);
	}
}


/**
 * @brief	Mbed callback function for external interrupt ID4 event
 * @return	none
 */
static void mbed_ext_int_id4_callback(void)
{
	if (mbed_irq_callbacks.callback_ext_int_id4.callback) {
		mbed_irq_callbacks.callback_ext_int_id4.callback(
			mbed_irq_callbacks.callback_ext_int_id4.ctx, EXTERNAL_INT_ID4, NULL);
	}
}


/**
 * @brief	Mbed callback function for external interrupt ID5 event
 * @return	none
 */
static void mbed_ext_int_id5_callback(void)
{
	if (mbed_irq_callbacks.callback_ext_int_id5.callback) {
		mbed_irq_callbacks.callback_ext_int_id5.callback(
			mbed_irq_callbacks.callback_ext_int_id5.ctx, EXTERNAL_INT_ID5, NULL);
	}
}


/**
 * @brief	Mbed callback function for UART Rx ID1 event
 * @return	none
 */
static void mbed_uart_rx_id1_callback(void)
{
	if (mbed_irq_callbacks.callback_uart_rx_id1.callback) {
		mbed_irq_callbacks.callback_uart_rx_id1.callback(
			mbed_irq_callbacks.callback_uart_rx_id1.ctx, UART_RX_INT_ID1, NULL);
	}
}


/**
 * @brief	Mbed callback function for ticker ID event
 * @return	none
 */
static void mbed_ticker_id_callback(void)
{
	if (mbed_irq_callbacks.callback_ticker_id.callback) {
		mbed_irq_callbacks.callback_ticker_id.callback(
			mbed_irq_callbacks.callback_ticker_id.ctx, TICKER_INT_ID, NULL);
	}
}


/**
 * @brief	Initialized the controller for the peripheral interrupts
 * @param	desc[in, out] - Pointer where the configured instance is stored
 * @param	param[in] - Configuration information for the instance
 * @return	SUCCESS in case of success, FAILURE otherwise.
 */
int32_t irq_ctrl_init(struct irq_ctrl_desc **desc,
		      const struct irq_init_param *param)
{
	mbed::Ticker *ticker;
	InterruptIn *ext_interrupt;
	mbed_irq_desc *new_mbed_desc;

	if (!desc || !param) {
		return FAILURE;
	}

	irq_ctrl_desc *new_desc = (irq_ctrl_desc *)malloc(sizeof(irq_ctrl_desc));
	if (!new_desc) {
		goto err_new_desc;
	}

	new_mbed_desc = (mbed_irq_desc *)malloc(sizeof(mbed_irq_desc));
	if (!new_mbed_desc) {
		goto err_new_mbed_desc;
	}

	new_desc->irq_ctrl_id = param->irq_ctrl_id;
	new_mbed_desc->int_mode = ((mbed_irq_init_param *)param->extra)->int_mode;
	new_mbed_desc->ext_int_pin = ((mbed_irq_init_param *)
				      param->extra)->ext_int_pin;

	switch (param->irq_ctrl_id) {
	case EXTERNAL_INT_ID1:
	case EXTERNAL_INT_ID2:
	case EXTERNAL_INT_ID3:
	case EXTERNAL_INT_ID4:
	case EXTERNAL_INT_ID5:
		/* Create a new external interrupt object */
		ext_interrupt = new InterruptIn((PinName)(new_mbed_desc->ext_int_pin));
		if (!ext_interrupt) {
			goto err_interrupt;
		}

		new_mbed_desc->int_obj = (mbed::InterruptIn *)ext_interrupt;
		break;

	case TICKER_INT_ID:
		/* Ticker is a special mbed class used for generating recurring interrupt.
		* The object of this class is created during interrupt initialization as:
		* 1) To avoid having seperate module for it.
		* 2) To avoid having multiple instances of Ticker class
		* */
		ticker = new Ticker();
		if (!ticker) {
			goto err_interrupt;
		}

		new_mbed_desc->int_obj = (mbed::Ticker *)ticker;
		new_mbed_desc->ticker_period_usec = ((mbed_irq_init_param *)
						     param->extra)->ticker_period_usec;
		break;

	case UART_RX_INT_ID1:
		/* UART object must be created from uart.cpp module by an application */
		new_mbed_desc->int_obj = ((mbed_irq_init_param *)
					  param->extra)->int_obj_type;
		break;

	default:
		goto err_interrupt;
	}

	new_desc->extra = (irq_ctrl_desc *)new_mbed_desc;

	*desc = new_desc;

	return SUCCESS;

err_interrupt:
	free(new_mbed_desc);
err_new_mbed_desc:
	free(new_desc);
err_new_desc:
	// Nothing to free

	return FAILURE;
}


/**
 * @brief	Free the resources allocated by irq_ctrl_init()
 * @param	desc[in, out] - Interrupt controller descriptor.
 * @return	SUCCESS in case of success, FAILURE otherwise.
 */
int32_t irq_ctrl_remove(struct irq_ctrl_desc *desc)
{
	uint8_t irq_id;

	if (!desc) {
		return FAILURE;
	}

	if (((mbed_irq_desc *)desc->extra)->int_obj) {
		free(((mbed_irq_desc *)desc->extra)->int_obj);
	}

	/* Unregister all callbacks */
	for (irq_id = 0; irq_id < (uint8_t)NB_INTERRUPTS; irq_id++) {
		if (irq_unregister(desc, irq_id) != SUCCESS)
			return FAILURE;
	}

	if ((irq_ctrl_desc *)desc->extra) {
		free((irq_ctrl_desc *)desc->extra);
	}

	free(desc);

	return SUCCESS;
}


/**
 * @brief	Registers a IRQ callback function to irq controller.
 * @param	desc[in] - The IRQ controller descriptor.
 * @param	irq_id[in] - Interrupt identifier.
 * @param	callback_desc - Descriptor of the callback. If it is NULL, the
 *			callback will be unregistered
 * @return	SUCCESS in case of success, FAILURE otherwise.
 */
int32_t irq_register_callback(struct irq_ctrl_desc *desc,
			      uint32_t irq_id,
			      struct callback_desc *callback_desc)
{
	InterruptIn *ext_interrupt;
	mbed_callback_func mbed_callback;

	if (!desc || !callback_desc) {
		return FAILURE;
	}

	switch (irq_id) {
	case EXTERNAL_INT_ID1:
	case EXTERNAL_INT_ID2:
	case EXTERNAL_INT_ID3:
	case EXTERNAL_INT_ID4:
	case EXTERNAL_INT_ID5:
		switch (irq_id) {
		case EXTERNAL_INT_ID1:
			mbed_irq_callbacks.callback_ext_int_id1.callback = callback_desc->callback;
			mbed_irq_callbacks.callback_ext_int_id1.ctx = callback_desc->ctx;
			mbed_callback = mbed_ext_int_id1_callback;
			break;

		case EXTERNAL_INT_ID2:
			mbed_irq_callbacks.callback_ext_int_id2.callback = callback_desc->callback;
			mbed_irq_callbacks.callback_ext_int_id2.ctx = callback_desc->ctx;
			mbed_callback = mbed_ext_int_id2_callback;
			break;

		case EXTERNAL_INT_ID3:
			mbed_irq_callbacks.callback_ext_int_id3.callback = callback_desc->callback;
			mbed_irq_callbacks.callback_ext_int_id3.ctx = callback_desc->ctx;
			mbed_callback = mbed_ext_int_id3_callback;
			break;

		case EXTERNAL_INT_ID4:
			mbed_irq_callbacks.callback_ext_int_id4.callback = callback_desc->callback;
			mbed_irq_callbacks.callback_ext_int_id4.ctx = callback_desc->ctx;
			mbed_callback = mbed_ext_int_id4_callback;
			break;

		case EXTERNAL_INT_ID5:
			mbed_irq_callbacks.callback_ext_int_id5.callback = callback_desc->callback;
			mbed_irq_callbacks.callback_ext_int_id5.ctx = callback_desc->ctx;
			mbed_callback = mbed_ext_int_id5_callback;
			break;

		default:
			return FAILURE;
		}

		ext_interrupt = (InterruptIn *)(((mbed_irq_desc *)(desc->extra))->int_obj);

		/* Select interrupt mode */
		if (((mbed_irq_desc *)(desc->extra))->int_mode == EXT_IRQ_FALL) {
			ext_interrupt->fall(mbed_callback);
		} else if (((mbed_irq_desc *)(desc->extra))->int_mode == EXT_IRQ_RISE) {
			ext_interrupt->rise(mbed_callback);
		} else {
			return FAILURE;
		}

		break;

	case UART_RX_INT_ID1:
		mbed_irq_callbacks.callback_uart_rx_id1.callback = callback_desc->callback;
		mbed_irq_callbacks.callback_uart_rx_id1.ctx = callback_desc->ctx;
		break;

	case TICKER_INT_ID:
		mbed_irq_callbacks.callback_ticker_id.callback = callback_desc->callback;
		mbed_irq_callbacks.callback_ticker_id.ctx = callback_desc->ctx;
		break;

	default:
		return FAILURE;
	}

	return SUCCESS;
}


/**
 * @brief	Unregister a IRQ callback function.
 * @param	desc[in] - The IRQ controller descriptor.
 * @param	irq_id[in] - Interrupt identifier.
 * @return	SUCCESS in case of success, FAILURE otherwise.
 */
int32_t irq_unregister(struct irq_ctrl_desc *desc, uint32_t irq_id)
{
	if (!desc) {
		return FAILURE;
	}

	switch (irq_id) {
	case  EXTERNAL_INT_ID1:
		mbed_irq_callbacks.callback_ext_int_id1.callback = NULL;
		break;

	case EXTERNAL_INT_ID2:
		mbed_irq_callbacks.callback_ext_int_id2.callback = NULL;
		break;

	case EXTERNAL_INT_ID3:
		mbed_irq_callbacks.callback_ext_int_id3.callback = NULL;
		break;

	case EXTERNAL_INT_ID4:
		mbed_irq_callbacks.callback_ext_int_id4.callback = NULL;
		break;

	case EXTERNAL_INT_ID5:
		mbed_irq_callbacks.callback_ext_int_id5.callback = NULL;
		break;

	case UART_RX_INT_ID1:
		mbed_irq_callbacks.callback_uart_rx_id1.callback = NULL;
		break;

	case TICKER_INT_ID:
		mbed_irq_callbacks.callback_ticker_id.callback = NULL;
		break;

	default:
		return FAILURE;
	}

	return SUCCESS;
}

/**
 * @brief	Enable specific interrupt
 * @param	desc[in] - The IRQ controller descriptor.
 * @param	irq_id[in] - Interrupt identifier.
 * @return	SUCCESS in case of success, FAILURE otherwise.
 */
int32_t irq_enable(struct irq_ctrl_desc *desc, uint32_t irq_id)
{
	InterruptIn *ext_interrupt;
	mbed::UnbufferedSerial *uart_rx_port;
	mbed::Ticker *ticker;

	if (!desc || !desc->extra) {
		return FAILURE;
	}

	switch (irq_id) {
	case EXTERNAL_INT_ID1:
	case EXTERNAL_INT_ID2:
	case EXTERNAL_INT_ID3:
	case EXTERNAL_INT_ID4:
	case EXTERNAL_INT_ID5:
		ext_interrupt = (InterruptIn *)(((mbed_irq_desc *)(desc->extra))->int_obj);
		ext_interrupt->enable_irq();
		break;

	case UART_RX_INT_ID1:
		uart_rx_port = (mbed::UnbufferedSerial *)(((mbed_irq_desc *)(
					desc->extra))->int_obj);
		uart_rx_port->attach(mbed_uart_rx_id1_callback, UnbufferedSerial::RxIrq);
		break;

	case TICKER_INT_ID:
		ticker = (mbed::Ticker *)(((mbed_irq_desc *)(desc->extra))->int_obj);
		ticker->attach(mbed_ticker_id_callback,
			       microseconds(((mbed_irq_desc *)(desc->extra))->ticker_period_usec));
		break;

	default:
		return FAILURE;
	}

	return SUCCESS;
}

/**
 * @brief	Disable specific interrupt
 * @param	desc[in] - The IRQ controller descriptor.
 * @param	irq_id[in] - Interrupt identifier.
 * @return	SUCCESS in case of success, FAILURE otherwise.
 */
int32_t irq_disable(struct irq_ctrl_desc *desc, uint32_t irq_id)
{
	InterruptIn *ext_interrupt;
	mbed::UnbufferedSerial *uart_rx_port;
	mbed::Ticker *ticker;

	if (!desc || !desc->extra) {
		return FAILURE;
	}

	switch (irq_id) {
	case EXTERNAL_INT_ID1:
	case EXTERNAL_INT_ID2:
	case EXTERNAL_INT_ID3:
	case EXTERNAL_INT_ID4:
	case EXTERNAL_INT_ID5:
		ext_interrupt = (InterruptIn *)(((mbed_irq_desc *)(desc->extra))->int_obj);
		ext_interrupt->disable_irq();
		break;

	case UART_RX_INT_ID1:
		uart_rx_port = (mbed::UnbufferedSerial *)(((mbed_irq_desc *)(
					desc->extra))->int_obj);
		uart_rx_port->attach(NULL, UnbufferedSerial::RxIrq);
		break;

	case TICKER_INT_ID:
		ticker = (mbed::Ticker *)(((mbed_irq_desc *)(desc->extra))->int_obj);
		ticker->detach();
		break;

	default:
		return FAILURE;
	}

	return SUCCESS;
}

#ifdef __cplusplus // Closing extern c
}
#endif
