/***************************************************************************//**
 *   @file   ain.cpp
 *   @brief  Implementation of mbed specific analog input functionality
********************************************************************************
 * Copyright (c) 2021 Analog Devices, Inc.
 * All rights reserved.
 *
 * This software is proprietary to Analog Devices, Inc. and its licensors.
 * By using this software you agree to the terms of the associated
 * Analog Devices Software License Agreement.
*******************************************************************************/

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <mbed.h>

// Platform drivers needs to be C-compatible to work with other drivers
#ifdef __cplusplus
extern "C"
{
#endif //  _cplusplus
#include "ain.h"
#include "mbed_ain_aout_extra.h"
#include "error.h"

/******************************************************************************/
/********************** Variables and User defined data types *****************/
/******************************************************************************/

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/

/******************************************************************************/
/************************ Functions Definitions *******************************/
/******************************************************************************/
/**
 * @brief Read voltage from analog input pin
 * @param desc[in] - Analog input descriptor
 * @param value[out] - Buffer to pass analog reading.
 * @return SUCCESS in case of success, FAILURE otherwise
 */
int32_t ain_get_voltage(struct ain_desc *desc, float *value)
{
	/* Pointer to analog input object */
	mbed::AnalogIn *analog_input;

	if (desc && desc->extra) {
		analog_input = (AnalogIn *)((mbed_analog_in_desc *)
					    desc->extra)->analog_in_obj;
		*value = analog_input->read_voltage();

		return SUCCESS;
	}
	return FAILURE;
}

/**
 * @brief Initialize the Analog Input Pin
 * @param desc[in, out] - Analog input descriptor structure
 * @param param[in] - Structure that contains analog input
 *                    initialization parameters
 * @return SUCCESS in case of success, FAILURE otherwise
 */
int32_t ain_init(struct ain_desc **desc,
		 const struct ain_init_param *param)
{
	mbed::AnalogIn *analog_input;
	struct ain_desc *new_analog_in_desc;
	struct mbed_analog_in_desc *new_mbed_analog_in_desc;

	if (!desc || !param ) {
		return FAILURE;
	}

	/* Allocate memory for general analog input descriptor */
	new_analog_in_desc = (ain_desc *)malloc(sizeof(ain_desc));
	if (!new_analog_in_desc) {
		goto err_new_in_desc;
	}

	/* Configure pin number and reference voltage*/
	new_analog_in_desc->number = param->number;
	new_analog_in_desc->vref = param->vref;

	/* Allocate memory for mbed specific analog input descriptor
	 * for future use */
	new_mbed_analog_in_desc = (mbed_analog_in_desc *)malloc(sizeof(
					  mbed_analog_in_desc));
	if (!new_mbed_analog_in_desc) {
		goto err_mbed_analog_in_desc;
	}

	/* Create and initialize mbed analog input object */
	analog_input = new AnalogIn((PinName)param->number, param->vref);
	if (analog_input) {
		new_mbed_analog_in_desc->analog_in_obj = analog_input;
	}

	else {
		goto err_analog_in_instance;
	}

	new_analog_in_desc->extra = (mbed_analog_in_desc *)new_mbed_analog_in_desc;

	*desc = new_analog_in_desc;
	return SUCCESS;

err_analog_in_instance:
	free(new_mbed_analog_in_desc);
err_mbed_analog_in_desc:
	free(new_analog_in_desc);
err_new_in_desc:
	// Nothing to free

	return FAILURE;
}

/**
 * @brief Deallocate resources allocated by ain_init()
 * @param desc[in, out] - Analog input descriptor
 * @return SUCCESS in case of success, FAILURE otherwise.
 */
int32_t ain_remove(struct ain_desc *desc)
{
	if (desc) {

		if (((mbed_analog_in_desc *)desc->extra)->analog_in_obj) {
			delete((mbed::AnalogIn *)((mbed_analog_in_desc *)desc->
						  extra)->analog_in_obj);
		}

		if ((mbed_analog_in_desc *)desc->extra) {
			free((mbed_analog_in_desc *)desc->extra);
		}

		if (((mbed_analog_in_desc *)desc->extra)->analog_in_obj) {
			delete((mbed::AnalogOut *)((mbed_analog_in_desc *)
						   desc->extra)->analog_in_obj);
		}

		if ((mbed_analog_in_desc *)desc->extra) {
			free((mbed_analog_in_desc *)desc->extra);
		}

		free(desc);

		return SUCCESS;
	}

	return FAILURE;
}

/**
 * @brief Mbed specific analog input platform ops structure
 */
const struct ain_platform_ops mbed_ain_ops = {
	.init = &ain_init,
	.read = &ain_get_voltage,
	.remove = &ain_remove
};

// Platform drivers needs to be C-compatible to work with other drivers
#ifdef __cplusplus
}
#endif //  _cplusplus
