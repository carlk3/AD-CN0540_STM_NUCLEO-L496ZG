/***************************************************************************//**
 *   @file   aout.cpp
 *   @brief  Implementation of mbed specific analog output functionality
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
#include "aout.h"
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
 * @brief Write voltage to analog output pin
 * @param desc[in] - Analog output descriptor
 * @param value[in] - Analog output value in volts
 * @return SUCCESS in case of success, FAILURE otherwise.
 */
int32_t aout_set_voltage(struct aout_desc *desc, float value)
{
	/* Pointer to analog output object */
	mbed::AnalogOut *analog_output;
	float volt_mapped;

	/* Mbed AnalogOut APIs accept voltage in range from 0.0f
	 * (representing DAC min output range / 0%) to 1.0f (representing
	 * DAC max output range / 100%). Hence we need to map the voltage levels
	 * before passing it to mbed APIs */
	if (value >= desc->aout_min_v && value <= desc->aout_max_v) {
		volt_mapped = ((value - desc->aout_min_v) /
			       (desc->aout_max_v - desc->aout_min_v));
	} else {
		return -EINVAL;
	}

	if (desc && desc->extra) {
		analog_output = (AnalogOut *)((mbed_analog_out_desc *)
					      desc->extra)->analog_out_obj ;
		analog_output->write(volt_mapped);

		return SUCCESS;
	}
	return FAILURE;
}

/**
 * @brief Initialize the analog output pin
 * @param desc[in, out] - Analog output descriptor structure
 * @param param[in] - Structure that contains analog output
 *                    initialization parameters
 * @return SUCCESS in case of success, FAILURE otherwise
 */
int32_t aout_init(struct aout_desc **desc,
		  const struct aout_init_param *param)
{
	mbed::AnalogOut *analog_output;
	struct aout_desc *new_analog_out_desc;
	struct mbed_analog_out_desc *new_mbed_analog_out_desc;

	if (!desc || !param ) {
		return FAILURE;
	}

	/* Allocate memory for general analog output descriptor */
	new_analog_out_desc = (aout_desc *)malloc(sizeof(aout_desc));
	if (!new_analog_out_desc) {
		goto err_new_out_desc;
	}

	/* Configure pin number,DAC max and min range */
	new_analog_out_desc->number = param->number;
	new_analog_out_desc->aout_min_v = param->aout_min_v;
	new_analog_out_desc->aout_max_v = param->aout_max_v;

	/* Allocate memory for mbed specific analog output descriptor
	 * for future use */
	new_mbed_analog_out_desc = (mbed_analog_out_desc *)malloc
				   (sizeof(new_mbed_analog_out_desc));
	if (!new_mbed_analog_out_desc) {
		goto err_mbed_analog_out_desc;
	}

	/* Create and initialize mbed analog output object */
	analog_output = new AnalogOut((PinName)param->number);
	if (analog_output) {
		new_mbed_analog_out_desc->analog_out_obj = analog_output;
	} else {
		goto err_analog_out_instance;
	}

	new_analog_out_desc->extra = (mbed_analog_out_desc *)
				     new_mbed_analog_out_desc;

	*desc = new_analog_out_desc;
	return SUCCESS;

err_analog_out_instance:
	free(new_mbed_analog_out_desc);
err_mbed_analog_out_desc:
	free(new_analog_out_desc);
err_new_out_desc:
	// Nothing to free

	return FAILURE;
}

/**
 * @brief Deallocate resources allocated by aout_init()
 * @param desc[in, out] - Analog output descriptor
 * @return SUCCESS in case of success, FAILURE otherwise.
 */
int32_t aout_remove(struct aout_desc *desc)
{
	if (desc) {

		if (((mbed_analog_out_desc *)desc->extra)->analog_out_obj) {
			delete((mbed::AnalogIn *)((mbed_analog_out_desc *)desc->
						  extra)->analog_out_obj);
		}

		if ((mbed_analog_out_desc *)desc->extra) {
			free((mbed_analog_out_desc *)desc->extra);
		}

		if (((mbed_analog_out_desc *)desc->extra)->analog_out_obj) {
			delete((mbed::AnalogOut *)((mbed_analog_out_desc *)
						   desc->extra)->analog_out_obj);
		}

		if ((mbed_analog_out_desc *)desc->extra) {
			free((mbed_analog_out_desc *)desc->extra);
		}

		free(desc);

		return SUCCESS;
	}

	return FAILURE;
}

/**
 * @brief Mbed specific analog output platform ops structure
 */
const struct aout_platform_ops mbed_aout_ops = {
	.init = &aout_init,
	.write = &aout_set_voltage,
	.remove = &aout_remove
};


// Platform drivers needs to be C-compatible to work with other drivers
#ifdef __cplusplus
}
#endif //  _cplusplus
