/***************************************************************************//**
 *   @file   pwm.cpp
 *   @brief  Implementation of PWM Mbed platform driver interfaces
********************************************************************************
 * Copyright (c) 2021 Analog Devices, Inc.
 *
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

// Platform drivers needs to be C-compatible to work with other drivers
#ifdef __cplusplus
extern "C"
{
#endif //  _cplusplus

#include "error.h"
#include "pwm.h"
#include "gpio.h"
#include "pwm_extra.h"

/******************************************************************************/
/********************** Macros and Constants Definitions **********************/
/******************************************************************************/

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
 * @brief	Initialized the PWM interface
 * @param	desc[in, out] - Pointer where the configured instance is stored
 * @param	param[in] - Configuration information for the instance
 * @return	SUCCESS in case of success, FAILURE otherwise.
 */
int32_t pwm_init(struct pwm_desc **desc,
		 const struct pwm_init_param *param)
{
	struct pwm_desc *new_pwm_desc;
	mbed::PwmOut *pwm;
	struct mbed_pwm_desc *new_mbed_pwm_desc;

	if (!desc || !param) {
		return FAILURE;
	}

	/* Allocate memory for general PWM descriptor */
	new_pwm_desc = (pwm_desc *)malloc(sizeof(pwm_desc));
	if (!new_pwm_desc) {
		goto err_new_pwm_desc;
	}

	new_pwm_desc->id = param->id;	// PWM Id
	new_pwm_desc->period_ns = param->period_ns;	// PWM period

	/* Create and initialize Mbed PWM object */
	pwm = new PwmOut((PinName)((mbed_pwm_init_param *)(param->extra))->pwm_pin);
	if (!pwm) {
		goto err_pwm;
	}

	/* Allocate memory for Mbed specific PWM descriptor for future use */
	new_mbed_pwm_desc = (mbed_pwm_desc *)malloc(sizeof(mbed_pwm_desc));
	if (!new_mbed_pwm_desc) {
		goto err_new_mbed_pwm_desc;
	}

	new_mbed_pwm_desc->pwm_obj = (mbed::PwmOut *)pwm;
	new_pwm_desc->extra = (mbed_pwm_desc *)new_mbed_pwm_desc;

	/* Configure Mbed PWM parameters */
	pwm->period_us(param->period_ns / 1000); 			// Period in usec
	pwm->pulsewidth_us(param->duty_cycle_ns / 1000);	// Duty cycle in usec
	pwm_disable(new_pwm_desc);

	*desc = new_pwm_desc;

	return SUCCESS;

err_new_mbed_pwm_desc:
	free(pwm);
err_pwm:
	free(new_pwm_desc);
err_new_pwm_desc:
	// Nothing to free

	return FAILURE;
}


/**
 * @brief	Enable the PWM interface
 * @param	desc[in, out] - Pointer where the configured instance is stored
 * @return	SUCCESS in case of success, FAILURE otherwise.
 */
int32_t pwm_enable(struct pwm_desc *desc)
{
	mbed::PwmOut *pwm;

	if (desc) {
		pwm = (mbed::PwmOut *)(((mbed_pwm_desc *)desc->extra)->pwm_obj);
		if (!pwm) {
			return FAILURE;
		}

		pwm->resume();

		return SUCCESS;
	}

	return FAILURE;
}


/**
 * @brief	Disable the PWM interface
 * @param	desc[in, out] - Pointer where the configured instance is stored
 * @return	SUCCESS in case of success, FAILURE otherwise.
 */
int32_t pwm_disable(struct pwm_desc *desc)
{
	mbed::PwmOut *pwm;

	if (desc) {
		pwm = (mbed::PwmOut *)(((mbed_pwm_desc *)desc->extra)->pwm_obj);

		if (!pwm) {
			return FAILURE;
		}

		pwm->suspend();

		return SUCCESS;
	}

	return FAILURE;
}


/**
 * @brief	Remove the memory allocated for PWM device descriptors
 * @param	desc[in, out] - Pointer where the configured instance is stored
 * @return	SUCCESS in case of success, FAILURE otherwise.
 */
int32_t pwm_remove(struct pwm_desc *desc)
{
	if (!desc) {
		return FAILURE;
	}

	if (((mbed_pwm_desc *)desc->extra)->pwm_obj) {
		delete((mbed::PwmOut *)((mbed_pwm_desc *)desc->extra)->pwm_obj);
	}

	free(desc);

	return SUCCESS;
}


#ifdef __cplusplus  // Closing extern c
}
#endif //  _cplusplus
