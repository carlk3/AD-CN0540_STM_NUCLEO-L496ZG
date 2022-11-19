/***************************************************************************//**
 *   @file   gpio.cpp
 *   @brief  Implementation of GPIO Mbed platform driver interfaces
********************************************************************************
 * Copyright (c) 2019 - 2021 Analog Devices, Inc.
 * All rights reserved.
 *
 * This software is proprietary to Analog Devices, Inc. and its licensors.
 * By using this software you agree to the terms of the associated
 * Analog Devices Software License Agreement.
*******************************************************************************/

/******************************************************************************/
/************************ Includes Files *******************************/
/******************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <mbed.h>

// Platform drivers needs to be C-compatible to work with other drivers
#ifdef __cplusplus
extern "C"
{
#endif //  _cplusplus

#include "error.h"
#include "gpio.h"
#include "gpio_extra.h"

/******************************************************************************/
/************************ Functions Definitions *******************************/
/******************************************************************************/

/**
 * @brief Obtain the GPIO decriptor.
 * @param desc - The GPIO descriptor.
 * @param gpio_number - The number of the GPIO.
 * @return SUCCESS in case of success, FAILURE otherwise.
 */
int32_t gpio_get(struct gpio_desc **desc, const struct gpio_init_param *param)
{
	gpio_desc *new_gpio;

	if (desc && param) {
		// Create the gpio description object for the device
		new_gpio = (gpio_desc *)calloc(1, sizeof(gpio_desc)) ;
		if (!new_gpio) {
			goto err_new_gpio;
		}

		new_gpio->number = param->number;

		// Create the gpio extra descriptor object to store extra mbed gpio info
		mbed_gpio_desc *gpio_desc_extra = (mbed_gpio_desc *)calloc(1,
						  sizeof(mbed_gpio_desc));
		if (!gpio_desc_extra) {
			goto err_gpio_desc_extra;
		}

		gpio_desc_extra->direction = GPIO_IN;
		gpio_desc_extra->gpio_pin = NULL;

		if (param->extra) {
			gpio_desc_extra->pin_mode = ((mbed_gpio_init_param *)param->extra)->pin_mode;
		} else {
			gpio_desc_extra->pin_mode = NULL;
		}

		new_gpio->extra = gpio_desc_extra;
		*desc = new_gpio;

		return SUCCESS;
	}

err_gpio_desc_extra:
	free(new_gpio);
err_new_gpio:
	// Nothing to free

	return FAILURE;
}


/**
 * @brief Get the value of an optional GPIO.
 * @param desc - The GPIO descriptor.
 * @param param - GPIO Initialization parameters.
 * @return SUCCESS in case of success, FAILURE otherwise.
 */
int32_t gpio_get_optional(struct gpio_desc **desc,
			  const struct gpio_init_param *param)
{
	if (param) {
		return gpio_get(desc, param);
	} else {
		*desc = NULL;
		return SUCCESS;
	}
}


/**
 * @brief Free the resources allocated by gpio_get().
 * @param desc - The GPIO descriptor.
 * @return SUCCESS in case of success, FAILURE otherwise.
 */
int32_t gpio_remove(struct gpio_desc *desc)
{
	if (desc) {
		// Free the gpio object
		if(((mbed_gpio_desc *)(desc->extra))->gpio_pin) {
			free(((mbed_gpio_desc *)(desc->extra))->gpio_pin);
		}

		// Free the gpio extra descriptor object
		if((mbed_gpio_desc *)(desc->extra)) {
			free((mbed_gpio_desc *)(desc->extra));
		}

		// Free the gpio descriptor object
		free(desc);

		return SUCCESS;
	}

	return FAILURE;
}


/**
 * @brief Enable the input direction of the specified GPIO.
 * @param desc - The GPIO descriptor.
 * @return SUCCESS in case of success, FAILURE otherwise.
 * @note does not support reconfiguration of already set pin direction
 */
int32_t gpio_direction_input(struct gpio_desc *desc)
{
	DigitalIn *gpio_input;   	// pointer to gpio input object
	mbed_gpio_desc *gpio_desc_extra;   // pointer to gpio desc extra parameters

	if (desc && desc->extra) {
		gpio_desc_extra = (mbed_gpio_desc *)(desc->extra);
		if (gpio_desc_extra->gpio_pin == NULL) {
			// Configure and instantiate GPIO pin as input
			gpio_input = new DigitalIn((PinName)desc->number);
			if (!gpio_input) {
				goto err_gpio_input;
			}

			gpio_desc_extra->gpio_pin = (mbed_gpio_desc *)gpio_input;
			gpio_desc_extra->direction = GPIO_IN;

			// Set the gpio pin mode
			gpio_input->mode((PinMode)((mbed_gpio_desc *)desc->extra)->pin_mode);

			return SUCCESS;
		}
	}

err_gpio_input:
	// Nothing to free

	return FAILURE;
}


/**
 * @brief Enable the output direction of the specified GPIO.
 * @param desc - The GPIO descriptor.
 * @param value - The value.
 *                Example: GPIO_HIGH
 *                         GPIO_LOW
 * @return SUCCESS in case of success, FAILURE otherwise.
 * @note does not support reconfiguration of already set pin direction
 */
int32_t gpio_direction_output(struct gpio_desc *desc, uint8_t value)
{
	DigitalOut *gpio_output;    	// pointer to gpio output object
	mbed_gpio_desc *gpio_desc_extra;   // pointer to gpio desc extra parameters

	if(desc && desc->extra) {
		gpio_desc_extra = (mbed_gpio_desc *)(desc->extra);
		if (gpio_desc_extra->gpio_pin == NULL) {

			// Configure and instantiate GPIO pin as output
			gpio_output = new DigitalOut((PinName)desc->number);

			if (!gpio_output) {
				goto err_gpio_output;
			}

			gpio_desc_extra->gpio_pin = (mbed_gpio_desc *)gpio_output;
			gpio_desc_extra->direction = GPIO_OUT;

			// Set the GPIO value
			if(gpio_set_value(desc, value) == SUCCESS) {
				return SUCCESS;
			}
		}
	}

err_gpio_output:
	// Nothing to free

	return FAILURE;
}


/**
 * @brief Get the direction of the specified GPIO.
 * @param desc - The GPIO descriptor.
 * @param direction - The direction.
 *                    Example: GPIO_OUT
 *                             GPIO_IN
 * @return SUCCESS in case of success, FAILURE otherwise.
 */
int32_t gpio_get_direction(struct gpio_desc *desc, uint8_t *direction)
{
	mbed_gpio_desc *gpio_desc_extra;     // pointer to gpio desc extra parameters

	if(desc && desc->extra) {
		gpio_desc_extra = (mbed_gpio_desc *)(desc->extra);

		if (gpio_desc_extra->gpio_pin) {
			*direction = gpio_desc_extra->direction;
		}

		return SUCCESS;
	}

	return FAILURE;
}


/**
 * @brief Set the value of the specified GPIO.
 * @param desc - The GPIO descriptor.
 * @param value - The value.
 *                Example: GPIO_HIGH
 *                         GPIO_LOW
 * @return SUCCESS in case of success, FAILURE otherwise.
 */
int32_t gpio_set_value(struct gpio_desc *desc, uint8_t value)
{
	DigitalOut *gpio_output; 		// pointer to gpio output object
	mbed_gpio_desc *gpio_desc_extra;    // pointer to gpio desc extra parameters

	if(desc && desc->extra) {
		gpio_desc_extra = (mbed_gpio_desc *)(desc->extra);

		if (gpio_desc_extra->gpio_pin) {
			gpio_output = (DigitalOut *)((mbed_gpio_desc *)desc->extra)->gpio_pin;
			gpio_output->write(value);
		}

		return SUCCESS;
	}

	return FAILURE;
}


/**
 * @brief Get the value of the specified GPIO.
 * @param desc - The GPIO descriptor.
 * @param value - The value.
 *                Example: GPIO_HIGH
 *                         GPIO_LOW
 * @return SUCCESS in case of success, FAILURE otherwise.
 */
int32_t gpio_get_value(struct gpio_desc *desc, uint8_t *value)
{
	DigitalIn *gpio_input;    	// pointer to gpio input object
	uint8_t returnVal = FAILURE;

	if (desc && desc->extra) {
		gpio_input = (DigitalIn *)((mbed_gpio_desc *)desc->extra)->gpio_pin;
		*value = (uint8_t)gpio_input->read();
		returnVal = gpio_input->is_connected() ? SUCCESS : FAILURE;

		return returnVal;
	}

	return FAILURE;
}

#ifdef __cplusplus
}
#endif //  _cplusplus
