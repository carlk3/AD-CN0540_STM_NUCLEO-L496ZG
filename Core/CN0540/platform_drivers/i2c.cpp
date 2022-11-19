/***************************************************************************//**
 *   @file   i2c.cpp
 *   @brief  Implementation of I2C Mbed platform driver interfaces
********************************************************************************
 * Copyright (c) 2019 - 2021 Analog Devices, Inc.
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
#include "i2c.h"
#include "i2c_extra.h"

/******************************************************************************/
/********************** Macros and Constants Definitions **********************/
/******************************************************************************/

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/

/******************************************************************************/
/************************ Functions Definitions *******************************/
/******************************************************************************/

/**
 * @brief Initialize the I2C communication peripheral.
 * @param desc - The I2C descriptor.
 * @param param - The structure that contains the I2C parameters.
 * @return SUCCESS in case of success, FAILURE otherwise.
 */
int32_t i2c_init_noos(struct i2c_desc **desc,
		      const struct i2c_init_param *param)
{
	mbed::I2C *i2c;		// pointer to new I2C instance
	mbed_i2c_desc *mbed_desc;	// pointer to mbed i2c desc
	i2c_desc *new_desc;

	if (desc) {
		// Create an i2c descriptor object for the device
		new_desc = (i2c_desc *)malloc(sizeof(i2c_desc));
		if (!new_desc) {
			goto err_new_desc;
		}

		// Address passed in parameter shifted left by 1 to form
		// 7-bit i2c slave address (7 MSBs) and the LSB acts as
		// r/w bit during i2c read/write operations
		new_desc->slave_address = ((param->slave_address) << 1);

		// Configure and instantiate I2C protocol
		i2c = new I2C(
			(PinName)(((mbed_i2c_init_param *)param->extra)->i2c_sda_pin),
			(PinName)(((mbed_i2c_init_param *)param->extra)->i2c_scl_pin));

		if (!i2c) {
			goto err_i2c;
		}

		// Create the i2c mbed descriptor object to store new i2c instance
		mbed_desc = (mbed_i2c_desc *)malloc(sizeof(mbed_i2c_desc));
		if (!mbed_desc) {
			goto err_mbed_desc;
		}

		mbed_desc->i2c_port = (I2C *)i2c;
		new_desc->extra = (mbed_i2c_desc *)mbed_desc;

		*desc = new_desc;

		return SUCCESS;
	}

err_mbed_desc:
	free(i2c);
err_i2c:
	free(new_desc);
err_new_desc:
	// Nothing to free

	return FAILURE;
}


/**
 * @brief Free the resources allocated by i2c_init_noos().
 * @param desc - The I2C descriptor.
 * @return SUCCESS in case of success, FAILURE otherwise.
 */
int32_t i2c_remove(struct i2c_desc *desc)
{
	if (desc) {
		// Free the I2C port object
		if ((I2C *)(((mbed_i2c_desc *)(desc->extra))->i2c_port)) {
			delete((I2C *)(((mbed_i2c_desc *)(desc->extra))->i2c_port));
		}

		// Free the I2C extra descriptor object
		if ((mbed_i2c_desc *)(desc->extra)) {
			free((mbed_i2c_desc *)(desc->extra));
		}

		// Free the I2C descriptor object
		free(desc);

		return SUCCESS;
	}

	return FAILURE;
}


/**
 * @brief Write data to a slave device.
 * @param desc - The I2C descriptor.
 * @param data - Buffer that stores the transmission data.
 * @param bytes_number - Number of bytes to write.
 * @param stop_bit - Stop condition control.
 *                   Example: 0 - A stop condition will not be generated;
 *                            1 - A stop condition will be generated.
 * @return SUCCESS in case of success, FAILURE otherwise.
 */

int32_t i2c_write_noos(struct i2c_desc *desc,
		       uint8_t *data,
		       uint8_t bytes_number,
		       uint8_t stop_bit)
{
	mbed::I2C *i2c;
	i2c = (I2C *)(((mbed_i2c_desc *)(desc->extra))->i2c_port);

	/**
	    The MBED I2C API is reversed for parameter 4
	    Instead of stop_bit - it has
	    @param repeated   - Repeated start, true - don't send stop at end default value is false.
	    Inverting here to keep the no-OS/platform_drivers API
	 */
	if (!(i2c->write(desc->slave_address, (char *)data, bytes_number, !stop_bit))) {
		return SUCCESS;
	} else {
		return FAILURE;
	}
}


/**
 * @brief Read data from a slave device.
 * @param desc - The I2C descriptor.
 * @param data - Buffer that will store the received data.
 * @param bytes_number - Number of bytes to read.
 * @param stop_bit - Stop condition control.
 *                   Example: 0 - A stop condition will not be generated;
 *                            1 - A stop condition will be generated.
 * @return SUCCESS in case of success, FAILURE otherwise.
 */
int32_t i2c_read_noos(struct i2c_desc *desc,
		      uint8_t *data,
		      uint8_t bytes_number,
		      uint8_t stop_bit)
{
	mbed::I2C *i2c;
	i2c = (I2C *)(((mbed_i2c_desc *)(desc->extra))->i2c_port);

	/**
	    The MBED I2C API is reversed for parameter 4
	    Instead of stop_bit - it has
	    @param repeated   - Repeated start, true - don't send stop at end default value is false.
	    Inverting here to keep the no-OS/platform_drivers API
	 */
	if (!(i2c->read(desc->slave_address, (char *)data, bytes_number, !stop_bit))) {
		return SUCCESS;
	} else {
		return FAILURE;
	}
}

#ifdef __cplusplus
}
#endif //  _cplusplus
