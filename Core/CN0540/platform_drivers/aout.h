/***************************************************************************//**
 *   @file   aout.h
 *   @author PMallick (Pratyush.Mallick@analog.com)
********************************************************************************
 * Copyright (c) 2021 Analog Devices, Inc.
 * All rights reserved.
 *
 * This software is proprietary to Analog Devices, Inc. and its licensors.
 * By using this software you agree to the terms of the associated
 * Analog Devices Software License Agreement.
*******************************************************************************/

#ifndef AOUT_H
#define AOUT_H

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/

#include <stdint.h>

/******************************************************************************/
/********************** Macros and Constants Definitions **********************/
/******************************************************************************/

/******************************************************************************/
/*************************** Types Declarations *******************************/
/******************************************************************************/
/**
 * @struct aout_init_param
 * @brief Structure holding the parameters for analog output initialization
 */
struct aout_init_param {
	/* Analog output pin number */
	int32_t	number;
	/* Min output range of DAC in volts */
	float aout_min_v;
	/* Max output range of DAC in volts */
	float aout_max_v;
	/* Analog output reference voltage */
	float vref;
	/* Analog output platform specific functions */
	const struct aout_platform_ops *platform_ops;
};

/**
 * @struct aout_desc
 * @brief Structure holding analog output descriptor
 */
struct aout_desc {
	/* Analog output pin number */
	int32_t	number;
	/* Min output value of DAC in volts */
	float aout_min_v;
	/* Max output value of DAC in volts */
	float aout_max_v;
	/* Analog output reference voltage */
	float vref;
	/* Analog output platform specific functions */
	const struct aout_platform_ops *platform_ops;
	/* Analog extra parameters (device specific) */
	void *extra;
};

/**
 * @struct aout_platform_ops
 * @brief Structure holding analog output function pointers that 
 * point to the platform specific function
 */
struct aout_platform_ops {
	/** Analog output initialization function pointer */
	int32_t(*init)(struct aout_desc **, const struct aout_init_param *);
	/** Analog output write function pointer */
	int32_t(*write)(struct aout_desc *, float);
	/** Analog output remove function pointer */
	int32_t(*remove)(struct aout_desc *);
};

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/
/* Write analog output voltage */
int32_t aout_set_voltage(struct aout_desc *desc, float value);

/* Initialize the analog output pin */
int32_t aout_init(struct aout_desc **desc,
		  const struct aout_init_param *param);

/* Free the resources allocated by analog_out_init() */
int32_t aout_remove(struct aout_desc *desc);

#endif
