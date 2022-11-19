/***************************************************************************//**
 *   @file   ain.h
 *   @author PMallick (Pratyush.Mallick@analog.com)
********************************************************************************
 * Copyright (c) 2021 Analog Devices, Inc.
 * All rights reserved.
 *
 * This software is proprietary to Analog Devices, Inc. and its licensors.
 * By using this software you agree to the terms of the associated
 * Analog Devices Software License Agreement.
*******************************************************************************/

#ifndef AIN_H
#define AIN_H

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
 * @struct ain_init_param
 * @brief Structure holding parameters for analog input initialization
 */
struct ain_init_param {
	/* Analog input pin number */
	int32_t	number;
	/* Analog input reference voltage */
	float vref;
	/* Analog input platform specific functions */
	const struct ain_platform_ops *platform_ops;
};

/**
 * @struct ain_desc
 * @brief Structure holding analog input descriptor
 */
struct ain_desc {
	/* Analog input pin number */
	int32_t	number;
	/* Analog input reference voltage */
	float vref;
	/* Analog input platform specific functions */
	const struct ain_platform_ops *platform_ops;
	/* Analog extra parameters (device specific) */
	void *extra;
};

/**
 * @struct ain_platform_ops
 * @brief Structure holding analog input function pointers that 
 *        point to the platform specific function
 */
struct ain_platform_ops {
	/** Analog input initialization function pointer */
	int32_t(*init)(struct ain_desc **, const struct ain_init_param *);
	/** Analog input read function pointer */
	int32_t(*read)(struct ain_desc *, float *);
	/** Analog input remove function pointer */
	int32_t(*remove)(struct ain_desc *);
};

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/
/* Read analog input voltage */
int32_t ain_get_voltage(struct ain_desc *desc, float *value);

/* Initialize the analog input pin */
int32_t ain_init(struct ain_desc **desc,
		 const struct ain_init_param *param);

/* Free the resources allocated by ain_init() */
int32_t ain_remove(struct ain_desc *desc);

#endif
