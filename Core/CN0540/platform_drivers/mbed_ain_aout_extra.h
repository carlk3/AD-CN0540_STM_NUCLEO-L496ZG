/***************************************************************************//*
* @file    mbed_ain_aout_extra.h
* @brief   Header containing extra types required for
*          analog in/output functionality
******************************************************************************
* Copyright (c) 2021 Analog Devices, Inc.
* All rights reserved.
*
* This software is proprietary to Analog Devices, Inc. and its licensors.
* By using this software you agree to the terms of the associated
* Analog Devices Software License Agreement.
******************************************************************************/

#ifndef MBED_AIN_AOUT_EXTRA_H_
#define MBED_AIN_AOUT_EXTRA_H_

// Platform support needs to be C-compatible to work with other drivers
#ifdef __cplusplus
extern "C"
{
#endif

/*****************************************************************************/
/***************************** Include Files *********************************/
/*****************************************************************************/

#include "stdio.h"
#include "stdint.h"

/*****************************************************************************/
/********************** Macros and Constants Definition **********************/
/*****************************************************************************/

/******************************************************************************/
/********************** Variables and User defined data types *****************/
/******************************************************************************/
/**
 * @struct mbed_analog_in_desc
 * @brief  Analog input pin specific descriptor for the mbed platform.
 */
struct mbed_analog_in_desc {
	/* Analog Input instance (mbed::AnalogIn) */
	void *analog_in_obj;		
};

/**
 * @struct mbed_analog_out_desc
 * @brief  Analog output pin specific descriptor for the mbed platform.
 */
struct mbed_analog_out_desc {
	/* Analog Output instance (mbed::AnalogOut) */
	void *analog_out_obj;		
};

/**
 * @brief mbed specific analog input platform ops structure
 */
extern const struct ain_platform_ops mbed_ain_ops;
	
/**
 * @brief mbed specific analog output platform ops structure
 */
extern const struct aout_platform_ops mbed_out_ops;

/******************************************************************************/
/*****************************Function Declarations****************************/
/******************************************************************************/

#ifdef __cplusplus // Closing extern c
}
#endif

#endif /* MBED_AIN_AOUT_EXTRA_H_ */

