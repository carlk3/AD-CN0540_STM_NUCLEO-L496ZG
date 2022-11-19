/***************************************************************************//*
 * @file    timer_extra.h
 * @brief   Header containing extra types for Timer interface
******************************************************************************
 * Copyright (c) 2021 Analog Devices, Inc.
 * All rights reserved.
 *
 * This software is proprietary to Analog Devices, Inc. and its licensors.
 * By using this software you agree to the terms of the associated
 * Analog Devices Software License Agreement.
******************************************************************************/

#ifndef _TIMER_EXTRA_H_
#define _TIMER_EXTRA_H_

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
* @struct mbed_timer_desc
* @brief Structure holding the mbed specific Timer parameters
*/
typedef struct mbed_timer_desc {
	void *timer;		// Object to the mbed Timer class
} mbed_timer_desc;

/******************************************************************************/
/*****************************Function Declarations****************************/
/******************************************************************************/

/* Function to capture the elapsed time in nanoseconds */
int32_t	get_elapsed_time_in_nsec(struct timer_desc *desc,
				 uint64_t *elapsed_time);

#ifdef __cplusplus // Closing extern c
}
#endif

#endif /*_TIMER_EXTRA_H_ */
