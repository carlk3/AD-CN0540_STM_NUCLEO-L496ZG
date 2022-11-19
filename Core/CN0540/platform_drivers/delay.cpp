/***************************************************************************//**
 *   @file   delay.cpp
 *   @brief  Implementation of Mbed specific delay functionality
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

#include <mbed.h>

// Platform drivers needs to be C-compatible to work with other drivers
#ifdef __cplusplus
extern "C"
{
#endif //  _cplusplus 

#include "delay.h"

/******************************************************************************/
/********************** Macros and Constants Definitions **********************/
/******************************************************************************/

/******************************************************************************/
/************************ Functions Definitions *******************************/
/******************************************************************************/

/**
 * @brief Generate microseconds delay.
 * @param usecs - Delay in microseconds.
 * @return None.
 */
void udelay(uint32_t usecs)
{
	/* wait_ns is more time efficient function compared to wait_us
	 * Note: For higher values of delay (more than few msec), it is better to use
	 * mdelay() function as no error/limit checking is done in this function */
	wait_ns(usecs * 1000);
}

/**
 * @brief Generate miliseconds delay.
 * @param msecs - Delay in miliseconds.
 * @return None.
 */
void mdelay(uint32_t msecs)
{
	if (msecs) {
		thread_sleep_for(msecs);
	}
}

#ifdef __cplusplus  // Closing extern c
}
#endif //  _cplusplus
