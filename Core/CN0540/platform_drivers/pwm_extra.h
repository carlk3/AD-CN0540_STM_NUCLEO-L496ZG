/***************************************************************************//**
 *   @file     pwm_extra.h
 *   @brief:   Header containing extra types required for Mbed PWM interface
********************************************************************************
 * Copyright (c) 2021 Analog Devices, Inc.
 *
 * All rights reserved.
 *
 * This software is proprietary to Analog Devices, Inc. and its licensors.
 * By using this software you agree to the terms of the associated
 * Analog Devices Software License Agreement.
*******************************************************************************/

#ifndef PWM_EXTRA_H
#define PWM_EXTRA_H

// Platform support needs to be C-compatible to work with other drivers
#ifdef __cplusplus
extern "C"
{
#endif

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/

#include <stdio.h>

/******************************************************************************/
/********************** Macros and Constants Definitions **********************/
/******************************************************************************/

/******************************************************************************/
/********************** Variables and User defined data types *****************/
/******************************************************************************/

typedef struct {
	uint16_t pwm_pin;	// PWM pin number
} mbed_pwm_init_param;

struct mbed_pwm_desc {
	void *pwm_obj;	/* Mbed PWM instance/object */
};

/******************************************************************************/
/************************ Public Declarations *********************************/
/******************************************************************************/

#ifdef __cplusplus // Closing extern c
}
#endif

#endif /* PWM_EXTRA_H */
