/******************************************************************************
 *Copyright (c)2020 Analog Devices, Inc.  
 *
 * Licensed under the 2020-04-27-CN0540EC License(the "License");
 * you may not use this file except in compliance with the License.
 *
 ****************************************************************************/

#ifndef _INIT_PARAMS_H_
#define _INIT_PARAMS_H_

#ifdef __cplusplus
extern "C"
{
#endif

//#include <mbed.h>
#include <stdint.h>
#include "AD77681/ad77681.h"
#include "LTC26X6/ltc26x6.h"
//#include "cn0540_piezo.h"
#include "platform_drivers.h"
#include "platform_support.h"
#include "spi_extra.h"
#include "i2c_extra.h"
//#include "gpio.h"

/******************************************************************************/
/********************** Macros and Constants Definitions **********************/
/******************************************************************************/

//DAC address
#define LTC2606_I2C_ADDRESS 0x10

#if 0
//GPIOs pins
#define ARD_RED_LED_PIN		D0
#define ARD_BLUE_LED_PIN	D1
#define DRDY_PIN			D2
#define ADC_RST_PIN			D7
#define ARD_BUF_EN_PIN		D9
#endif

extern struct ad77681_init_param init_params;
extern struct ltc26x6_init_param init_params_dac;
extern const float programmable_FIR[56];
extern const uint8_t count_of_active_coeffs;

	
#ifdef __cplusplus 
}				  
#endif // __cplusplus 	
#endif // !_INIT_PARAMS_H_


