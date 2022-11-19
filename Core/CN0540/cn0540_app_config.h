/******************************************************************************
 *Copyright (c)2020 Analog Devices, Inc.  
 *
 * Licensed under the 2020-04-27-CN0540EC License(the "License");
 * you may not use this file except in compliance with the License.
 *
 ****************************************************************************/

#ifndef _APP_CONFIG_H_
#define _APP_CONFIG_H_

#include <stdint.h>
#include "platform_drivers.h"
#ifdef __cplusplus
extern "C"
{
#endif
#ifdef __cplusplus
#include "ad77681.h"
}
#endif

#define  ARDUINO

/**
  The ADI SDP_K1 can be used with both arduino headers
  or the 120-pin SDP connector found on ADI evaluation
  boards. The default is the SDP connector

  Uncomment the ARDUINO #define above to enable the ARDUINO connector

*/
//#warning  check this
/*
#ifdef ARDUINO
	#define I2C_SCL     D15
	#define I2C_SDA     D14

	#define SPI_CS		D10
	#define SPI_MISO	D12
	#define SPI_MOSI	D11
	#define SPI_SCK		D13

#else
	
	#define I2C_SCL     SDP_I2C_SCL
	#define I2C_SDA     SDP_I2C_SDA

	#define SPI_CS		SDP_SPI_CS_A
	#define SPI_MISO	SDP_SPI_MISO
	#define SPI_MOSI	SDP_SPI_MOSI
	#define SPI_SCK		SDP_SPI_SCK

#endif
*/
#endif //_APP_CONFIG_H_




