/***************************************************************************//**
 *   @file   spi.h
 *   @author MPhalke (Mahesh.Phalke@analog.com)
********************************************************************************
********************************************************************************
 * Copyright (c) 2021 Analog Devices, Inc.
 * All rights reserved.
 *
 * This software is proprietary to Analog Devices, Inc. and its licensors.
 * By using this software you agree to the terms of the associated
 * Analog Devices Software License Agreement.
*******************************************************************************/

#ifndef MBED_SPI_H_
#define MBED_SPI_H_

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/

#define  spi_init(desc, init_param)         spi_init_noos(desc, init_param)

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/

#include "no-OS/include/spi.h"

#endif // MBED_SPI_H_
