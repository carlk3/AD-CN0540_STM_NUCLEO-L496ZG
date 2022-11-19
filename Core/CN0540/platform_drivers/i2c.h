/***************************************************************************//**
 *   @file   i2c.h
 *   @author MPhalke (Mahesh.Phalke@analog.com)
********************************************************************************
 * Copyright (c) 2021 Analog Devices, Inc.
 * All rights reserved.
 *
 * This software is proprietary to Analog Devices, Inc. and its licensors.
 * By using this software you agree to the terms of the associated
 * Analog Devices Software License Agreement.
*******************************************************************************/

#ifndef MBED_I2C_H_
#define MBED_I2C_H_

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/

#define  i2c_init(desc, init_param)                      i2c_init_noos(desc, init_param)
#define  i2c_write(desc, data, bytes_number, stop_bits)  i2c_write_noos(desc, data, bytes_number, stop_bits)
#define  i2c_read(desc, data, bytes_number, stop_bits)   i2c_read_noos(desc, data, bytes_number, stop_bits)

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/

#include "no-OS/include/i2c.h"

#endif // MBED_I2C_H_