/***************************************************************************//**
 *   @file   adc_data_capture.h
 *   @brief  Header for ADC data capture interfaces
********************************************************************************
 * Copyright (c) 2021 Analog Devices, Inc.
 * All rights reserved.
 *
 * This software is proprietary to Analog Devices, Inc. and its licensors.
 * By using this software you agree to the terms of the associated
 * Analog Devices Software License Agreement.
*******************************************************************************/

#ifndef _ADC_DATA_CAPTURE_H_
#define _ADC_DATA_CAPTURE_H_

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/******************************************************************************/
/********************** Macros and Constants Definition ***********************/
/******************************************************************************/

/******************************************************************************/
/********************** Variables and User Defined Data Types *****************/
/******************************************************************************/

/**
 * @struct data_capture_ops
 * @brief Structure holding function pointers that points to device specific
 * data capture functions
 */
struct data_capture_ops {
	/* Perform operations required before single sample read */
	int32_t(*single_sample_read_start_ops)(uint8_t input_chn);

	/* Wait for conversion to finish on enabled channels and read conversion data */
	int32_t(*perform_conv_and_read_sample)(uint32_t *read_data, uint8_t chn);

	/* Enable operations required post single sample read */
	int32_t(*single_sample_read_stop_ops)(uint8_t input_chn);

	/* Perform operations required before continuous sample read */
	int32_t(*continuous_sample_read_start_ops)(uint32_t chn_mask);

	/* Read ADC raw sample/data */
	int32_t(*read_converted_sample)(uint32_t *read_data, uint8_t input_chn);

	/* Perform operations required post continuous sample read */
	int32_t(*continuous_sample_read_stop_ops)(void);

	/* Trigger next data conversion */
	int32_t(*trigger_next_conversion)(void);
};

/******************************************************************************/
/************************ Public Declarations *********************************/
/******************************************************************************/

int32_t read_single_sample(uint8_t input_chn, uint32_t *raw_data);
size_t read_buffered_data(char *pbuf,
			  size_t bytes,
			  size_t offset,
			  uint32_t active_chns_mask,
			  uint8_t sample_size_in_bytes);
void store_requested_samples_count(size_t bytes, uint8_t sample_size_in_bytes);
void start_data_capture(uint32_t ch_mask, uint8_t num_of_chns);
void stop_data_capture(void);
void data_capture_callback(void *ctx, uint32_t event, void *extra);

#define SUCCESS 1
#define FAILURE 0

#endif /* _ADC_DATA_CAPTURE_H_ */
