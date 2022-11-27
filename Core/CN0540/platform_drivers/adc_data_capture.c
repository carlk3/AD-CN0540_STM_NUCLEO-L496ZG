/***************************************************************************//**
 *   @file    adc_data_capture.c
 *   @brief   ADC common data capture interface for IIO based applications
 *   @details This module handles the ADC data capturing for IIO client
********************************************************************************
 * Copyright (c) 2021 Analog Devices, Inc.
 * All rights reserved.
 *
 * This software is proprietary to Analog Devices, Inc. and its licensors.
 * By using this software you agree to the terms of the associated
 * Analog Devices Software License Agreement.
*******************************************************************************/

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "adc_data_capture.h"
//#include "error.h"

/******************************************************************************/
/********************** Macros and Constants Definition ***********************/
/******************************************************************************/

/* Max available channels for continuous data capture. Actual number of channels
 * to be captured are supplied from an application */
#define MAX_AVAILABLE_CHANNELS		(16)

/* Max size of the acquisition buffer (in terms of bytes) */
#define DATA_BUFFER_SIZE	(32768)		// 32Kbytes

/* Timeout count to avoid stuck into potential infinite loop while checking
 * for new data into an acquisition buffer. The actual timeout factor is determined
 * through 'sampling_frequency' attribute of IIO app, but this period here makes sure
 * we are not stuck into a forever loop in case data capture is interrupted
 * or failed in between.
 * Note: This timeout factor is dependent upon the MCU clock frequency. Below timeout
 * is tested for SDP-K1 platform @180Mhz default core clock */
#define BUF_READ_TIMEOUT	(100000000)

/******************************************************************************/
/********************** Variables and User Defined Data Types *****************/
/******************************************************************************/

/* Extern declaration for device specific data capture operations structure
 * (actual definition should be present in an application) */
extern struct data_capture_ops data_capture_ops;

/*
 *@enum		acq_buffer_state_e
 *@details	Enum holding the data acquisition buffer states
 **/
typedef enum {
	BUF_AVAILABLE,
	BUF_EMPTY,
	BUF_FULL
} acq_buffer_state_e;

/*
 *@struct	acq_buf_t
 *@details	Structure holding the data acquisition buffer parameters
 **/
typedef struct {
	acq_buffer_state_e state;         	// Buffer state
	bool refill_buffer;        			// Flag to start refilling acquisition buffer
	uint32_t rd_indx;          			// Buffer read index (incremented per sample transmit)
	uint32_t wr_indx;          			// Buffer write index (incremented per sample read)
	uint8_t sample_size;        		// ADC sample/raw data size received from application
	uint8_t chn_indx;        			// ADC channel index into acquisition buffer
	uint8_t active_chn[MAX_AVAILABLE_CHANNELS];   	// Active channel number sequence
	uint8_t data[DATA_BUFFER_SIZE];   				// buffer data (adc raw values)
	uint8_t *pdata;   								// Pointer to data buffer
} acq_buf_t;

/* ADC data acquisition buffers */
static volatile acq_buf_t acq_buffer;

/* Flag to indicate data capture status */
static volatile bool start_adc_data_capture = false;

/* Number of active channels in any data buffer read request */
static volatile uint8_t num_of_active_channels = 0;

/* Count to track number of actual samples requested by IIO client */
static volatile uint16_t num_of_requested_samples = 0;

/* Channel alignment offset */
static volatile uint8_t chn_alignment_offset = 0;

/* Actual or max available size of acquisition buffer */
static volatile uint16_t max_available_buffer_size = 0;

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/

/******************************************************************************/
/************************ Functions Definitions *******************************/
/******************************************************************************/

/*!
 * @brief	Function to read the single ADC sample (raw data) for input channel
 * @param	input_chn[in] - Input channel to sample and read data for
 * @param	raw_data[in, out]- ADC raw data
 * @return	SUCCESS in case of success, FAILURE otherwise
 */
int32_t read_single_sample(uint8_t input_chn, uint32_t *raw_data)
{
	uint32_t read_adc_data = 0;
	int32_t status = SUCCESS;

	do {
		/* Perform operations required before single sample conversion read */
		if (data_capture_ops.single_sample_read_start_ops) {
			if (data_capture_ops.single_sample_read_start_ops(input_chn) != SUCCESS) {
				status = FAILURE;
				break;
			}
		}

		/* Perform ADC conversion and read the converted sample after EOC */
		if (data_capture_ops.perform_conv_and_read_sample(&read_adc_data, input_chn) !=
		    SUCCESS) {
			status = FAILURE;
			break;
		}
	} while (0);

	/* Perform operations required post single sample conversion read */
	if (data_capture_ops.single_sample_read_stop_ops) {
		if (data_capture_ops.single_sample_read_stop_ops(input_chn) != SUCCESS) {
			status = FAILURE;
		}
	}

	*raw_data = read_adc_data;
	return status;
}


/*!
 * @brief	Function to store the number of actul requested ADC samples from IIO client
 * @param	bytes[in] - Number of bytes corresponding to requested samples
 * @param	sample_size_in_bytes[in] - Size of each sample in bytes (eqv to ADC resolution)
 * @return	none
 * @note	The information about sample and buffer size is required for continuous
 *			data acquisition
 */
void store_requested_samples_count(size_t bytes, uint8_t sample_size_in_bytes)
{
	/* This gets the number of samples requested by IIO client for all active channels */
	num_of_requested_samples = bytes / sample_size_in_bytes;

	/* Store the ADC sample size */
	acq_buffer.sample_size = sample_size_in_bytes;

	/* Get the actual available size of buffer by aligning with number of requested samples.
	 * e.g. if requested samples are 400 and sample size is 2 bytes, the max available
	 * size of buffer is: available size = ((32768 / 2) / 400) * 400 = 40 * 400 = 16000.
	 * The max samples to be requested should always be less than half the max size of buffer
	 * (in this case: (32768/2) / 2 = 8192).
	 * */
	max_available_buffer_size = ((DATA_BUFFER_SIZE / sample_size_in_bytes) /
				     num_of_requested_samples) * num_of_requested_samples;
}


/*!
 * @brief	Function to read acquired samples into buffer without IIO request timeout
 * @param	input_buffer[in] - Input data acquisition buffer
 * @param	output_buffer[in, out] - Output data buffer
 * @param	samples_to_read[in] - Number of samples to read
 * @param	sample_size_in_bytes[in] - Size of each sample in bytes (eqv to ADC resolution)
 * @return	none
 */
static void read_acquired_samples(char *output_buffer,
				  size_t samples_to_read,
				  uint8_t sample_size_in_bytes)
{
	int32_t buff_rd_wr_indx_offset; 			// Offset b/w buffer read and write indexes
	uint32_t timeout = BUF_READ_TIMEOUT; 	// Buffer new data read timeout count
	size_t bytes =  samples_to_read * sample_size_in_bytes;

	/* Copy the bytes into buffer provided there is enough offset b/w read and write counts.
	 * If there is overlap b/w read and write indexes (read is faster than write), empty buffer
	 * should be returned to IIO client to avoid IIO request timeout */
	do {
		buff_rd_wr_indx_offset = (acq_buffer.wr_indx - acq_buffer.rd_indx);
		timeout--;
	} while (((buff_rd_wr_indx_offset < (int32_t)(samples_to_read))
		  || (acq_buffer.wr_indx < acq_buffer.rd_indx)) && (timeout > 0)
		 && start_adc_data_capture);

	if ((timeout == 0) || (acq_buffer.wr_indx <= 0)) {
		/* This returns the empty buffer */
		return;
	}

	memcpy(output_buffer,
	       (void const *)&acq_buffer.data[acq_buffer.rd_indx * sample_size_in_bytes],
	       bytes);
	acq_buffer.rd_indx += samples_to_read;
}


/*!
 * @brief	Function to read and align the ADC buffered raw data
 * @param	device[in]- Device instance
 * @param	pbuf[out] - Buffer to load ADC raw data
 * @param	bytes[in] - Number of bytes to be read
 * @param	active_chns_mask[in] - Active channels mask
 * @return	Number of bytes read
 */
size_t read_buffered_data(char *pbuf,
			  size_t bytes,
			  size_t offset,
			  uint32_t active_chns_mask,
			  uint8_t sample_size_in_bytes)
{
	(void) active_chns_mask;

	size_t samples_to_read = bytes /
				 sample_size_in_bytes; 		// Bytes to sample conversion

	/* Make sure requested samples size is less than ADC buffer size. Return constant
	 * value to avoid IIO client getting timed-out */
	if(num_of_requested_samples >= max_available_buffer_size) {
		memset(pbuf, 1, bytes);
		return bytes;
	}

	/* Increment read counter to point to next acquired data of 1st active channel */
	if ((offset == 0) && (acq_buffer.rd_indx > 0)) {
		acq_buffer.rd_indx += chn_alignment_offset;
	}

	read_acquired_samples(pbuf, samples_to_read, sample_size_in_bytes);

	/* Make buffer available again once it is read/transmited completely */
	if (acq_buffer.rd_indx >= max_available_buffer_size) {
		acq_buffer.rd_indx = 0;
		acq_buffer.wr_indx = 0;
		acq_buffer.pdata = (uint8_t *)acq_buffer.data;
		acq_buffer.state = BUF_AVAILABLE;
		acq_buffer.refill_buffer = true;
	}

	return bytes;
}


/*!
 * @brief	This is an ISR (Interrupt Service Routine) to monitor end of conversion event.
 * @param	*ctx[in] - Callback context (unused)
 * @param	event[in] - Callback event (unused)
 * @param	extra[in] - Callback extra (unused)
 * @return	none
 * @details	This is an Interrupt callback function/ISR invoked in synchronous/asynchronous
 *			manner depending upon the application implementation. The conversion results
 *			are read into acquisition buffer and control continue to sample next channel.
 *			This continues until conversion is stopped (through IIO client command)
 * @note	This function also handles the logic to align the first channel data after
 *			every 'n' sample transmission. This is required to visualize data properly
 *			on IIO client application.
 */
void data_capture_callback(void *ctx, uint32_t event, void *extra)
{
	(void) ctx;
	(void) event;
	(void) extra;

	uint32_t adc_sample;

	if (start_adc_data_capture == true) {
		/* Read the sample(s) for channel(s) which has/have been sampled recently and
		 * get the number of samples read count */
		if (data_capture_ops.read_converted_sample(&adc_sample,
				acq_buffer.active_chn[acq_buffer.chn_indx]) != FAILURE) {
			do {
				if (acq_buffer.state == BUF_AVAILABLE) {
					if (acq_buffer.refill_buffer) {
						/* Buffer refilling must start with first active channel data
						 * for IIO client to synchronize the buffered data */
						if (acq_buffer.chn_indx != 0) {
							break;
						}
						acq_buffer.refill_buffer = false;
					}

					/* Copy adc samples into acquisition buffer to transport over
					 * communication link */
					memcpy(acq_buffer.pdata, &adc_sample, acq_buffer.sample_size);
					acq_buffer.pdata += acq_buffer.sample_size;

					/* Check for acquisition buffer full condition */
					acq_buffer.wr_indx++;
					if (acq_buffer.wr_indx >= max_available_buffer_size) {
						acq_buffer.state = BUF_FULL;
					}
				}
			} while (0);

			/* Keep tracking channel index as it is needed to refill the buffer
			 * starting with first channel data */
			acq_buffer.chn_indx++;
			if (acq_buffer.chn_indx >= num_of_active_channels) {
				acq_buffer.chn_indx = 0;
			}
		}

		/* Trigger next continuous conversion (optional or device dependent) */
		if (data_capture_ops.trigger_next_conversion) {
			data_capture_ops.trigger_next_conversion();
		}
	}
}


/*!
 * @brief	Reset the data capture specific variables
 * @return	none
 */
static void reset_data_capture(void)
{
	/* Reset data capture flags */
	start_adc_data_capture = false;
	num_of_active_channels = 0;

	/* Reset acquisition buffer states and clear old data */
	acq_buffer.state = BUF_EMPTY;

	acq_buffer.wr_indx = 0;
	acq_buffer.rd_indx = 0;
	acq_buffer.chn_indx = 0;
	acq_buffer.refill_buffer = false;
	acq_buffer.pdata = (uint8_t *)acq_buffer.data;
}


/*!
 * @brief	Function to trigger ADC conversion for new READBUFF
 *              request from IIO client (for active channels)
 * @param	ch_mask[in] - Channels to enable for data capturing
 * @param	num_of_chns[in] - ADC channel count
 * @return	none
 */
void start_data_capture(uint32_t ch_mask, uint8_t num_of_chns)
{
	uint32_t mask = 0x1;
	uint8_t index = 0;

	/* Make sure requested samples size is less than max available buffer size */
	if (num_of_requested_samples >= max_available_buffer_size) {
		return;
	}

	/* Reset data capture module specific flags and variables */
	reset_data_capture();

	/* Count active channels based on channel mask set in the IIO client */
	for (uint8_t chn = 0; chn < num_of_chns; chn++) {
		if (ch_mask & mask) {
			acq_buffer.active_chn[index++] = chn;
			num_of_active_channels++;
		}

		mask <<= 1;
	}

	/* Note: As shown below, after nth sample read, next sample must be for the first
	 *       channel present in the list of enabled channels */
	/*+-----------------------+-------------------------+---------------------------+
	 *| 0 | 1 | 2 | ------| n | 0 | 1 | 2 | --------| n | 0 | 1 | 2 |-----------| l |
	 *+-^-------------------^-+-^---------------------^-+-^-----------------------^-+
	 *  |                   |   |                     |   |                       |
	 * 1st chn data   nth data  1st chn data    nth data  1st chn data         last data
	 * n = number of requested samples. l = last data (channel unknown)
	 * To achieve this, offset value is determined based on the requested samples count
	 * and number of active channels. The read index is then incremented by offset value
	 * to read the data for first enabled channel by mapping into acquisition buffer.
	 **/
	if (num_of_requested_samples % num_of_active_channels) {
		chn_alignment_offset = (num_of_requested_samples - ((num_of_requested_samples /
					num_of_active_channels) * num_of_active_channels)) + 1;
	} else {
		chn_alignment_offset = 0;
	}

	/* Make acquisition buffer available and start continuous conversion */
	acq_buffer.state = BUF_AVAILABLE;
	if (data_capture_ops.continuous_sample_read_start_ops) {
		if (data_capture_ops.continuous_sample_read_start_ops(ch_mask) != FAILURE) {
			start_adc_data_capture = true;
		}
	} else {
		start_adc_data_capture = true;
	}
}


/*!
 * @brief	Function to stop ADC data capture
 * @return	none
 */
void stop_data_capture(void)
{
	start_adc_data_capture = false;

	/* Enable operations required post continuous sample read */
	if (data_capture_ops.continuous_sample_read_stop_ops) {
		data_capture_ops.continuous_sample_read_stop_ops();
	}

	/* Reset data capture module specific flags and variables */
	reset_data_capture();
}
