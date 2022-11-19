/***************************************************************************//**
 *   @file   spi.cpp
 *   @brief  Implementation of SPI Mbed platform driver interfaces
********************************************************************************
 * Copyright (c) 2019-2021 Analog Devices, Inc.
 * All rights reserved.
 *
 * This software is proprietary to Analog Devices, Inc. and its licensors.
 * By using this software you agree to the terms of the associated
 * Analog Devices Software License Agreement.
*******************************************************************************/

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/

#include <stdio.h>
#include <mbed.h>

// Platform drivers needs to be C-compatible to work with other drivers
#ifdef __cplusplus
extern "C"
{
#endif //  _cplusplus

#include "error.h"
#include "spi.h"
#include "gpio.h"
#include "spi_extra.h"

/******************************************************************************/
/********************** Macros and Constants Definitions **********************/
/******************************************************************************/

#define		SPI_8_BIT_FRAME			8		// SPI 8-bit frame size

/******************************************************************************/
/********************** Variables and User defined data types *****************/
/******************************************************************************/

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/

/******************************************************************************/
/************************ Functions Definitions *******************************/
/******************************************************************************/

/**
 * @brief Initialize the SPI communication peripheral.
 * @param desc - The SPI descriptor.
 * @param init_param - The structure that contains the SPI parameters.
 * @return SUCCESS in case of success, FAILURE otherwise.
 */
int32_t spi_init_noos(struct spi_desc **desc,
		      const struct spi_init_param *param)
{
	mbed::SPI *spi;					// Pointer to new spi instance
	DigitalOut *csb;  				// pointer to new CSB gpio instance
	mbed_spi_desc *mbed_desc;		// Pointer to mbed spi descriptor
	use_gpio_ssel_t use_gpio_ssel;	// For CSB hardware (implicit) control
	spi_desc *new_desc;

	if ((desc) && (param) && (param->extra)) {
		// Create the spi description object for the device
		new_desc = (spi_desc *)malloc(sizeof(spi_desc));
		if (!new_desc) {
			goto err_new_desc;
		}

		new_desc->chip_select = param->chip_select;
		new_desc->mode = param->mode;
		new_desc->max_speed_hz = param->max_speed_hz;

		// Create the SPI extra descriptor object to store new SPI instances
		mbed_desc = (mbed_spi_desc *)malloc(sizeof(mbed_spi_desc));
		if (!mbed_desc) {
			goto err_mbed_desc;
		}

		// Configure and instantiate SPI protocol
		if (((mbed_spi_init_param *)param->extra)->use_sw_csb) {
			/* CSB pin toggled explicitly (s/w controlled) during SPI transaction */
			spi = new SPI(
				(PinName)(((mbed_spi_init_param *)param->extra)->spi_mosi_pin),
				(PinName)(((mbed_spi_init_param *)param->extra)->spi_miso_pin),
				(PinName)(((mbed_spi_init_param *)param->extra)->spi_clk_pin));

			if (spi) {
				/* Configure and instantiate chip select pin */
				csb = new DigitalOut((PinName)(new_desc->chip_select));
				if (csb) {
					mbed_desc->use_sw_csb = true;
					mbed_desc->csb_gpio = (DigitalOut *)csb;
					csb->write(GPIO_HIGH);
				} else {
					goto err_csb;
				}
			}
		} else {
			/* CSB pin toggled implicitly (through HAL layer) during SPI transaction */
			spi = new SPI(
				(PinName)(((mbed_spi_init_param *)param->extra)->spi_mosi_pin),
				(PinName)(((mbed_spi_init_param *)param->extra)->spi_miso_pin),
				(PinName)(((mbed_spi_init_param *)param->extra)->spi_clk_pin),
				(PinName)(param->chip_select),
				use_gpio_ssel);

			mbed_desc->use_sw_csb = false;
			mbed_desc->csb_gpio = NULL;
		}

		if (!spi) {
			goto err_spi;
		}

		mbed_desc->spi_port = (SPI *)spi;

		new_desc->extra = (mbed_spi_desc *)mbed_desc;
		*desc = new_desc;

		/**
		    NOTE: Actual frequency of SPI clk will be somewhat device
		    dependent, relating to clock-settings, prescalars etc. If absolute
		    SPI frequency is required, consult your device documentation.
		  **/
		spi->frequency(param->max_speed_hz);
		spi->format(SPI_8_BIT_FRAME, param->mode);   // data write/read format
		spi->set_default_write_value(0x00);          // code to write when reading back

		return SUCCESS;
	}

err_spi:
	if (((mbed_spi_init_param *)param->extra)->use_sw_csb) {
		free(csb);
	}
err_csb:
	free(spi);
	free(mbed_desc);
err_mbed_desc:
	free(new_desc);
err_new_desc:
	// Nothing to free

	return FAILURE;
}


/**
 * @brief Free the resources allocated by spi_init().
 * @param desc - The SPI descriptor.
 * @return SUCCESS in case of success, FAILURE otherwise.
 */
int32_t spi_remove(struct spi_desc *desc)
{
	if (desc) {
		if (((mbed_spi_desc *)desc->extra)->use_sw_csb) {
			// Free the CSB gpio object
			if((DigitalOut *)(((mbed_spi_desc *)(desc->extra))->csb_gpio)) {
				delete((DigitalOut *)(((mbed_spi_desc *)(desc->extra))->csb_gpio));
			}
		}

		// Free the SPI port object
		if ((SPI *)(((mbed_spi_desc *)(desc->extra))->spi_port)) {
			delete((SPI *)(((mbed_spi_desc *)(desc->extra))->spi_port));
		}

		// Free the SPI extra descriptor object
		if ((mbed_spi_desc *)(desc->extra)) {
			free((mbed_spi_desc *)(desc->extra));
		}

		// Free the SPI descriptor object
		free(desc);

		return SUCCESS;
	}

	return FAILURE;
}


/**
 * @brief Write and read data to/from SPI.
 * @param desc - The SPI descriptor.
 * @param data - The buffer with the transmitted/received data.
 * @param bytes_number - Number of bytes to write/read.
 * @return SUCCESS in case of success, FAILURE otherwise.
 */
int32_t spi_write_and_read(struct spi_desc *desc,
			   uint8_t *data,
			   uint16_t bytes_number)
{
	mbed::SPI *spi; 			// pointer to new spi instance
	mbed::DigitalOut *csb;   	// pointer to new CSB instance

	if (desc) {
		spi = (SPI *)(((mbed_spi_desc *)(desc->extra))->spi_port);

		if (((mbed_spi_desc *)desc->extra)->use_sw_csb) {
			csb = (DigitalOut *)(((mbed_spi_desc *)(desc->extra))->csb_gpio);
			csb->write(GPIO_LOW);
		}

		/* Perform synchronous SPI write and read */
		spi->write((const char *)data, bytes_number, (char *)data, bytes_number);

		if (((mbed_spi_desc *)desc->extra)->use_sw_csb) {
			csb->write(GPIO_HIGH);
		}

		return SUCCESS;
	}

	return FAILURE;
}


/**
 * @brief Transfer (write/read) the number of SPI messages
 * @param desc - The SPI descriptor
 * @param msgs - Pointer to SPI messages
 * @param num_of_msgs - Number of SPI messages
 * @return SUCCESS in case of success, FAILURE otherwise.
 * @note Use of this function requires CSB pin to be software controlled
 */
int32_t spi_transfer(struct spi_desc *desc, struct spi_msg *msgs,
		     uint32_t num_of_msgs)
{
	mbed::SPI *spi; 			// pointer to new spi instance
	mbed::DigitalOut *csb;   	// pointer to new CSB instance
	uint8_t msg_cnt;			// SPI message counter

	if (desc) {
		if (!((mbed_spi_desc *)desc->extra)->use_sw_csb)
			return FAILURE;

		spi = (SPI *)(((mbed_spi_desc *)(desc->extra))->spi_port);
		csb = (DigitalOut *)(((mbed_spi_desc *)(desc->extra))->csb_gpio);

		if (!spi || !csb)
			return FAILURE;

		for (msg_cnt = 0; msg_cnt < num_of_msgs; msg_cnt++) {
			csb->write(GPIO_LOW);

			/* Perform synchronous SPI write and read */
			if (!msgs[msg_cnt].tx_buff) {
				spi->write(NULL, 0,
					   (char *)msgs[msg_cnt].rx_buff, msgs[msg_cnt].bytes_number);
			} else {
				spi->write((const char *)msgs[msg_cnt].tx_buff, msgs[msg_cnt].bytes_number,
					   (char *)msgs[msg_cnt].rx_buff, msgs[msg_cnt].bytes_number);
			}

			if (msgs[msg_cnt].cs_change) {
				csb->write(GPIO_HIGH);
			}
		}

		csb->write(GPIO_HIGH);
		return SUCCESS;
	}

	return FAILURE;
}

#ifdef __cplusplus  // Closing extern c
}
#endif //  _cplusplus
