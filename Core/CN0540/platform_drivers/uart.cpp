/***************************************************************************//**
 * @file  uart.cpp
 * @brief Implementation of UART Mbed platform driver interfaces
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
#include <stdio.h>
#include <mbed.h>
#include <USBCDC.h>

// Platform drivers needs to be C-compatible to work with other drivers
#ifdef __cplusplus
extern "C"
{
#endif //  _cplusplus

#include "error.h"
#include "delay.h"
#include "uart.h"
#include "uart_extra.h"

/******************************************************************************/
/************************ Macros/Constants ************************************/
/******************************************************************************/

/* Max size for USB CDC packet during transmit/receive */
#define USB_CDC_MAX_PACKET_SIZE		(64)

/* Max allowed length of USB serial number in characters */
#define USB_SERIAL_NUM_MAX_LENGTH	(100)

/******************************************************************************/
/*************************** Types Declarations *******************************/
/******************************************************************************/

/* Derived USBCDC class to access protected members of USBCDC class */
class platform_usbcdc :public USBCDC
{
private:
	uint8_t usb_iserial_descriptor[(USB_SERIAL_NUM_MAX_LENGTH * 2) + 2];

public :
	/* Call parent class (USBCDC) constructor explicitly */
	platform_usbcdc(bool connect_blocking, uint16_t vendor_id, uint16_t product_id,
			const char *serial_number)
		: USBCDC(connect_blocking, vendor_id, product_id)
	{
		uint8_t usb_iserial_len;	// USB serial number length
		uint8_t i, j = 0;

		usb_iserial_len = strlen(serial_number);
		if (usb_iserial_len > USB_SERIAL_NUM_MAX_LENGTH) {
			usb_iserial_len = USB_SERIAL_NUM_MAX_LENGTH;
		}

		this->usb_iserial_descriptor[j++] = (usb_iserial_len * 2) + 2;	/* bLength */
		this->usb_iserial_descriptor[j++] = STRING_DESCRIPTOR;			/* bDescriptorType */

		/* bString iSerial */
		for (i = 0; i < usb_iserial_len; i++) {
			this->usb_iserial_descriptor[j++] = serial_number[i];
			this->usb_iserial_descriptor[j++] = 0;
		}
	}

	/* Override the virtual function that sets the USB serial number
	 * The custom (user provided) USB serial number is passed through this function */
	virtual const uint8_t *string_iserial_desc()
	{
		return this->usb_iserial_descriptor;
	}

	void change_terminal_connection(bool connect_status);
	bool data_received(uint32_t rx_size);
	bool data_transmited(void);
};

/******************************************************************************/
/************************ Functions Definitions *******************************/
/******************************************************************************/

/**
 * @brief Change the terminal connection status (for non-terminal) based USB interface
 * @param connect_status- new connection status
 * @note  This functions is used to change the terminal connection status of USB client
 *        interface which is different than the 'console terminal'. The console terminals acknowledge
 *        back to USB host when USB connection is opened on the console terminal and Mbed USBCDC
 *        class automatically changes the '_terminal_connected' status accordingly. However, for
 *        custom PC applications (non terminal), the terminal connection status needs to be changed
 *        manually. The '_terminal_connected' is protected member of USBCDC parent class and thus can
 *        be accessed through 'platform_usbcdc' derived class using below function.
 */
void platform_usbcdc::change_terminal_connection(bool connect_status)
{
	_terminal_connected = connect_status;
}


/**
 * @brief	Check if new USB data is received/available
 * @param	bytes[in] - Number of expected bytes to be received
 * @return	true if expected number of bytes received, else false
 */
bool platform_usbcdc::data_received(uint32_t bytes)
{
	volatile static uint32_t *rx_size = &_rx_size;

	if (*rx_size >= bytes) {
		return true;
	} else {
		return false;
	}
}


/**
 * @brief  Check if old USB data was transmitted
 * @return true if transmit not in progress, else false
 */
bool platform_usbcdc::data_transmited(void)
{
	volatile static bool *tx_in_progress = &_tx_in_progress;

	if (*tx_in_progress) {
		return false;
	} else {
		return true;
	}
}


/**
 * @brief Read data from UART device.
 * @param desc - Instance of UART.
 * @param data - Pointer to buffer containing data.
 * @param bytes_number - Number of bytes to read.
 * @return SUCCESS in case of success, FAILURE otherwise.
 */
int32_t uart_read(struct uart_desc *desc, uint8_t *data, uint32_t bytes_number)
{
	uint8_t cnt;
	mbed::UnbufferedSerial *uart;	// pointer to UnbufferedSerial/UART instance
	platform_usbcdc *usb_cdc_dev;	// Pointer to usb cdc device class instance
	uint32_t size_rd;

	if (desc && data) {
		if (((mbed_uart_desc *)(desc->extra))->uart_port) {
			if (((mbed_uart_desc *)desc->extra)->virtual_com_enable) {
				usb_cdc_dev = (platform_usbcdc *)((mbed_uart_desc *)(
						desc->extra))->uart_port;

				while (!usb_cdc_dev->data_received(bytes_number)) {
					/* Wait until new data is available */
				}

				/* Change terminal connection status manually */
				usb_cdc_dev->change_terminal_connection(true);

				usb_cdc_dev->receive_nb(data, bytes_number, &size_rd);
			} else {
				uart = (UnbufferedSerial *)(((mbed_uart_desc *)(desc->extra))->uart_port);

				for (cnt = 0; cnt < bytes_number; cnt++) {
					uart->read(data + cnt, 1);
				}
			}

			return bytes_number;
		}
	}

	return FAILURE;
}


/**
 * @brief Write data to UART device.
 * @param desc - Instance of UART.
 * @param data - Pointer to buffer containing data.
 * @param bytes_number - Number of bytes to read.
 * @return SUCCESS in case of success, FAILURE otherwise.
 */
int32_t uart_write(struct uart_desc *desc, const uint8_t *data,
		   uint32_t bytes_number)
{
	mbed::UnbufferedSerial *uart;	// pointer to UnbufferedSerial/UART instance
	platform_usbcdc *usb_cdc_dev;	// Pointer to usb cdc device class instance
	uint32_t d_sz;
	uint32_t indx = 0;

	if (desc && data) {
		if (((mbed_uart_desc *)(desc->extra))->uart_port) {
			if (((mbed_uart_desc *)desc->extra)->virtual_com_enable) {
				usb_cdc_dev = (platform_usbcdc *)((mbed_uart_desc *)(
						desc->extra))->uart_port;

				while (bytes_number) {
					while (!usb_cdc_dev->data_transmited()) {
						/* Wait until old data is transmitted */
					}

					/* Make sure packet size is less than max CDC packet size during data transmit */
					d_sz = (bytes_number > (USB_CDC_MAX_PACKET_SIZE - 1)) ?
					       (USB_CDC_MAX_PACKET_SIZE - 1) :
					       bytes_number;

					/* Change terminal connection status manually */
					usb_cdc_dev->change_terminal_connection(true);

					usb_cdc_dev->send_nb((uint8_t *)&data[indx], d_sz, &d_sz);

					bytes_number -= d_sz;
					indx += d_sz;
				}

				return bytes_number;
			} else {
				uart = (UnbufferedSerial *)(((mbed_uart_desc *)(desc->extra))->uart_port);
				return uart->write(data, bytes_number);
			}
		}
	}

	return FAILURE;
}


/**
 * @brief Submit reading buffer to the UART driver.
 *
 * Buffer is used until bytes_number bytes are read.
 * @param desc:	Descriptor of the UART device
 * @param data:	Buffer where data will be read
 * @param bytes_number:	Number of bytes to be read.
 * @return \ref SUCCESS in case of success, \ref FAILURE otherwise.
 */
int32_t uart_read_nonblocking(struct uart_desc *desc,
			      uint8_t *data,
			      uint32_t bytes_number)
{
	mbed::UnbufferedSerial *uart;		// pointer to UnbufferedSerial/UART instance

	if (desc) {
		if (((mbed_uart_desc *)(desc->extra))->uart_port) {
			uart = (UnbufferedSerial *)(((mbed_uart_desc *)(desc->extra))->uart_port);

			if (data) {
				for (size_t i = 0; i < bytes_number; i++) {
					if (uart->readable() > 0) {
						uart->read(&data[i], 1);
					}
				}

				return bytes_number;
			}
		}
	}

	return FAILURE;
}


/**
 * @brief Submit writting buffer to the UART driver.
 *
 * Data from the buffer is sent over the UART, the function returns imediatly.
 * @param desc:	Descriptor of the UART device
 * @param data:	Buffer where data will be written
 * @param bytes_number:	Number of bytes to be written.
 * @return \ref SUCCESS in case of success, \ref FAILURE otherwise.
 */
int32_t uart_write_nonblocking(struct uart_desc *desc,
			       const uint8_t *data,
			       uint32_t bytes_number)
{
	mbed::UnbufferedSerial *uart;		// pointer to UnbufferedSerial/UART instance

	if (desc) {
		if (((mbed_uart_desc *)(desc->extra))->uart_port) {
			uart = (UnbufferedSerial *)(((mbed_uart_desc *)(desc->extra))->uart_port);

			if (data) {
				for (size_t i = 0; i < bytes_number; i++) {
					uart->write(&data[i], 1);
				}

				return bytes_number;
			}
		}
	}

	return FAILURE;
}


/**
 * @brief Initialize the UART communication peripheral.
 * @param desc - The UART descriptor.
 * @param param - The structure that contains the UART parameters.
 * @return SUCCESS in case of success, FAILURE otherwise.
 */
int32_t uart_init(struct uart_desc **desc, struct uart_init_param *param)
{
	mbed::UnbufferedSerial *uart;	// Pointer to new UnbufferedSerial/UART instance
	platform_usbcdc *usb_cdc_dev;	// Pointer to usb cdc device class instance
	mbed_uart_desc *mbed_desc;  	// Pointer to mbed uart descriptor
	uart_desc *new_desc;			// UART new descriptor

	if (desc && param && param->extra) {
		// Create the UART description object for the device
		new_desc = (uart_desc *)malloc(sizeof(uart_desc));
		if (!new_desc) {
			goto err_new_desc;
		}

		new_desc->baud_rate = param->baud_rate;

		if (((mbed_uart_init_param *)param->extra)->virtual_com_enable) {
			// Create a new instance of platform_usbcdc class
			usb_cdc_dev = new platform_usbcdc(false,
							  ((mbed_uart_init_param *)param->extra)->vendor_id,
							  ((mbed_uart_init_param *)param->extra)->product_id,
							  ((mbed_uart_init_param *)param->extra)->serial_number);
			if (!usb_cdc_dev) {
				goto err_usb_cdc_dev;
			}
		} else {
			// Create and configure a new instance of UnbufferedSerial/UART port
			uart = new UnbufferedSerial(
				(PinName)(((mbed_uart_init_param *)param->extra)->uart_tx_pin),
				(PinName)(((mbed_uart_init_param *)param->extra)->uart_rx_pin),
				(int)param->baud_rate);

			if (!uart) {
				goto err_uart;
			}
		}

		// Create a new mbed descriptor to store new UART instances
		mbed_desc = (mbed_uart_desc *)malloc(sizeof(mbed_uart_desc));
		if (!mbed_desc) {
			goto err_mbed_desc;
		}

		if (((mbed_uart_init_param *)param->extra)->virtual_com_enable) {
			mbed_desc->uart_port = (platform_usbcdc *)usb_cdc_dev;

			/* Establish connection with the USB CDC communication port */
			usb_cdc_dev->connect();
			mdelay(2000);
		} else {
			mbed_desc->uart_port = (UnbufferedSerial *)uart;
		}

		mbed_desc->virtual_com_enable = ((mbed_uart_init_param *)
						 param->extra)->virtual_com_enable;
		new_desc->extra = (mbed_uart_desc *)mbed_desc;
		*desc = new_desc;

		return SUCCESS;
	}

err_mbed_desc:
	if (uart) {
		free(uart);
	}
	if (usb_cdc_dev) {
		free(usb_cdc_dev);
	}
err_uart:
err_usb_cdc_dev:
	free(new_desc);
err_new_desc:
	// Nothing to free

	return FAILURE;
}


/**
 * @brief Free the resources allocated by uart_init().
 * @param desc - The UART descriptor.
 * @return SUCCESS in case of success, FAILURE otherwise.
 */
int32_t uart_remove(struct uart_desc *desc)
{
	if (desc) {
		// Free the UART port object
		if (((mbed_uart_desc *)desc->extra)->virtual_com_enable) {
			if ((platform_usbcdc *)((mbed_uart_desc *)desc->extra)->uart_port)
				delete((platform_usbcdc *)(platform_usbcdc *)((mbed_uart_desc *)
						desc->extra)->uart_port);
		} else {
			if ((UnbufferedSerial *)(((mbed_uart_desc *)(desc->extra))->uart_port)) {
				delete((UnbufferedSerial *)(((mbed_uart_desc *)(desc->extra))->uart_port));
			}
		}

		// Free the UART extra descriptor object
		if ((mbed_uart_desc *)(desc->extra)) {
			free((mbed_uart_desc *)(desc->extra));
		}

		// Free the UART descriptor object
		free(desc);

		return SUCCESS;
	}

	return FAILURE;
}


/**
 * @brief Get number of UART errors.
 * @param desc - The UART descriptor.
 * @return number of errors.
 */
uint32_t uart_get_errors(struct uart_desc *desc)
{
	if (desc) {
		// Unused variable - fix compiler warning
	}

	return SUCCESS;
}

#ifdef __cplusplus  // Closing extern c
}
#endif //  _cplusplus
