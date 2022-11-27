/*
 * cn0540_init_params.c
 *
 *  Created on: Jun 14, 2022
 *      Author: carlk
 */

#include "cn0540_init_params.h"

// Init params

// Init SPI extra parameters structure
//mbed_spi_init_param spi_init_extra_params ;
//= {
//	.spi_clk_pin =	SPI_SCK,
//	.spi_miso_pin = SPI_MISO,
//	.spi_mosi_pin = SPI_MOSI
//};
// SPI bus init parameters
//spi_init_param spi_params
//= {
//	20000000,		// SPI Speed
//	SPI_CS,			// SPI CS select index
//	SPI_MODE_3,		// SPI Mode
//	&spi_init_extra_params, // SPI extra configurations
//};

// Initial parameters for the ADC AD7768-1
struct ad77681_init_param init_params = {

//	spi_params,					// SPI parameters
	AD77681_ECO,				// power_mode
	AD77681_MCLK_DIV_16,		// mclk_div
	AD77681_CONV_CONTINUOUS,	// conv_mode
	AD77681_AIN_SHORT,			// diag_mux_sel
	false,						// conv_diag_sel
	AD77681_CONV_24BIT,			// conv_len
	AD77681_NO_CRC,				// crc_sel
	0,							// status bit
	AD77681_VCM_HALF_VCC,		// VCM setup
	AD77681_AINn_ENABLED,		// AIN- precharge buffer
	AD77681_AINp_ENABLED,		// AIN+ precharge buffer
	AD77681_BUFn_ENABLED,		// REF- buffer
	AD77681_BUFp_ENABLED,		// REF+ buffer
	AD77681_FIR,				// FIR Filter
	AD77681_SINC5_FIR_DECx32,	// Decimate by 32
	0,							// OS ratio of SINC3
	4096,						// Reference voltage
	16384,						// MCLK in kHz
	32000,						// Sample rate in Hz
	1,							// Data frame bytes
};

//Extra mbed_i2c_init_param
//mbed_i2c_init_param mbed_i2c_init_dac_extra_params
//= {
//	.i2c_sda_pin =	I2C_SDA,
//	.i2c_scl_pin = I2C_SCL,
//};

// Initial parameters for the DAC LTC2606's I2C bus
//const i2c_init_param i2c_params_dac = {
//		100000, 									// I2C speed (hz)
//		.slave_address = LTC26X6_WRITE_ADDRESS(LTC2606_I2C_ADDRESS),	// I2C slave address
//		&mbed_i2c_init_dac_extra_params,
//};

// Initial parameters for the DAC LTC2606 itself
struct ltc26x6_init_param init_params_dac = {
//		i2c_params_dac,	// I2C parameters
		16,				// Resolution (LTC2606)
		2.5,			// Reference Voltage
		-0.001			// Typical offset
};

/*
 *  User-defined coefficients for programmable FIR filter, max 56 coeffs
 *
 *  Please note that, inserted coefficiets will be mirrored afterwards,
 *  so you must insert only one half of all the coefficients.
 *
 *  Please note your original filer must have ODD count of coefficients,
 *  allowing internal ADC circuitry to mirror the coefficients properly.
 *
 *	In case of usage lower count of coeffs than 56, please make sure, that
 *	the variable 'count_of_active_coeffs' bellow, carries the correct number
 *	of coeficients, allowing to fill the rest of the coeffs by zeroes
 *
 *	Default coeffs:
 **/
const uint8_t count_of_active_coeffs = 56;

const float programmable_FIR[56] = {

	-9.53674E-07,
	 3.33786E-06,
	 5.48363E-06,
   - 5.48363E-06,
   - 1.54972E-05,
	 5.24521E-06,
	 3.40939E-05,
	 3.57628E-06,
   - 6.17504E-05,
   - 3.05176E-05,
	 9.56059E-05,
	 8.74996E-05,
   - 0.000124693,
   - 0.000186205,
	 0.000128746,
	 0.000333548,
   - 7.70092E-05,
   - 0.000524998,
   - 6.98566E-05,
     0.000738144,
	 0.000353813,
   - 0.000924349,
   - 0.000809193,
	 0.001007795,
	 0.00144887,
   - 0.000886202,
   - 0.002248049,
	 0.000440598,
	 0.00312829,
	 0.000447273,
   - 0.00394845,
   - 0.001870632,
	 0.004499197,
	 0.003867388,
   - 0.004512072,
   - 0.006392241,
	 0.003675938,
	 0.009288311,
   - 0.001663446,
   - 0.012270451,
   - 0.001842737,
	 0.014911652,
	 0.007131577,
   - 0.016633987,
   - 0.014478207,
	 0.016674042,
	 0.024231672,
   - 0.013958216,
   - 0.037100792,
	 0.006659508,
	 0.055086851,
	 0.009580374,
   - 0.085582495,
   - 0.052207232,
	 0.177955151,
	 0.416601658,
};
