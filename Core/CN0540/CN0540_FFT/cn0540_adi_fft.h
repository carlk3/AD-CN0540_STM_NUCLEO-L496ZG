/******************************************************************************
 *Copyright (c)2020 Analog Devices, Inc.  
 *
 * Licensed under the 2020-04-27-CN0540EC License(the "License");
 * you may not use this file except in compliance with the License.
 *
 ****************************************************************************/

#ifndef CN0540_ADI_FFT_H_
#define CN0540_ADI_FFT_H_

#include "stdint.h"
#include "arm_math.h"
#include "stdbool.h"

/**
 ******************************************************************************************************************************************************************
 ******************************************************************************************************************************************************************
 * 
 *			MACROS AND CONSTANT DEFINITIONS
 */

#define N_BITS					24								// Define resolution of the ADC


#define SQRT_2					sqrt(2)
#define ADC_FULL_SCALE			(1 << N_BITS)					// Full scale of the ADC depends on the N_BITS macro, for 24-bit ADC, ADC_FULL_SCALE = 16777216
#define ADC_ZERO_SCALE			(1 << (N_BITS - 1))				// Zero scale of the ADC depends on the N_BITS macro, for 24-bit ADC, ADC_ZERO_SCALE = 8388608

#define DC_BINS					10								// Ignoring certain amount of DC bins for for noise and other calculations
#define FUND_BINS				10								// Power spread of the fundamental, 10 bins from either side of the fundamental
#define HARM_BINS				3								// Power spread of the harmonic, 10 bins from either side of the harmonic


/**
 ******************************************************************************************************************************************************************
 ******************************************************************************************************************************************************************
 * 
 *			TYPES DECLARATIONS
 */

enum fft_windowing_type
{
	BLACKMAN_HARRIS_7TERM,
	RECTANGULAR
};

struct fft_entry									// Structure carying all the necessary data, the FFT work with
{
	float	vref;
	uint16_t	mclk;
	float	bin_width;
	uint32_t	sample_rate;						// Sample rate based on MCLK, MCLK_DIV and Digital filter settings
	uint16_t	fft_length;							// Length of fft = sample_count / 2
	int32_t		codes[4096];						// Codes in range 0 +/- ZERO SCALE
	int32_t		zero_scale_codes[4096];				// Codes shifted up by ZERO SCALE - range ZERO SCALE +/- ZERO SCALE
	float	voltage[4096];						// Voltage before windowing
	float	fft_magnitude[2048];				// Maximum length of FFT magnitude supported by the on-chip DSP = 4096 samples
	float	fft_magnitude_corrected[2048];		// Maginute with windowing correction
	float	fft_dB[2048];						// dB fro plot
	float	fft_input[8192];					// Maximum length of FFT input array supporred by the on-chip DSP = 4096 Real + 4096 Imaginary samples
	float	noise_bins[2048];					// FFT bins excluding DC, fundamental and Harmonics
	enum fft_windowing_type window;					// WIndow type
	bool		fft_done;
};

struct fft_measurements								// Structure carying all the FFT measurements
{
	float	harmonics_power[6]; 				// Harmonics, including their power leakage
	float	harmonics_mag_dbfs[6]; 				// Harmonic magnitudes for THD
	uint16_t	harmonics_freq[6]; 					// Harmonic frequencies for THD
	float	fundamental;  						// Fundamental in volts
	float	pk_spurious_noise; 					// Peak spurious noise (amplitude)
	uint16_t	pk_spurious_freq; 					// Peak Spurious Frequency
	float	THD; 								// Total Harmonic Distortion
	float	SNR; 								// Signal to Noise Ratio
	float	DR; 								// Dynamic Range
	float	SINAD; 								// Signal to Noise And Distortion ratio
	float	SFDR_dbc; 							// Spurious Free Dynamic Range, dBc
	float	SFDR_dbfs; 							// Spurious Free Dynamic Range, dbFS
	float	ENOB; 								// ENOB - Effective Number Of Bits
	float	RMS_noise; 							// same as transition noise
	float	average_bin_noise;
	float	max_amplitude;
	float	min_amplitude;
	float	pk_pk_amplitude;
	float	DC;
	float	transition_noise;
	uint32_t	max_amplitude_LSB;
	uint32_t	min_amplitude_LSB;
	uint32_t	pk_pk_amplitude_LSB;
	int32_t		DC_LSB;
	float	transition_noise_LSB;
};

/**
 ******************************************************************************************************************************************************************
 ******************************************************************************************************************************************************************
 * 
 *			FUNCTION DECLARATIONS
 */

int32_t FFT_init_params(struct fft_entry **fft_entry_init, struct fft_measurements **fft_meas);
void perform_FFT(uint32_t *data, struct fft_entry *fft_data, struct fft_measurements *fft_meas, uint32_t sample_rate);
void FFT_init(uint16_t sample_count, struct fft_entry *fft_data);
void update_FFT_enviroment(uint16_t reference, uint16_t master_clock, uint16_t sampling_rate, struct fft_entry *fft_data);
#endif // !ADI_FFT_H_

