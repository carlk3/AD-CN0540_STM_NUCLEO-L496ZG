/******************************************************************************
 *Copyright (c)2020 Analog Devices, Inc.  
 *
 * Licensed under the 2020-04-27-CN0540EC License(the "License");
 * you may not use this file except in compliance with the License.
 *
 ****************************************************************************/
 
#include "cn0540_adi_fft.h"
#include "cn0540_windowing.h"
#include "stdio.h"
#include "stdlib.h"
#include <math.h>
#include <stdbool.h>

static arm_cfft_radix4_instance_f32 S;

void static FFT_maginutde_do_dB(struct fft_entry *fft_data, double sum);
void static FFT_calculate_THD(struct fft_entry *fft_data, struct fft_measurements *fft_meas);
void static FFT_calculate_noise(struct fft_entry *fft_data, struct fft_measurements *fft_meas);
float static dbfs_to_volts(float vref, float value);
void static FFT_windowing(struct fft_entry *fft_data, double *sum);
void static FFT_waveform_stat(struct fft_entry *fft_data, struct fft_measurements *fft_meas);

/**
 * Initialize the FFT structure
 * @param **fft_entry_init - the FFT data structure
 * @param **fft_meas - the FFT measurements structure
 */
int32_t FFT_init_params(struct fft_entry **fft_entry_init, struct fft_measurements **fft_meas)
{
	struct fft_entry *fft_data_init;
	struct fft_measurements *fft_meas_init;

	fft_data_init = (struct fft_entry *)malloc(sizeof(*fft_data_init));
	if (!fft_data_init) {
		return -1;
	}	
	fft_meas_init = (struct fft_measurements *)malloc(sizeof(*fft_meas_init));
	if (!fft_meas_init) {
		return -1;
	}
	fft_data_init->window = BLACKMAN_HARRIS_7TERM;
	fft_data_init->vref = (float)(4096);
	fft_data_init->sample_rate = 32000;
	fft_data_init->mclk = 16384;
	fft_data_init->fft_length = 4096;
	fft_data_init->bin_width = 0.0;
	fft_data_init->fft_done = false;
	
	*fft_entry_init = fft_data_init;
	
	fft_meas_init->fundamental = 0.0;
	fft_meas_init->pk_spurious_noise = 0.0;
	fft_meas_init->pk_spurious_freq = 0;	
	fft_meas_init->THD = 0.0;
	fft_meas_init->SNR = 0.0;
	fft_meas_init->DR = 0.0;
	fft_meas_init->SINAD = 0.0;
	fft_meas_init->SFDR_dbc = 0.0;
	fft_meas_init->SFDR_dbfs = 0.0;
	fft_meas_init->ENOB = 0.0;
	fft_meas_init->RMS_noise = 0.0;
	fft_meas_init->average_bin_noise = 0.0;
	fft_meas_init->max_amplitude = 0.0;
	fft_meas_init->min_amplitude = 0.0; 
	fft_meas_init->pk_pk_amplitude = 0.0;
	fft_meas_init->DC = 0.0;
	fft_meas_init->transition_noise = 0.0;
	fft_meas_init->max_amplitude_LSB = 0;
	fft_meas_init->min_amplitude_LSB = 0; 
	fft_meas_init->pk_pk_amplitude_LSB = 0;
	fft_meas_init->DC_LSB = 0;
	fft_meas_init->transition_noise_LSB = 0.0;
	
	for (uint8_t i = 0; i < 7; i++) {
		fft_meas_init->harmonics_mag_dbfs[i] = 0.0;
		fft_meas_init->harmonics_freq[i] = 0;
		fft_meas_init->harmonics_power[i] = 0.0;
	}
	
	*fft_meas = fft_meas_init;
	
	return 0;
}

/**
 * Initialize the FFT module
 * The funciton has to be called, everytime when user wants to change the number of samples
 * @param sample_count - desired FFT samples
 * @param *fft_data - fft_data structure
 */
void FFT_init(uint16_t sample_count, struct fft_entry *fft_data)
{
	arm_cfft_radix4_init_f32(&S, sample_count, 0, 1);
	fft_data->fft_length = sample_count / 2;
}

/**
 * Update reference voltage and Master clock
 * The funciton has to be called, everytime when user wants to change Vref or Mclk	
 * @param referemce - The reference voltage in mV
 * @param master_clock - The master clock frequnecy in kHz
 * @param sampling_rate - The samplingrate frequnecy in kHz
 * @param *fft_data - fft_data structure
 */
void update_FFT_enviroment(uint16_t reference, uint16_t master_clock, uint16_t sampling_rate, struct fft_entry *fft_data)
{
	fft_data->vref = ((float)(reference)) / ((float)(1000.0)); 			// Convert reference voltage to V
	fft_data->mclk = master_clock;											    // MCLK in kHz
	fft_data->sample_rate = sampling_rate;										// Sampling rate
}




/**
 * Perform the FFT
 * @param *data - pointer to sampled data
 * @param *fft_data		-	fft_data structure
 * @param *fft_meas		-	fft_meas structure
 * @param sample_rate	-	sample rate based on MCLK, MCLK_DIV and Digital filter settings
 */
void perform_FFT(uint32_t *data, struct fft_entry *fft_data, struct fft_measurements *fft_meas, uint32_t sample_rate)
{
	uint32_t i;
	int32_t shifted_data = 0;	
	double coeffs_sum = 0.0;
	
	fft_data->fft_done = false;
	fft_data->sample_rate = sample_rate;														// get sample rate
	fft_data->bin_width = (float)(fft_data->sample_rate) / ((float)(fft_data->fft_length)*2);	// get bin width
	
	
	//Converting RAW adc data to codes
	for(i = 0 ; i < fft_data->fft_length * 2 ; i++) {
		if (data[i] & 0x800000)
			shifted_data = (int32_t)((0xFF << 24) | data[i]);	
		else
			shifted_data = (int32_t)((0x00 << 24) | data[i]);	
		
		fft_data->codes[i] = shifted_data;  													// Codes in range 0 +/- ZERO SCALE
		fft_data->zero_scale_codes[i] = shifted_data + ADC_ZERO_SCALE;							// Codes shifted up by ZERO SCALE - range ZERO SCALE +/- ZERO SCALE
	}	
	// Find max, min, pk-pk amplitude, DC offset, Tranition noise
	FFT_waveform_stat(fft_data, fft_meas);	
	
	// Converting codes without DC offset to "volts" without respect to Vref voltage
	for (i = 0; i < fft_data->fft_length * 4; i++) {
		// Filling array for FFT, conversion to voltage withour respect to Vref voltage		
		fft_data->fft_input[i] = ((((float)(fft_data->codes[i / 2])) / ADC_ZERO_SCALE));
				
		// Voltage array with respect to full scale voltage, Vpp
		fft_data->voltage[i / 2] = (float)((2*fft_data->vref / ADC_ZERO_SCALE) * fft_data->codes[i / 2]);
				
		// Imaginary part
		fft_data->fft_input[++i] = 0;
	}	
	// Apply windowing
	FFT_windowing(fft_data, &coeffs_sum);	
	//perform FFT, passing the input array, complex FFT values will be stored to the iput array
	arm_cfft_radix4_f32(&S, fft_data->fft_input);	
	//transform from complex FFT to magnitude
	arm_cmplx_mag_f32(fft_data->fft_input, fft_data->fft_magnitude, fft_data->fft_length);	
	// Convert FFT magnitude to dB for plot
	FFT_maginutde_do_dB(fft_data, coeffs_sum);	
	//Calculate THD
	FFT_calculate_THD(fft_data, fft_meas);	
	// Calculate noise and its parameters from FFT points
	FFT_calculate_noise(fft_data, fft_meas);	
	// FFT finish flag
	fft_data->fft_done = true;
}

/**
 * Windowing function
 * 7-term Blackman-Harris and Rectangular widow for now, prepared for more windowing functions if needed
 * You can use precalculated coeficients for 4096 sample length
 * @param *sample			-	pointer to sample
 * @param sample_length		-	2 * FFT_length, because working with samples, not with FFT yet
 * @param *sum				-	pointer to sum of all the coeffs
 * 
 */
void static FFT_windowing(struct fft_entry *fft_data, double *sum)
{
	uint8_t j;
	uint16_t i;
	double term = 0.0;
	const double sample_count = (double)((fft_data->fft_length * 2) - 1);
	
	for (i = 0; i < fft_data->fft_length * 4; i++)
	{
		switch (fft_data->window) {																	// Switch prepard for other windowing functions
		case BLACKMAN_HARRIS_7TERM:		
			if (fft_data->fft_length == 2048)													// For 4096 samples, precalculated coefficients are used
				term = Seven_Term_Blackman_Harris_4096[i / 2];			
			else {																			// 7-term BH windowing formula 
				for (j = 0; j < 7; j++)
					term += seven_term_BH_coefs[j] * cos((double)((2.0 * PI * j * (i / 2))) / sample_count);
			}
			break;
		case RECTANGULAR:																			// No window, all terms = 1
			term = 1;
			break;			
		default:
			break;
		}
		*sum += term;																				// Getting sum of all terms, which will be used for amplitude correction
		fft_data->fft_input[i] *= (float)(term);												// Multiplying each (real) sample by windowing term
		term = 0.0;
		i++;																						// +1, to consider only real (not imaginary) samples
	}
}


/**
 * Transfer magnitude to dB
 * @param *fft_data -	fft_data structure
 * @param sum		-	sum of all windowing coeffs
 */
void static FFT_maginutde_do_dB(struct fft_entry *fft_data, double sum)
{
	uint16_t i;
	float correction = 0;
	
	// Getting sum of coeffs
	// If rectangular window is choosen = no windowing, sum of coeffs is number of samples
	const float coeff_sum = ((fft_data->fft_length == 2048) && 
							(fft_data->window == BLACKMAN_HARRIS_7TERM)) ? ((float)(Seven_Term_Blackman_Harris_4096_sum)) :
							((fft_data->window == RECTANGULAR) ? ((float)(fft_data->fft_length * 2.0)) : (float)(sum));
	
	for (i = 0; i < fft_data->fft_length; i++) {
		// Apply a correction factor
		// Divide magnigude by a sum of the windowing function coefficients
		// Multiple by 2 because of power spread over spectrum below and above the Nyquist frequency
		correction = (fft_data->fft_magnitude[i] * 2.0) / coeff_sum;
		
		// FFT magnitude with windowing correction
		fft_data->fft_magnitude_corrected[i] = correction;
		
		//Convert to dB without respect to Vref
		fft_data->fft_dB[i] = 20.0 * (log10f(correction));
	}
}

/**
 * THD Calculation with support of harmonics folding to 1-st nyquist zone
 * @param *fft_data - fft_data structure
 * @param *fft_meas - fft_meas structure
 */
void static FFT_calculate_THD(struct fft_entry *fft_data, struct fft_measurements *fft_meas)
{
	const uint16_t first_nyquist_zone = fft_data->fft_length;
	uint16_t i, j, k = 0, fund_freq = 0, harmonic_position;
	int8_t m, nyquist_zone;
	float mag_helper = -200.0, freq_helper, sum = 0.0, fund_mag = -200.0; 
	float fund_pow_bins[21], harm_pow_bins[5][7];
	
	// Looking for the fundamental frequency and amplitude
	for(i = DC_BINS ; i < fft_data->fft_length ; i++) {												// Not counting DC bins
		if (fft_data->fft_dB[i] > fund_mag) {
			fund_mag = fft_data->fft_dB[i];
			fund_freq = i;
		}
	}
	
	fft_meas->harmonics_freq[0] = fund_freq; 													// Fundamental frequency bin
	fft_meas->harmonics_mag_dbfs[0] = fund_mag;  												// Fundamental magnitude in dBFS
	fft_meas->fundamental = dbfs_to_volts(fft_data->vref, fund_mag); 							// Fundamental magnitude in V

	for(i = 1 ; i < 6 ; i++) {		
		if (fft_meas->harmonics_freq[0] * (i + 1) < first_nyquist_zone)							// Checking if 2nd harmonic is outside of the first nyquist zone
			harmonic_position = fft_meas->harmonics_freq[0] * (i + 1);
		else {
			nyquist_zone = 1 + (fft_meas->harmonics_freq[0] * (i + 1) / first_nyquist_zone); 	// Determine the nyquist zone
			if(nyquist_zone % 2)																// Odd nyquist zones: 3, 5, 7...
				harmonic_position = first_nyquist_zone - (first_nyquist_zone * nyquist_zone - fft_meas->harmonics_freq[0] * (i + 1));
			else																				// Even nyquist zones: 2, 4, 6...
				harmonic_position = first_nyquist_zone * nyquist_zone - fft_meas->harmonics_freq[0] * (i + 1);
		}
		// Extend searching range by 3 bins around expected position of the harmonic
		for(m = -HARM_BINS ; m <= HARM_BINS ; m++) {
			if (fft_data->fft_dB[harmonic_position + m] > mag_helper) {
				mag_helper = fft_data->fft_dB[harmonic_position + m];
				freq_helper = (harmonic_position + m);
			}	
		}
		
		fft_meas->harmonics_freq[i] = freq_helper;
		fft_meas->harmonics_mag_dbfs[i]  = mag_helper;
		mag_helper = -200.0;
	}	
	// Power leakage of the fundamental
	for(i = fft_meas->harmonics_freq[0] - FUND_BINS ; i <= fft_meas->harmonics_freq[0] + FUND_BINS ; i++) {
		sum += powf(((fft_data->fft_magnitude_corrected[i] / (2.0*SQRT_2))), 2.0);
		fund_pow_bins[k] = fft_data->fft_magnitude_corrected[i];
		k++;
	}		
	// Finishing the RSS of power-leaked fundamental
	sum = sqrt(sum);
	fft_meas->harmonics_power[0] = sum * 2.0 * SQRT_2;
	sum = 0.0;
	k = 0;	
	// Power leakage of the harmonics
	for(j = 1 ; j <= 5 ; j++) {
		for (i = fft_meas->harmonics_freq[j] - HARM_BINS; i <= fft_meas->harmonics_freq[j] + HARM_BINS; i++) {
			sum += powf(((fft_data->fft_magnitude_corrected[i] / (2.0*SQRT_2))), 2.0);
			harm_pow_bins[j - 1][k] = fft_data->fft_magnitude_corrected[i];
			k++;
		}		
		// Finishing the RSS of power-leaked harmonics
		k = 0;
		sum = sqrt(sum);
		fft_meas->harmonics_power[j] = sum * 2.0 * SQRT_2;
		sum = 0.0;
	}	
	// The THD formula
	fft_meas->THD = sqrtf(powf(fft_meas->harmonics_power[1], 2.0) + powf(fft_meas->harmonics_power[2], 2.0) + powf(fft_meas->harmonics_power[3], 2.0) + powf(fft_meas->harmonics_power[4], 2.0) + powf(fft_meas->harmonics_power[5], 2.0)) / fft_meas->harmonics_power[0];
	// Back from volts to dB
	fft_meas->THD = 20.0 * log10f(fft_meas->THD);
}

/**
 * Calculate amplitudes: min, max, pk-pk amplitude and DC part
 * @param *fft_data - fft_data structure
 * @param *fft_meas - fft_meas structure
 */
void static FFT_waveform_stat(struct fft_entry *fft_data, struct fft_measurements *fft_meas)
{
	uint16_t i;
	int16_t max_position, min_position;
	int32_t max = -ADC_ZERO_SCALE, min = ADC_ZERO_SCALE, offset_correction;
	int64_t sum = 0;
	double deviation = 0.0, mean;
	
	// summ of all coeffs, to find the Mean value
	for(i = 0; i < fft_data->fft_length * 2; i++)
		sum += fft_data->codes[i];
	
	// Calculating mean value = DC offset
	mean = (sum / (fft_data->fft_length * 2));
	
	// DC part in LSBs
	fft_meas->DC_LSB = (int32_t)(mean) + ADC_ZERO_SCALE;
	offset_correction = (int32_t)(mean);
	
	// Min, Max amplitudes + Deviation
	for (i = 0; i < fft_data->fft_length * 2; i++) {						// Working with codes = fft_length * 2
		// Calculating the Deviation for Transition noise
		deviation += pow(fft_data->codes[i] - mean, 2.0);
		
		// Looking for MAX value
		if (fft_data->codes[i] > max) {
			max = fft_data->codes[i];
			max_position = i;
		}
		// Looking for MIN value
		if (fft_data->codes[i] < min) {
			min = fft_data->codes[i];
			min_position = i;
		}	
	}	
	// Amplitudes in Volts
	fft_meas->max_amplitude = (2.0 * fft_data->vref * fft_data->codes[max_position]) / ADC_FULL_SCALE; 
	fft_meas->min_amplitude = (2.0 * fft_data->vref * fft_data->codes[min_position]) / ADC_FULL_SCALE; 
	fft_meas->pk_pk_amplitude = fft_meas->max_amplitude - fft_meas->min_amplitude;
	fft_meas->DC = (2.0 * fft_data->vref * ((float)(((int32_t)(fft_meas->DC_LSB) - ADC_ZERO_SCALE)))) / ADC_FULL_SCALE;
	
	// Amplitudes in LSBs
	fft_meas->max_amplitude_LSB = fft_data->codes[max_position] + ADC_ZERO_SCALE;
	fft_meas->min_amplitude_LSB = fft_data->codes[min_position] + ADC_ZERO_SCALE;
	fft_meas->pk_pk_amplitude_LSB = fft_meas->max_amplitude_LSB - fft_meas->min_amplitude_LSB;
	
	// Transition noise
	deviation = (sqrt(deviation / (fft_data->fft_length * 2.0)));
	fft_meas->transition_noise_LSB = (uint32_t)(deviation);
	fft_meas->transition_noise = (2.0 * fft_data->vref * fft_meas->transition_noise_LSB) / ADC_FULL_SCALE;
	
	// RMS noise
	fft_meas->RMS_noise = fft_meas->transition_noise;
	
	// Applying mean value to each sample = removing DC offset
	for(i = 0 ; i < fft_data->fft_length * 2 ; i++)
		fft_data->codes[i] -= offset_correction;
	
}

/**
 * Calculate the RMS noise from the FFT plot
 * @param *fft_data - fft_data structure
 * @param *fft_meas - fft_meas structure
 */
void static FFT_calculate_noise(struct fft_entry *fft_data, struct fft_measurements *fft_meas)
{
	const float LW_DR_correction_const = 4.48;																		// Magic constant from the LabView FFT core correcting only dynamic range
	uint16_t i; //, j;
	float biggest_spur = -300;
	double RSS = 0.0, mean = 0.0;
	
	// Initalizing pk_spurious variables
	fft_meas->pk_spurious_noise = -200.0;
	fft_meas->pk_spurious_freq = 0;
	
	for (i = 0; i < DC_BINS; i++)																						// Ignoring DC bins
		fft_data->noise_bins[i] = 0.0;
	for (i = DC_BINS; i < fft_data->fft_length; i++) {
		// Ignoring spread near the fundamental
		if ((i <= fft_meas->harmonics_freq[0] + FUND_BINS) && (i >= fft_meas->harmonics_freq[0] - FUND_BINS))			
			fft_data->noise_bins[i] = 0.0;
			
		else if((i <= fft_meas->harmonics_freq[1] + HARM_BINS) && (i >= fft_meas->harmonics_freq[1] - HARM_BINS))		// Ignoring spread near harmonics
			fft_data->noise_bins[i] = 0.0;
		
		else if((i <= fft_meas->harmonics_freq[2] + HARM_BINS) && (i >= fft_meas->harmonics_freq[2] - HARM_BINS))
			fft_data->noise_bins[i] = 0.0;

		else if((i <= fft_meas->harmonics_freq[3] + HARM_BINS) && (i >= fft_meas->harmonics_freq[3] - HARM_BINS))
			fft_data->noise_bins[i] = 0.0;
		
		else if((i <= fft_meas->harmonics_freq[4] + HARM_BINS) && (i >= fft_meas->harmonics_freq[4] - HARM_BINS))
			fft_data->noise_bins[i] = 0.0;

		else if((i <= fft_meas->harmonics_freq[5] + HARM_BINS) && (i >= fft_meas->harmonics_freq[5] - HARM_BINS))
			fft_data->noise_bins[i] = 0.0;

		else {		
			// Root Sum Square = RSS for noise calculations
			fft_data->noise_bins[i] = fft_data->fft_magnitude_corrected[i];
			RSS += pow(((double)(fft_data->fft_magnitude_corrected[i] / (2.0*SQRT_2))), 2.0);
			
			// Average bin noise
			mean += fft_data->fft_magnitude_corrected[i];
			
			// Peak spurious amplitude
			if(fft_data->fft_magnitude_corrected[i] > fft_meas->pk_spurious_noise) {
				fft_meas->pk_spurious_noise = fft_data->fft_magnitude_corrected[i];
				fft_meas->pk_spurious_freq = i;
			}
			
		}	
	}	
	mean /= (double)(fft_data->fft_length);
	
	// RSS of FFT spectrum without DC, Fund. and Harmonics
	RSS = sqrt(RSS);
	RSS = RSS * 2.0 * SQRT_2;
	
	// Peak spurious amplitude = Highest amplitude excluding DC, the Fundamental and the Harmonics
	fft_meas->pk_spurious_noise = 20.0 * log10f(1.0 / fft_meas->pk_spurious_noise);
	
	// Looking for the biggest spur among harmonics
	for(i = 1 ; i < 6 ; i++) {
		if (fft_meas->harmonics_mag_dbfs[i] > biggest_spur)
			biggest_spur = fft_meas->harmonics_mag_dbfs[i]; 
	}	
	// Looking for the biggest spur among harmonics and pk_spurious_noise
	if(biggest_spur > fft_meas->pk_spurious_noise)
		biggest_spur = fft_meas->pk_spurious_noise;
	
	// Spurious Free Dynamic Range SFDR related to the carrer = biggest spur - the Fundamental, [dBc] - Decibels related to the carrier
	fft_meas->SFDR_dbc = biggest_spur - fft_meas->harmonics_mag_dbfs[0];

	// Spurious Free Dynamic Range SFDR related to the full-scale = biggest spur - full-scale [dBFS], where full-scale is 0 dBFS
	fft_meas->SFDR_dbfs = biggest_spur;
	
	// Average bin noise = Mean value of FFT spectrum excluding DC, the Fundamental and the Harmonics
	fft_meas->average_bin_noise = (float)(20.0 * log10(mean));
	
	// DR = 1 / RSS of FFT spectrum without DC, Fund. and Harmonics + Magic constant from the Labview FFT core
	fft_meas->DR = (20.0 * log10f(1.0 / (float)(RSS))) + LW_DR_correction_const;
	
	// SNR = Power of the fundamental / RSS of FFT spectrum without DC, Fund. and Harmonics
	fft_meas->SNR = 20.0 * log10f((fft_meas->harmonics_power[0]) / (RSS));
	
	// SINAD
	fft_meas->SINAD = -10.0 * log10f(powf(10.0, (fabs(fft_meas->SNR))*(-1.0) / 10.0) + powf(10.0, fabs(fft_meas->THD)*(-1.0) / 10.0));
	
	// ENOB - Effective number of bits
	fft_meas->ENOB = (fft_meas->SINAD - 1.67 + fabs(fft_meas->harmonics_mag_dbfs[0])) / 6.02; 
}

/**
 * Convert dBFS to volts in Pk-Pk
 * @param vref - reference voltage in volts
 * @param *fft_meas - fft_meas structure
 */
float static dbfs_to_volts(float vref, float value)
{
	return ( 2 * vref * powf(10.0, value / 20.0) );
}
