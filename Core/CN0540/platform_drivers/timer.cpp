/**************************************************************************//**
 *   @file    timer.cpp
 *   @brief   Implementation of Timer MBED Platform Driver Interfaces
*******************************************************************************
 * Copyright (c) 2021 Analog Devices, Inc.
 * All rights reserved.
 *
 * This software is proprietary to Analog Devices, Inc. and its licensors.
 * By using this software you agree to the terms of the associated
 * Analog Devices Software License Agreement.
******************************************************************************/

/*****************************************************************************/
/***************************** Include Files *********************************/
/*****************************************************************************/

#include <stdio.h>
#include <mbed.h>

//Platform Drivers need to be C-compatible to work with other drivers
#ifdef __cplusplus
extern "C"
{
#endif	//__cplusplus

#include "timer.h"
#include "error.h"
#include "timer_extra.h"
using namespace std::chrono;

/*****************************************************************************/
/******************** Variables and User Defined Data Types ******************/
/*****************************************************************************/

/*****************************************************************************/
/************************** Functions Declarations ***************************/
/*****************************************************************************/

/*****************************************************************************/
/************************** Functions Definitions ****************************/
/*****************************************************************************/

/**
 * @brief Initialize the timer peripheral
 * @param desc[in, out] - The Timer descriptor
 * @param init_param[in] - Structure that contains Timer Initialization Parameters
 * @return SUCCESS in case of success, FAILURE otherwise
 */
int32_t timer_init(struct timer_desc **desc,
		   struct timer_init_param *param)
{
	timer_desc *new_timer;			// Pointer to timer descriptor
	mbed_timer_desc *mbed_desc;		// Pointer to mbed timer descriptor
	mbed::Timer *timer;				// Pointer to new Timer instance

	if (desc && param) {
		new_timer = (timer_desc *)malloc(sizeof(timer_desc));
		if (!new_timer) {
			goto err_new_desc;
		}
		new_timer->freq_hz = param->freq_hz;
		new_timer->id = param->id;
		new_timer->load_value = param->load_value;

		mbed_desc = (mbed_timer_desc *)malloc(sizeof(mbed_timer_desc));
		if (!mbed_desc) {
			goto err_mbed_desc;
		}

		timer  = new Timer();
		if (!timer) {
			goto err_timer_instance;
		}

		mbed_desc->timer  = timer;
		new_timer->extra = (mbed_timer_desc *) mbed_desc;

		*desc = new_timer;
		return SUCCESS;
	}
	return FAILURE;

err_timer_instance:
	free(mbed_desc);
	free(new_timer);
err_mbed_desc:
	free(new_timer);
err_new_desc:
	// Nothing to free

	return FAILURE;
}


/**
* @brief Start Timer.
* @param desc[in] - The Timer Descriptor.
* @return SUCCESS in case of success, FAILURE otherwise.
*/
int32_t timer_start(struct timer_desc *desc)
{
	mbed::Timer *timer;

	if (!desc) {
		return FAILURE;
	}
	timer = (Timer *)(((mbed_timer_desc *)(desc->extra))->timer);
	if (!timer) {
		return FAILURE;
	}
	timer->start();

	return SUCCESS;
}


/**
* @brief Stop Timer.
* @param desc[in] - The Timer Descriptor.
* @return SUCCESS in case of success, FAILURE otherwise.
*/
int32_t timer_stop(struct timer_desc *desc)
{
	mbed::Timer *timer;

	if (!desc) {
		return FAILURE;
	}
	timer = (Timer *)(((mbed_timer_desc *)(desc->extra))->timer);
	if (!timer) {
		return FAILURE;
	}
	timer->stop();

	return SUCCESS;
}


/**
* @brief Release all the resources allocated by Timer.
* @param desc[in] - The Timer Descriptor.
* @return SUCCESS in case of success, FAILURE otherwise
*/
int32_t timer_remove(struct timer_desc *desc)
{
	if (desc) {
		// Free the timer object
		if ((Timer *)(((mbed_timer_desc *)(desc->extra))->timer)) {
			delete((Timer *)(((mbed_timer_desc *)(desc->extra))->timer));
		}
		// Free the extra descriptor object
		if ((mbed_timer_desc *)(desc->extra)) {
			free((mbed_timer_desc *)(desc->extra));
		}
		// Free the Timer descriptor
		free(desc);

		return SUCCESS;
	}
	return FAILURE;
}


/**
* @brief Get the elapsed time in nanoseconds.
* @param desc[in] - The Timer descriptor
* @param elapsed_time[out] - Pointer where the elapsed time value is stored
* @return SUCCESS in case of success, FAILURE otherwise
*/
int32_t get_elapsed_time_in_nsec(struct timer_desc *desc,
				 uint64_t *elapsed_time)
{
	mbed::Timer *timer;

	if (!desc) {
		return FAILURE;
	}

	timer = (Timer *)(((mbed_timer_desc *)(desc->extra))->timer);
	if (!timer) {
		return FAILURE;
	}
	*elapsed_time = duration_cast<nanoseconds>(timer->elapsed_time()).count();

	return SUCCESS;
}

#ifdef __cplusplus
}
#endif // __cplusplus
