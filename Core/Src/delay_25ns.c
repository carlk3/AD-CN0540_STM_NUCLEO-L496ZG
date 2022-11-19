/*
 * delay_100ns.c
 *
 *  Created on: Apr 25, 2022
 *      Author: carlk
 */

#include "main.h"
//#include <FreeRTOS.h>
//#include <task.h>
//
//#include <math.h> //TEST
#include <assert.h>

/* This implementation is not sharable. Only one task at a time can use it. */

/*
 * TIM3 is on APB1 which is clocked at 40 MHz ==> period 25 ns
 * ==> 1 us is 40 clocks
 */

#if 0
static TaskHandle_t htask;

void TIM3_PeriodElapsedCallback() {

//    	HAL_TIM_Base_Stop_IT(&htim3);
	LL_TIM_DisableIT_UPDATE(TIM3);
	LL_TIM_DisableCounter(TIM3);
	LL_TIM_ClearFlag_UPDATE(TIM3);

	/* The xHigherPriorityTaskWoken parameter must be initialized to pdFALSE
	 as it will get set to pdTRUE inside the interrupt safe API function if
	 a context switch is required. */
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	/* Send a notification directly to the task to which interrupt
	 processing is being deferred. */
	vTaskNotifyGiveFromISR(htask,  // The handle of the task to which
								   // the notification is being sent.
			&xHigherPriorityTaskWoken);

	/* Pass the xHigherPriorityTaskWoken value into portYIELD_FROM_ISR().
	 If xHigherPriorityTaskWoken was set to pdTRUE inside
	 vTaskNotifyGiveFromISR() then calling portYIELD_FROM_ISR() will
	 request a context switch. If xHigherPriorityTaskWoken is still
	 pdFALSE then calling portYIELD_FROM_ISR() will have no effect. */
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

static void delay_25nsIT(size_t multiple) {
	configASSERT(0 < multiple && multiple < 0xFFFF);
	/* Ensure this task does not already have a notification pending by calling
	 ulTaskNotifyTake() with the xClearCountOnExit parameter set to pdTRUE, and
	 a block time of 0 (don't block). */
	BaseType_t rc = ulTaskNotifyTake(pdTRUE, 0);
	configASSERT(!rc);

	htask = xTaskGetCurrentTaskHandle();

	LL_TIM_SetAutoReload(TIM3, multiple - 1);
	LL_TIM_DisableIT_UPDATE(TIM3);
	asm volatile("" ::: "memory");
	LL_TIM_GenerateEvent_UPDATE(TIM3); // Cause load of ARR
	asm volatile("" ::: "memory");
	LL_TIM_ClearFlag_UPDATE(TIM3);
	asm volatile("" ::: "memory");
	LL_TIM_EnableIT_UPDATE(TIM3);
	asm volatile("" ::: "memory");
	LL_TIM_EnableCounter(TIM3);
	asm volatile("" ::: "memory");

	/* Timeout 1 sec */
	uint32_t timeOut = 1000;
	/* Wait until master completes transfer or time out has occured. */
	rc = ulTaskNotifyTake(pdFALSE, pdMS_TO_TICKS(timeOut)); // Wait for notification from ISR
	configASSERT(rc);
}
#endif
static void delay_25nsSpin(size_t multiple) {
	assert(0 < multiple && multiple < 0xFFFF);
	LL_TIM_SetAutoReload(TIM3, multiple - 1);
	LL_TIM_DisableIT_UPDATE(TIM3);
	asm volatile("" ::: "memory");
	LL_TIM_GenerateEvent_UPDATE(TIM3); // Cause load of ARR
	asm volatile("" ::: "memory");
	LL_TIM_ClearFlag_UPDATE(TIM3);
	asm volatile("" ::: "memory");
	LL_TIM_EnableCounter(TIM3);
	asm volatile("" ::: "memory");
	while (!LL_TIM_IsActiveFlag_UPDATE(TIM3));  // wait for overflow
}
#if 0
void delay_25ns(size_t multiple) {
//	multiple = ceil(0.7 * multiple); //TEST
//	LL_GPIO_SetOutputPin(Trig_GPIO_Port, Trig_Pin);  //DEBUG
	if (multiple  < 10 * 40) {
		delay_25nsSpin(multiple);
	} else {
		delay_25nsIT(multiple);
	}
//	LL_GPIO_ResetOutputPin(Trig_GPIO_Port, Trig_Pin); //DEBUG
}
#endif
void udelay(size_t n) {
	delay_25nsSpin(n * 40);
}
