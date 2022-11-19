/*
 * console.c
 *
 *  Created on: Nov 3, 2021
 *      Author: carlk
 */

#include "main.h"
//
#include <string.h>
#include <stdbool.h>
//
#include "console.h"
//
#include "printf.h"

int __io_putchar(int ch) {
	// Wait until LPUART Transmit Data Register Empty Flag is set
	while (!LL_LPUART_IsActiveFlag_TXE(LPUART1))
//		if (!xPortIsInsideInterrupt())
//			vTaskDelay(pdMS_TO_TICKS(1));
		;
	if ('\n' == ch)
		LL_LPUART_TransmitData8(LPUART1, '\r');
	while (!LL_LPUART_IsActiveFlag_TXE(LPUART1))
//		if (!xPortIsInsideInterrupt())
//			vTaskDelay(pdMS_TO_TICKS(1));
		;
	LL_LPUART_TransmitData8(LPUART1, ch);
	return ch;
}

// For Marco Paland's printf
void _putchar(char character) {
	if ('\n' == character)
		__io_putchar('\r');
	__io_putchar(character);
}

volatile atomic_bool cnsl_buf_full;
volatile char cnsl_buf[16];
static size_t ix;

/**
 * @brief  Function called from LPUART IRQ Handler when RXNE flag is set
 *         Function is in charge of reading character received on USART RX line.
 * @param  None
 * @retval None
 */
void LPUART_CharReception_Callback() {

	while (LL_LPUART_IsActiveFlag_RXNE(LPUART1)) {
		uint8_t received_char = 0;

		/* Read Received character. RXNE flag is cleared by reading of RDR register */
		received_char = LL_LPUART_ReceiveData8(LPUART1);

		/* Echo received character on TX */
		if (received_char) {
			__io_putchar(received_char);
			if (received_char == '\r') {
				cnsl_buf[ix] = '\0';
				cnsl_buf_full = true;
			} else {
				/* Queue the character */
				if (ix < sizeof cnsl_buf - 1)
					cnsl_buf[ix++] = received_char;
			}
		}
	}
}

void cnsl_buf_clear() {
	ix = 0;
	cnsl_buf[ix] = '\0';
	cnsl_buf_full = false;
}
