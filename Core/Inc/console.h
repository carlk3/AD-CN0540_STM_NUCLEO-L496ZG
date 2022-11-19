/*
 * console.h
 *
 *  Created on: Nov 3, 2021
 *      Author: carlk
 */

#ifndef INC_CONSOLE_H_X_
#define INC_CONSOLE_H_X_

#include <stdatomic.h>

int __io_putchar(int ch);
void LPUART_CharReception_Callback();
void console_init();
void cnsl_buf_clear();

extern volatile atomic_bool cnsl_buf_full;
extern volatile char cnsl_buf[16];

#endif /* INC_CONSOLE_H_X_ */
