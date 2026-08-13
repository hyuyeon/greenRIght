/*
 * uart_debug.h
 *
 *  Created on: 2026. 7. 3.
 *      Author: 한국전파진흥협회
 */

#ifndef INC_UART_DEBUG_H_
#define INC_UART_DEBUG_H_

void _send_char(char ch);
void _send_string(char *p);
void _uart_printf(char *fmt, ...);

#endif /* INC_UART_DEBUG_H_ */
