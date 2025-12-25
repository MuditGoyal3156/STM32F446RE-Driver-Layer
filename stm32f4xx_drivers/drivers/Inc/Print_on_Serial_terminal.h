/*
 * Print_on_Serial_terminal.h
 *
 *  Created on: Dec 25, 2025
 *      Author: mudit
 */

#ifndef INC_PRINT_ON_SERIAL_TERMINAL_H_
#define INC_PRINT_ON_SERIAL_TERMINAL_H_



#define UART_BAUDRATE    115200



void uart2_write(int ch);
void uart2_tx_init(void);

#endif /* INC_PRINT_ON_SERIAL_TERMINAL_H_ */
