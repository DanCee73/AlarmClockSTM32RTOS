/*
 * debug.c
 *
 *  Created on: Apr 10, 2018
 *      Author: eric
 */
#include "debug.h"

#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"


extern UART_HandleTypeDef huart2;


/* FOR DEBUGGING PURPOSES
 * USE ARDUINO / UART DEVICE
 * BAUD: 115200 -- 8N1
 * PA2 : TX pin for USART2
 */

int __io_putchar(int ch)
{
	HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);
	return ch;
}

/*
 * Write a string/char to USART2
 */
int _write(char *ptr)
{
	int DataIdx;
	int len;
	len = strlen(ptr);
	for (DataIdx = 0; DataIdx < len; DataIdx++)
	{
		__io_putchar( *ptr++ );
	}
	return len;
}

/*
 * Write a string/char to USART2 and include CRLF
 */
int _writeln(char *ptr)
{
	int DataIdx;
	int len;
	len = strlen(ptr);
	for (DataIdx = 0; DataIdx < len; DataIdx++)
	{
		__io_putchar( *ptr++ );
	}
	__io_putchar('\n');
	__io_putchar('\r');
	return len;
}
