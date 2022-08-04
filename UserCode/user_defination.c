#include <stdio.h>
#include "usart.h"

#include "user_defination.h"

UART_HandleTypeDef *UART_Printf_Config_huart = &huart6;

int fputc(int ch, FILE *stream)
{
	while (HAL_UART_Transmit(&UART_Printf_Config_huart, (uint8_t *)&ch, 1, 0xffff) == HAL_BUSY);
	return ch;
}

DJI_Motor_s motor[4];