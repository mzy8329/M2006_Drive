/**
 * @file uart_serial.h
 * @author mzy (mzy8329@163.com)
 * @brief 进行串口消息的收发，传输电机的控制量和反馈量
 * @version 0.1
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef UART_SERIAL_H
#define UART_SERIAL_H

#include "usart.h"
#include "cmsis_os.h"

#include "user_defination.h"


void SerialTaskStart();


#endif // !UART_SERIAL_H