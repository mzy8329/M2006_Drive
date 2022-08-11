/**
 * @file can_serial.h
 * @author mzy (mzy8329@163.com)
 * @brief can通信线程，直接获取电机反馈信息以及控制电机
 * @version 0.1
 * 
 * @copyright Copyright (c) 2022
 * 
 */


#ifndef CAN_SERIAL_H
#define CAN_SERIAL_H

#include "can.h"
#include "cmsis_os.h"

#include "user_defination.h"

void CanSerialTaskStart();

#endif // !CAN_SERIAL_H