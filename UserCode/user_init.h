/**
 * @file user_init.h
 * @author mzy (mzy8329@163.com)
 * @brief 只需要在main中包含这一个头文件，调用USER_INIT()，初始化三个线程
 * @version 0.1
 * 
 * @copyright Copyright (c) 2022
 * 
 */


#ifndef USER_INIT_H
#define USER_INIT_H

#include "user_defination.h"
#include "can_serial.h"
#include "uart_serial.h"

void USER_INIT();


#endif // !USER_INIT_H


