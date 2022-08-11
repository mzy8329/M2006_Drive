/**
 * @file user_init.c
 * @author mzy (mzy8329@163.com)
 * @brief 调用USER_INIT()，初始化三个线程
 * @version 0.1
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "user_init.h"

/**
 * @brief 闪灯线程，方便观察是否上电
 * 
 */
void lightTask()
{
    for(;;)
    {
        HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_14);
        osDelay(500);

    }
}

/**
 * @brief 注册闪灯线程
 * 
 */
void lightTaskStart()
{
    osThreadDef(light, lightTask, osPriorityNormal, 0, 512);
	osThreadCreate(osThread(light), NULL);
}

/**
 * @brief 初始化电机，注册闪灯线程，uart通信线程，can通信线程
 * 
 */
void USER_INIT()
{
    MOTOR_INIT();

    lightTaskStart();
    CanSerialTaskStart();
    SerialTaskStart();
}


