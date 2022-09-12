/**
 * @file user_defination.h
 * @author mzy (mzy8329@163.com)
 * @brief  定义了各线程运行频率及电机相关结构体
 * @version 0.1
 * 
 * @copyright Copyright (c) 2022
 * 
 */


#ifndef USER_DEFINATION_H
#define USER_DEFINATION_H

#include <stdio.h>
#include "usart.h"
#include "gpio.h"


#define CAN_SERIAL_FREQUENCY 500
#define UART_SERIAL_FREQUENCY 80

#define USE_MOTOR_NUM 3
#define M2006_CURRENT_MAX 10000
#define M2006_KT 0.18


extern int MOTOR_IS_POS[USE_MOTOR_NUM];
extern float MOTOR_MIN[USE_MOTOR_NUM];
extern float MOTOR_MAX[USE_MOTOR_NUM];

int _write(int fd, char *pBuffer, int size);
int fputc(int ch, FILE *stream);


typedef struct
{
    float Kp;
    float Ki;
    float Kd;
    float err[2];

    float output;
    float outputMax;
    float outputMin;
}PID_s;


void PID_Cal(PID_s *pid, float ref, float fdb);

/**
 * @brief 包含了电机的反馈量，控制量和输出量
 * 
 */
typedef struct
{
    uint8_t id;
    struct
    {
        float angle;
        float rpm;
        float torque; 
        int msg_cnt;
    }FdbData;

    struct
    {
        float axisAngleAll;
        float axisRpm;
    }AxisData;
    
    struct
    {
        float angleAll;
        int round;
        float angleOffset;
    }globalAngle;

    struct
    {
        float angle_ref;
        float rpm_ref;
        float current_ref;   
    }RefData;   

    struct
    {
        PID_s angle_pid;
        PID_s rpm_pid;
    }PID;

    float current_out;
}DJI_Motor_s;


void MOTOR_INIT();


extern DJI_Motor_s motor[4] ;

#endif // !USER_DEFINATION_H