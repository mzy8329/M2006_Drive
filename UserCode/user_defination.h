#ifndef USER_DEFINATION_H
#define USER_DEFINATION_H

#include <stdio.h>
#include "usart.h"
#include "gpio.h"


#define CAN_SERIAL_FREQUENCY 500
#define UART_SERIAL_FREQUENCY 80

#define USE_IMPE_CTRL 0

#define USE_MOTOR_NUM 1
#define M2006_CURRENT_MAX 10000
#define M2006_KT 0.18


int _write(int fd, char *pBuffer, int size);
int fputc(int ch, FILE *stream);


#if !USE_IMPE_CTRL
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
#endif


typedef struct
{
    /* data */
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
    
#if !USE_IMPE_CTRL
    struct
    {
        PID_s angle_pid;
        PID_s rpm_pid;
    }PID;
#endif

#if USE_IMPE_CTRL
    struct
    {
        float Md;
        float Dd;
        float Kd;

        float Mq;
        float Cq;
        float gq;
    }IMPE;
    
#endif


    float current_out;
}DJI_Motor_s;



extern DJI_Motor_s motor[4] ;




void MOTOR_INIT();

#endif // !USER_DEFINATION_H
