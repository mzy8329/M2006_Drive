#ifndef USER_DEFINATION_H
#define USER_DEFINATION_H

#include <stdio.h>
#include "usart.h"


#define USE_MOTOR_NUM 1
#define M2006_CURRENT_MAX 10000
#define M2006_KT 0.18

#define CAN_SERIAL_FREQUENCY 500

typedef struct
{
    /* data */
    struct
    {
        float angle;
        float rpm;
        float current; 
        int msg_cnt;
    }FdbData;

    struct
    {
        float angleAll;
        int round;
        float angleOffset;
    }globalAngle;

    struct
    {
        float currentRef;
        float currentMax;
        float currentOut;
    }CtrlData;
    
}DJI_Motor_s;

extern DJI_Motor_s motor[4] ;

#endif // !USER_DEFINATION_H
