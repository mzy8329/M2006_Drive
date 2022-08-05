#ifndef USER_DEFINATION_H
#define USER_DEFINATION_H

#include <stdio.h>
#include "usart.h"
#include "gpio.h"


#define USE_MOTOR_NUM 1
#define M2006_CURRENT_MAX 10000
#define M2006_KT 0.18

#define CAN_SERIAL_FREQUENCY 500
#define UART_SERIAL_FREQUENCY 200

int _write(int fd, char *pBuffer, int size);
int fputc(int ch, FILE *stream);

typedef struct
{
    /* data */
    uint8_t id;
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

extern UART_HandleTypeDef *UART_Mavlink;

void MOTOR_INIT();

#endif // !USER_DEFINATION_H
