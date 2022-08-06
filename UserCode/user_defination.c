#include "user_defination.h"


UART_HandleTypeDef *UART_Mavlink = &huart6;

int _write (int fd, char *pBuffer, int size)  
{  
    for (int i = 0; i < size; i++)  
    {  
        while((UART8->SR&0X40)==0);          //等待上一次串口数据发送完成  
        UART8->DR = (uint8_t) pBuffer[i];    //写DR,串口1将发送数据
    }  
    return size;  
}

void MOTOR_INIT()
{
    for(int i = 0; i < 4; i++)
    {
        motor[i].id = i;
        motor[i].CtrlData.currentOut = 0;
        motor[i].CtrlData.currentMax = 8000;
    }
}

DJI_Motor_s motor[4];