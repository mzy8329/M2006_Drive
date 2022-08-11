/**
 * @file user_defination.c
 * @author mzy (mzy8329@163.com)
 * @brief  定义了各线程运行频率及电机相关结构体
 * @version 0.1
 * 
 * @copyright Copyright (c) 2022
 * 
 */


#include "user_defination.h"


UART_HandleTypeDef *UART_Mavlink = &huart6;

//linux下 gcc对应的printf重定向，将uart8作为输出，debug用
int _write (int fd, char *pBuffer, int size)  
{  
    for (int i = 0; i < size; i++)  
    {  
        while((UART8->SR&0X40)==0);          //等待上一次串口数据发送完成  
        UART8->DR = (uint8_t) pBuffer[i];    //写DR,串口8将发送数据
    }  
    return size;  
}

//windows MDK对应的printf重定向
int fputc(int ch, FILE *stream)
{
	while (HAL_UART_Transmit(&UART_Printf_Config_huart, (uint8_t *)&ch, 1, 0xffff) == HAL_BUSY);
	return ch;
}


/**
 * @brief 初始化所有电机
 * 
 */
void MOTOR_INIT()
{
    for(int i = 0; i < 4; i++)
    {
        motor[i].id = i;
        motor[i].globalAngle.round = 0;
        motor[i].RefData.angle_ref = 0;
        motor[i].RefData.rpm_ref = 0;
        motor[i].RefData.current_ref = 0;

        motor[i].PID.angle_pid.Kp = 5.5;
        motor[i].PID.angle_pid.Ki = 0.35;
        motor[i].PID.angle_pid.Kd = 25;
        motor[i].PID.angle_pid.output = 0;
        motor[i].PID.angle_pid.outputMax = 15000;
        motor[i].PID.angle_pid.outputMin = -15000;
        motor[i].PID.angle_pid.err[0] = 0;
        motor[i].PID.angle_pid.err[1] = 0;
        
        motor[i].PID.rpm_pid.Kp = 2.5;
        motor[i].PID.rpm_pid.Ki = 0.08;
        motor[i].PID.rpm_pid.Kd = 1.0;
        motor[i].PID.rpm_pid.output = 0;
        motor[i].PID.rpm_pid.outputMax = 4000;
        motor[i].PID.rpm_pid.outputMin = -4000;        
        motor[i].PID.rpm_pid.err[0] = 0;
        motor[i].PID.rpm_pid.err[1] = 0;
    }
}


/**
 * @brief 进行PID计算，将输出结果保存到pid.output
 * 
 * @param pid {PID_s}
 * @param ref {float}
 * @param fdb {float}
 */
void PID_Cal(PID_s *pid, float ref, float fdb)
{
    float err_now = ref - fdb;
    pid->output += pid->Kp*(err_now - pid->err[1]) + pid->Ki*err_now + pid->Kd*(err_now - 2*pid->err[0] + pid->err[1]);
    if(pid->output > pid->outputMax)
    {
        pid->output = pid->outputMax;
    }
    if(pid->output < pid->outputMin)
    {
        pid->output = pid->outputMin;
    } 

    pid->err[1] = pid->err[0];
    pid->err[0] = err_now;
}


/**
 * @brief 将电机期望值转化为电流输出值
 * 
 */
void MotorCtrl()
{
    for(int i = 0; i < 4; i++)
    {
        //位置伺服
        if(motor[i].RefData.angle_ref!=-1)
        {
            PID_Cal(&motor[i].PID.angle_pid, motor[i].RefData.angle_ref * 36.0, motor[i].AxisData.axisAngleAll);
            PID_Cal(&motor[i].PID.rpm_pid, motor[i].PID.angle_pid.output, motor[i].AxisData.axisRpm);
            motor[i].current_out = motor[i].PID.rpm_pid.output;
        }
        //速度伺服
        else if(motor[i].RefData.rpm_ref!=-1)
        {
            PID_Cal(&motor[i].PID.rpm_pid, motor[i].RefData.rpm_ref * 36.0, motor[i].AxisData.axisRpm);
            motor[i].current_out = motor[i].PID.rpm_pid.output;
        }
        //电流控制
        else if(motor[i].RefData.current_ref!=-1)
        {
            if(motor[i].RefData.current_ref>motor[i].PID.rpm_pid.outputMax)
            {
                motor[i].current_out = motor[i].PID.rpm_pid.outputMax;
            }
            else if(motor[i].RefData.current_ref<motor[i].PID.rpm_pid.outputMin)
            {
                motor[i].current_out = motor[i].PID.rpm_pid.outputMin;
            }
            else
            {
                motor[i].current_out = motor[i].RefData.current_ref;
            }
        }
        //无控制信号传入
        else
        {
            motor[i].current_out = 0;
        }
    }
}



DJI_Motor_s motor[4];