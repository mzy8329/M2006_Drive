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



void MOTOR_INIT()
{
    for(int i = 0; i < 4; i++)
    {
        motor[i].id = i;
        motor[i].globalAngle.round = 0;
        motor[i].RefData.angle_ref = 0;
        motor[i].RefData.rpm_ref = 0;
        motor[i].RefData.current_ref = 0;

        motor[i].PID.angle_pid.Kp = 0.76;
        motor[i].PID.angle_pid.Ki = 0.02;
        motor[i].PID.angle_pid.Kd = 0;
        motor[i].PID.angle_pid.output = 0;
        motor[i].PID.angle_pid.outputMax = 15000;
        motor[i].PID.angle_pid.outputMin = -15000;
        motor[i].PID.angle_pid.err[0] = 0;
        motor[i].PID.angle_pid.err[1] = 0;
        
        motor[i].PID.rpm_pid.Kp = 1.1;
        motor[i].PID.rpm_pid.Ki = 0.00039;
        motor[i].PID.rpm_pid.Kd = 0;
        motor[i].PID.rpm_pid.output = 0;
        motor[i].PID.rpm_pid.outputMax = 8000;
        motor[i].PID.rpm_pid.outputMin = -8000;        
        motor[i].PID.rpm_pid.err[0] = 0;
        motor[i].PID.rpm_pid.err[1] = 0;
    }
}


void MotorCtrl()
{

    for(int i = 0; i < 4; i++)
    {
        if(motor[i].RefData.angle_ref!=-1)
        {
            PID_Cal(&motor[i].PID.angle_pid, motor[i].RefData.angle_ref * 36.0, motor[i].AxisData.axisAngleAll);
            PID_Cal(&motor[i].PID.rpm_pid, motor[i].PID.angle_pid.output, motor[i].AxisData.axisRpm);
            motor[i].current_out = motor[i].PID.rpm_pid.output;
        }
        else if(motor[i].RefData.rpm_ref!=-1)
        {
            PID_Cal(&motor[i].PID.rpm_pid, motor[i].RefData.rpm_ref * 36.0, motor[i].AxisData.axisRpm);
            motor[i].current_out = motor[i].PID.rpm_pid.output;
        }
        else if(motor[i].RefData.current_ref!=-1)
        {
            if(motor[i].RefData.current_ref>8000)
            {
                motor[i].current_out = 8000;
            } 
            else if(motor[i].RefData.current_ref<-8000)
            {
                motor[i].current_out = -8000;   
            }
            else
            {
                motor[i].current_out = motor[i].RefData.current_ref;
            }
        }
        else
        {
            motor[i].current_out = 0;
        }
    }
}

DJI_Motor_s motor[4];