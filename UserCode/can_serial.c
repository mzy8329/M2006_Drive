/**
 * @file can_serial.c
 * @author mzy (mzy8329@163.com)
 * @brief can通信线程，直接获取电机反馈信息以及控制电机
 * @version 0.1
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "can_serial.h"


/**
 * @brief 一次性发送四个电机的控制电流
 * 
 * @param motor0_Iq {int16_t}
 * @param motor1_Iq {int16_t}
 * @param motor2_Iq {int16_t}
 * @param motor3_Iq {int16_t}
 */
void CanTransmitMotor0123(int16_t motor0_Iq, int16_t motor1_Iq, int16_t motor2_Iq, int16_t motor3_Iq)
{
    CAN_TxHeaderTypeDef TxMessage;

    TxMessage.DLC=0x08;
	TxMessage.StdId= 0x200;
	TxMessage.IDE=CAN_ID_STD;
	TxMessage.RTR=CAN_RTR_DATA;

    uint8_t TxData[8] = {0};
    TxData[0] = (motor0_Iq >> 8);
	TxData[1] = motor0_Iq;
	TxData[2] = (motor1_Iq >> 8);
	TxData[3] = motor1_Iq;
	TxData[4] = (motor2_Iq >> 8);
	TxData[5] = motor2_Iq;
	TxData[6] = (motor3_Iq >> 8);
	TxData[7] = motor3_Iq;

	while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0) ;
	if(HAL_CAN_AddTxMessage(&hcan1,&TxMessage,TxData, 0)!=HAL_OK)
	{
		 Error_Handler();       
	}
}

/**
 * @brief 将can通信消息转化成电机信息
 * 
 * @param Motor 
 * @param CanData 
 */
void UpdataMotor(DJI_Motor_s *Motor, uint8_t *CanData)
{
    
    Motor->FdbData.angle = (int16_t)(CanData[0] << 8 | CanData[1]);
    Motor->AxisData.axisRpm = (int16_t)(CanData[2] << 8 | CanData[3]);
    Motor->FdbData.torque = (int16_t)(CanData[4] << 8 | CanData[5]);

    //M2006减速比为36
    Motor->FdbData.rpm = Motor->AxisData.axisRpm/36.0;

    //M2006转子转一圈（360）对应的返回值为8191
    static float angle_last[4] = {4000,4000,4000,4000};
    if(Motor->FdbData.angle - angle_last[Motor->id] > 5000)
    {
        Motor->globalAngle.round--;
    }
    if(Motor->FdbData.angle - angle_last[Motor->id] < -5000)
    {
        Motor->globalAngle.round++;
    }
    Motor->AxisData.axisAngleAll = (Motor->FdbData.angle + Motor->globalAngle.round * 8191.0 - Motor->globalAngle.angleOffset)/8191.0*360;
    Motor->globalAngle.angleAll = Motor->AxisData.axisAngleAll/36.0;

    angle_last[Motor->id] = Motor->FdbData.angle;
}


/**
 * @brief can通信Callback
 * 
 * @param hcan 
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef RxHeader;
    uint8_t CanReceiveData[8] = {0};
    if(HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, CanReceiveData) != HAL_OK)
    {   
        Error_Handler();     
    }

    int id = RxHeader.StdId - 0x201;
    if(id >= 0 && id < USE_MOTOR_NUM)
    {
        //获取绝对编码器的初始值
        if(motor[id].FdbData.msg_cnt < 20)
        {
            motor[id].globalAngle.angleOffset = (int16_t)(CanReceiveData[0] << 8 | CanReceiveData[1]);
            motor[id].FdbData.msg_cnt++;
        }
        else
        {
            UpdataMotor(&motor[id], CanReceiveData);
        }
    }
    else
    {
        Error_Handler(); 
    }
}


/**
 * @brief 初始化can通信
 * 
 */
void CAN_INIT()
{
    CAN_FilterTypeDef sFilterConfig;
    sFilterConfig.FilterBank = 0;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterIdHigh = 0x0000;
    sFilterConfig.FilterIdLow = 0x0000;
    sFilterConfig.FilterMaskIdHigh = 0x0000;
    sFilterConfig.FilterMaskIdLow = 0x0000;
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 14;

    HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
}

/**
 * @brief can通信线程，以固定频率给电机发送控制电流
 * 
 * @param argument 
 */
void CanSerialTask(void const *argument)
{
    CAN_INIT();
    osDelay(200);

    int i = 0;
    for(;;)
    {
        MotorCtrl();
        //位置保护
        for(int i = 0; i < USE_MOTOR_NUM; i++)
        {
            if(MOTOR_IS_POS[i])
            {
                if(motor[i].globalAngle.angleAll < MOTOR_MIN[i] || motor[i].globalAngle.angleAll > MOTOR_MAX[i])
                {
                    motor[i].current_out = 0;
                }
            }
        }
        if(i++ > 10)
        {
            printf("1:%.2f, 2:%.2f, 3:%.2f \n", motor[0].RefData.current_ref, motor[1].RefData.current_ref, motor[2].RefData.current_ref);
            i = 0;
        }
        
        CanTransmitMotor0123(motor[0].current_out, motor[1].current_out,  motor[2].current_out,  motor[3].current_out);
        // CanTransmitMotor0123(0, 0, 0, 0);

        osDelay(1000/(float)CAN_SERIAL_FREQUENCY);
    }

}

/**
 * @brief 注册can通信线程
 * 
 */
void CanSerialTaskStart()
{
    osThreadDef(CanSerial, CanSerialTask, osPriorityNormal, 0, 512);
	osThreadCreate(osThread(CanSerial), NULL);
}