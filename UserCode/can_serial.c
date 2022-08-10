#include "can_serial.h"



void CanTransmitMotor(int16_t motor0_Iq, int16_t motor1_Iq, int16_t motor2_Iq, int16_t motor3_Iq)
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

void UpdataMotor(DJI_Motor_s *Motor, uint8_t *CanData)
{
    Motor->FdbData.angle = (int16_t)(CanData[0] << 8 | CanData[1]);
    Motor->AxisData.axisRpm = (int16_t)(CanData[2] << 8 | CanData[3]);
    Motor->FdbData.torque = (int16_t)(CanData[4] << 8 | CanData[5]);

    Motor->FdbData.rpm = Motor->AxisData.axisRpm/36.0;

    static float angle_last = 0;
    if(Motor->FdbData.angle - angle_last > 5000)
    {
        Motor->globalAngle.round--;
    }
    if(Motor->FdbData.angle - angle_last < -5000)
    {
        Motor->globalAngle.round++;
    }
    Motor->AxisData.axisAngleAll = (Motor->FdbData.angle + Motor->globalAngle.round * 8191.0 - Motor->globalAngle.angleOffset)/8191.0*360;
    Motor->globalAngle.angleAll = Motor->AxisData.axisAngleAll/36.0;

    angle_last = Motor->FdbData.angle;
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef RxHeader;
    uint8_t CanReceiveData[8] = {0};
    if(HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, CanReceiveData) != HAL_OK)
    {   
        Error_Handler();     
    }

    int id = RxHeader.StdId - 0x201;
    if(id >= 0 && id <= 4)
    {
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
}



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

void CanSerialTask(void const *argument)
{
    CAN_INIT();
    osDelay(200);
    uint32_t PreviousWakeTime = osKernelSysTick();

    int i = 0;
    int up = 1;
    int a = 0;
    for(;;)
    {
#if !USE_IMPE_CTRL        
        // motor[0].RefData.angle_ref = -1;
        // motor[0].RefData.rpm_ref = -1;
        // motor[0].RefData.current_ref = -1;
        // if(++i>5)
        // {
        //     // if(up && a < 100)
        //     // {
        //     //     a++;
        //     // }
        //     // else if(up && a >= 100)
        //     // {
        //     //     up = 0;
        //     // }
        //     // else if(!up && a > -100)
        //     // {
        //     //     a--;
        //     // }
        //     // else if(!up && a <= -100)
        //     // {
        //     //     up = 1;
        //     // }
        //     i = 0;
        //     // printf("%f,%f \n", motor[0].globalAngle.angleAll,motor[0].RefData.angle_ref);
        //     printf("%f,%f \n", motor[0].RefData.current_ref, motor[0].FdbData.torque);
        // }
        MotorCtrl();
        if(motor[0].globalAngle.angleAll < -240 || motor[0].globalAngle.angleAll > 1)
        {
            motor[0].current_out = 0;
        }

#endif
        
#if USE_IMPE_CTRL
        IMPE_CTRL();
#endif
        CanTransmitMotor(motor[0].current_out, motor[1].current_out,  motor[2].current_out,  motor[3].current_out);
        
        // osDelayUntil(&PreviousWakeTime, 1000/(float)CAN_SERIAL_FREQUENCY);
        osDelay(1000/(float)CAN_SERIAL_FREQUENCY);
    }

}

void CanSerialTaskStart()
{
    osThreadDef(CanSerial, CanSerialTask, osPriorityNormal, 0, 512);
	osThreadCreate(osThread(CanSerial), NULL);
}