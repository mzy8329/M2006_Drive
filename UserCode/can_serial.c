#include "can_serial.h"



void CanTransmitMotor(int16_t motor0_Iq, int16_t motor1_Iq, int16_t motor2_Iq, int16_t motor3_Iq)
{
    CAN_TxHeaderTypeDef TxMessage;

    TxMessage.DLC=0x08;
	TxMessage.StdId= 0x200;
	TxMessage.IDE=CAN_ID_STD;
	TxMessage.RTR=CAN_RTR_DATA;

    uint8_t TxData[8] = {0};
    TxData[0] = (uint8_t)(motor0_Iq >> 8);
	TxData[1] = (uint8_t)motor0_Iq;
	TxData[2] = (uint8_t)(motor1_Iq >> 8);
	TxData[3] = (uint8_t)motor1_Iq;
	TxData[4] = (uint8_t)(motor2_Iq >> 8);
	TxData[5] = (uint8_t)motor2_Iq;
	TxData[6] = (uint8_t)(motor3_Iq >> 8);
	TxData[7] = (uint8_t)motor3_Iq;

	while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0) ;
	if(HAL_CAN_AddTxMessage(&hcan1,&TxMessage,TxData, 0)!=HAL_OK)
	{
		 Error_Handler();       
	}
}

void UpdataMotor(DJI_Motor_s *Motor, uint8_t *CanData)
{
    Motor->FdbData.angle = (int16_t)(CanData[0] << 8 | CanData[1]);
    Motor->FdbData.rpm = (int16_t)(CanData[2] << 8 | CanData[3]);
    Motor->FdbData.current = (int16_t)(CanData[4] << 8 | CanData[5])/M2006_KT;

    static float angle_last = 0;
    if(Motor->FdbData.angle - angle_last > 5000)
    {
        Motor->globalAngle.round--;
    }
    if(Motor->FdbData.angle - angle_last < -5000)
    {
        Motor->globalAngle.round++;
    }
    Motor->globalAngle.angleAll = (Motor->FdbData.angle + Motor->globalAngle.round * 8191.0 - Motor->globalAngle.angleOffset)/8191.0/36.0 * 360.0;

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

    for(;;)
    {
        CanTransmitMotor(motor[0].CtrlData.currentOut, motor[1].CtrlData.currentOut,  motor[2].CtrlData.currentOut,  motor[3].CtrlData.currentOut);

        osDelayUntil(&PreviousWakeTime, 1000/(float)CAN_SERIAL_FREQUENCY);
    }

}