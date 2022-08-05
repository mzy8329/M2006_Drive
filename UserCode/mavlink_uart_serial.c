#include "uart_serial.h"

MavlinkMsg_s mavlinkMsg;

void MavlinkUartTransmin(MavlinkMsg_s *MavlinkMsg, DJI_Motor_s *Motor)
{
    mavlink_msg_dji_motor_pack(mavlink_system.sysid, mavlink_system.compid, &MavlinkMsg->Tx.msg, Motor->id, Motor->globalAngle.angleAll, Motor->FdbData.rpm, Motor->FdbData.current, Motor->CtrlData.currentOut);
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    mavlink_dji_motor_t mavlink_motor;
    if(huart == mavlinkMsg.huart)
    {
        if (mavlink_parse_char(mavlinkMsg.Rx.chan, mavlinkMsg.Rx.RxCh, &mavlinkMsg.Rx.msg, &mavlinkMsg.Rx.statu))
        {
            switch (mavlinkMsg.Rx.msg.msgid)
            {
            case 42:
                mavlink_msg_dji_motor_decode(&mavlinkMsg.Rx.msg, &mavlink_motor);
                motor[mavlink_motor.id].CtrlData.currentRef = mavlink_motor.current_out;
                break;
            
            default:
                break;
            }
        }
    }
    HAL_UART_Receive_IT(mavlinkMsg.huart, &mavlinkMsg.Rx.RxCh, 1);
}

void Mavlink_INIT()
{
    mavlinkMsg.huart = UART_Mavlink;
    mavlinkMsg.Rx.chan = 0;
    mavlinkMsg.Tx.chan = 1;
}

void MavlinkSerialTask()
{
    Mavlink_INIT();

    for(;;)
    {
        for(int id = 0; id < USE_MOTOR_NUM; id++)
        {
            MavlinkUartTransmin(&mavlinkMsg, &motor[id]);
            osDelay( 1000 / (float)UART_SERIAL_FREQUENCY/(float)USE_MOTOR_NUM);
        }
        
    }
}

void MavlinkSerialTaskStart(mavlink_con)
{
    osThreadDef(MavlinkSerial, MavlinkSerialTask, osPriorityNormal, 0, 512);
	osThreadCreate(osThread(MavlinkSerial), NULL);
}