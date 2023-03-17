/**
 * @file uart_serial.c
 * @author mzy (mzy8329@163.com)
 * @brief 进行串口消息的收发，传输电机的控制量和反馈量
 * @version 0.1
 * 
 * @copyright Copyright (c) 2022
 * 
 */


#include "uart_serial.h"

#define HEAD_LENGTH 2
#define PAYLOAD_LENGTH 13
#define BAG_LENGTH (HEAD_LENGTH + PAYLOAD_LENGTH)


uint8_t HEADER[2] = {0x44, 0x22};
uint8_t RxBuffer[BAG_LENGTH*2] = {0};

/**
 * @brief 电机反馈包，发送给上位机
 * 
 */
typedef union
{
    uint8_t data[BAG_LENGTH];
    struct 
    {
        uint8_t header[HEAD_LENGTH];
        union
        {
            uint8_t payload[PAYLOAD_LENGTH];
            struct
            {
                uint8_t id;
                float angle_fdb;
                float rpm_fdb;
                float torque_fdb;
            }__attribute__((packed));
        };
    }__attribute__((packed));
}__attribute__((packed)) SEND_Bag_u;

/**
 * @brief 电机控制包，从上位机接受
 * 
 */
typedef union
{
    uint8_t data[BAG_LENGTH];
    struct 
    {
        uint8_t header[HEAD_LENGTH];
        union
        {
            uint8_t payload[PAYLOAD_LENGTH];
            struct
            {
                uint8_t id;
                float angle_ref;
                float rpm_ref;
                float current_ref;
            }__attribute__((packed));
        };
    }__attribute__((packed));
}__attribute__((packed)) RECV_Bag_u;



/**
 * @brief 将某个电机的反馈量发送给上位机
 * 
 * @param Motor
 */
void UartTransmit(DJI_Motor_s *Motor)
{
    SEND_Bag_u UartBag;

    UartBag.header[0] = HEADER[0];
    UartBag.header[1] = HEADER[1];

    UartBag.id = Motor->id;
    UartBag.angle_fdb = Motor->globalAngle.angleAll;
    UartBag.rpm_fdb = Motor->FdbData.rpm;
    UartBag.torque_fdb = Motor->FdbData.torque;


    HAL_UART_Transmit(&huart1, &UartBag, BAG_LENGTH, 10);
}



/**
 * @brief uart的Callback函数
 * 
 * @param huart 
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    //usart6为上下位机通信所用串口
    if(huart == &huart1)
    {
        for (int i = 0; i < sizeof(RxBuffer); ++i)
        {
            RECV_Bag_u tempBag = *(RECV_Bag_u*) (void *) (&(RxBuffer[i]));
            if(tempBag.header[0] == HEADER[0] && tempBag.header[1] == HEADER[1])
            {
                motor[tempBag.id].RefData.angle_ref = tempBag.angle_ref;
                motor[tempBag.id].RefData.rpm_ref = tempBag.rpm_ref;
                motor[tempBag.id].RefData.current_ref = tempBag.current_ref;         
                break;
            }
        }

        //发和收每次都是两个包，降低频率换取通信准确性
        HAL_UART_Receive_IT(&huart1, &RxBuffer, 2*BAG_LENGTH);
    }


}


/**
 * @brief uart初始化，开启uart6的中断
 * 
 */
void UART_INIT()
{
    HAL_UART_Receive_IT(&huart1, &RxBuffer, 2*BAG_LENGTH);
    printf("uart_init \n");
}

/**
 * @brief uart线程，以固定频率发送电机的反馈信号
 * 
 */
void SerialTask()
{
    UART_INIT();

    for(;;)
    {
        for(int id = 0; id < USE_MOTOR_NUM; id++)
        {
            UartTransmit(&motor[id]);
            osDelay(1000 / (float)UART_SERIAL_FREQUENCY/(float)USE_MOTOR_NUM);
        }
        
    }
}

/**
 * @brief 开启uart线程
 * 
 */
void SerialTaskStart(mavlink_con)
{
    osThreadDef(Serial, SerialTask, osPriorityNormal, 0, 512);
	osThreadCreate(osThread(Serial), NULL);
}