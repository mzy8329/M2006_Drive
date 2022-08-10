#include "uart_serial.h"

#define HEAD_LENGTH 2
#define PAYLOAD_LENGTH 13
#define BAG_LENGTH (HEAD_LENGTH + PAYLOAD_LENGTH)


uint8_t HEADER[2] = {0x44, 0x22};

uint8_t RxBuffer[BAG_LENGTH*2] = {0};

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




void UartTransmit(DJI_Motor_s *Motor)
{
    SEND_Bag_u UartBag;

    UartBag.header[0] = HEADER[0];
    UartBag.header[1] = HEADER[1];

    UartBag.id = Motor->id;
    UartBag.angle_fdb = Motor->globalAngle.angleAll;
    UartBag.rpm_fdb = Motor->FdbData.rpm;
    UartBag.torque_fdb = Motor->FdbData.torque;


    HAL_UART_Transmit(&huart6, &UartBag, BAG_LENGTH, 10);
}




void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    
    if(huart == &huart6)
    {
        for (int i = 0; i < sizeof(RxBuffer); ++i)
        {
            RECV_Bag_u tempBag = *(RECV_Bag_u*) (void *) (&(RxBuffer[i]));
            if(tempBag.header[0] == HEADER[0] && tempBag.header[1] == HEADER[1])
            {
                motor[tempBag.id].RefData.angle_ref = tempBag.angle_ref;
                motor[tempBag.id].RefData.rpm_ref = tempBag.rpm_ref;
                motor[tempBag.id].RefData.current_ref = tempBag.current_ref;         

                // printf("%f \n", motor[0].RefData.angle_ref);
                break;
            }
        }

        HAL_UART_Receive_IT(&huart6, &RxBuffer, 2*BAG_LENGTH);
    }


}

void UART_INIT()
{
    HAL_UART_Receive_IT(&huart6, &RxBuffer, 2*BAG_LENGTH);
    printf("uart_init \n");
}

void SerialTask()
{
    UART_INIT();

    for(;;)
    {
        for(int id = 0; id < USE_MOTOR_NUM; id++)
        {
            UartTransmit(&motor[id]);
            // osDelay( 1000 / (float)UART_SERIAL_FREQUENCY/(float)USE_MOTOR_NUM);
            osDelay(1000 / (float)UART_SERIAL_FREQUENCY/(float)USE_MOTOR_NUM);
        }
        
    }
}

void SerialTaskStart(mavlink_con)
{
    osThreadDef(Serial, SerialTask, osPriorityNormal, 0, 512);
	osThreadCreate(osThread(Serial), NULL);
}