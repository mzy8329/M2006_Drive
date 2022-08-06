#include "uart_serial.h"

#define HEAD_LENGTH 2
#define PAYLOAD_LENGTH 17
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
                float current_fdb;
                float current_out;
            }__attribute__((packed));
        };
    }__attribute__((packed));
}__attribute__((packed)) UART_Bag_u;

UART_Bag_u UartBag;


void UartTransmit(DJI_Motor_s *Motor)
{
    UartBag.id = Motor->id;
    UartBag.angle_fdb = Motor->globalAngle.angleAll;
    UartBag.rpm_fdb = Motor->FdbData.rpm;
    UartBag.current_fdb = Motor->FdbData.current;
    UartBag.current_out = Motor->CtrlData.currentOut;


    HAL_UART_Transmit(&huart6, &UartBag, BAG_LENGTH, 10);
}




void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    for (int i = 0; i < sizeof(RxBuffer); ++i) {
        UART_Bag_u tempBag = *(UART_Bag_u*) (void *) (&(RxBuffer[i]));
        if(
                tempBag.header[0] == HEADER[0] &&
                tempBag.header[1] == HEADER[1]
                ){
            motor[tempBag.id].CtrlData.currentRef = tempBag.current_out;
            printf("current_out: %f", tempBag.current_out);
            break;
        }
    }

    HAL_UART_Receive_IT(&huart6, &RxBuffer, 2*BAG_LENGTH);
}

void UART_INIT()
{
    UartBag.header[0] = HEADER[0];
    UartBag.header[1] = HEADER[1];

    HAL_UART_Receive_IT(&huart6, &RxBuffer, 2*BAG_LENGTH);
}

void SerialTask()
{
    UART_INIT();

    for(;;)
    {
        for(int id = 0; id < USE_MOTOR_NUM; id++)
        {
            UartTransmit(&motor[id]);
            osDelay( 1000 / (float)UART_SERIAL_FREQUENCY/(float)USE_MOTOR_NUM);
        }
        
    }
}

void SerialTaskStart(mavlink_con)
{
    osThreadDef(Serial, SerialTask, osPriorityNormal, 0, 512);
	osThreadCreate(osThread(Serial), NULL);
}