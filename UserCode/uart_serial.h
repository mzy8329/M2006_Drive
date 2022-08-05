#ifndef UART_SERIAL_H
#define UART_SERIAL_H

#include "usart.h"
#include "cmsis_os.h"

#include "user_defination.h"
#include "mavlink.h"



static mavlink_system_t mavlink_system = 
{
    1,  // System ID
    1    // Component ID
};


typedef struct
{
    UART_HandleTypeDef *huart;

    struct
    {
        uint8_t RxCh;
        mavlink_channel_t chan;
        mavlink_message_t msg;
        mavlink_status_t statu;
    }Rx;
    
    struct
    {
        mavlink_channel_t chan;
        mavlink_message_t msg;
    }Tx;
    
    
    
}MavlinkMsg_s;

extern MavlinkMsg_s mavlinkMsg;

void MavlinkSerialTaskStart();


#endif // !UART_SERIAL_H