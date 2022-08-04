#include "uart_serial.h"

mavlink_system_t mavlink_system = 
{
    42,  // System ID
    1    // Component ID
};

mavlink_message_t UartReceiveData;
