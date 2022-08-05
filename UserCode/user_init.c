#include "user_init.h"



void lightTask()
{
    for(;;)
    {
        HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_14);
        osDelay(500);
    }
}


void lightTaskStart()
{
    osThreadDef(light, lightTask, osPriorityNormal, 0, 512);
	osThreadCreate(osThread(light), NULL);
}


void USER_INIT()
{
    MOTOR_INIT();

    lightTaskStart();
    CanSerialTaskStart();
    MavlinkSerialTaskStart();
}


