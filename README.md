# M2006_Drive
> [上位机代码地址: motor_serial](https://github.com/mzy8329/motor_serial "motor_serial")

该仓库为大疆A板(STM32F427)和大疆C板(STM32F407)的下位机程序，支持上位机通过串口通信，将其转为Can信号并发送给电机，实现对电机的控制。

当前支持至多4个电机，所用Can通信协议与C610电调所用协议一致。
如需更改电机数量或Can通信协议，可在*UserCode/can_serial.c:CanTransmitMotor0123*处进行修改

串口通信协议*UserCode/uart_serial.c*中定义，目前支持电机的位置，速度和电流控制。

电机限位在*UserCode/user_defination.c*中定义，可按需进行更改或在*UserCode/user_defination.c:MOTOR_IS_POS*中更改其控制类型