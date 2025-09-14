//
// Created by 21481 on 2025/3/16.
//

#include <stdio.h>
#include <string.h>
#include "main.h"
#include "cmsis_os.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"
#include "uart_printf.h"
#include "uart_sent.h"
#include "bsp_rc.h"
#include "remote_control.h"
#include "CAN_receive.h"
#include "jy61p.h"
#include "chassis_motor_control.h"
#include "auto_aim.h"






void uart_sent_debug()
{
    while (1)
    {
        usart6_printf("%d,%f,%d \r\n",motor_can1_data[4].speed_rpm,PITCH_3510_ID5_GIVEN_SPEED,PITCH_3510_ID5_GIVEN_CURRENT);



        osDelay(5);




    }

}


void aim_uart_sent()
{
    while (1)
    {
        //2025.8.30完成了串口接收上位机发过来的数据，差下位机发送TF给上位机了，在这接着完成

        osDelay(5);
        osDelay(1);
    }
}




