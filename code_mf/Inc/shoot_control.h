//
// Created by 21481 on 2025/3/22.
//

#ifndef BUBING_RM2025_SHOOT_CONTROL_H
#define BUBING_RM2025_SHOOT_CONTROL_H



#include "main.h"
#include "cmsis_os.h"
#include "can.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"

#include <stdio.h>
#include "board_LED.h"
#include "uart_printf.h"
#include "uart_sent.h"
#include "bsp_rc.h"
#include "remote_control.h"
#include "get_rc.h"
#include "bsp_can.h"
#include "CAN_receive.h"
#include "jy61p.h"
#include "pid.h"
#include "chassis_motor_control.h"
#include "gimbal_motor_control.h"
#include <math.h>



#define SHOOT_2006_ID3_SPEED_PID_KP        5.0f
#define SHOOT_2006_ID3_SPEED_PID_KI        0.1f
#define SHOOT_2006_ID3_SPEED_PID_KD        0.0f
#define SHOOT_2006_ID3_SPEED_PID_OUT_MAX   16000.0f
#define SHOOT_2006_ID3_SPEED_PID_KI_MAX    4000.0f


#define SHOOT_TURN_ON_SPEED  2000
#define SHOOT_TURN_OFF_SPEED (-5000)

#define SHOOT_SPEED_CHECK_TIME 100
#define SHOOT_STOP_CHECK_SPEED 100  //+-500,卡弹检测速度区间，区间内都视为卡弹
#define SHOOT_TURN_OFF_TIME 100


#define SHOOT_MAX_STOP_TIME 500 //ms

void shoot_speed_compute();
void shoot_stop_check();
void shoot_pid_control();

void shoot_2006_id3_speed_pid_init(void);
int16_t shoot_3510_id7_speed_pid_loop(float shoot_3510_ID7_speed_set_loop);




#endif //BUBING_RM2025_SHOOT_CONTROL_H
