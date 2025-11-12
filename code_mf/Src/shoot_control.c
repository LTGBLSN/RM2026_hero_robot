//
// Created by 21481 on 2025/3/22.
//

#include "main.h"
#include "cmsis_os.h"
#include "CAN_receive.h"
#include "pid.h"
#include "gimbal_motor_control.h"
#include "shoot_control.h"

void shoot_control()
{
    while (1)
    {

        //此任务不阻塞


        shoot_speed_compute();//拨弹盘目标速度控制
        shoot_stop_check();//拨弹盘堵转反转
        //拨弹盘的速度闭环在不阻塞的gimbal_motor_control里面



        osDelay(1);
    }
}




void shoot_speed_compute()
{
    if(rc_s1 == 2 | rc_s0 == 2)
    {
        SHOOT_3510_ID7_GIVEN_SPEED = 0 ;

    }
    else
    {
        if(rc_ch4 < -300 | mouse_press_l == 1)
        {
            SHOOT_3510_ID7_GIVEN_SPEED = SHOOT_TURN_ON_SPEED ;
        }
        else
        {
            SHOOT_3510_ID7_GIVEN_SPEED = 0 ;
        }
    }

}

void shoot_stop_check()
{
    if(0)
    {
        //如果卡住了

    }
}





void shoot_pid_control()
{
    //shoot
    SHOOT_3510_ID7_GIVEN_CURRENT = shoot_3510_id7_speed_pid_loop(SHOOT_3510_ID7_GIVEN_SPEED);
}







//shoot
void shoot_2006_id3_speed_pid_init(void)
{
    static fp32 shoot_2006_id3_speed_kpkikd[3] = {SHOOT_2006_ID3_SPEED_PID_KP,SHOOT_2006_ID3_SPEED_PID_KI,SHOOT_2006_ID3_SPEED_PID_KD};
    PID_init(&shoot_3510_ID7_speed_pid, PID_POSITION, shoot_2006_id3_speed_kpkikd, SHOOT_2006_ID3_SPEED_PID_OUT_MAX, SHOOT_2006_ID3_SPEED_PID_KI_MAX);

}

int16_t shoot_3510_id7_speed_pid_loop(float shoot_3510_ID7_speed_set_loop)
{
    PID_calc(&shoot_3510_ID7_speed_pid, motor_can1_data[6].speed_rpm , shoot_3510_ID7_speed_set_loop);
    int16_t shoot_3510_id7_given_current_loop = (int16_t)(shoot_3510_ID7_speed_pid.out);

    return shoot_3510_id7_given_current_loop ;

}





