/*
 * 轮腿式平衡车姿态与运动控制软件 — 硬实时 ISR 入口（软著 01 专用摘录）
 * 自 cm7_0_isr.c 抽取 1ms/5ms/20ms 控制节拍。
 */
#include "zf_common_headfile.h"
void pit0_ch0_isr(void)
{
    pit_isr_flag_clear(PIT_CH0);
    dip_switch_motor_sync_from_hw();
    EKF_UpData();
    EKF_V_UPData();
    if (steer_yaw_request_pending)
    {
        if (!spin_enable)
        {
            steer_set_target_yaw(steer_yaw_request_deg);
            steer_yaw_request_pending = 0;
        }
    }
    yaw_hold_poweron_request_if_needed();
    pid_ctrl_Run();
    Left_Motor_Pwm = -motor_value.receive_left_speed_data;
    Right_Motor_Pwm = motor_value.receive_right_speed_data;
}
void pit0_ch1_isr(void)
{
    pit_isr_flag_clear(PIT_CH1);
    leg_control();
    buzzer_beep_poll();
}
void pit0_ch10_isr(void)
{
    pit_isr_flag_clear(PIT_CH10);
    jump_control();
    Left_Motor_Speed = -motor_value.receive_left_speed_data;
    Right_Motor_Speed = motor_value.receive_right_speed_data;
    car_speed = (Left_Motor_Speed + Right_Motor_Speed) / 2;
}
