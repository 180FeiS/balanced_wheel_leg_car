/*
 * 运动控制核初始化摘录（自 init.c）
 */
#include "zf_common_headfile.h"
void all_init_cm7_0_control(void)
{
    buzzer_init();
    imu660rc_init(IMU660RC_QUARTERNION_DISABLE);
    servo_init();
    small_driver_uart_init();
    pid_ctrl_Init();
    EKF_Init();
    key_init(10);
    pit_ms_init(PIT_CH0, 1);
    pit_ms_init(PIT_CH1, 5);
    pit_ms_init(PIT_CH10, 20);
    dip_switch_motor_sync_from_hw();
}
