#ifndef CODE_INTERFACES_CONTROL_PORT_H_
#define CODE_INTERFACES_CONTROL_PORT_H_
/*
 * 运动控制软件对外 API（供导航/GPS/人机交互模块调用）。
 * 软著 01 主归属 control.c；其他模块申报材料仅保留本头文件声明。
 */
#include "zf_common_typedef.h"
/* 速度目标：由导航模块计算后供速度环使用 */
float Nag_GetControlSpeedTarget(void);
/* 航向/转向 */
void steer_request_target_yaw(float target_yaw_deg);
void steer_request_relative_yaw(float delta_deg);
void steer_set_target_yaw(float target_yaw_deg);
void steer_task_stop(void);
/* 自旋 */
void spin_task_start(float turns, int8 dir);
void spin_set_rate_max_dps(float rate_dps);
void spin_task_stop(void);
/* 跳跃 */
extern uint8 jump_flag;
uint8 jump_is_allowed(void);
void jump_stop(void);
void Nag_NotifyStairJumpDone(void);
/* 速度基准 */
extern float motor_user_speed_cmd;
extern float run_launch_speed;
extern float speed_target_effective;
extern uint8 roll_balance_en;
/* 遥控门控 */
uint8 remote_lora_nav_allows_heading_override(void);
uint8 remote_lora_nav_allows_spin_request(void);
extern volatile float remote_lora_steer_rate_cmd_dps;
extern volatile uint8 remote_lora_steer_snapshot_valid;
/* 周期入口（ISR 调用） */
void pid_ctrl_Init(void);
void pid_ctrl_Run(void);
void leg_control(void);
void jump_control(void);
#endif
