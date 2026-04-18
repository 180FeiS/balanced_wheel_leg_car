/*********************************************************************************************************************
 * LoRa 遥控器独立模块（与 Menu / navigation 解耦）
 *
 * 约定：
 * - CM7_1：lora3a22 解析、边沿检测、写入 g_dualcore_blob.remote、投递 dualcore_ui_cmd 瞬时命令。
 * - CM7_0：1ms 节拍里在拨码同步之后调用 remote_control_apply_after_dip()；pid_ctrl_Run() 里混入遥控角速度差速。
 *
 * 硬件冲突必读：lora3a22 与 wireless_uart（VOFA 无线串口）在工程中默认共用 UART1 / P04_0~1。
 * 启用本模块后 lora3a22_init() 会覆盖无线串口中断回调，调试上位机串口与遥控器接收机勿同时接在同一路 UART。
 *********************************************************************************************************************/
#ifndef CODE_REMOTE_CONTROL_H_
#define CODE_REMOTE_CONTROL_H_

#include "zf_common_typedef.h"

/* 置 0 可整模块编译为空调用，便于无 LoRa 时减小耦合排查 */
#ifndef REMOTE_CONTROL_ENABLE
#define REMOTE_CONTROL_ENABLE  (1)
#endif

/* 摇杆机械中位对应的“原始 ADC 值”（解析前 lora 帧里的 int16）。
 * 代码里会做：offset = raw - REMOTE_JOYSTICK_CENTER，再死区与映射。
 * 你当前接收机静止约为 0，故默认 0；若某批次中位在 2000 附近，在工程选项里加
 * -DREMOTE_JOYSTICK_CENTER=2000 或在此处改默认值即可。 */
#ifndef REMOTE_JOYSTICK_CENTER
#define REMOTE_JOYSTICK_CENTER  (0)
#endif

/* 摇杆在中位附近的死区（原始 ADC 单位） */
#ifndef REMOTE_JOYSTICK_DEADBAND
#define REMOTE_JOYSTICK_DEADBAND  (120)
#endif

/* 认为“离开中位”的最大行程，用于把偏移线性映射到内部标准量程（可按上位机最大偏移微调） */
#ifndef REMOTE_JOYSTICK_HALF_SPAN
#define REMOTE_JOYSTICK_HALF_SPAN  (2000)
#endif

/* 内部速度命令量程：左摇杆前后映射到 [-REMOTE_SPEED_CMD_MAX, +REMOTE_SPEED_CMD_MAX] */
#ifndef REMOTE_SPEED_CMD_MAX
#define REMOTE_SPEED_CMD_MAX  (2000)
#endif

/* 内部角速度命令量程：右摇杆左右映射到同量级，再由 CM7_0 换算为 steer_cmd */
#ifndef REMOTE_YAW_RATE_CMD_MAX
#define REMOTE_YAW_RATE_CMD_MAX  (2000)
#endif

/* 左摇杆前后方向：若实际“前推”对应 ADC 减小，置为 -1 */
#ifndef REMOTE_LEFT_Y_SIGN
#define REMOTE_LEFT_Y_SIGN  (1)
#endif

/* 右摇杆左右方向：若“右推”希望车向右转对应负号，置为 -1 */
#ifndef REMOTE_RIGHT_X_SIGN
#define REMOTE_RIGHT_X_SIGN  (1)
#endif

/* 连续量失效：主循环若干次未收到完整帧则置 offline（主循环越快该值应略大） */
#ifndef REMOTE_LINK_STALE_LOOPS
#define REMOTE_LINK_STALE_LOOPS  (80u)
#endif

/* 左摇杆下压切换 motor_arm 的最小间隔（按 LoRa 完整帧计数），抑制按键抖动导致 0/1 连翻 */
#ifndef REMOTE_MOTOR_ARM_TOGGLE_COOLDOWN_FRAMES
#define REMOTE_MOTOR_ARM_TOGGLE_COOLDOWN_FRAMES  (8u)
#endif

/* motor_user_speed_cmd = speed_norm * REMOTE_SPEED_TO_MOTOR_SCALE（与工程里常用 500~1500 量级匹配，实车标定） */
#ifndef REMOTE_SPEED_TO_MOTOR_SCALE
#define REMOTE_SPEED_TO_MOTOR_SCALE  (0.5f)
#endif

/* 遥控角速度差速混入：steer_cmd 增量系数（越大越灵敏，注意 STEER_CMD_MAX 限幅） */
#ifndef REMOTE_YAW_TO_STEER_GAIN
#define REMOTE_YAW_TO_STEER_GAIN  (0.65f)
#endif

/* 横滚平衡拨键允许的车速上限（用 CM7_0 发布的 |car_speed|，与 control.c 中速度单位一致） */
#ifndef REMOTE_ROLL_TOGGLE_SPEED_MAX
#define REMOTE_ROLL_TOGGLE_SPEED_MAX  (120.0f)
#endif

#if REMOTE_CONTROL_ENABLE

#if defined(CY_CORE_CM7_1)
void remote_control_init(void);
void remote_control_task(void);
#endif

#if defined(CY_CORE_CM7_0)
void remote_control_apply_after_dip(void);
float remote_control_get_yaw_steer_mix(void);
#endif

#else /* REMOTE_CONTROL_ENABLE */

#if defined(CY_CORE_CM7_1)
static inline void remote_control_init(void) {}
static inline void remote_control_task(void) {}
#endif
#if defined(CY_CORE_CM7_0)
static inline void remote_control_apply_after_dip(void) {}
static inline float remote_control_get_yaw_steer_mix(void) { return 0.0f; }
#endif

#endif /* REMOTE_CONTROL_ENABLE */

#endif /* CODE_REMOTE_CONTROL_H_ */
