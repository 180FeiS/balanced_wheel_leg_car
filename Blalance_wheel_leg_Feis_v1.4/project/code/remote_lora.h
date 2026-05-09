/*
 * LORA3A22 遥控：CM7_1 接收，整理后写入双核共享区 remote 快照。
 * 串口引脚默认与 zf_device_lora3a22.h 一致（UART_1 / P04_0 / P04_1），可在包含驱动头之前重定义宏。
 */
#ifndef CODE_REMOTE_LORA_H_
#define CODE_REMOTE_LORA_H_

#include "zf_common_typedef.h"
/* 不直接包含 dualcore_shared.h，避免 step_detection→headfile→remote_lora 循环导致类型未完整 */
struct dualcore_remote_to_ctrl;

/* 摇杆中位为 0，绝对值裁剪上限（与遥控端量程一致，便于后续自定义映射） */
#ifndef REMOTE_LORA_JOYSTICK_ABS_MAX
#define REMOTE_LORA_JOYSTICK_ABS_MAX  (2000)
#endif

/* 验证用：左摇杆 left_y → 线速度用户指令的满偏幅值（±REMOTE_LORA_VALIDATE_SPEED_ABS_MAX） */
#ifndef REMOTE_LORA_VALIDATE_SPEED_ABS_MAX
#define REMOTE_LORA_VALIDATE_SPEED_ABS_MAX  (1500.0f)
#endif
/* 1：前进方向与摇杆相反时整体取反 */
#ifndef REMOTE_LORA_VALIDATE_SPEED_INVERT
#define REMOTE_LORA_VALIDATE_SPEED_INVERT  0
#endif

/* 右摇杆 right_x：目标偏航角速度满偏（°/s），与 control.c 中 STEER_RATE_TARGET_MAX_DPS 同量级便于实车调参 */
#ifndef REMOTE_LORA_VALIDATE_STEER_RATE_MAX_DPS
#define REMOTE_LORA_VALIDATE_STEER_RATE_MAX_DPS  (200.0f)
#endif

/* 右摇杆按下：lora3a22 协议为 key[1]（key0 左杆键、key1 右杆键、key2/key3 左右侧向键，见逐飞 E6_13_lora_remote_control_demo） */
#ifndef REMOTE_LORA_KEY_INDEX_RIGHT_STICK
#define REMOTE_LORA_KEY_INDEX_RIGHT_STICK  (1u)
#endif
/* 右杆键上升沿触发的原地自旋圈数与方向（spin_task_start；dir>0 为沿 yaw 正方向，默认 2 圈） */
#ifndef REMOTE_LORA_SPIN_TURNS_VALIDATE
#define REMOTE_LORA_SPIN_TURNS_VALIDATE  (2.0f)
#endif
#ifndef REMOTE_LORA_SPIN_DIR
#define REMOTE_LORA_SPIN_DIR  (1)
#endif

/* 左上/左侧向键（逐飞协议 key2）：上升沿切换横滚平衡 on/off，语义同 dualcore DUALCORE_UI_CMD_ROLL_BALANCE_TOGGLE */
#ifndef REMOTE_LORA_KEY_INDEX_ROLL_BALANCE
#define REMOTE_LORA_KEY_INDEX_ROLL_BALANCE  (2u)
#endif
/* 右上/右侧向键（逐飞 key3）：上升沿触发跳跃，语义同 DUALCORE_UI_CMD_JUMP（须 jump_is_allowed 且当前未在跳） */
#ifndef REMOTE_LORA_KEY_INDEX_JUMP
#define REMOTE_LORA_KEY_INDEX_JUMP  (3u)
#endif

/* 左边两路拨码：与逐飞 lora3a22 左边 switch_key[0/1] 一致；0/1 为协议里“上拉为 1/按下为 0”等逻辑，与实物箭头一致可改下标/取反 */
/* 计划语义：非回放=1、回放档=0 → 沿 1→0 调 Nag_Begin_Replay。若你实物“停在上面=0、常态=1”，与计划一致。 */
/* 若需改为 0→1 进回放，置 REMOTE_LORA_REPLAY_ON_SW0_RISING=1。首次联网先同步前态，不产假沿。 */
#ifndef REMOTE_LORA_LEFT_SWITCH0_INDEX
#define REMOTE_LORA_LEFT_SWITCH0_INDEX  (0u)
#endif
#ifndef REMOTE_LORA_LEFT_SWITCH1_INDEX
#define REMOTE_LORA_LEFT_SWITCH1_INDEX  (1u)
#endif
#ifndef REMOTE_LORA_REPLAY_ON_SW0_RISING
#define REMOTE_LORA_REPLAY_ON_SW0_RISING  (0u)
#endif

/* MENU_INPUT_REMOTE_MENU_FIRST==1 时：switch_key 下标 REMOTE_LORA_DEBUG_MODE_SWITCH_INDEX 与宏一致电平 = 板载按键/拨码调试 */
#ifndef REMOTE_LORA_DEBUG_MODE_SWITCH_INDEX
#define REMOTE_LORA_DEBUG_MODE_SWITCH_INDEX  (3u)
#endif
#ifndef REMOTE_LORA_LOCAL_KEYS_ACTIVE_LEVEL
#define REMOTE_LORA_LOCAL_KEYS_ACTIVE_LEVEL  (0u)
#endif

void remote_lora_init(void);
void remote_lora_update_from_driver_and_publish(void);

/* 与 Menu.h 中 MENU_INPUT_REMOTE_MENU_FIRST 一致：1 表示遥控菜单/遥控数据对控制核生效 */
uint8 remote_lora_is_remote_menu_enabled(void);

/* 最近一次已发布快照（CM7_1 侧调试可读；主通路为 dualcore_remote_publish）；实参为 dualcore_remote_to_ctrl_t* */
void remote_lora_get_last_published(struct dualcore_remote_to_ctrl *out);

#if defined(CY_CORE_CM7_0)
/* 验证用：CM7_0 读 remote；左拨码 0/1 录/停/回放见 REMOTE_LORA_LEFT_SWITCH* */
void remote_lora_apply_validate_motor(void);
#endif

#endif /* CODE_REMOTE_LORA_H_ */
