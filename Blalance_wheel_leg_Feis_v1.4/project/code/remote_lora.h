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

/* 验证用：左摇杆 left_y 映射到速度的满偏幅值（±REMOTE_LORA_VALIDATE_SPEED_ABS_MAX） */
#ifndef REMOTE_LORA_VALIDATE_SPEED_ABS_MAX
#define REMOTE_LORA_VALIDATE_SPEED_ABS_MAX  (1500.0f)
#endif
/* 1：前进方向与摇杆相反时整体取反 */
#ifndef REMOTE_LORA_VALIDATE_SPEED_INVERT
#define REMOTE_LORA_VALIDATE_SPEED_INVERT  0
#endif

/* 左杆横向：目标偏航角速度满偏（°/s），与 STEER_RATE_TARGET_MAX_DPS 同量级便于实车调参 */
#ifndef REMOTE_LORA_VALIDATE_STEER_RATE_MAX_DPS
#define REMOTE_LORA_VALIDATE_STEER_RATE_MAX_DPS  (200.0f)
#endif

/* 右摇杆按键在 key[0..3] 中的下标（按实际遥控改） */
#ifndef REMOTE_LORA_KEY_INDEX_RIGHT_STICK
#define REMOTE_LORA_KEY_INDEX_RIGHT_STICK  (2u)
#endif
/* 验证用：右杆键上升沿触发的自旋圈数与方向（dir>0 沿 yaw 正方向） */
#ifndef REMOTE_LORA_SPIN_TURNS_VALIDATE
#define REMOTE_LORA_SPIN_TURNS_VALIDATE  (2.0f)
#endif
#ifndef REMOTE_LORA_SPIN_DIR
#define REMOTE_LORA_SPIN_DIR  (1)
#endif

void remote_lora_init(void);
void remote_lora_update_from_driver_and_publish(void);

/* 与 Menu.h 中 MENU_INPUT_REMOTE_MENU_FIRST 一致：1 表示遥控菜单/遥控数据对控制核生效 */
uint8 remote_lora_is_remote_menu_enabled(void);

/* 最近一次已发布快照（CM7_1 侧调试可读；主通路为 dualcore_remote_publish）；实参为 dualcore_remote_to_ctrl_t* */
void remote_lora_get_last_published(struct dualcore_remote_to_ctrl *out);

#if defined(CY_CORE_CM7_0)
/* 验证用：CM7_0 读共享区 remote，左杆纵向→速度、横向→目标偏航角速度；左杆键翻转 Motor_Switch；右杆键自旋 */
void remote_lora_apply_validate_motor(void);
#endif

#endif /* CODE_REMOTE_LORA_H_ */
