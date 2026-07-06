#ifndef CODE_REMOTE_LORA_H_
#define CODE_REMOTE_LORA_H_
#include "zf_common_typedef.h"
struct dualcore_remote_to_ctrl;
#ifndef REMOTE_LORA_JOYSTICK_ABS_MAX
#define REMOTE_LORA_JOYSTICK_ABS_MAX  (2000)
#endif
#ifndef REMOTE_LORA_VALIDATE_SPEED_ABS_MAX
#define REMOTE_LORA_VALIDATE_SPEED_ABS_MAX  (1500.0f)
#endif
#ifndef REMOTE_LORA_VALIDATE_SPEED_INVERT
#define REMOTE_LORA_VALIDATE_SPEED_INVERT  0
#endif
#ifndef REMOTE_LORA_VALIDATE_STEER_RATE_MAX_DPS
#define REMOTE_LORA_VALIDATE_STEER_RATE_MAX_DPS  (200.0f)
#endif
#ifndef REMOTE_LORA_KEY_INDEX_RIGHT_STICK
#define REMOTE_LORA_KEY_INDEX_RIGHT_STICK  (1u)
#endif
#ifndef REMOTE_LORA_SPIN_TURNS_VALIDATE
#define REMOTE_LORA_SPIN_TURNS_VALIDATE  (2.0f)
#endif
#ifndef REMOTE_LORA_SPIN_DIR
#define REMOTE_LORA_SPIN_DIR  (1)
#endif
#ifndef REMOTE_LORA_KEY_INDEX_ROLL_BALANCE
#define REMOTE_LORA_KEY_INDEX_ROLL_BALANCE  (2u)
#endif
#ifndef REMOTE_LORA_KEY_INDEX_JUMP
#define REMOTE_LORA_KEY_INDEX_JUMP  (3u)
#endif
#ifndef REMOTE_LORA_LEFT_SWITCH0_INDEX
#define REMOTE_LORA_LEFT_SWITCH0_INDEX  (0u)
#endif
#ifndef REMOTE_LORA_LEFT_SWITCH1_INDEX
#define REMOTE_LORA_LEFT_SWITCH1_INDEX  (1u)
#endif
#ifndef REMOTE_LORA_REPLAY_ON_SW0_RISING
#define REMOTE_LORA_REPLAY_ON_SW0_RISING  (0u)
#endif
#ifndef REMOTE_LORA_DEBUG_MODE_SWITCH_INDEX
#define REMOTE_LORA_DEBUG_MODE_SWITCH_INDEX  (3u)
#endif
#ifndef REMOTE_LORA_LOCAL_KEYS_ACTIVE_LEVEL
#define REMOTE_LORA_LOCAL_KEYS_ACTIVE_LEVEL  (0u)
#endif
void remote_lora_init(void);
void remote_lora_update_from_driver_and_publish(void);
uint8 remote_lora_is_remote_menu_enabled(void);
void remote_lora_get_last_published(struct dualcore_remote_to_ctrl *out);
#if defined(CY_CORE_CM7_0)
void remote_lora_apply_validate_motor(void);
#endif
#endif
