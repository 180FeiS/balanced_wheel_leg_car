/*********************************************************************************************************************
 * 遥控验证（仅 CM7_0 编译）：左摇杆 left_y → motor_user_speed_cmd（±REMOTE_LORA_VALIDATE_SPEED_ABS_MAX），
 * left_x → remote_lora_steer_rate_cmd_dps（目标偏航角速度 °/s，pid_ctrl_Run 角速度环，松杆为 0、不拉固定航向）；
 * 左杆按键 key[0] 上升沿翻转 Motor_Switch；右杆键上升沿在满足门控时 spin_task_start；
 * 失控锁存时不允许遥控置 ON。
 *********************************************************************************************************************/
#include "dualcore_shared.h"
#include "remote_lora.h"
#include "control.h"
#include "small_driver_uart_control.h"

#if defined(CY_CORE_CM7_0)

static float remote_lora_validate_clamp_speed(float v)
{
    const float lim = REMOTE_LORA_VALIDATE_SPEED_ABS_MAX;
    if (v > lim)
    {
        return lim;
    }
    if (v < -lim)
    {
        return -lim;
    }
    return v;
}

static float remote_lora_clamp_steer_rate_dps(float w)
{
    const float lim = REMOTE_LORA_VALIDATE_STEER_RATE_MAX_DPS;
    if (w > lim)
    {
        return lim;
    }
    if (w < -lim)
    {
        return -lim;
    }
    return w;
}

void remote_lora_apply_validate_motor(void)
{
    dualcore_remote_to_ctrl_t r;
    static uint8 s_prev_key0;
    static uint8 s_prev_key_right_stick;
    float cmd;
    float rate_cmd;
    uint8 k0;
    uint8 rising;
    uint8 kr;
    uint8 rising_r;

    dualcore_remote_pull(&r);

    if ((r.enabled == 0u) || (r.online == 0u))
    {
        remote_lora_steer_snapshot_valid = 0u;
        return;
    }

    cmd = (float)r.left_y * (REMOTE_LORA_VALIDATE_SPEED_ABS_MAX / (float)REMOTE_LORA_JOYSTICK_ABS_MAX);
#if REMOTE_LORA_VALIDATE_SPEED_INVERT
    cmd = -cmd;
#endif
    motor_user_speed_cmd = remote_lora_validate_clamp_speed(cmd);

    rate_cmd = (float)r.left_x * (REMOTE_LORA_VALIDATE_STEER_RATE_MAX_DPS / (float)REMOTE_LORA_JOYSTICK_ABS_MAX);
    remote_lora_steer_rate_cmd_dps = remote_lora_clamp_steer_rate_dps(rate_cmd);
    remote_lora_steer_snapshot_valid = 1u;

    k0 = (r.key[0] != 0u) ? 1u : 0u;
    rising = (uint8)((k0 != 0u) && (s_prev_key0 == 0u));
    s_prev_key0 = k0;

    if (rising != 0u)
    {
        uint8 next_sw = (uint8)((Motor_Switch == MOTOR_ON) ? MOTOR_OFF : MOTOR_ON);
        if ((next_sw == MOTOR_ON) && (Motor_Runaway_Latch != 0u))
        {
            /* 失控锁存时禁止遥控重新上使能，清锁存后再用拨码/其它途径 */
        }
        else
        {
            Motor_Switch = next_sw;
        }
    }

#if REMOTE_LORA_KEY_INDEX_RIGHT_STICK < 4u
    kr = (r.key[REMOTE_LORA_KEY_INDEX_RIGHT_STICK] != 0u) ? 1u : 0u;
#else
    kr = 0u;
#endif
    rising_r = (uint8)((kr != 0u) && (s_prev_key_right_stick == 0u));
    s_prev_key_right_stick = kr;

    if ((rising_r != 0u) && (spin_enable == 0u) && (remote_lora_nav_allows_spin_request() != 0u))
    {
        spin_task_start(REMOTE_LORA_SPIN_TURNS_VALIDATE, (int8)REMOTE_LORA_SPIN_DIR);
    }
}

#endif /* CY_CORE_CM7_0 */
