#include "dualcore_shared.h"
#include "Menu.h"
#include "remote_lora.h"
#include "control.h"
#include "navigation.h"
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
    static uint8 s_prev_key_roll_balance;
    static uint8 s_prev_key_jump;
    static uint8 s_prev_left_sw0;
    static uint8 s_prev_left_sw1;
    static uint8 s_left_dip_synced;
    float cmd;
    float rate_cmd;
    uint8 k0;
    uint8 rising;
    uint8 kr;
    uint8 rising_r;
    uint8 nav_recording_active;
    dualcore_remote_pull(&r);
    if (g_menu_input_remote_first != 0u)
    {
        if ((r.enabled == 0u) || (r.online == 0u))
        {
            g_remote_local_keys_debug = 0u;
        }
        else
        {
            uint8 swv = 0u;
#if REMOTE_LORA_DEBUG_MODE_SWITCH_INDEX < 4u
            swv = (r.switch_key[REMOTE_LORA_DEBUG_MODE_SWITCH_INDEX] != 0u) ? 1u : 0u;
#endif
            g_remote_local_keys_debug = (uint8)((swv == REMOTE_LORA_LOCAL_KEYS_ACTIVE_LEVEL) ? 1u : 0u);
        }
        if (g_remote_local_keys_debug != 0u)
        {
            remote_lora_steer_snapshot_valid = 0u;
            remote_lora_steer_rate_cmd_dps = 0.0f;
            return;
        }
    }
    if ((r.enabled == 0u) || (r.online == 0u))
    {
        remote_lora_steer_snapshot_valid = 0u;
        s_left_dip_synced = 0u;
        return;
    }
    cmd = (float)r.left_y * (REMOTE_LORA_VALIDATE_SPEED_ABS_MAX / (float)REMOTE_LORA_JOYSTICK_ABS_MAX);
#if REMOTE_LORA_VALIDATE_SPEED_INVERT
    cmd = -cmd;
#endif
    motor_user_speed_cmd = remote_lora_validate_clamp_speed(cmd);
    rate_cmd = (float)r.right_x * (REMOTE_LORA_VALIDATE_STEER_RATE_MAX_DPS / (float)REMOTE_LORA_JOYSTICK_ABS_MAX);
    remote_lora_steer_rate_cmd_dps = remote_lora_clamp_steer_rate_dps(rate_cmd);
    remote_lora_steer_snapshot_valid = 1u;
    nav_recording_active = (uint8)((N.Nag_SystemRun_Index == 1u) && (N.End_f == 0));
    k0 = (r.key[0] != 0u) ? 1u : 0u;
    rising = (uint8)((k0 != 0u) && (s_prev_key0 == 0u));
    s_prev_key0 = k0;
    if (rising != 0u)
    {
        uint8 next_sw = (uint8)((Motor_Switch == MOTOR_ON) ? MOTOR_OFF : MOTOR_ON);
        if ((next_sw == MOTOR_ON) && (Motor_Runaway_Latch != 0u))
        {
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
#if REMOTE_LORA_KEY_INDEX_ROLL_BALANCE < 4u
    {
        uint8 k_roll;
        uint8 rising_roll;
        k_roll = (r.key[REMOTE_LORA_KEY_INDEX_ROLL_BALANCE] != 0u) ? 1u : 0u;
        rising_roll = (uint8)((k_roll != 0u) && (s_prev_key_roll_balance == 0u));
        s_prev_key_roll_balance = k_roll;
        if (rising_roll != 0u)
        {
            if (nav_recording_active != 0u)
            {
                Nag_Cycle_Record_Event_Type();
            }
            else
            {
                roll_balance_en = (uint8)(roll_balance_en ? 0u : 1u);
            }
        }
    }
#endif
#if REMOTE_LORA_KEY_INDEX_JUMP < 4u
    {
        uint8 k_j;
        uint8 rising_j;
        k_j = (r.key[REMOTE_LORA_KEY_INDEX_JUMP] != 0u) ? 1u : 0u;
        rising_j = (uint8)((k_j != 0u) && (s_prev_key_jump == 0u));
        s_prev_key_jump = k_j;
        if (rising_j != 0u)
        {
            if (nav_recording_active != 0u)
            {
                Nag_Request_Event_Mark();
            }
            else if ((jump_is_allowed() != 0u) && (jump_flag == 0u))
            {
                jump_flag = 1u;
            }
        }
    }
#endif
#if (REMOTE_LORA_LEFT_SWITCH0_INDEX < 4u) && (REMOTE_LORA_LEFT_SWITCH1_INDEX < 4u)
    {
        uint8 sw0;
        uint8 sw1;
        uint8 edge_replay;
        uint8 edge_rec_on;
        uint8 edge_rec_stop;
        sw0 = (r.switch_key[REMOTE_LORA_LEFT_SWITCH0_INDEX] != 0u) ? 1u : 0u;
        sw1 = (r.switch_key[REMOTE_LORA_LEFT_SWITCH1_INDEX] != 0u) ? 1u : 0u;
        if (s_left_dip_synced == 0u)
        {
            s_prev_left_sw0 = sw0;
            s_prev_left_sw1 = sw1;
            s_left_dip_synced = 1u;
        }
        else
        {
#if REMOTE_LORA_REPLAY_ON_SW0_RISING
            edge_replay = (uint8)((s_prev_left_sw0 == 0u) && (sw0 == 1u));
#else
            edge_replay = (uint8)((s_prev_left_sw0 == 1u) && (sw0 == 0u));
#endif
            if (edge_replay != 0u)
            {
                Nag_Begin_Replay();
            }
            if (sw0 == 1u)
            {
                edge_rec_on = (uint8)((s_prev_left_sw1 == 1u) && (sw1 == 0u));
                edge_rec_stop = (uint8)((s_prev_left_sw1 == 0u) && (sw1 == 1u));
                if (edge_rec_on != 0u)
                {
                    Nag_Begin_Record();
                }
                if (edge_rec_stop != 0u)
                {
                    Nag_Request_Stop_Record();
                }
            }
            s_prev_left_sw0 = sw0;
            s_prev_left_sw1 = sw1;
        }
    }
#endif
}
#endif
