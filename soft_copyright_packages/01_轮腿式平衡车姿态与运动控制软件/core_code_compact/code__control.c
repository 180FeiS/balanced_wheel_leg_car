#include "zf_common_headfile.h"
ins_struct ins;
double TempLat_Now=0,TempLon_Now=0;
uint16 pwm_4 = 0;
uint16 pwm_1 = 0;
int16 pwm_ph1 = 0;
int16 pwm_ph2 = 0;
int16 pwm_ph3 = 0;
int16 pwm_ph4 = 0;
const float LQR_K[8] = {
    -0.0585, -0.6200, -1.5706, -0.1139,
    -0.0585, -0.6200, -1.5706, -0.1139
};
const float Lmoto_K = 4980;
const float Rmoto_K = 4980;
pid_t leg_hight, turn_angle, turn_gyro, gyro, angle, speed, turn;
float angle_kd = 0;
float pitch_mid = 1.0;
float roll_mid = 4.2;
float dt_pid_gyro = 0.002f;
float dt_pid_angle = 0.01f;
float dt_pid_speed = 0.02f;
float dt_pid_turn = 0.01f;
float dt_leg = 0.005f;
float dt_pid_turn_angle = 0.001f;
float dt_pid_turn_gyro = 0.001f;
float leg_long = 3.5f;
float motor_user_speed_cmd = 0.0f;
float run_launch_speed = 0.0f;
float speed_target_effective = 0.0f;
uint8 jump_flag = 0;
uint8 jump_step_index = 0;
static int jump_time = 0;
uint8 speed_flag = 0;
static uint8 stair_jump_speed_boost_phase;
static void stair_jump_on_normal_sequence_done(void)
{
    if (stair_jump_speed_boost_phase == 0u)
    {
        stair_jump_speed_boost_phase = 1u;
    }
    else
    {
        stair_jump_speed_boost_phase = 0u;
    }
}
void stair_jump_reset_boost_phase(void)
{
    stair_jump_speed_boost_phase = 0u;
}
uint8 Motor_Runaway_Latch = 0;
uint8 jump_is_allowed(void)
{
    if (Motor_Switch != MOTOR_ON)
    {
        return 0u;
    }
    if (Motor_Runaway_Latch != 0u)
    {
        return 0u;
    }
    return 1u;
}
void jump_stop(void)
{
    jump_flag = 0u;
    jump_step_index = 0u;
    jump_time = 0;
    leg_long = 5.5f;
}
void motor_user_speed_cmd_set_from_pc(float cmd)
{
    run_launch_speed = cmd;
}
float speed_loop_leg_tilt = 0.0f;
float roll_debug_roll = 0;
float roll_debug_pid_out = 0;
float roll_debug_pid_err = 0;
float roll_debug_desired_left = 0;
float roll_debug_desired_right = 0;
float roll_debug_out_left = 0;
float roll_debug_out_right = 0;
float roll_debug_left_offset = 0;
float roll_debug_right_offset = 0;
float turn_out = 0;
float KP = 5;
float KPP = 0;
float KD = 0;
float KDD = 0;
float steer_cmd = 0.0f;
float spin_cmd = 0.0f;
float turn_mix_cmd = 0.0f;
uint8 steer_enable = 0;
float steer_target_yaw_deg = 0.0f;
float steer_angle_err = 0.0f;
float steer_rate_target_dps = 0.0f;
float steer_rate_meas_dps = 0.0f;
vuint8 steer_yaw_request_pending = 0;
vuint8 steer_yaw_delayed_by_spin = 0;
volatile float steer_yaw_request_deg = 0.0f;
#define YAW_POWERON_REF_LATCH_MS  100u
uint8 yaw_hold_poweron_en = 0;
float yaw_poweron_ref = 0.0f;
static uint8 yaw_poweron_ref_latched = 0;
static uint16 yaw_poweron_latch_count = 0;
volatile float remote_lora_steer_rate_cmd_dps = 0.0f;
volatile uint8 remote_lora_steer_snapshot_valid = 0u;
uint8 g_remote_local_keys_debug = 0u;
uint8 g_menu_input_remote_first = 0u;
uint8 g_menu_vofa_enable = 0u;
uint8 g_menu_nav_fusion_enable = 1u;
uint8 remote_lora_nav_allows_heading_override(void)
{
    if (nav_heading_mode == NAV_HEADING_MODE_GPS &&
        gps_nav_state == GPS_NAV_STATE_RUNNING)
    {
        return 0u;
    }
#if NAV_FUSION_ENABLE && NAV_FUSION_HEADING_CALIB_ENABLE
    if (NavFusion_IsRuntimeEnabled() != 0u && NavFusion_IsHeadingCalibrating() != 0u)
    {
        return 0u;
    }
#endif
    if (N.Nag_SystemRun_Index == 3)
    {
        return 0u;
    }
    if (N.Event_Active)
    {
        return 0u;
    }
    if (N.Nag_Stop_f)
    {
        return 0u;
    }
    return 1u;
}
uint8 remote_lora_nav_allows_spin_request(void)
{
    if (remote_lora_nav_allows_heading_override() == 0u)
    {
        return 0u;
    }
    if (Motor_Switch != MOTOR_ON)
    {
        return 0u;
    }
    if (Motor_Runaway_Latch != 0u)
    {
        return 0u;
    }
    return 1u;
}
#define STEER_ANGLE_SETTLE_DEG       3.0f
#define STEER_RATE_SETTLE_DPS        6.0f
#define STEER_RATE_TARGET_MAX_DPS   200.0f
#define STEER_CMD_MAX              1500.0f
#define BRIDGE_STEER_KP            0.015f
#define BRIDGE_STEER_KPP           0.00008f
#define BRIDGE_STEER_KD            0.4f
#define BRIDGE_GYRO_SUPPRESS_K     0.25f
static float bridge_image_steer_last_err = 0.0f;
static uint8 g_bridge_vision_track_valid = 0u;
static uint8 g_bridge_prev_zone_active = 0u;
static uint8 g_bridge_cam_steer_active = 0u;
static float bridge_image_steer_ppd(float center_err)
{
    float kp1;
    float kp2;
    float kd;
    float out;
    kp1 = center_err * BRIDGE_STEER_KP;
    kp2 = center_err * fabsf(center_err) * BRIDGE_STEER_KPP;
    kd = (center_err - bridge_image_steer_last_err) * BRIDGE_STEER_KD;
    bridge_image_steer_last_err = center_err;
    out = kp1 + kp2 + kd - imu_data.gyro_z * DEG_TO_RAD * BRIDGE_GYRO_SUPPRESS_K;
    return clip(out, -STEER_CMD_MAX, STEER_CMD_MAX);
}
void bridge_image_steer_reset(void)
{
    bridge_image_steer_last_err = 0.0f;
}
uint8 control_bridge_vision_track_valid(void)
{
    return g_bridge_vision_track_valid;
}
#define STEER_SETTLE_COUNT_MAX      20u
#define SPIN_ANGLE_OUT_MAX_DPS_DEFAULT 200.0f
#define SPIN_RATE_MIN_DPS               30.0f
#define SPIN_RATE_MAX_DPS              1000.0f
#define SPIN_ANGLE_SETTLE_DEG         70.0f
#define SPIN_RATE_SETTLE_DPS         20.0f
#define SPIN_SETTLE_COUNT_MAX        25u
#define SPIN_TIMEOUT_BASE_MS       3000u
#define SPIN_PITCH_ABORT_DEG         20.0f
float spin_rate_max_dps = SPIN_ANGLE_OUT_MAX_DPS_DEFAULT;
uint8 spin_enable = 0;
uint8 spin_done = 0;
int8 spin_dir = 1;
float spin_target_deg = 0.0f;
float spin_accum_deg = 0.0f;
float spin_angle_err = 0.0f;
float spin_rate_target_dps = 0.0f;
float spin_rate_meas_dps = 0.0f;
static float spin_last_yaw = 0.0f;
static uint8 spin_brake_phase = 0;
static uint8 spin_settle_count = 0;
static uint32 spin_timeout_ms = 0;
static uint32 spin_timeout_limit_ms = 0;
static void spin_reset_pid_state(pid_t *pid)
{
    pid->target = 0;
    pid->observation = 0;
    pid->error = 0;
    pid->last_error = 0;
    pid->prev_error = 0;
    pid->integral = 0;
    pid->differential = 0;
    pid->last_differential = 0;
    pid->out = 0;
}
static float wrap_yaw_deg(float yaw_deg)
{
    while (yaw_deg > 180.0f)
    {
        yaw_deg -= 360.0f;
    }
    while (yaw_deg < -180.0f)
    {
        yaw_deg += 360.0f;
    }
    return yaw_deg;
}
static void steer_finish(uint8 done)
{
    (void)done;
    steer_enable = 0;
    steer_angle_err = 0.0f;
    steer_cmd = 0.0f;
    turn_mix_cmd = spin_enable ? spin_cmd : 0.0f;
    spin_reset_pid_state(&turn_angle);
    spin_reset_pid_state(&turn_gyro);
}
static void spin_finish(uint8 done)
{
    spin_enable = 0;
    spin_done = done;
    spin_rate_target_dps = 0.0f;
    spin_settle_count = 0;
    spin_timeout_ms = 0;
    spin_brake_phase = 0;
    spin_cmd = 0.0f;
    turn_mix_cmd = 0.0f;
    spin_reset_pid_state(&turn_angle);
    spin_reset_pid_state(&turn_gyro);
}
void spin_task_start(float turns, int8 dir)
{
    if (turns <= 0.0f)
    {
        spin_finish(0);
        spin_target_deg = 0.0f;
        spin_accum_deg = 0.0f;
        spin_angle_err = 0.0f;
        return;
    }
    spin_dir = (dir >= 0) ? 1 : -1;
    spin_enable = 1;
    spin_done = 0;
    spin_target_deg = turns * 360.0f * (float)spin_dir;
    spin_accum_deg = 0.0f;
    spin_angle_err = spin_target_deg;
    spin_rate_target_dps = 0.0f;
    spin_rate_meas_dps = 0.0f;
    spin_last_yaw = (float)euler_angle.yaw;
    spin_brake_phase = 0;
    spin_settle_count = 0;
    spin_timeout_ms = 0;
    {
        float rate_dps = spin_rate_max_dps;
        uint32 per_turn_ms = 0u;
        if (rate_dps < SPIN_RATE_MIN_DPS)
        {
            rate_dps = SPIN_RATE_MIN_DPS;
        }
        per_turn_ms = (uint32)(360.0f / rate_dps * 1000.0f);
        spin_timeout_limit_ms = SPIN_TIMEOUT_BASE_MS + (uint32)(turns * (float)per_turn_ms);
    }
    spin_reset_pid_state(&turn_angle);
    spin_reset_pid_state(&turn_gyro);
    steer_enable = 0;
    steer_angle_err = 0.0f;
    steer_cmd = 0.0f;
    spin_cmd = 0.0f;
    turn_mix_cmd = 0.0f;
}
void spin_set_rate_max_dps(float rate_dps)
{
    if (rate_dps < SPIN_RATE_MIN_DPS)
    {
        rate_dps = SPIN_RATE_MIN_DPS;
    }
    else if (rate_dps > SPIN_RATE_MAX_DPS)
    {
        rate_dps = SPIN_RATE_MAX_DPS;
    }
    spin_rate_max_dps = rate_dps;
}
void spin_task_stop(void)
{
    spin_target_deg = 0.0f;
    spin_accum_deg = 0.0f;
    spin_angle_err = 0.0f;
    spin_finish(0);
}
void steer_set_target_yaw(float target_yaw_deg)
{
    float curr_yaw = (float)euler_angle.yaw;
    float target_yaw = wrap_yaw_deg(target_yaw_deg);
    float target_err = (float)ange_deviation1(target_yaw, curr_yaw);
    steer_yaw_request_pending = 0;
    steer_yaw_delayed_by_spin = 0;
    if (ABS(target_err) <= 0.001f)
    {
        steer_finish(0);
        steer_target_yaw_deg = target_yaw;
        return;
    }
    if (spin_enable)
    {
        spin_task_stop();
    }
    steer_enable = 1;
    steer_target_yaw_deg = target_yaw;
    steer_angle_err = target_err;
    steer_cmd = 0.0f;
    turn_mix_cmd = 0.0f;
    spin_reset_pid_state(&turn_angle);
    spin_reset_pid_state(&turn_gyro);
}
void steer_request_target_yaw(float target_yaw_deg)
{
    steer_yaw_request_deg = target_yaw_deg;
    steer_yaw_request_pending = 1;
    steer_yaw_delayed_by_spin = 0;
}
void steer_request_relative_yaw(float delta_deg)
{
    float curr_yaw = (float)euler_angle.yaw;
    if (delta_deg == 0.0f)
    {
        steer_request_target_yaw(curr_yaw);
        return;
    }
    steer_request_target_yaw(curr_yaw + delta_deg);
}
void steer_task_start(float delta_deg)
{
    float curr_yaw = (float)euler_angle.yaw;
    if (delta_deg == 0.0f)
    {
        steer_finish(0);
        steer_target_yaw_deg = curr_yaw;
        return;
    }
    steer_set_target_yaw(curr_yaw + delta_deg);
}
void steer_task_stop(void)
{
    steer_finish(0);
}
static void control_yaw_soft_reset_sync(void)
{
    steer_yaw_request_pending = 0u;
    steer_yaw_delayed_by_spin = 0u;
    steer_yaw_request_deg = 0.0f;
    steer_task_stop();
    spin_task_stop();
    yaw_poweron_ref = (float)euler_angle.yaw;
    yaw_poweron_ref_latched = 1u;
    yaw_poweron_latch_count = YAW_POWERON_REF_LATCH_MS;
    steer_target_yaw_deg = (float)euler_angle.yaw;
    steer_angle_err = 0.0f;
}
void motor_poll_switch2_speed_baseline(void)
{
    static uint8 s_switch2_synced = 0u;
    static uint8 s_switch2_prev = 1u;
    uint8 sw2_now;
    sw2_now = (gpio_get_level(SWITCH2) == GPIO_LOW) ? 0u : 1u;
    if (s_switch2_synced == 0u)
    {
        s_switch2_prev = sw2_now;
        s_switch2_synced = 1u;
        return;
    }
    if (sw2_now == s_switch2_prev)
    {
        return;
    }
    s_switch2_prev = sw2_now;
    Yaw_ResetZero();
    control_yaw_soft_reset_sync();
    gpio_toggle_level(LED1);
}
void yaw_hold_poweron_request_if_needed(void)
{
    float yaw_err;
    if (yaw_hold_poweron_en == 0u || yaw_poweron_ref_latched == 0u)
    {
        return;
    }
    if (N.HeadingHold_Enable && N.HeadingHold_Event_Allowed)
    {
        return;
    }
    if (spin_enable)
    {
        return;
    }
    if (remote_lora_steer_snapshot_valid != 0u)
    {
        return;
    }
    if (steer_yaw_request_pending)
    {
        return;
    }
    if (steer_enable == 0u)
    {
        steer_request_target_yaw(yaw_poweron_ref);
        return;
    }
    if (fabsf((float)ange_deviation1(yaw_poweron_ref, steer_target_yaw_deg)) > 0.01f)
    {
        steer_request_target_yaw(yaw_poweron_ref);
        return;
    }
    yaw_err = fabsf((float)ange_deviation1(yaw_poweron_ref, (float)euler_angle.yaw));
    if (yaw_err > Nag_HeadingHold_Reissue_Error)
    {
        steer_request_target_yaw(yaw_poweron_ref);
    }
}
void set_steer_cmd(float cmd)
{
    steer_cmd = cmd;
}
#define LEG_P_MIN           2.4f
#define LEG_P_MAX          14.5f
#define LEG_STEP_P_MAX      1.0f
#define LEG_STEP_ANGLE_MAX  1.0f
#define LEG_RIGHT_ANGLE_INVERT  1
uint8 roll_balance_en = 0;
#define ROLL_LEG_SCALE          1.0f
#define ROLL_LEG_OFFSET_MAX      8.5f
#define LEG_SERVO_SPEED_TILT_EN  1
#define LEG_TILT_K              0.016f
#define LEG_TILT_MAX             VMC_A_EXT_MAX
#define JUMP_PID_SCALE          0.4f
#define JUMP_TAKEOFF_P_DEFAULT   12.5f
#define JUMP_RETRACT_P_DEFAULT   5.5f
#define JUMP_PREPARE_P_DEFAULT   7.5f
#define JUMP_BUFFER_P_DEFAULT    5.5f
#define JUMP_BUFFER_STEP_PER_20MS_DEFAULT  (JUMP_BUFFER_STEP_P_MAX_DEFAULT * 4.0f)
#define JUMP_BUFFER_MARGIN      1
#define JUMP_BUFFER_CYCLES_DEFAULT  ((int)(((JUMP_PREPARE_P_DEFAULT - JUMP_BUFFER_P_DEFAULT) / JUMP_BUFFER_STEP_PER_20MS_DEFAULT) + 0.999f) + JUMP_BUFFER_MARGIN)
float jump_takeoff_p  = JUMP_TAKEOFF_P_DEFAULT;
float jump_retract_p  = JUMP_RETRACT_P_DEFAULT;
float jump_prepare_p  = JUMP_PREPARE_P_DEFAULT;
float jump_buffer_p   = JUMP_BUFFER_P_DEFAULT;
float jump_buffer_step_p_max = JUMP_BUFFER_STEP_P_MAX_DEFAULT;
float jump_stage_takeoff_cycles  = 4.0f;
float jump_stage_retract_cycles  = 3.0f;
float jump_stage_prepare_cycles  = 2.0f;
float jump_stage_buffer_cycles   = (float)JUMP_BUFFER_CYCLES_DEFAULT;
static jump_control_struct jump_control_config[4] =
    {
        {0, 0, jump_set_step, "起跳"},
        {0, 0, jump_set_step, "收腿"},
        {0, 0, jump_set_step, "准备缓冲"},
        {0, 0, jump_set_step, "执行缓冲"},
};
static const uint8 jump_step_num = 4u;
static int16 jump_cycles_round(float cycles)
{
    int16 r = (int16)(cycles + 0.5f);
    if (r < 1)
    {
        r = 1;
    }
    if (r > 255)
    {
        r = 255;
    }
    return r;
}
static void jump_control_config_sync(void)
{
    int16 t0 = jump_cycles_round(jump_stage_takeoff_cycles);
    int16 t1 = (int16)(t0 + jump_cycles_round(jump_stage_retract_cycles));
    int16 t2 = (int16)(t1 + jump_cycles_round(jump_stage_prepare_cycles));
    int16 t3_max = (int16)(t2 + jump_cycles_round(jump_stage_buffer_cycles) - 1);
    jump_control_config[0].min = 0;
    jump_control_config[0].max = t0;
    jump_control_config[1].min = t0;
    jump_control_config[1].max = t1;
    jump_control_config[2].min = t1;
    jump_control_config[2].max = t2;
    jump_control_config[3].min = t2;
    jump_control_config[3].max = t3_max;
}
static float jump_param_clamp_leg(float value)
{
    if (value < 3.0f)
    {
        return 3.0f;
    }
    if (value > 15.0f)
    {
        return 15.0f;
    }
    return value;
}
static float jump_param_clamp_time(float value)
{
    if (value < 1.0f)
    {
        return 1.0f;
    }
    if (value > 255.0f)
    {
        return 255.0f;
    }
    return value;
}
static float jump_param_clamp_step(float value)
{
    if (value < 0.01f)
    {
        return 0.01f;
    }
    if (value > 1.0f)
    {
        return 1.0f;
    }
    return value;
}
static void jump_recalc_buffer_cycles_from_leg(void)
{
    float delta = jump_prepare_p - jump_buffer_p;
    float step_per_20ms = jump_buffer_step_p_max * 4.0f;
    if (delta < 0.0f)
    {
        delta = 0.0f;
    }
    if (step_per_20ms < 0.001f)
    {
        step_per_20ms = 0.001f;
    }
    jump_stage_buffer_cycles = jump_param_clamp_time(
        (float)((int)((delta / step_per_20ms) + 0.999f) + JUMP_BUFFER_MARGIN));
}
void JumpParamRecalcBufferTimeFromLeg(void)
{
    jump_recalc_buffer_cycles_from_leg();
    jump_control_config_sync();
}
void JumpParamApplyDefaults(void)
{
    jump_takeoff_p = JUMP_TAKEOFF_P_DEFAULT;
    jump_retract_p = JUMP_RETRACT_P_DEFAULT;
    jump_prepare_p = JUMP_PREPARE_P_DEFAULT;
    jump_buffer_p = JUMP_BUFFER_P_DEFAULT;
    jump_buffer_step_p_max = JUMP_BUFFER_STEP_P_MAX_DEFAULT;
    jump_stage_takeoff_cycles = 4.0f;
    jump_stage_retract_cycles = 3.0f;
    jump_stage_prepare_cycles = 2.0f;
    jump_recalc_buffer_cycles_from_leg();
    jump_control_config_sync();
}
float JumpParamGet(uint8 field_index)
{
    switch (field_index)
    {
    case Run_Jump_Field_Takeoff_P:
        return jump_takeoff_p;
    case Run_Jump_Field_Retract_P:
        return jump_retract_p;
    case Run_Jump_Field_Prepare_P:
        return jump_prepare_p;
    case Run_Jump_Field_Buffer_P:
        return jump_buffer_p;
    case Run_Jump_Field_Takeoff_T:
        return jump_stage_takeoff_cycles;
    case Run_Jump_Field_Retract_T:
        return jump_stage_retract_cycles;
    case Run_Jump_Field_Prepare_T:
        return jump_stage_prepare_cycles;
    case Run_Jump_Field_Buffer_T:
        return jump_stage_buffer_cycles;
    case Run_Jump_Field_Buffer_Step:
        return jump_buffer_step_p_max;
    default:
        return 0.0f;
    }
}
void JumpParamSet(uint8 field_index, float value)
{
    switch (field_index)
    {
    case Run_Jump_Field_Takeoff_P:
        jump_takeoff_p = jump_param_clamp_leg(value);
        break;
    case Run_Jump_Field_Retract_P:
        jump_retract_p = jump_param_clamp_leg(value);
        break;
    case Run_Jump_Field_Prepare_P:
        jump_prepare_p = jump_param_clamp_leg(value);
        jump_recalc_buffer_cycles_from_leg();
        break;
    case Run_Jump_Field_Buffer_P:
        jump_buffer_p = jump_param_clamp_leg(value);
        jump_recalc_buffer_cycles_from_leg();
        break;
    case Run_Jump_Field_Takeoff_T:
        jump_stage_takeoff_cycles = jump_param_clamp_time(value);
        break;
    case Run_Jump_Field_Retract_T:
        jump_stage_retract_cycles = jump_param_clamp_time(value);
        break;
    case Run_Jump_Field_Prepare_T:
        jump_stage_prepare_cycles = jump_param_clamp_time(value);
        break;
    case Run_Jump_Field_Buffer_T:
        jump_stage_buffer_cycles = jump_param_clamp_time(value);
        break;
    case Run_Jump_Field_Buffer_Step:
        jump_buffer_step_p_max = jump_param_clamp_step(value);
        jump_recalc_buffer_cycles_from_leg();
        break;
    default:
        break;
    }
    jump_control_config_sync();
}
void JumpParamAdjust(uint8 field_index, float delta)
{
    JumpParamSet(field_index, JumpParamGet(field_index) + delta);
}
double victual_point_lat[] = {0};
double victual_point_lon[] = {0};
uint8 Temp_num = 0;
double Angle_Z_Quaternions = 0;
void pid_ctrl_Init(void)
{
    pid_init(&leg_hight, 0.25f, 0.12f, 0.0, dt_leg, 500, 0, 0, 50, Position_pid);
    pid_init(&turn_angle, 20.0f, 2.0f, 0.0f, dt_pid_turn_angle, 0, 0, 0, SPIN_ANGLE_OUT_MAX_DPS_DEFAULT, Position_pid);
    pid_init(&turn_gyro, 30.0f, 2.0f, 0.0f, dt_pid_turn_gyro, 0, 0, 0, 2200, Position_pid);
     pid_init(&gyro, 1.1, 0, 0, 0.002, 0, 0, 0, 10000, Position_pid);
     pid_init(&angle, 500.0, 0, 0, 0.01, 0, 0, 0, 10000, Position_pid);
     pid_init(&speed, 3.0, 0.001, 0.01, 0.02, 0, 0, 0, 10000, Position_pid);
    pid_set_target(&leg_hight, roll_mid);
    pid_set_target(&speed, 0);
    jump_control_config_sync();
}
void leg_debug_init_pwm(void)
{
#if LEG_DEBUG_MODE
    servo_control_table(leg_long, 0, &pwm_ph4, &pwm_ph3);
    servo_control_table(leg_long, 0, &pwm_ph1, &pwm_ph2);
#endif
}
void LQR_control(float V_target, float th)
{
    float L_min = 0.0353019121;
    float TL = 0, TR = 0;
    static float x_hat_last = 0;
    static float last_image_error = 0;
    float Tangle = -(euler_angle.pitch - th) / DEG_TO_RAD;
    float gy = -imu660rc_gyro_transition(imu660rc_gyro_y) / DEG_TO_RAD;
    float v_t = (v_hat - V_target);
    TL = LQR_K[3] * gy + LQR_K[2] * Tangle + LQR_K[1] * v_t + LQR_K[0] * ((x_hat + L_min * sin(euler_angle.pitch / DEG_TO_RAD) - (x_hat_last + v_hat)));
    TR = LQR_K[7] * gy + LQR_K[6] * Tangle + LQR_K[5] * v_t + LQR_K[4] * ((x_hat + L_min * sin(euler_angle.pitch / DEG_TO_RAD) - (x_hat_last + v_hat)));
    x_hat_last = x_hat;
    int16 LO = (int16)(Lmoto_K * TL - turn_out);
    int16 RO = (int16)(Rmoto_K * TR + turn_out);
    Left_Motor_Pwm = LO;
    Right_Motor_Pwm = RO;
    dead_compensate(&LO, &RO);
    if (Motor_Switch)
    {
        if (jump_flag != 1)
        {
            if (30 >= euler_angle.pitch)
            {
                small_driver_set_duty(LO, -RO);
            }
            if (euler_angle.pitch >= -50)
            {
                small_driver_set_duty(LO, -RO);
            }
            else
            {
                small_driver_set_duty(0, 0);
            }
        }
    }
    else
    {
        small_driver_set_duty(0, 0);
    }
}
float turn_control(float image_error)
{
    (void)image_error;
    return steer_cmd;
}
void pid_ctrl_Run(void)
{
    static uint16 pid_time_gyro = 0;
    static uint16 pid_time_angle = 0;
    static uint16 pid_time_speed = 0;
    static uint16 pid_time_turn = 0;
    static uint32 timer_flag = 0;
    static float Angle_Out = 0;
    imu660rc_get_gyro();
    if (yaw_poweron_ref_latched == 0u)
    {
        if (++yaw_poweron_latch_count >= YAW_POWERON_REF_LATCH_MS)
        {
            yaw_poweron_ref = (float)euler_angle.yaw;
            yaw_poweron_ref_latched = 1u;
        }
    }
    if (0 == timer_flag)
    {
        speed_target_effective = Nag_GetControlSpeedTarget();
        if (stair_jump_speed_boost_phase != 0u)
        {
            float dir = motor_user_speed_cmd;
            if (dir == 0.0f)
            {
                dir = speed_target_effective;
            }
            speed_target_effective += ((dir >= 0.0f) ? 1.0f : -1.0f) * STAIR_JUMP_SPEED_BOOST_AFTER_FIRST;
        }
        pid_set_target(&speed, speed_target_effective);
        pid_get_observation(&speed, -motor_value.receive_left_speed_data + motor_value.receive_right_speed_data);
        pid_set_dt(&speed, dt_pid_speed);
        pid_run(&speed);
        speed_loop_leg_tilt = (jump_flag == 1) ? (speed.out * JUMP_PID_SCALE) : speed.out;
    }
    if (spin_enable)
    {
        pid_set_target(&speed, 0);
        speed_loop_leg_tilt = 0.0f;
    }
    if (0 == timer_flag % 5)
    {
        pid_set_target(&angle, pitch_mid);
        pid_get_observation(&angle, euler_angle.pitch);
        pid_set_dt(&angle, dt_pid_angle);
        pid_run(&angle);
        Angle_Out = angle.out + angle_kd * imu660rc_gyro_y * dt_pid_angle;
        if (jump_flag == 1)
            Angle_Out *= JUMP_PID_SCALE;
    }
    pid_set_target(&gyro, Angle_Out);
    pid_get_observation(&gyro, imu660rc_gyro_y);
    pid_set_dt(&gyro, dt_pid_gyro);
    pid_run(&gyro);
    spin_rate_meas_dps = imu_data.gyro_z * DEG_TO_RAD;
    if (spin_enable)
    {
        if (Motor_Switch)
        {
            float curr_yaw = (float)euler_angle.yaw;
            float delta_yaw = (float)ange_deviation1(curr_yaw, spin_last_yaw);
            float abs_spin_err = 0.0f;
            spin_last_yaw = curr_yaw;
            spin_accum_deg += delta_yaw;
            spin_angle_err = spin_target_deg - spin_accum_deg;
            abs_spin_err = ABS(spin_angle_err);
            if (!spin_brake_phase)
            {
                if ((spin_target_deg >= 0.0f && spin_angle_err <= 0.0f) ||
                    (spin_target_deg < 0.0f && spin_angle_err >= 0.0f) ||
                    abs_spin_err <= SPIN_ANGLE_SETTLE_DEG)
                {
                    spin_brake_phase = 1;
                }
            }
            if (spin_brake_phase)
            {
                spin_rate_target_dps = 0.0f;
            }
            else
            {
                float spin_err_sign = (spin_angle_err >= 0.0f) ? 1.0f : -1.0f;
                spin_rate_target_dps = spin_err_sign * spin_rate_max_dps;
            }
            pid_set_target(&turn_gyro, spin_rate_target_dps);
            pid_get_observation(&turn_gyro, spin_rate_meas_dps);
            pid_set_dt(&turn_gyro, dt_pid_turn_gyro);
            pid_run(&turn_gyro);
            spin_cmd = turn_gyro.out;
            spin_timeout_ms++;
            if (ABS(euler_angle.pitch - pitch_mid) > SPIN_PITCH_ABORT_DEG || spin_timeout_ms > spin_timeout_limit_ms)
            {
                spin_finish(0);
            }
            else if (spin_brake_phase && ABS(spin_rate_meas_dps) < SPIN_RATE_SETTLE_DPS)
            {
                if (++spin_settle_count >= SPIN_SETTLE_COUNT_MAX)
                {
                    spin_finish(1);
                }
            }
            else
            {
                spin_settle_count = 0;
            }
        }
        else
        {
            spin_last_yaw = (float)euler_angle.yaw;
            spin_rate_target_dps = 0.0f;
            spin_cmd = 0.0f;
        }
    }
    else
    {
        spin_angle_err = spin_target_deg - spin_accum_deg;
        spin_rate_target_dps = 0.0f;
        spin_cmd = 0.0f;
    }
    {
        float bridge_center_err = 0.0f;
        uint8 bridge_track_valid = 0u;
        uint8 bridge_fresh = 0u;
        dualcore_bridge_vision_pull(&bridge_center_err, &bridge_track_valid, &bridge_fresh);
        g_bridge_vision_track_valid = bridge_track_valid;
        if ((N.Bridge_Zone_Active == 0u) && (g_bridge_prev_zone_active != 0u))
        {
            bridge_image_steer_reset();
            g_bridge_cam_steer_active = 0u;
        }
        g_bridge_prev_zone_active = N.Bridge_Zone_Active;
        g_bridge_cam_steer_active = 0u;
        if (!spin_enable &&
                 (remote_lora_steer_snapshot_valid != 0u) &&
                 (remote_lora_nav_allows_heading_override() != 0u))
        {
            if (steer_enable != 0u)
            {
                steer_task_stop();
            }
            steer_rate_meas_dps = imu_data.gyro_z * DEG_TO_RAD;
            steer_rate_target_dps = clip(
                remote_lora_steer_rate_cmd_dps,
                -STEER_RATE_TARGET_MAX_DPS,
                STEER_RATE_TARGET_MAX_DPS);
            pid_set_target(&turn_gyro, steer_rate_target_dps);
            pid_get_observation(&turn_gyro, steer_rate_meas_dps);
            pid_set_dt(&turn_gyro, dt_pid_turn_gyro);
            pid_run(&turn_gyro);
            set_steer_cmd(clip(turn_gyro.out, -STEER_CMD_MAX, STEER_CMD_MAX));
        }
        else if (!spin_enable && steer_enable)
        {
            static uint8 steer_settle_count = 0;
            steer_rate_meas_dps = imu_data.gyro_z * DEG_TO_RAD;
            steer_rate_target_dps = 0.0f;
            steer_angle_err = (float)ange_deviation1(steer_target_yaw_deg, euler_angle.yaw);
            pid_set_target(&turn_angle, 0.0f);
            pid_get_observation(&turn_angle, -steer_angle_err);
            pid_set_dt(&turn_angle, dt_pid_turn_angle);
            pid_run(&turn_angle);
            steer_rate_target_dps = clip(turn_angle.out, -STEER_RATE_TARGET_MAX_DPS, STEER_RATE_TARGET_MAX_DPS);
            pid_set_target(&turn_gyro, steer_rate_target_dps);
            pid_get_observation(&turn_gyro, steer_rate_meas_dps);
            pid_set_dt(&turn_gyro, dt_pid_turn_gyro);
            pid_run(&turn_gyro);
            set_steer_cmd(clip(turn_gyro.out, -STEER_CMD_MAX, STEER_CMD_MAX));
            if (ABS(steer_angle_err) <= STEER_ANGLE_SETTLE_DEG &&
                ABS(steer_rate_meas_dps) <= STEER_RATE_SETTLE_DPS)
            {
                if (++steer_settle_count >= STEER_SETTLE_COUNT_MAX)
                {
                    steer_finish(1);
                    steer_settle_count = 0;
                }
            }
            else
            {
                steer_settle_count = 0;
            }
        }
        else if (!spin_enable)
        {
            steer_rate_meas_dps = imu_data.gyro_z * DEG_TO_RAD;
            steer_rate_target_dps = 0.0f;
            steer_angle_err = (float)ange_deviation1(steer_target_yaw_deg, euler_angle.yaw);
            if (!steer_enable)
            {
                steer_cmd = 0.0f;
            }
        }
    }
    turn_mix_cmd = spin_enable ? spin_cmd : steer_cmd;
    if(Motor_Switch)
    {
        if ((-motor_value.receive_left_speed_data + motor_value.receive_right_speed_data) / 2 > 3000 || (-motor_value.receive_left_speed_data + motor_value.receive_right_speed_data) / 2 < -3000)
        {
            Motor_Switch = 0;
            Motor_Runaway_Latch = 1;
            jump_stop();
         }
        else
        {
            float scale = (jump_flag == 1) ? JUMP_PID_SCALE : 1.0f;
            small_driver_set_duty((int16)((gyro.out + turn_mix_cmd) * scale), (int16)(-(gyro.out - turn_mix_cmd) * scale));
        }
    }
    else
    {
        small_driver_set_duty(0, 0);
    }
    timer_flag = (timer_flag + 1) % 20;
}
static void leg_servo_step_update(float desired_left_p, float desired_right_p, float desired_angle,
                                  float *out_left_p, float *out_right_p,
                                  float *out_left_angle, float *out_right_angle)
{
    static float current_left_p = 0;
    static float current_right_p = 0;
    static float current_left_angle = 0;
    static float current_right_angle = 0;
    static uint8 first_run = 1;
    if (first_run)
    {
        current_left_p = desired_left_p;
        current_right_p = desired_right_p;
        current_left_angle = desired_angle;
        current_right_angle = desired_angle;
        first_run = 0;
    }
    uint8 use_step = 0;
    if (jump_flag == 0)
        use_step = 1;
    else if (jump_step_index == 3)
        use_step = 1;
    if (use_step)
    {
        float step_p = (jump_step_index == 3) ? jump_buffer_step_p_max : LEG_STEP_P_MAX;
        float delta;
        delta = desired_left_p - current_left_p;
        current_left_p += clip2(delta, step_p);
        delta = desired_right_p - current_right_p;
        current_right_p += clip2(delta, step_p);
        delta = desired_angle - current_left_angle;
        current_left_angle += clip2(delta, LEG_STEP_ANGLE_MAX);
        delta = desired_angle - current_right_angle;
        current_right_angle += clip2(delta, LEG_STEP_ANGLE_MAX);
    }
    else
    {
        current_left_p = desired_left_p;
        current_right_p = desired_right_p;
        current_left_angle = desired_angle;
        current_right_angle = desired_angle;
    }
    current_left_p = clip(current_left_p, LEG_P_MIN, LEG_P_MAX);
    current_right_p = clip(current_right_p, LEG_P_MIN, LEG_P_MAX);
    *out_left_p = current_left_p;
    *out_right_p = current_right_p;
    *out_left_angle = current_left_angle;
    *out_right_angle = current_right_angle;
}
static float leg_servo_get_desired_tilt_angle(void)
{
#if LEG_SERVO_SPEED_TILT_EN
    float a = LEG_TILT_K * speed_loop_leg_tilt;
    return clip(a, -LEG_TILT_MAX, LEG_TILT_MAX);
#else
    return 0.0f;
#endif
}
void leg_control(void)
{
    float desired_left_p, desired_right_p;
    if ((Motor_Switch != MOTOR_ON) && (jump_flag != 0u))
    {
        jump_stop();
    }
    if (jump_flag == 1)
    {
        desired_left_p = desired_right_p = leg_long;
        roll_debug_left_offset = 0;
        roll_debug_right_offset = 0;
    }
    else if (roll_balance_en)
    {
        pid_get_observation(&leg_hight, euler_angle.roll);
        pid_set_dt(&leg_hight, dt_leg);
        pid_run(&leg_hight);
        float roll_angle_out = leg_hight.out;
        float left_offset, right_offset;
        if (roll_angle_out > 0)
        {
            left_offset = 0;
            right_offset = clip2(ROLL_LEG_SCALE * roll_angle_out, ROLL_LEG_OFFSET_MAX);
        }
        else
        {
            left_offset = clip2(ROLL_LEG_SCALE * (-roll_angle_out), ROLL_LEG_OFFSET_MAX);
            right_offset = 0;
        }
        desired_left_p = leg_long + left_offset;
        desired_right_p = leg_long + right_offset;
        roll_debug_left_offset = left_offset;
        roll_debug_right_offset = right_offset;
        roll_debug_pid_out = leg_hight.out;
        roll_debug_pid_err = leg_hight.error;
    }
    else
    {
        desired_left_p = desired_right_p = leg_long;
        roll_debug_left_offset = 0;
        roll_debug_right_offset = 0;
        roll_debug_pid_out = 0;
        roll_debug_pid_err = 0;
    }
    roll_debug_roll = euler_angle.roll;
    roll_debug_desired_left = desired_left_p;
    roll_debug_desired_right = desired_right_p;
    float desired_angle = leg_servo_get_desired_tilt_angle();
    float out_left_p, out_right_p, out_left_angle, out_right_angle;
    leg_servo_step_update(desired_left_p, desired_right_p, desired_angle,
                          &out_left_p, &out_right_p, &out_left_angle, &out_right_angle);
    roll_debug_out_left = out_left_p;
    roll_debug_out_right = out_right_p;
#if LEG_RIGHT_ANGLE_INVERT
    left_leg_control(out_left_p, out_left_angle);
    right_leg_control(out_right_p, -out_right_angle);
#else
    left_leg_control(out_left_p, -out_left_angle);
    right_leg_control(out_right_p, out_right_angle);
#endif
}
void jump_set_step(int step_num)
{
    switch (step_num)
    {
    case 0:
        leg_long = jump_takeoff_p;
        break;
    case 1:
        leg_long = jump_retract_p;
        break;
    case 2:
        leg_long = jump_prepare_p;
        break;
    case 3:
        leg_long = jump_buffer_p;
        break;
    default:
        break;
    }
}
void jump_control(void)
{
    if (jump_flag == 1)
    {
        if (jump_is_allowed() == 0u)
        {
            jump_stop();
            return;
        }
        if (jump_time == 0)
        {
            jump_control_config_sync();
        }
        jump_time++;
        if (jump_time < jump_control_config[jump_step_num - 1].max)
        {
            for (int i = 0; i < jump_step_num; i++)
            {
                if (jump_time >= jump_control_config[i].min && jump_time <= jump_control_config[i].max)
                {
                    jump_control_config[i].handler(i);
                    jump_step_index = (uint8)i;
                    break;
                }
            }
        }
        else
        {
            stair_jump_on_normal_sequence_done();
            Nag_NotifyStairJumpDone();
            jump_stop();
        }
    }
}
void dead_compensate(int16 *input_L, int16 *input_R)
{
    if (*input_L > 0)
    {
        *input_L = clip2(*input_L + L_dead_zone_correct, 10000);
    }
    else if (*input_L < 0)
    {
        *input_L = clip2(*input_L + L_dead_zone_negative, 10000);
    }
    else
    {
        *input_L = 0;
    }
    if (*input_R > 0)
    {
        *input_R = clip2(*input_R + R_dead_zone_correct, 10000);
    }
    else if (*input_R < 0)
    {
        *input_R = clip2(*input_R + R_dead_zone_negative, 10000);
    }
    else
    {
        *input_R = 0;
    }
}
void left_leg_control(float p, float angle)
{
#if !LEG_DEBUG_MODE
    servo_control_table(p, -angle, &pwm_ph4, &pwm_ph3);
#endif
      if(10000 == pwm_ph3 || 10000 == pwm_ph4)
    {
        ASSERT(10000 == pwm_ph4 || 10000 == pwm_ph3);
        return;
    }
    pwm_set_duty(SERVO_3, SERVO3_MID - pwm_ph3);
    pwm_set_duty(SERVO_4, SERVO4_MID + pwm_ph4);
}
void right_leg_control(float p, float angle)
{
#if !LEG_DEBUG_MODE
    servo_control_table(p, -angle, &pwm_ph1, &pwm_ph2);
#endif
   if(10000 == pwm_ph1 || 10000 == pwm_ph2)
    {
        ASSERT(10000 == pwm_ph2 || 10000 == pwm_ph1);
        return;
    }
    pwm_set_duty(SERVO_1, SERVO1_MID + pwm_ph1);
    pwm_set_duty(SERVO_2, SERVO2_MID - pwm_ph2);
    pwm_4 = pwm_ph2;
    pwm_1 = pwm_ph1;
}
double get_fang_wei_jiao(double X_now, double Y_now, double X_next, double Y_next)
{
    double X_err, Y_err, angle;
    X_err = X_next - X_now;
    Y_err = Y_next - Y_now;
    if(X_err == 0)
    {
        if(Y_err > 0) return 90;
        if(Y_err < 0) return 270;
    }
    angle = atan(Y_err / X_err);
    angle = angle / PI * 180;
    if(angle > 0)
    {
        if(Y_err > 0) return angle;
        else if(Y_err < 0) return angle + 180;
    }
    else if(angle < 0)
    {
        if(Y_err > 0) return angle + 180;
        else if(Y_err < 0) return angle + 360;
    }
    return 0;
}
double ange_deviation1(double angel1, double angel2)
{
    double x = angel1 - angel2;
    while (x > 180.0)
    {
        x -= 360.0;
    }
    while (x < -180.0)
    {
        x += 360.0;
    }
    return x;
}
float Get_Final_Angle(void)
{
    float direction;
    direction = get_fang_wei_jiao(TempLat_Now, TempLon_Now,
                                  victual_point_lat[Temp_num],
                                  victual_point_lon[Temp_num]);
    float final_angle = 0;
    final_angle = ange_deviation1(euler_angle.yaw, direction);
    return final_angle;
}
void get_car_xy(void)
{
        double speed_x = 0 , speed_y = 0;
        speed_x = car_speed * cos(euler_angle.yaw/180.0*3.1415926);
        speed_y = car_speed * sin(euler_angle.yaw/180.0*3.1415926);
        ins.distance_x += speed_x * Nag_Sample_Dt * 1.9;
        ins.distance_y += speed_y * Nag_Sample_Dt * 1.9;
        TempLat_Now=ins.distance_x;
        TempLon_Now=ins.distance_y;
}
void ins_init(void)
{
   ins.distance_x = 0;
   ins.distance_y = 0;
   ins.distance = 0;
   car_speed = 0;
}
