#include "zf_common_headfile.h"
#ifndef CODE_CONTROL_H_
#define CODE_CONTROL_H_
extern float pitch_mid;
extern float roll_mid;
#define DEG_TO_RAD      (57.295779513082320876798154814105f)
#define K               (1.0f)
typedef void (*HandlerFunc)(int value);
typedef struct
{
        int16           min;
        int16           max;
        HandlerFunc     handler;
        const char      *description;
}jump_control_struct;
extern float dt_pid_gyro;
extern float dt_pid_angle;
extern float dt_pid_speed;
extern float dt_pid_turn;
extern float dt_leg;
extern float dt_pid_turn_angle;
extern float dt_pid_turn_gyro;
extern float motor_user_speed_cmd;
extern float run_launch_speed;
extern float speed_target_effective;
#ifndef STAIR_JUMP_SPEED_BOOST_AFTER_FIRST
#define STAIR_JUMP_SPEED_BOOST_AFTER_FIRST  (100.0f)
#endif
void stair_jump_reset_boost_phase(void);
void motor_user_speed_cmd_set_from_pc(float cmd);
void motor_poll_switch2_speed_baseline(void);
extern uint8 jump_flag;
uint8 jump_is_allowed(void);
void jump_stop(void);
extern float jump_takeoff_p;
extern float jump_retract_p;
extern float jump_prepare_p;
extern float jump_buffer_p;
#define JUMP_BUFFER_STEP_P_MAX_DEFAULT  0.25f
extern float jump_buffer_step_p_max;
extern float jump_stage_takeoff_cycles;
extern float jump_stage_retract_cycles;
extern float jump_stage_prepare_cycles;
extern float jump_stage_buffer_cycles;
#define Run_Jump_Param_Count 9u
#define Run_Jump_Field_Takeoff_P   0u
#define Run_Jump_Field_Retract_P     1u
#define Run_Jump_Field_Prepare_P     2u
#define Run_Jump_Field_Buffer_P      3u
#define Run_Jump_Field_Takeoff_T     4u
#define Run_Jump_Field_Retract_T     5u
#define Run_Jump_Field_Prepare_T     6u
#define Run_Jump_Field_Buffer_T      7u
#define Run_Jump_Field_Buffer_Step   8u
float JumpParamGet(uint8 field_index);
void JumpParamSet(uint8 field_index, float value);
void JumpParamAdjust(uint8 field_index, float delta);
void JumpParamApplyDefaults(void);
void JumpParamRecalcBufferTimeFromLeg(void);
static inline float JumpParamGetStep(uint8 field_index)
{
    if (field_index == Run_Jump_Field_Buffer_Step)
    {
        return 0.01f;
    }
    return 0.5f;
}
extern float leg_long;
extern uint8 speed_flag;
extern float speed_loop_leg_tilt;
#define L_dead_zone_correct           (140)
#define L_dead_zone_negative          (-148)
#define R_dead_zone_correct           (140)
#define R_dead_zone_negative          (-140)
extern float turn_out;
extern float KP;
extern float KPP;
extern float KD;
extern float KDD;
extern uint16 pwm_4;
extern uint16 pwm_1;
extern int16 LO_S;
extern int16 RO_S;
extern uint8 roll_balance_en;
extern uint8 Motor_Runaway_Latch;
extern float roll_debug_roll;
extern float roll_debug_pid_out;
extern float roll_debug_pid_err;
extern float roll_debug_desired_left;
extern float roll_debug_desired_right;
extern float roll_debug_out_left;
extern float roll_debug_out_right;
extern float roll_debug_left_offset;
extern float roll_debug_right_offset;
extern float steer_cmd;
extern float spin_cmd;
extern float turn_mix_cmd;
extern uint8 steer_enable;
extern float steer_target_yaw_deg;
extern float steer_angle_err;
extern float steer_rate_target_dps;
extern float steer_rate_meas_dps;
extern vuint8 steer_yaw_request_pending;
extern vuint8 steer_yaw_delayed_by_spin;
extern volatile float steer_yaw_request_deg;
extern uint8 yaw_hold_poweron_en;
extern float yaw_poweron_ref;
void yaw_hold_poweron_request_if_needed(void);
extern uint8 spin_enable;
extern uint8 spin_done;
extern int8 spin_dir;
extern float spin_target_deg;
extern float spin_accum_deg;
extern float spin_angle_err;
extern float spin_rate_max_dps;
extern float spin_rate_target_dps;
extern float spin_rate_meas_dps;
void pid_ctrl_Init(void);
void LQR_control(float V_target, float th);
float turn_control(float image_error);
void set_steer_cmd(float cmd);
void steer_set_target_yaw(float target_yaw_deg);
void steer_request_target_yaw(float target_yaw_deg);
void steer_request_relative_yaw(float delta_deg);
void steer_task_start(float delta_deg);
void steer_task_stop(void);
uint8 control_bridge_vision_track_valid(void);
void bridge_image_steer_reset(void);
void pid_ctrl_Run(void);
void leg_control(void);
void jump_set_step(int step_num);
void jump_control(void);
void dead_compensate(int16 *input_L, int16 *input_R);
void left_leg_control(float p, float angle);
void right_leg_control(float p, float angle);
void leg_debug_init_pwm(void);
void spin_task_start(float turns, int8 dir);
void spin_set_rate_max_dps(float rate_dps);
void spin_task_stop(void);
extern volatile float remote_lora_steer_rate_cmd_dps;
extern volatile uint8 remote_lora_steer_snapshot_valid;
extern uint8 g_remote_local_keys_debug;
extern uint8 g_menu_input_remote_first;
extern uint8 g_menu_vofa_enable;
extern uint8 g_menu_nav_fusion_enable;
uint8 remote_lora_nav_allows_heading_override(void);
uint8 remote_lora_nav_allows_spin_request(void);
typedef struct
{
    double angle;
    double speed;
    double distance;
    double distance_x,distance_y;
    double ins_x[400];
    double ins_y[400];
}ins_struct;
extern ins_struct ins;
extern double TempLat_Now;
extern double TempLon_Now;
extern double victual_point_lat[];
extern double victual_point_lon[];
extern uint8 Temp_num;
extern double Angle_Z_Quaternions;
void get_car_xy(void);
void ins_init(void);
float Get_Final_Angle(void);
double ange_deviation1(double angel1, double angel2);
#endif
