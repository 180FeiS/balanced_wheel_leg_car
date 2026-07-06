#ifndef CODE_DUALCORE_SHARED_H_
#define CODE_DUALCORE_SHARED_H_
#include "zf_common_typedef.h"
#include "step_detection.h"
#include "my_gps.h"
#ifndef DUALCORE_SHARED_PHYS_ADDR
#define DUALCORE_SHARED_PHYS_ADDR  (0x28001000u)
#endif
#ifndef DUALCORE_UI_ON_CM7_1
#define DUALCORE_UI_ON_CM7_1  (1)
#endif
#define DUALCORE_UI_CMD_QUEUE_DEPTH  (32u)
typedef enum
{
  DUALCORE_UI_CMD_NONE = 0,
  DUALCORE_UI_CMD_JUMP = 1,
  DUALCORE_UI_CMD_ROLL_BALANCE_TOGGLE = 2,
  DUALCORE_UI_CMD_NAG_BEGIN_RECORD = 3,
  DUALCORE_UI_CMD_NAG_BEGIN_REPLAY = 4,
  DUALCORE_UI_CMD_NAG_STOP_RECORD = 5,
  DUALCORE_UI_CMD_NAG_VOFA_GROUP_NEXT = 6,
  DUALCORE_UI_CMD_SPIN_START = 7,
  DUALCORE_UI_CMD_STEER_REL_DEG = 8,
  DUALCORE_UI_CMD_SPEED_DELTA = 9,
  DUALCORE_UI_CMD_SPEED_SET_ABS = 10,
  DUALCORE_UI_CMD_NAG_EVENT_MARK = 11,
  DUALCORE_UI_CMD_NAG_CYCLE_EVENT_TYPE = 12,
  DUALCORE_UI_CMD_NAG_EVENT_DONE = 13,
  DUALCORE_UI_CMD_MOTOR_SPEED_FROM_PC = 14,
  DUALCORE_UI_CMD_KEY_NAV_RECORD = 15,
  DUALCORE_UI_CMD_KEY_NAV_STOP_REC = 16,
  DUALCORE_UI_CMD_KEY_NAV_KEY3 = 17,
  DUALCORE_UI_CMD_RUN_LAUNCH_SPEED_SET_ABS = 18,
  DUALCORE_UI_CMD_RUN_LAUNCH_SPEED_SAVE_FLASH = 19,
  DUALCORE_UI_CMD_RUN_LAUNCH_PARAM_DELTA = 26,
  DUALCORE_UI_CMD_GPS_SAVE_POINT = 20,
  DUALCORE_UI_CMD_GPS_CYCLE_ELEMENT = 21,
  DUALCORE_UI_CMD_GPS_SAVE_FLASH = 22,
  DUALCORE_UI_CMD_GPS_BEGIN_RECORD = 23,
  DUALCORE_UI_CMD_GPS_END_SAVE_FLASH = 24,
  DUALCORE_UI_CMD_GPS_LAUNCH = 25,
  DUALCORE_UI_CMD_RUN_CONFIG_TOGGLE = 27,
  DUALCORE_UI_CMD_RUN_JUMP_PARAM_DELTA = 28,
} dualcore_ui_cmd_op_t;
typedef struct
{
  volatile uint32 seq;
  float euler_pitch;
  float euler_roll;
  float euler_yaw;
  float gyro_z_bias_mean;
  float car_speed;
  float left_motor_speed;
  float right_motor_speed;
  int16 left_motor_pwm;
  int16 right_motor_pwm;
  uint8 motor_switch;
  uint8 nag_system_run_index;
  uint8 end_f;
  uint8 nav_recording_active;
  uint8 event_active;
  uint8 event_state;
  uint8 event_active_type;
  uint8 event_record_type;
  uint8 nag_vofa_group;
  float mileage_debug_total;
  uint32 save_index;
  uint32 flash_page_index;
  float motor_user_speed_cmd;
  float run_launch_speed;
  float nag_spin_target_speed;
  float nag_spin_pre_decel_dist_cm;
  float nag_enter_turn_target_speed;
  float nag_enter_turn_pre_decel_dist_cm;
  float nag_exit_turn_recovery_speed;
  float nag_exit_turn_pre_accel_dist_cm;
  float nag_enter_cones_target_speed;
  float nag_enter_cones_pre_decel_dist_cm;
  float nag_enter_stair_pre_decel_dist_cm;
  float nag_enter_bridge_target_speed;
  float nag_enter_bridge_pre_decel_dist_cm;
  float spin_rate_max_dps;
  float speed_target_effective;
  uint8 spin_enable;
  uint8 spin_done;
  uint8 jump_allowed;
  uint8 jump_active;
  uint8 remote_local_keys_debug;
  uint8 menu_input_remote_first;
  uint8 menu_vofa_enable;
  uint8 menu_nav_fusion_enable;
  float jump_takeoff_p;
  float jump_retract_p;
  float jump_prepare_p;
  float jump_buffer_p;
  float jump_stage_takeoff_cycles;
  float jump_stage_retract_cycles;
  float jump_stage_prepare_cycles;
  float jump_stage_buffer_cycles;
  float jump_buffer_step_p_max;
  uint8 gps_valid;
  uint16 gps_year;
  uint8 gps_month;
  uint8 gps_day;
  uint8 gps_hour;
  uint8 gps_minute;
  uint8 gps_second;
  uint8 gps_state;
  uint8 gps_satellite_used;
  double gps_latitude;
  double gps_longitude;
  float gps_speed;
  float gps_direction;
  float gps_height;
  uint8 gps_point_count;
  uint8 gps_save_point;
  uint8 gps_show_point;
  uint8 gps_recording_active;
  uint32 gps_current_yuansu;
  double gps_latitude_point[GPS_POINT_MAX];
  double gps_longitude_point[GPS_POINT_MAX];
  uint32 gps_yuansu[GPS_POINT_MAX];
  uint8 gps_nav_state;
  uint8 gps_nav_protect_reason;
  uint8 gps_nav_target_index;
  double gps_nav_target_latitude;
  double gps_nav_target_longitude;
  float gps_nav_distance_m;
  float gps_nav_geo_bearing_deg;
  float gps_nav_body_target_yaw_deg;
  float gps_nav_target_imu_yaw_deg;
  float gps_nav_imu_yaw_deg;
  float gps_nav_yaw_err_deg;
  float gps_nav_heading_bias_deg;
  uint8 gps_nav_align_state;
  uint8 gps_drift_corr_valid;
  float gps_drift_delta_lat;
  float gps_drift_delta_lon;
  float gps_nav_gps_first_deg;
  float gps_nav_euler_ref_at_first_deg;
  float gps_nav_dist_from_launch_m;
  float dbg_run_index;
  float dbg_prospect_index;
  float dbg_angle_run;
  float dbg_read_yaw;
  float dbg_nag_stop;
  float dbg_final_out;
  float dbg_curve_strength;
  float dbg_nav_speed_target;
  float dbg_steer_target_yaw_deg;
  float dbg_steer_angle_err;
  float dbg_steer_cmd;
  float dbg_steer_enable;
  float dbg_steer_yaw_request_pending;
  float dbg_steer_yaw_request_deg;
  float fusion_x_m;
  float fusion_y_m;
  float fusion_v_mps;
  float fusion_gps_residual_m;
  float fusion_gps_weight;
  uint8 fusion_gps_used;
  uint8 fusion_valid;
  uint8 fusion_origin_calibrating;
  float fusion_origin_accepted;
  float fusion_origin_rejected;
  float fusion_heading_bias_deg;
  uint8 fusion_calib_state;
  float fusion_hold_dist_m;
  float fusion_cog_deg;
  float fusion_imu_ref_deg;
  uint8 fusion_calib_failed;
  uint8 stair_enter_active;
  uint8 bridge_zone_active;
  uint8 bridge_detect_arm;
  float odo_wheel_left_cmps;
  float odo_wheel_right_cmps;
  float odo_gyro_z_dps;
  float odo_vc_from_l_cmps;
  float odo_vc_from_r_cmps;
  float odo_corr_speed_cmps;
  float odo_slip_state;
  float odo_protected_step_cm;
  float odo_rollback_applied_cm;
} dualcore_ctrl_to_ui_t;
typedef struct
{
  float center_err;
  uint8 track_valid;
  uint8 fresh;
  float road_w_avg;
  uint8 pin_left;
  uint8 pin_right;
  uint8 enter_ready;
  uint8 exit_ready;
  uint8 detect_enter;
  uint8 detect_exit;
  uint8 detect_side;
} dualcore_bridge_vision_snapshot_t;
typedef struct
{
  volatile uint32 seq;
  step_info_t step;
  uint32 frame_seq;
  float bridge_center_err;
  uint8 bridge_track_valid;
  uint8 bridge_frame_fresh;
  uint8 bridge_detect_enter;
  uint8 bridge_detect_exit;
  float bridge_road_w_avg;
  uint8 bridge_pin_left;
  uint8 bridge_pin_right;
  uint8 bridge_enter_ready;
  uint8 bridge_exit_ready;
  uint8 bridge_detect_side;
  uint32 bridge_frame_seq;
  float blob_center_err;
  uint8 blob_track_valid;
  uint8 blob_frame_fresh;
  uint32 blob_frame_seq;
  uint8 vision_guidance_mode;
} dualcore_vision_to_ctrl_t;
typedef struct
{
  uint8 op;
  uint8 _pad[3];
  uint32 arg_u32;
  float arg_f32;
} dualcore_ui_cmd_slot_t;
typedef struct
{
  volatile uint32 head;
  volatile uint32 tail;
  dualcore_ui_cmd_slot_t slot[DUALCORE_UI_CMD_QUEUE_DEPTH];
} dualcore_ui_cmd_fifo_t;
struct dualcore_remote_to_ctrl
{
  volatile uint32 seq;
  uint8 enabled;
  uint8 online;
  uint8 fresh;
  uint8 _pad;
  int16 left_x;
  int16 left_y;
  int16 right_x;
  int16 right_y;
  uint8 key[4];
  uint8 switch_key[4];
};
typedef struct dualcore_remote_to_ctrl dualcore_remote_to_ctrl_t;
typedef struct
{
  dualcore_ctrl_to_ui_t ctrl;
  dualcore_vision_to_ctrl_t vision;
  dualcore_ui_cmd_fifo_t fifo;
  dualcore_remote_to_ctrl_t remote;
} dualcore_shared_blob_t;
extern dualcore_shared_blob_t g_dualcore_blob;
void dualcore_shared_dcache_clean(const void *addr, uint32 size);
void dualcore_shared_dcache_invalidate(const void *addr, uint32 size);
#if defined(CY_CORE_CM7_0)
void dualcore_ctrl_to_ui_publish(void);
void dualcore_vision_to_ctrl_pull_step(step_info_t *out, uint32 *frame_seq_out, uint32 *vision_seq_out);
void dualcore_bridge_vision_pull(float *center_err, uint8 *track_valid, uint8 *fresh);
void dualcore_bridge_vision_pull_snapshot(dualcore_bridge_vision_snapshot_t *out);
void dualcore_white_blob_pull(float *center_err, uint8 *track_valid, uint8 *fresh);
uint8 dualcore_white_blob_read_track_valid(void);
uint8 dualcore_vision_guidance_pull_mode(void);
void dualcore_ui_cmd_consume_all(void);
void dualcore_remote_pull(dualcore_remote_to_ctrl_t *out);
#endif
#if defined(CY_CORE_CM7_1)
void dualcore_ctrl_to_ui_pull(dualcore_ctrl_to_ui_t *out);
uint8 dualcore_ui_cmd_push(dualcore_ui_cmd_op_t op, uint32 arg_u32, float arg_f32);
void dualcore_vision_publish_after_step(uint32 frame_seq);
void dualcore_bridge_vision_publish(float center_err, uint8 track_valid, uint32 frame_seq);
void dualcore_bridge_vision_publish_detect(float center_err, uint8 track_valid,
                                           float road_w_avg,
                                           uint8 pin_left, uint8 pin_right,
                                           uint8 enter_ready, uint8 exit_ready,
                                           uint8 detect_enter, uint8 detect_exit,
                                           uint8 detect_side,
                                           uint32 frame_seq);
void dualcore_bridge_vision_publish_inactive(void);
void dualcore_white_blob_publish(float center_err, uint8 track_valid, uint32 frame_seq);
void dualcore_white_blob_publish_inactive(void);
void dualcore_remote_publish(const dualcore_remote_to_ctrl_t *in);
#endif
#endif
