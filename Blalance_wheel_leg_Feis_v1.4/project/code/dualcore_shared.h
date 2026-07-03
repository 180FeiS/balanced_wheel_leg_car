/*********************************************************************************************************************
 * 双核共享内存与 UI->控制 命令队列（CM7_0 运动控制 / CM7_1 图像+菜单）
 * 共享区放在链接脚本保留的 0x28001000 起 8KB（.global_ram_data），与逐飞 E15 例程一致。
 *********************************************************************************************************************/
#ifndef CODE_DUALCORE_SHARED_H_
#define CODE_DUALCORE_SHARED_H_

#include "zf_common_typedef.h"
#include "step_detection.h"
#include "my_gps.h"

/* 与 project/iar/icf/linker_directives_tviibh.icf 中 ICFEDIT_region_RAM 起始地址一致 */
#ifndef DUALCORE_SHARED_PHYS_ADDR
#define DUALCORE_SHARED_PHYS_ADDR  (0x28001000u)
#endif

/* 1：CM7_1 负责 IPS/菜单/摄像头/台阶/无线；CM7_0 仅运动控制并发布 ctrl 快照。 */
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
  DUALCORE_UI_CMD_RUN_LAUNCH_SPEED_SET_ABS = 18, /* 菜单 Launch 页：arg_u32=字段索引，arg_f32=绝对值 */
  DUALCORE_UI_CMD_RUN_LAUNCH_SPEED_SAVE_FLASH = 19, /* Run/Save 菜单：保存全部 Launch 参数到 flash 页 47 */
  DUALCORE_UI_CMD_RUN_LAUNCH_PARAM_DELTA = 26, /* Launch 页：arg_u32=字段索引，arg_f32=增量 */
  DUALCORE_UI_CMD_GPS_SAVE_POINT = 20,
  DUALCORE_UI_CMD_GPS_CYCLE_ELEMENT = 21,
  DUALCORE_UI_CMD_GPS_SAVE_FLASH = 22,
  DUALCORE_UI_CMD_GPS_BEGIN_RECORD = 23,
  DUALCORE_UI_CMD_GPS_END_SAVE_FLASH = 24,
  DUALCORE_UI_CMD_GPS_LAUNCH = 25,
  DUALCORE_UI_CMD_RUN_CONFIG_TOGGLE = 27, /* Config 页：arg_u32=字段索引，切换当前字段取值 */
  DUALCORE_UI_CMD_RUN_JUMP_PARAM_DELTA = 28, /* Jump 页：arg_u32=字段索引，arg_f32=增量 */
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
  uint8 nav_recording_active; /* Nag_SystemRun_Index==1 && End_f==0 */
  uint8 event_active;
  uint8 event_state;
  uint8 event_active_type; /* 0=SPIN … 5=BRIDGE_IN 6=BRIDGE_OUT 7=BUMP 8=ENTER_STAIR 9=EXIT_STAIR … */
  uint8 event_record_type; /* N.Event_Record_Type，录制时 KEY3 循环切换 */
  uint8 nag_vofa_group;
  float mileage_debug_total;
  uint32 save_index;
  uint32 flash_page_index;
  float motor_user_speed_cmd;
  float run_launch_speed; /* 无元素速度，仅惯导回放进入执行态时装载到 motor_user_speed_cmd */
  float nag_spin_target_speed;
  float nag_spin_pre_decel_dist_cm;
  float nag_enter_turn_target_speed;
  float nag_enter_turn_pre_decel_dist_cm;
  float nag_exit_turn_recovery_speed;
  float nag_exit_turn_pre_accel_dist_cm;
  float nag_enter_cones_target_speed;
  float nag_enter_cones_pre_decel_dist_cm;
  float nag_enter_stair_pre_decel_dist_cm;
  float nag_enter_bridge_target_speed;      /* 单边桥进目标速度（Launch/Flash） */
  float nag_enter_bridge_pre_decel_dist_cm; /* 单边桥进预减速距离 cm */
  float spin_rate_max_dps;
  float speed_target_effective;
  uint8 spin_enable;
  uint8 spin_done;
  uint8 jump_allowed; /* jump_is_allowed()，为 0 时不应再发视觉跳跃命令 */
  uint8 jump_active;  /* jump_flag!=0，跳跃流程进行中 */
  uint8 remote_local_keys_debug; /* 1：g_menu_input_remote_first==1 且遥控拨码=板载调试电平；CM7_1 菜单/拨码路径对齐 0。仅 CM7_0 写入 */
  uint8 menu_input_remote_first; /* 0=按键+拨码 1=遥控优先；Run Config 可配，Save 写 Flash */
  uint8 menu_vofa_enable;      /* 0=关 1=开 VOFA 无线调试；Run Config 可配，Save 写 Flash V6 */
  uint8 menu_nav_fusion_enable; /* 0=关 1=开 GPS+惯导融合；Run Config 可配，Save 写 Flash V10 */
  float jump_takeoff_p;
  float jump_retract_p;
  float jump_prepare_p;
  float jump_buffer_p;
  float jump_stage_takeoff_cycles;
  float jump_stage_retract_cycles;
  float jump_stage_prepare_cycles;
  float jump_stage_buffer_cycles;
  float jump_buffer_step_p_max;
  uint8 gps_valid; /* 1：已解析到至少一帧 GNSS 数据 */
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
  float gps_nav_heading_bias_deg; /* Wrap180(地理 bearing − IMU)，标定 TRACKING 后有效 */
  uint8 gps_nav_align_state;       /* gps_nav_align_state_enum：WAIT=0 / TRACKING=1 */
  uint8 gps_drift_corr_valid;    /* 1：本次发车已计算经纬度漂移修正量 */
  float gps_drift_delta_lat;      /* 与 gps_drift_delta_lon：度，见 my_gps 头文件说明 */
  float gps_drift_delta_lon;
  float gps_nav_gps_first_deg;    /* RMC COG→±180°，≥ GPS_NAV_GPS_FIRST_DISTANCE_M 后锁定；WAIT 段常为 0 */
  float gps_nav_euler_ref_at_first_deg; /* 锁 GPS_first 当帧 IMU yaw（subject2 锚） */
  float gps_nav_dist_from_launch_m; /* 当前距发车锁存点位移（m），屏显 Lm */
  /* 以下 dbg_* 字段保留结构体布局，当前 VOFA 已不使用 */
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
  /* VOFA 组 2：GPS+惯导融合调试（nav_fusion） */
  float fusion_x_m;
  float fusion_y_m;
  float fusion_v_mps;
  float fusion_gps_residual_m;
  float fusion_gps_weight;
  uint8 fusion_gps_used;
  uint8 fusion_valid;
  /* 发车原点平均调试：calibrating=采集中；accepted 目标 NAV_FUSION_ORIGIN_SAMPLE_COUNT(50) */
  uint8 fusion_origin_calibrating;
  float fusion_origin_accepted;
  float fusion_origin_rejected;
  /* 北向角标定调试：state 见 NAV_FUSION_CALIB_*；hold_dist 为编码器累计里程(m) */
  float fusion_heading_bias_deg;
  uint8 fusion_calib_state;
  float fusion_hold_dist_m;
  float fusion_cog_deg;
  float fusion_imu_ref_deg;
  uint8 fusion_calib_failed;
  /* 1=惯导 ENTER_STAIR 接管中；CM7_1 仅此时跑 step_detect / step_visual_jump_after_step */
  uint8 stair_enter_active;
  /* 1=惯导 BridgeIn～BridgeOut；CM7_1 仅此时跑 single_bridge 并 publish 视觉误差 */
  uint8 bridge_zone_active;
  /* 1=桥预区或桥上：CM7_1 跑寻边+detect */
  uint8 bridge_detect_arm;
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
  /* jump_active==1 或 stair_enter_active==0 时 publish 写全 0 无效帧，勿作控车/决策 */
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
  volatile uint32 head; /* CM7_1 写 */
  volatile uint32 tail; /* CM7_0 读 */
  dualcore_ui_cmd_slot_t slot[DUALCORE_UI_CMD_QUEUE_DEPTH];
} dualcore_ui_cmd_fifo_t;

/* CM7_1 汇总 LORA 遥控快照 -> CM7_0；与 MENU_INPUT_REMOTE_MENU_FIRST 联动见 remote_lora.c */
struct dualcore_remote_to_ctrl
{
  volatile uint32 seq;
  uint8 enabled; /* 1：MENU_INPUT_REMOTE_MENU_FIRST==1，控制核可采信本快照 */
  uint8 online;  /* 1：lora3a22 链路判定在线 */
  uint8 fresh;   /* 1：本周期有新完整帧（每帧仅置位一次） */
  uint8 _pad;
  int16 left_x;  /* 左摇杆水平，中位 0，幅值见 REMOTE_LORA_JOYSTICK_ABS_MAX */
  int16 left_y;  /* 左摇杆垂直 */
  int16 right_x; /* 右摇杆水平 */
  int16 right_y; /* 右摇杆垂直 */
  uint8 key[4];       /* 按下 1 松开 0；离线默认为 0 */
  uint8 switch_key[4]; /* 遥控拨码 0/1；离线默认为 1（与需求默认一致） */
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
/* 由 cm7_0_isr / 主循环调用：执行队列中所有待处理命令 */
void dualcore_ui_cmd_consume_all(void);
void dualcore_remote_pull(dualcore_remote_to_ctrl_t *out);
#endif

#if defined(CY_CORE_CM7_1)
void dualcore_ctrl_to_ui_pull(dualcore_ctrl_to_ui_t *out);
uint8 dualcore_ui_cmd_push(dualcore_ui_cmd_op_t op, uint32 arg_u32, float arg_f32);
/* frame_seq 递增发布台阶快照；jump_active 或 !stair_enter_active 时 step 写无效零帧 */
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
void dualcore_remote_publish(const dualcore_remote_to_ctrl_t *in);
#endif

#endif /* CODE_DUALCORE_SHARED_H_ */
