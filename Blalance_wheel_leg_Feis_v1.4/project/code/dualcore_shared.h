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
  DUALCORE_UI_CMD_RUN_LAUNCH_SPEED_SET_ABS = 18, /* 菜单发车速度页专用：只更新 run_launch_speed，不立即改变运行速度 */
  DUALCORE_UI_CMD_RUN_LAUNCH_SPEED_SAVE_FLASH = 19, /* Run/Flash 菜单专用：保存 run_launch_speed 到独立 flash 参数页 */
  DUALCORE_UI_CMD_GPS_SAVE_POINT = 20,
  DUALCORE_UI_CMD_GPS_CYCLE_ELEMENT = 21,
  DUALCORE_UI_CMD_GPS_SAVE_FLASH = 22,
  DUALCORE_UI_CMD_GPS_BEGIN_RECORD = 23,
  DUALCORE_UI_CMD_GPS_END_SAVE_FLASH = 24,
  DUALCORE_UI_CMD_GPS_LAUNCH = 25,
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
  uint8 event_active_type; /* N.Event_Active_Type：0=SPIN 1=TURNAROUND 2=ENTER_CONES 3=EXIT_CONES … */
  uint8 nag_vofa_group;
  float mileage_debug_total;
  uint32 save_index;
  uint32 flash_page_index;
  float motor_user_speed_cmd;
  float run_launch_speed; /* 发车速度设定值，仅惯导回放进入执行态时装载 */
  float speed_target_effective;
  uint8 spin_enable;
  uint8 spin_done;
  uint8 jump_allowed; /* jump_is_allowed()，为 0 时不应再发视觉跳跃命令 */
  uint8 jump_active;  /* jump_flag!=0，跳跃流程进行中 */
  uint8 remote_local_keys_debug; /* 1：MENU_INPUT_REMOTE_MENU_FIRST==1 且遥控拨码=板载调试电平；CM7_1 菜单/拨码路径对齐宏=0。仅 CM7_0 写入 */
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
  /* --- 以下仅由 CM7_0 publish，供 CM7_1 走无线 VOFA 复现 main_cm7_0.c:send_nav_debug_to_vofa 第 1~3 组缺失量（与 mileage_debug_total 等并存不重复） --- */
  float dbg_run_index;        /* N.Run_index → 浮点 */
  float dbg_prospect_index;   /* Nag_GetDebugProspectIndex() */
  float dbg_angle_run;        /* N.Angle_Run */
  float dbg_read_yaw;         /* Nag_GetDebugReadYaw() */
  float dbg_nag_stop;         /* N.Nag_Stop_f：0/1 */
  float dbg_final_out;        /* N.Final_Out */
  float dbg_curve_strength;   /* N.Curve_Strength */
  float dbg_nav_speed_target; /* Nag_GetControlSpeedTarget()，与 speed_target_effective 同源不同用途时见 navigation */
  /* VOFA 组 8：差速转向闭环（CM7_0 publish，供 CM7_1 JustFloat） */
  float dbg_steer_target_yaw_deg;
  float dbg_steer_angle_err;
  float dbg_steer_cmd;
  float dbg_steer_enable;
  float dbg_steer_yaw_request_pending;
  float dbg_steer_yaw_request_deg;
} dualcore_ctrl_to_ui_t;

typedef struct
{
  volatile uint32 seq;
  /* jump_active==1（CM7_0 jump_flag）时 dualcore_vision_publish_after_step 故意写全 0 无效帧，勿作控车/决策 */
  step_info_t step;
  uint32 frame_seq;
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
/* 由 cm7_0_isr / 主循环调用：执行队列中所有待处理命令 */
void dualcore_ui_cmd_consume_all(void);
void dualcore_remote_pull(dualcore_remote_to_ctrl_t *out);
#endif

#if defined(CY_CORE_CM7_1)
void dualcore_ctrl_to_ui_pull(dualcore_ctrl_to_ui_t *out);
uint8 dualcore_ui_cmd_push(dualcore_ui_cmd_op_t op, uint32 arg_u32, float arg_f32);
/* frame_seq 递增发布台阶快照；jump_active 时 step 写无效零帧（见 dualcore_vision_to_ctrl_t） */
void dualcore_vision_publish_after_step(uint32 frame_seq);
void dualcore_remote_publish(const dualcore_remote_to_ctrl_t *in);
#endif

#endif /* CODE_DUALCORE_SHARED_H_ */
