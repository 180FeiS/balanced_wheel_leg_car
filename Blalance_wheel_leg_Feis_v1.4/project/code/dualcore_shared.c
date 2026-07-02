/*********************************************************************************************************************
 * 双核共享区定义与缓存维护（两核各编一份，CM7_0 带初值，CM7_1 __no_init）
 *********************************************************************************************************************/
#include "dualcore_shared.h"
#include "cachel1_armv7.h"
#include "Menu.h"

#if defined(CY_CORE_CM7_0)
#include "matrix.h"
#include "ekf.h"
#include "control.h"
#include "navigation.h"
#include "flash.h"
#include "zf_device_gnss.h"
#include "nav_fusion.h"
#endif

#if defined(CY_CORE_CM7_0)
#pragma location = DUALCORE_SHARED_PHYS_ADDR
__root dualcore_shared_blob_t g_dualcore_blob = {0};
#elif defined(CY_CORE_CM7_1)
#pragma location = DUALCORE_SHARED_PHYS_ADDR
__no_init dualcore_shared_blob_t g_dualcore_blob;
#endif

void dualcore_shared_dcache_clean(const void *addr, uint32 size)
{
  if ((addr != NULL) && (size > 0u))
  {
    SCB_CleanDCache_by_Addr((void *)(uintptr_t)addr, (int32_t)size);
  }
}

void dualcore_shared_dcache_invalidate(const void *addr, uint32 size)
{
  if ((addr != NULL) && (size > 0u))
  {
    SCB_InvalidateDCache_by_Addr((void *)(uintptr_t)addr, (int32_t)size);
  }
}

#if defined(CY_CORE_CM7_0)

void dualcore_ctrl_to_ui_publish(void)
{
  dualcore_ctrl_to_ui_t *c = &g_dualcore_blob.ctrl;

  dualcore_shared_dcache_invalidate(c, sizeof(*c));

  c->euler_pitch = euler_angle.pitch;
  c->euler_roll = euler_angle.roll;
  c->euler_yaw = euler_angle.yaw;
  c->gyro_z_bias_mean = gyro_z_bias_mean;
  c->car_speed = (float)car_speed;
  c->left_motor_speed = (float)Left_Motor_Speed;
  c->right_motor_speed = (float)Right_Motor_Speed;
  c->left_motor_pwm = Left_Motor_Pwm;
  c->right_motor_pwm = Right_Motor_Pwm;
  c->motor_switch = Motor_Switch;
  c->nag_system_run_index = N.Nag_SystemRun_Index;
  c->end_f = (uint8)(N.End_f ? 1u : 0u);
  c->nav_recording_active = (uint8)((N.Nag_SystemRun_Index == 1u) && (N.End_f == 0));
  c->event_active = N.Event_Active;
  c->event_state = N.Event_State;
  c->event_active_type = N.Event_Active_Type;
  c->event_record_type = N.Event_Record_Type;
  c->nag_vofa_group = Nag_Vofa_Group;
  c->mileage_debug_total = N.Mileage_Debug_Total;
  c->save_index = (uint32)N.Save_index;
  c->flash_page_index = (uint32)N.Flash_page_index;
  c->motor_user_speed_cmd = motor_user_speed_cmd;
  c->run_launch_speed = run_launch_speed;
  c->nag_spin_target_speed = nag_spin_target_speed;
  c->nag_spin_pre_decel_dist_cm = nag_spin_pre_decel_dist_cm;
  c->nag_enter_turn_target_speed = nag_enter_turn_target_speed;
  c->nag_enter_turn_pre_decel_dist_cm = nag_enter_turn_pre_decel_dist_cm;
  c->nag_exit_turn_recovery_speed = nag_exit_turn_recovery_speed;
  c->nag_exit_turn_pre_accel_dist_cm = nag_exit_turn_pre_accel_dist_cm;
  c->nag_enter_cones_target_speed = nag_enter_cones_target_speed;
  c->nag_enter_cones_pre_decel_dist_cm = nag_enter_cones_pre_decel_dist_cm;
  c->nag_enter_stair_pre_decel_dist_cm = nag_enter_stair_pre_decel_dist_cm;
  c->nag_enter_bridge_target_speed = nag_enter_bridge_target_speed;
  c->nag_enter_bridge_pre_decel_dist_cm = nag_enter_bridge_pre_decel_dist_cm;
  c->spin_rate_max_dps = spin_rate_max_dps;
  c->speed_target_effective = speed_target_effective;
  c->spin_enable = spin_enable;
  c->spin_done = spin_done;
  c->jump_allowed = jump_is_allowed();
  c->jump_active = (uint8)(jump_flag ? 1u : 0u);
  c->stair_enter_active = (uint8)((N.Event_Active != 0u) &&
                                  (N.Event_Active_Type == NAG_EVENT_TYPE_ENTER_STAIR));
  c->remote_local_keys_debug = g_remote_local_keys_debug;
  c->menu_input_remote_first = g_menu_input_remote_first;
  c->menu_vofa_enable = g_menu_vofa_enable;
  c->menu_nav_fusion_enable = g_menu_nav_fusion_enable;
  c->jump_takeoff_p = jump_takeoff_p;
  c->jump_retract_p = jump_retract_p;
  c->jump_prepare_p = jump_prepare_p;
  c->jump_buffer_p = jump_buffer_p;
  c->jump_stage_takeoff_cycles = jump_stage_takeoff_cycles;
  c->jump_stage_retract_cycles = jump_stage_retract_cycles;
  c->jump_stage_prepare_cycles = jump_stage_prepare_cycles;
  c->jump_stage_buffer_cycles = jump_stage_buffer_cycles;
  c->jump_buffer_step_p_max = jump_buffer_step_p_max;
  c->gps_valid = (uint8)((gnss.time.year != 0u) || (gnss.state != 0u) || (gnss.satellite_used != 0u));
  c->gps_year = gnss.time.year;
  c->gps_month = gnss.time.month;
  c->gps_day = gnss.time.day;
  c->gps_hour = gnss.time.hour;
  c->gps_minute = gnss.time.minute;
  c->gps_second = gnss.time.second;
  c->gps_state = gnss.state;
  c->gps_satellite_used = gnss.satellite_used;
  c->gps_latitude = gnss.latitude;
  c->gps_longitude = gnss.longitude;
  c->gps_speed = gnss.speed;
  c->gps_direction = gnss.direction;
  c->gps_height = gnss.height;
  c->gps_point_count = gps_point_count;
  c->gps_save_point = save_point;
  c->gps_show_point = show_point;
  c->gps_recording_active = gps_recording_active;
  c->gps_current_yuansu = gps_current_yuansu;
  memcpy(c->gps_latitude_point, latitude_point, sizeof(c->gps_latitude_point));
  memcpy(c->gps_longitude_point, longitude_point, sizeof(c->gps_longitude_point));
  memcpy(c->gps_yuansu, u32yuansu, sizeof(c->gps_yuansu));
  c->gps_nav_state = gps_nav_state;
  c->gps_nav_protect_reason = gps_nav_protect_reason;
  c->gps_nav_target_index = gps_nav_target_index;
  c->gps_nav_target_latitude = gps_nav_target_latitude;
  c->gps_nav_target_longitude = gps_nav_target_longitude;
  c->gps_nav_distance_m = gps_nav_distance_m;
  c->gps_nav_geo_bearing_deg = gps_nav_geo_bearing_deg;
  c->gps_nav_body_target_yaw_deg = gps_nav_body_target_yaw_deg;
  c->gps_nav_target_imu_yaw_deg = gps_nav_target_imu_yaw_deg;
  c->gps_nav_imu_yaw_deg = gps_nav_imu_yaw_deg;
  c->gps_nav_yaw_err_deg = gps_nav_yaw_err_deg;
  c->gps_nav_heading_bias_deg = gps_nav_heading_bias_deg;
  c->gps_nav_align_state = gps_nav_align_state;
  c->gps_drift_corr_valid = gps_drift_corr_valid;
  c->gps_drift_delta_lat = (float)gps_drift_delta_lat;
  c->gps_drift_delta_lon = (float)gps_drift_delta_lon;
  c->gps_nav_gps_first_deg = gps_nav_gps_first_deg;
  c->gps_nav_euler_ref_at_first_deg = gps_nav_euler_ref_at_first_deg;
  c->gps_nav_dist_from_launch_m = gps_nav_dist_from_launch_m;

#if NAV_FUSION_ENABLE
  if (g_menu_nav_fusion_enable != 0u)
  {
    const NavFusionState *fusion_st = NavFusion_GetState();
    if (fusion_st != NULL)
    {
      c->fusion_x_m = fusion_st->x_m;
      c->fusion_y_m = fusion_st->y_m;
      c->fusion_v_mps = fusion_st->v_mps;
      c->fusion_gps_residual_m = fusion_st->gps_residual_m;
      c->fusion_gps_weight = fusion_st->gps_weight;
      c->fusion_gps_used = fusion_st->gps_used;
      c->fusion_valid = fusion_st->valid;
      c->fusion_origin_calibrating = NavFusion_IsOriginCalibrating();
      c->fusion_origin_accepted = (float)NavFusion_GetOriginAcceptedCount();
      c->fusion_origin_rejected = (float)NavFusion_GetOriginRejectedCount();
#if NAV_FUSION_HEADING_CALIB_ENABLE
      c->fusion_heading_bias_deg = NavFusion_GetHeadingBiasDeg();
      c->fusion_calib_state = NavFusion_GetHeadingCalibState();
      c->fusion_hold_dist_m = NavFusion_GetHoldDistM();
      c->fusion_cog_deg = NavFusion_GetHeadingCogDeg();
      c->fusion_imu_ref_deg = NavFusion_GetHeadingImuRefDeg();
      c->fusion_calib_failed = (uint8)(NavFusion_GetHeadingCalibState() == NAV_FUSION_CALIB_FAILED);
#endif
    }
    else
    {
      c->fusion_x_m = 0.0f;
      c->fusion_y_m = 0.0f;
      c->fusion_v_mps = 0.0f;
      c->fusion_gps_residual_m = 0.0f;
      c->fusion_gps_weight = 0.0f;
      c->fusion_gps_used = 0u;
      c->fusion_valid = 0u;
      c->fusion_origin_calibrating = 0u;
      c->fusion_origin_accepted = 0.0f;
      c->fusion_origin_rejected = 0.0f;
#if NAV_FUSION_HEADING_CALIB_ENABLE
      c->fusion_heading_bias_deg = 0.0f;
      c->fusion_calib_state = NAV_FUSION_CALIB_IDLE;
      c->fusion_hold_dist_m = 0.0f;
      c->fusion_cog_deg = 0.0f;
      c->fusion_imu_ref_deg = 0.0f;
      c->fusion_calib_failed = 0u;
#endif
    }
  }
  else
  {
    c->fusion_x_m = 0.0f;
    c->fusion_y_m = 0.0f;
    c->fusion_v_mps = 0.0f;
    c->fusion_gps_residual_m = 0.0f;
    c->fusion_gps_weight = 0.0f;
    c->fusion_gps_used = 0u;
    c->fusion_valid = 0u;
    c->fusion_origin_calibrating = 0u;
    c->fusion_origin_accepted = 0.0f;
    c->fusion_origin_rejected = 0.0f;
#if NAV_FUSION_HEADING_CALIB_ENABLE
    c->fusion_heading_bias_deg = 0.0f;
    c->fusion_calib_state = NAV_FUSION_CALIB_IDLE;
    c->fusion_hold_dist_m = 0.0f;
    c->fusion_cog_deg = 0.0f;
    c->fusion_imu_ref_deg = 0.0f;
    c->fusion_calib_failed = 0u;
#endif
  }
#else
  c->fusion_x_m = 0.0f;
  c->fusion_y_m = 0.0f;
  c->fusion_v_mps = 0.0f;
  c->fusion_gps_residual_m = 0.0f;
  c->fusion_gps_weight = 0.0f;
  c->fusion_gps_used = 0u;
  c->fusion_valid = 0u;
  c->fusion_origin_calibrating = 0u;
  c->fusion_origin_accepted = 0.0f;
  c->fusion_origin_rejected = 0.0f;
#if NAV_FUSION_HEADING_CALIB_ENABLE
  c->fusion_heading_bias_deg = 0.0f;
  c->fusion_calib_state = NAV_FUSION_CALIB_IDLE;
  c->fusion_hold_dist_m = 0.0f;
  c->fusion_cog_deg = 0.0f;
  c->fusion_imu_ref_deg = 0.0f;
  c->fusion_calib_failed = 0u;
#endif
#endif

  c->seq++;
  __DSB();

  dualcore_shared_dcache_clean(c, sizeof(*c));
}

void dualcore_vision_to_ctrl_pull_step(step_info_t *out, uint32 *frame_seq_out, uint32 *vision_seq_out)
{
  if (out != NULL)
  {
    dualcore_shared_dcache_invalidate(&g_dualcore_blob.vision, sizeof(g_dualcore_blob.vision));
    *out = g_dualcore_blob.vision.step;
    if (frame_seq_out != NULL)
    {
      *frame_seq_out = g_dualcore_blob.vision.frame_seq;
    }
    if (vision_seq_out != NULL)
    {
      *vision_seq_out = g_dualcore_blob.vision.seq;
    }
  }
}

static void dualcore_apply_one_ui_cmd(const dualcore_ui_cmd_slot_t *s)
{
  switch ((dualcore_ui_cmd_op_t)s->op)
  {
  case DUALCORE_UI_CMD_NONE:
    break;
  case DUALCORE_UI_CMD_JUMP:
    if ((jump_is_allowed() != 0u) && (jump_flag == 0u))
    {
      jump_flag = 1u;
    }
    break;
  case DUALCORE_UI_CMD_ROLL_BALANCE_TOGGLE:
    roll_balance_en = (uint8)(roll_balance_en ? 0u : 1u);
    break;
  case DUALCORE_UI_CMD_NAG_BEGIN_RECORD:
    Nag_Begin_Record();
    break;
  case DUALCORE_UI_CMD_NAG_BEGIN_REPLAY:
    Nag_Begin_Replay();
    break;
  case DUALCORE_UI_CMD_NAG_STOP_RECORD:
    Nag_Request_Stop_Record();
    break;
  case DUALCORE_UI_CMD_NAG_VOFA_GROUP_NEXT:
    Nag_Vofa_Group = (uint8)((Nag_Vofa_Group + 1u) % NAG_VOFA_GROUP_COUNT);
    break;
  case DUALCORE_UI_CMD_SPIN_START:
    /* arg_u32: dir as int8 符号扩展在发送端保证 */
    spin_task_start(s->arg_f32, (int8)s->arg_u32);
    break;
  case DUALCORE_UI_CMD_STEER_REL_DEG:
    steer_request_relative_yaw(s->arg_f32);
    break;
  case DUALCORE_UI_CMD_SPEED_DELTA:
    run_launch_speed += s->arg_f32;
    break;
  case DUALCORE_UI_CMD_SPEED_SET_ABS:
    run_launch_speed = s->arg_f32;
    break;
  case DUALCORE_UI_CMD_NAG_EVENT_MARK:
    Nag_Request_Event_Mark();
    break;
  case DUALCORE_UI_CMD_NAG_CYCLE_EVENT_TYPE:
    Nag_Cycle_Record_Event_Type();
    break;
  case DUALCORE_UI_CMD_NAG_EVENT_DONE:
    Nag_Notify_Event_Done();
    break;
  case DUALCORE_UI_CMD_MOTOR_SPEED_FROM_PC:
    motor_user_speed_cmd_set_from_pc(s->arg_f32);
    break;
  case DUALCORE_UI_CMD_RUN_LAUNCH_SPEED_SET_ABS:
    if (s->arg_u32 < Nag_Run_Launch_Param_Count)
    {
      Nag_LaunchParamSet((uint8)s->arg_u32, s->arg_f32);
    }
    break;
  case DUALCORE_UI_CMD_RUN_LAUNCH_SPEED_SAVE_FLASH:
    flash_RunLaunchSpeed_Write();
    flash_JumpParams_Write();
    break;
  case DUALCORE_UI_CMD_RUN_LAUNCH_PARAM_DELTA:
    if (s->arg_u32 < Nag_Run_Launch_Param_Count)
    {
      Nag_LaunchParamAdjust((uint8)s->arg_u32, s->arg_f32);
    }
    break;
  case DUALCORE_UI_CMD_RUN_CONFIG_TOGGLE:
    Menu_RunConfigToggleField((uint8)s->arg_u32);
    break;
  case DUALCORE_UI_CMD_RUN_JUMP_PARAM_DELTA:
    if (s->arg_u32 < Run_Jump_Param_Count)
    {
      JumpParamAdjust((uint8)s->arg_u32, s->arg_f32);
    }
    break;
  case DUALCORE_UI_CMD_GPS_SAVE_POINT:
    (void)GPS_SaveCurrentPointFromCoord(gnss.latitude, gnss.longitude);
    break;
  case DUALCORE_UI_CMD_GPS_CYCLE_ELEMENT:
    (void)GPS_CycleCurrentElement();
    break;
  case DUALCORE_UI_CMD_GPS_SAVE_FLASH:
    flash_GpsPoints_Write();
    break;
  case DUALCORE_UI_CMD_GPS_BEGIN_RECORD:
    GPS_BeginRecord();
    break;
  case DUALCORE_UI_CMD_GPS_END_SAVE_FLASH:
    GPS_EndRecord();
    flash_GpsPoints_Write();
    break;
  case DUALCORE_UI_CMD_GPS_LAUNCH:
    GPS_ApplyLaunchSpeed();
    break;
  case DUALCORE_UI_CMD_KEY_NAV_RECORD:
    Nag_Begin_Record();
    break;
  case DUALCORE_UI_CMD_KEY_NAV_STOP_REC:
    Nag_Request_Stop_Record();
    break;
  case DUALCORE_UI_CMD_KEY_NAV_KEY3:
  {
    uint8 nav_recording_active = (uint8)((N.Nag_SystemRun_Index == 1u) && (N.End_f == 0));
    if (nav_recording_active)
    {
      Nag_Cycle_Record_Event_Type();
    }
    else
    {
      Nag_Begin_Replay();
    }
    break;
  }
  default:
    break;
  }
}

void dualcore_ui_cmd_consume_all(void)
{
  dualcore_ui_cmd_fifo_t *f = &g_dualcore_blob.fifo;

  for (;;)
  {
    dualcore_shared_dcache_invalidate(f, sizeof(*f));
    if (f->tail == f->head)
    {
      break;
    }

    uint32 idx = f->tail % DUALCORE_UI_CMD_QUEUE_DEPTH;
    dualcore_ui_cmd_slot_t local = f->slot[idx];

    f->tail++;
    __DSB();
    dualcore_shared_dcache_clean(f, sizeof(*f));

    dualcore_apply_one_ui_cmd(&local);
  }
}

void dualcore_remote_pull(dualcore_remote_to_ctrl_t *out)
{
  if (out == NULL)
  {
    return;
  }
  dualcore_shared_dcache_invalidate(&g_dualcore_blob.remote, sizeof(g_dualcore_blob.remote));
  *out = g_dualcore_blob.remote;
}

#endif /* CY_CORE_CM7_0 */

#if defined(CY_CORE_CM7_1)

void dualcore_ctrl_to_ui_pull(dualcore_ctrl_to_ui_t *out)
{
  if (out == NULL)
  {
    return;
  }
  dualcore_shared_dcache_invalidate(&g_dualcore_blob.ctrl, sizeof(g_dualcore_blob.ctrl));
  *out = g_dualcore_blob.ctrl;
}

uint8 dualcore_ui_cmd_push(dualcore_ui_cmd_op_t op, uint32 arg_u32, float arg_f32)
{
  dualcore_ui_cmd_fifo_t *f = &g_dualcore_blob.fifo;

  dualcore_shared_dcache_invalidate(f, sizeof(*f));

  if ((f->head - f->tail) >= DUALCORE_UI_CMD_QUEUE_DEPTH)
  {
    return 0u;
  }

  uint32 idx = f->head % DUALCORE_UI_CMD_QUEUE_DEPTH;
  f->slot[idx].op = (uint8)op;
  f->slot[idx].arg_u32 = arg_u32;
  f->slot[idx].arg_f32 = arg_f32;
  f->head++;
  __DSB();

  dualcore_shared_dcache_clean(f, sizeof(*f));
  return 1u;
}

void dualcore_vision_publish_after_step(uint32 frame_seq)
{
  dualcore_ctrl_to_ui_t ctrl;
  dualcore_ctrl_to_ui_pull(&ctrl);

  dualcore_vision_to_ctrl_t *v = &g_dualcore_blob.vision;

  dualcore_shared_dcache_invalidate(v, sizeof(*v));

  extern step_info_t step_data;
  if ((ctrl.jump_active != 0u) || (ctrl.stair_enter_active == 0u))
  {
    step_info_t invalid_step = {0};
    invalid_step.detected = 0u;
    invalid_step.step_row = 0u;
    invalid_step.step_top_row = 0u;
    invalid_step.step_height_pix = 0u;
    invalid_step.distance_mm = 0.0f;
    invalid_step.distance_cm = 0.0f;
    invalid_step.bottom_row_raw = 0u;
    v->step = invalid_step;
  }
  else
  {
    v->step = step_data;
  }
  v->frame_seq = frame_seq;
  v->seq++;
  __DSB();

  dualcore_shared_dcache_clean(v, sizeof(*v));
}

void dualcore_remote_publish(const dualcore_remote_to_ctrl_t *in)
{
  if (in == NULL)
  {
    return;
  }
  dualcore_remote_to_ctrl_t *r = &g_dualcore_blob.remote;

  dualcore_shared_dcache_invalidate(r, sizeof(*r));
  *r = *in;
  r->seq++;
  __DSB();
  dualcore_shared_dcache_clean(r, sizeof(*r));
}

#endif /* CY_CORE_CM7_1 */
