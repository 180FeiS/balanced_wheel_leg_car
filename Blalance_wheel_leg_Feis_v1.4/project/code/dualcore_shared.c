/*********************************************************************************************************************
 * 双核共享区定义与缓存维护（两核各编一份，CM7_0 带初值，CM7_1 __no_init）
 *********************************************************************************************************************/
#include "dualcore_shared.h"
#include "cachel1_armv7.h"

#if defined(CY_CORE_CM7_0)
#include "matrix.h"
#include "ekf.h"
#include "control.h"
#include "navigation.h"
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
  c->nag_vofa_group = Nag_Vofa_Group;
  c->mileage_debug_total = N.Mileage_Debug_Total;
  c->save_index = (uint32)N.Save_index;
  c->flash_page_index = (uint32)N.Flash_page_index;
  c->motor_user_speed_cmd = motor_user_speed_cmd;
  c->speed_target_effective = speed_target_effective;
  c->spin_enable = spin_enable;
  c->spin_done = spin_done;
  c->jump_allowed = jump_is_allowed();
  c->jump_active = (uint8)(jump_flag ? 1u : 0u);

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
    Nag_Vofa_Group = (uint8)((Nag_Vofa_Group + 1u) % 6u);
    break;
  case DUALCORE_UI_CMD_SPIN_START:
    /* 元素接管或已在自旋时忽略，避免遥控/菜单调试互相抢状态 */
    if ((N.Event_Active != 0u) || (spin_enable != 0u))
    {
      break;
    }
    /* arg_u32: dir as int8 符号扩展在发送端保证 */
    spin_task_start(s->arg_f32, (int8)s->arg_u32);
    break;
  case DUALCORE_UI_CMD_STEER_REL_DEG:
    steer_request_relative_yaw(s->arg_f32);
    break;
  case DUALCORE_UI_CMD_SPEED_DELTA:
    motor_user_speed_cmd += s->arg_f32;
    break;
  case DUALCORE_UI_CMD_SPEED_SET_ABS:
    motor_user_speed_cmd = s->arg_f32;
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
  case DUALCORE_UI_CMD_NAG_IDLE_RESET:
    Init_Nag();
    break;
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
  dualcore_vision_to_ctrl_t *v = &g_dualcore_blob.vision;

  dualcore_shared_dcache_invalidate(v, sizeof(*v));

  extern step_info_t step_data;
  v->step = step_data;
  v->frame_seq = frame_seq;
  v->seq++;
  __DSB();

  dualcore_shared_dcache_clean(v, sizeof(*v));
}

#endif /* CY_CORE_CM7_1 */
