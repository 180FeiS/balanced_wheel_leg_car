/*********************************************************************************************************************
 * LoRa 遥控：CM7_1 解析 + CM7_0 应用（单文件双核条件编译）
 *********************************************************************************************************************/
#include "remote_control.h"

#if REMOTE_CONTROL_ENABLE

#include "dualcore_shared.h"
#include <math.h>
#include <string.h>

#if defined(CY_CORE_CM7_1)
#include "zf_device_lora3a22.h"
#endif

#if defined(CY_CORE_CM7_0)
#include "navigation.h"
#include "control.h"
#include "small_driver_uart_control.h"
#endif

#if defined(CY_CORE_CM7_1)

static uint8 s_edge_inited;
static uint8 s_prev_key[4];
static uint8 s_prev_sw[4];
static uint8 s_motor_arm;
static uint8 s_remote_takeover; /* 左摇杆下压接过权后才干预 CM7_0 电机与速度 */
static uint8 s_arm_toggle_cd;   /* 距上次切换 motor_arm 未满 N 帧则忽略边沿 */

static int16 rc_norm_axis(int16 raw, int out_max, int sign)
{
  int32 x = ((int32)raw - (int32)REMOTE_JOYSTICK_CENTER) * (int32)sign;
  if (x > -(int32)REMOTE_JOYSTICK_DEADBAND && x < (int32)REMOTE_JOYSTICK_DEADBAND)
  {
    x = 0;
  }
  int32 span = (int32)REMOTE_JOYSTICK_HALF_SPAN;
  if (span <= 0)
  {
    span = 1;
  }
  x = (x * (int32)out_max) / span;
  if (x > (int32)out_max)
  {
    x = (int32)out_max;
  }
  if (x < -(int32)out_max)
  {
    x = -(int32)out_max;
  }
  return (int16)x;
}

static void rc_publish_to_blob(const dualcore_remote_from_ui_t *in)
{
  dualcore_remote_from_ui_t *r = &g_dualcore_blob.remote;
  uint32 prev_seq = 0;

  dualcore_shared_dcache_invalidate(r, sizeof(*r));
  prev_seq = r->seq;
  dualcore_remote_from_ui_t tmp = *in;
  tmp.seq = prev_seq + 1u;
  __DSB();
  *r = tmp;
  dualcore_shared_dcache_clean(r, sizeof(*r));
}

void remote_control_init(void)
{
  s_edge_inited = 0u;
  s_motor_arm = 0u;
  s_remote_takeover = 0u;
  s_arm_toggle_cd = 0u;
  memset(s_prev_key, 0, sizeof(s_prev_key));
  memset(s_prev_sw, 0, sizeof(s_prev_sw));
  lora3a22_init();

  dualcore_remote_from_ui_t z;
  memset(&z, 0, sizeof(z));
  rc_publish_to_blob(&z);
}

static void rc_process_edges(const lora3a22_uart_transfer_dat_struct *d)
{
  uint8 i;

  if (!s_edge_inited)
  {
    for (i = 0u; i < 4u; i++)
    {
      s_prev_key[i] = d->key[i];
      s_prev_sw[i] = d->switch_key[i];
    }
    s_edge_inited = 1u;
    return;
  }

  /* 左摇杆下压：首次边沿置 takeover，之后每次有效边沿切换 motor_arm（带帧冷却防连翻） */
  if (d->key[0] != 0u && s_prev_key[0] == 0u && s_arm_toggle_cd == 0u)
  {
    s_remote_takeover = 1u;
    s_motor_arm = (uint8)(s_motor_arm ? 0u : 1u);
    s_arm_toggle_cd = REMOTE_MOTOR_ARM_TOGGLE_COOLDOWN_FRAMES;
  }

  /* 右摇杆下压：自旋两圈，方向默认 +1 */
  if (d->key[1] != 0u && s_prev_key[1] == 0u)
  {
    (void)dualcore_ui_cmd_push(DUALCORE_UI_CMD_SPIN_START, (uint32)(int8)1, 2.0f);
  }

  /* 左侧边键：低速允许时切换横滚平衡 */
  if (d->key[2] != 0u && s_prev_key[2] == 0u)
  {
    dualcore_ctrl_to_ui_t dc;
    dualcore_ctrl_to_ui_pull(&dc);
    if (fabsf(dc.car_speed) <= REMOTE_ROLL_TOGGLE_SPEED_MAX)
    {
      (void)dualcore_ui_cmd_push(DUALCORE_UI_CMD_ROLL_BALANCE_TOGGLE, 0, 0.0f);
    }
  }

  /* 右侧边键：跳跃 */
  if (d->key[3] != 0u && s_prev_key[3] == 0u)
  {
    dualcore_ctrl_to_ui_t dcj;
    dualcore_ctrl_to_ui_pull(&dcj);
    if ((dcj.jump_allowed != 0u) && (dcj.jump_active == 0u))
    {
      (void)dualcore_ui_cmd_push(DUALCORE_UI_CMD_JUMP, 0, 0.0f);
    }
  }

  /* 左拨码 1：上升沿开始录制，下降沿结束录制 */
  if (d->switch_key[0] != 0u && s_prev_sw[0] == 0u)
  {
    (void)dualcore_ui_cmd_push(DUALCORE_UI_CMD_NAG_BEGIN_RECORD, 0, 0.0f);
  }
  else if (d->switch_key[0] == 0u && s_prev_sw[0] != 0u)
  {
    (void)dualcore_ui_cmd_push(DUALCORE_UI_CMD_NAG_STOP_RECORD, 0, 0.0f);
  }

  /* 左拨码 2：上升沿准备回放，下降沿导航回待机 */
  if (d->switch_key[1] != 0u && s_prev_sw[1] == 0u)
  {
    (void)dualcore_ui_cmd_push(DUALCORE_UI_CMD_NAG_BEGIN_REPLAY, 0, 0.0f);
  }
  else if (d->switch_key[1] == 0u && s_prev_sw[1] != 0u)
  {
    (void)dualcore_ui_cmd_push(DUALCORE_UI_CMD_NAG_IDLE_RESET, 0, 0.0f);
  }

  /* 右拨码 1：录制态循环元素类型 */
  if (d->switch_key[2] != 0u && s_prev_sw[2] == 0u)
  {
    (void)dualcore_ui_cmd_push(DUALCORE_UI_CMD_NAG_CYCLE_EVENT_TYPE, 0, 0.0f);
  }

  /* 右拨码 2：录制态元素 enter/exit 打点 */
  if (d->switch_key[3] != 0u && s_prev_sw[3] == 0u)
  {
    (void)dualcore_ui_cmd_push(DUALCORE_UI_CMD_NAG_EVENT_MARK, 0, 0.0f);
  }

  for (i = 0u; i < 4u; i++)
  {
    s_prev_key[i] = d->key[i];
    s_prev_sw[i] = d->switch_key[i];
  }
}

void remote_control_task(void)
{
  static uint16 s_stale;
  static int16 s_last_speed;
  static int16 s_last_yaw;
  static uint8 s_got_frame_ever;
  dualcore_remote_from_ui_t out;

  memset(&out, 0, sizeof(out));

  if (lora3a22_finsh_flag != 0u)
  {
    lora3a22_uart_transfer_dat_struct *p = &lora3a22_uart_transfer;

    if (s_arm_toggle_cd > 0u)
    {
      s_arm_toggle_cd--;
    }

    s_got_frame_ever = 1u;
    s_stale = 0u;
    out.online = 1u;
    out.takeover = s_remote_takeover;
    out.motor_arm = s_motor_arm;
    s_last_speed = rc_norm_axis(p->joystick[1], REMOTE_SPEED_CMD_MAX, REMOTE_LEFT_Y_SIGN);
    (void)rc_norm_axis(p->joystick[0], REMOTE_SPEED_CMD_MAX, 1); /* 左右轴暂不使用，仅保持解析路径 */
    s_last_yaw = rc_norm_axis(p->joystick[2], REMOTE_YAW_RATE_CMD_MAX, REMOTE_RIGHT_X_SIGN);
    out.speed_cmd = s_last_speed;
    out.yaw_rate_cmd = s_last_yaw;

    rc_process_edges(p);

    lora3a22_finsh_flag = 0u;
  }
  else
  {
    if (s_got_frame_ever == 0u)
    {
      out.online = 0u;
      out.takeover = 0u;
      out.motor_arm = 0u;
      out.speed_cmd = 0;
      out.yaw_rate_cmd = 0;
      rc_publish_to_blob(&out);
      return;
    }

    if (s_stale < 0xFFFFu)
    {
      s_stale++;
    }
    if (s_stale >= REMOTE_LINK_STALE_LOOPS)
    {
      out.online = 0u;
      out.takeover = 0u;
      out.motor_arm = 0u;
      out.speed_cmd = 0;
      out.yaw_rate_cmd = 0;
      s_last_speed = 0;
      s_last_yaw = 0;
      s_motor_arm = 0u;
      s_remote_takeover = 0u;
      s_arm_toggle_cd = 0u;
      s_edge_inited = 0u;
      s_got_frame_ever = 0u;
    }
    else
    {
      /* 两帧之间保持上一拍连续量，避免主循环比 LoRa 帧快时速度/转向被反复清零 */
      out.online = 1u;
      out.takeover = s_remote_takeover;
      out.motor_arm = s_motor_arm;
      out.speed_cmd = s_last_speed;
      out.yaw_rate_cmd = s_last_yaw;
    }
  }

  rc_publish_to_blob(&out);
}

#elif defined(CY_CORE_CM7_0)

static uint8 s_rc_online;
static uint8 s_rc_allow_mix;
static int16 s_rc_yaw_norm;

void remote_control_apply_after_dip(void)
{
  dualcore_remote_from_ui_t *rp = &g_dualcore_blob.remote;
  dualcore_remote_from_ui_t r;

  dualcore_shared_dcache_invalidate(rp, sizeof(*rp));
  r = *rp;

  s_rc_online = r.online;
  if (r.online == 0u)
  {
    s_rc_allow_mix = 0u;
    s_rc_yaw_norm = 0;
    return;
  }

  /* 未下压左摇杆接过权前：不拉闸、不改 motor_user_speed_cmd，上电仅跟 SWITCH1/2 */
  if (r.takeover == 0u)
  {
    s_rc_allow_mix = 0u;
    s_rc_yaw_norm = 0;
    return;
  }

  /* takeover 后：motor_arm==0 表示遥控侧要停车 */
  if (r.motor_arm == 0u)
  {
    Motor_Switch = MOTOR_OFF;
  }

  /* 回放执行态与元素接管时不改写速度基准，避免干扰比赛/自动流程 */
  if ((N.Nag_SystemRun_Index != 3u) && (N.Event_Active == 0u))
  {
    motor_user_speed_cmd = (float)r.speed_cmd * REMOTE_SPEED_TO_MOTOR_SCALE;
  }

  s_rc_allow_mix = (uint8)((N.Nag_SystemRun_Index != 3u) && (N.Event_Active == 0u));
  s_rc_yaw_norm = r.yaw_rate_cmd;
}

float remote_control_get_yaw_steer_mix(void)
{
  float v;

  if ((s_rc_online == 0u) || (s_rc_allow_mix == 0u))
  {
    return 0.0f;
  }
  v = (float)s_rc_yaw_norm * REMOTE_YAW_TO_STEER_GAIN;
  if (v > 1500.0f)
  {
    v = 1500.0f;
  }
  if (v < -1500.0f)
  {
    v = -1500.0f;
  }
  return v;
}

#endif /* core */

#endif /* REMOTE_CONTROL_ENABLE */
