/*
 * navigation.c
 *
 *  Created on: 2024年10月16日
 *      Author: Monst
 *
 *
 */

#include "zf_common_headfile.h"
#include "navigation.h"
#include "nav_fusion.h"
#include "control.h"
#include "Menu.h"
#include "my_gps.h"
#include "init.h"
#include "dualcore_shared.h"
#include "flash.h"
#include "ekf.h"
#include "small_driver_uart_control.h"

#define NAG_STAIR_PAIRED_ENTER_INVALID    0xFFFFu
#define NAG_BUMP_PAIRED_ENTER_INVALID     0xFFFFu
#define NAG_BRIDGE_EXIT_INDEX_INVALID     0xFFFFu
#define NAG_STAIR2_EXIT_INDEX_INVALID     0xFFFFu
#define NAG_STAIR2_PAIRED_ENTER_INVALID   0xFFFFu

static uint8 Nag_FindNextEventOfType(uint16 run_index, uint8 event_type, uint16 *dist_points);
static uint16 Nag_DistanceToPoints(float distance_cm);
static uint16 s_bridge_blob_lost_ms;
static uint16 s_bridge_enter_grace_ms;   /* 进桥宽限计时（ms），Nag_BridgeTimeoutTick1ms 递增 */
static uint16 s_bridge_blob_no_frame_ms; /* 进桥后无 CM7_1 白块新帧保护计时 */
static uint16 s_stair2_blob_lost_ms;
static uint16 s_stair2_enter_grace_ms;
static uint16 s_stair2_blob_no_frame_ms;

static bool Nag_GetActiveBridgeZone(uint16 run_index,
                                    uint16 *enter_index,
                                    uint16 *exit_index,
                                    uint8 *exit_event_index);

static void Nag_UpdatePreviewAndSpeedTarget(void);
static void Nag_ClearEventRuntimeState(void);
static void Nag_Spin_HandleFailure(void);
static void Nag_BridgeConfirmEnter(void);
static void Nag_BridgeConfirmExit(uint8 allow_beep);
static void Nag_BridgeApplyBlobYaw(float center_err, uint8 track_valid);
static bool Nag_GetActiveStair2Zone(uint16 run_index,
                                    uint16 *enter_index,
                                    uint16 *exit_index,
                                    uint8 *exit_event_index);
static void Nag_Stair2ApplyBlobYaw(float center_err, uint8 track_valid);
static void Nag_Stair2ApplyHoldYaw(void);
static void Nag_Stair2OnFirstBlobFound(void);
static void Nag_Stair2ConfirmEnter(void);
static void Nag_Stair2ConfirmExit(void);
static uint8 Nag_Stair2_ShouldTrackInsYaw(void);
static uint16 Nag_FindPairedExitStair2Marker(uint16 enter_stair2_marker);
static uint16 Nag_ComputeStair2ResumeIndex(uint8 event_index, uint16 event_enter_index);
static void Nag_ConsumeTableExitStair2Events(void);
static void Nag_ActivateChainedExitStair2(void);

int32 Nav_read[Read_MaxSize];//每5cm的点，1000个点50m
Nag N;
uint8 nav_heading_mode = NAV_HEADING_MODE_INS;
NagEvent Nag_Event_Table[Nag_Event_Max];
uint8 Nag_Vofa_Group = 0;
/* 惯导三科目：Run→RecSubj/PlaySubj 配置，Flash V17 持久化；非法值钳位 1~3 */
uint8 g_nag_record_subject = 1u;
uint8 g_nag_replay_subject = 1u;

uint8 Nag_ClampSubject(uint8 subject)
{
    if (subject < 1u)
    {
        return 1u;
    }
    if (subject > NAG_SUBJECT_COUNT)
    {
        return NAG_SUBJECT_COUNT;
    }
    return subject;
}

uint8 Nag_GetSubjectMetaPage(uint8 subject)
{
    switch (Nag_ClampSubject(subject))
    {
    case 2u:
        return Nag_Subject2_Meta_Page;
    case 3u:
        return Nag_Subject3_Meta_Page;
    default:
        return Nag_Subject1_Meta_Page;
    }
}

uint8 Nag_GetSubjectYawStartPage(uint8 subject)
{
    switch (Nag_ClampSubject(subject))
    {
    case 2u:
        return Nag_Subject2_Yaw_Start_Page;
    case 3u:
        return Nag_Subject3_Yaw_Start_Page;
    default:
        return Nag_Subject1_Yaw_Start_Page;
    }
}

uint8 Nag_GetSubjectYawEndPage(uint8 subject)
{
    switch (Nag_ClampSubject(subject))
    {
    case 2u:
        return Nag_Subject2_Yaw_End_Page;
    case 3u:
        return Nag_Subject3_Yaw_End_Page;
    default:
        return Nag_Subject1_Yaw_End_Page;
    }
}

uint8 Nag_GetSubjectEventPage(uint8 subject)
{
    switch (Nag_ClampSubject(subject))
    {
    case 2u:
        return Nag_Subject2_Event_Page;
    case 3u:
        return Nag_Subject3_Event_Page;
    default:
        return Nag_Subject1_Event_Page;
    }
}

void Nag_SubjectPreviewStep(uint8 *preview_subject, int8 delta)
{
    int16 next = 0;

    if (preview_subject == NULL)
    {
        return;
    }
    next = (int16)Nag_ClampSubject(*preview_subject) + (int16)delta;
    if (next < 1)
    {
        next = (int16)NAG_SUBJECT_COUNT;
    }
    else if (next > (int16)NAG_SUBJECT_COUNT)
    {
        next = 1;
    }
    *preview_subject = (uint8)next;
}

void Nag_SetRecordSubject(uint8 subject)
{
    g_nag_record_subject = Nag_ClampSubject(subject);
}

void Nag_SetReplaySubject(uint8 subject)
{
    g_nag_replay_subject = Nag_ClampSubject(subject);
}

#if NAV_FUSION_ENABLE && NAG_USE_FUSION_MILEAGE && NAV_FUSION_ORIGIN_ENABLE
/* 惯导回放：flash 读完置 1；与原点采集均完成后才进入 index=3 */
static uint8 g_nag_replay_flash_ready = 0u;
#endif

/* Launch 页可调：无元素速度仍用 control.c 的 run_launch_speed */
float nag_spin_target_speed = Nag_Spin_Target_Speed_Default;
float nag_spin_pre_decel_dist_cm = Nag_Spin_PreDecel_Dist_cm_Default;
float nag_enter_turn_target_speed = Nag_EnterTurn_Target_Speed_Default;
float nag_enter_turn_pre_decel_dist_cm = Nag_EnterTurn_PreDecel_Dist_cm_Default;
float nag_exit_turn_recovery_speed = Nag_ExitTurn_Recovery_Speed_Default;
float nag_exit_turn_pre_accel_dist_cm = Nag_ExitTurn_PreAccel_Dist_cm_Default;
float nag_enter_cones_target_speed = Nag_EnterCones_Target_Speed_Default;
float nag_enter_cones_pre_decel_dist_cm = Nag_EnterCones_PreDecel_Dist_cm_Default;
float nag_enter_stair_target_speed = Nag_EnterStair_Target_Speed_Default;
float nag_enter_stair_pre_decel_dist_cm = Nag_EnterStair_PreDecel_Dist_cm_Default;
float nag_enter_bridge_target_speed = Nag_EnterBridge_Target_Speed_Default;
float nag_enter_bridge_pre_decel_dist_cm = Nag_EnterBridge_PreDecel_Dist_cm_Default;
float nag_enter_bump_target_speed = Nag_EnterBump_Target_Speed_Default;
float nag_bump_duration_sec = Nag_Bump_Duration_Sec_Default;
float nag_enter_bump_pre_decel_dist_cm = Nag_EnterBump_PreDecel_Dist_cm_Default;
float nag_enter_stair2_target_speed = Nag_EnterStair2_Target_Speed_Default;

void Nag_LaunchParamApplyDefaults(void)
{
    nag_spin_target_speed = Nag_Spin_Target_Speed_Default;
    nag_spin_pre_decel_dist_cm = Nag_Spin_PreDecel_Dist_cm_Default;
    nag_enter_turn_target_speed = Nag_EnterTurn_Target_Speed_Default;
    nag_enter_turn_pre_decel_dist_cm = Nag_EnterTurn_PreDecel_Dist_cm_Default;
    nag_exit_turn_recovery_speed = Nag_ExitTurn_Recovery_Speed_Default;
    nag_exit_turn_pre_accel_dist_cm = Nag_ExitTurn_PreAccel_Dist_cm_Default;
    nag_enter_cones_target_speed = Nag_EnterCones_Target_Speed_Default;
    nag_enter_cones_pre_decel_dist_cm = Nag_EnterCones_PreDecel_Dist_cm_Default;
    nag_enter_stair_target_speed = Nag_EnterStair_Target_Speed_Default;
    nag_enter_stair_pre_decel_dist_cm = Nag_EnterStair_PreDecel_Dist_cm_Default;
    nag_enter_bridge_target_speed = Nag_EnterBridge_Target_Speed_Default;
    nag_enter_bridge_pre_decel_dist_cm = Nag_EnterBridge_PreDecel_Dist_cm_Default;
    nag_enter_bump_target_speed = Nag_EnterBump_Target_Speed_Default;
    nag_bump_duration_sec = Nag_Bump_Duration_Sec_Default;
    nag_enter_bump_pre_decel_dist_cm = Nag_EnterBump_PreDecel_Dist_cm_Default;
    nag_enter_stair2_target_speed = Nag_EnterStair2_Target_Speed_Default;
    spin_set_rate_max_dps(Nag_Spin_Rate_Max_Dps_Default);
}

float Nag_LaunchParamGet(uint8 field_index)
{
    switch (field_index)
    {
    case Nag_Launch_Field_Base_Spd:
        return run_launch_speed;
    case Nag_Launch_Field_Spin_Spd:
        return nag_spin_target_speed;
    case Nag_Launch_Field_Spin_Dec:
        return nag_spin_pre_decel_dist_cm;
    case Nag_Launch_Field_TurnIn_Spd:
        return nag_enter_turn_target_speed;
    case Nag_Launch_Field_TurnIn_Dec:
        return nag_enter_turn_pre_decel_dist_cm;
    case Nag_Launch_Field_TurnOut_Spd:
        return nag_exit_turn_recovery_speed;
    case Nag_Launch_Field_TurnOut_Acc:
        return nag_exit_turn_pre_accel_dist_cm;
    case Nag_Launch_Field_Cone_Spd:
        return nag_enter_cones_target_speed;
    case Nag_Launch_Field_Cone_Dec:
        return nag_enter_cones_pre_decel_dist_cm;
    case Nag_Launch_Field_Spin_Rate:
        return spin_rate_max_dps;
    case Nag_Launch_Field_Stair_Spd:
        return nag_enter_stair_target_speed;
    case Nag_Launch_Field_Stair_Dec:
        return nag_enter_stair_pre_decel_dist_cm;
    case Nag_Launch_Field_BridgeIn_Spd:
        return nag_enter_bridge_target_speed;
    case Nag_Launch_Field_BridgeIn_Dec:
        return nag_enter_bridge_pre_decel_dist_cm;
    case Nag_Launch_Field_Bump_Dur:
        return nag_bump_duration_sec;
    case Nag_Launch_Field_Bump_Spd:
        return nag_enter_bump_target_speed;
    case Nag_Launch_Field_Stair2_Spd:
        return nag_enter_stair2_target_speed;
    default:
        return 0.0f;
    }
}

void Nag_LaunchParamSet(uint8 field_index, float value)
{
    switch (field_index)
    {
    case Nag_Launch_Field_Base_Spd:
        run_launch_speed = value;
        break;
    case Nag_Launch_Field_Spin_Spd:
        nag_spin_target_speed = value;
        break;
    case Nag_Launch_Field_Spin_Dec:
        nag_spin_pre_decel_dist_cm = value;
        break;
    case Nag_Launch_Field_TurnIn_Spd:
        nag_enter_turn_target_speed = value;
        break;
    case Nag_Launch_Field_TurnIn_Dec:
        nag_enter_turn_pre_decel_dist_cm = value;
        break;
    case Nag_Launch_Field_TurnOut_Spd:
        nag_exit_turn_recovery_speed = value;
        break;
    case Nag_Launch_Field_TurnOut_Acc:
        nag_exit_turn_pre_accel_dist_cm = value;
        break;
    case Nag_Launch_Field_Cone_Spd:
        nag_enter_cones_target_speed = value;
        break;
    case Nag_Launch_Field_Cone_Dec:
        nag_enter_cones_pre_decel_dist_cm = value;
        break;
    case Nag_Launch_Field_Spin_Rate:
        spin_set_rate_max_dps(value);
        break;
    case Nag_Launch_Field_Stair_Spd:
        nag_enter_stair_target_speed = value;
        break;
    case Nag_Launch_Field_Stair_Dec:
        nag_enter_stair_pre_decel_dist_cm = value;
        break;
    case Nag_Launch_Field_BridgeIn_Spd:
        nag_enter_bridge_target_speed = value;
        break;
    case Nag_Launch_Field_BridgeIn_Dec:
        nag_enter_bridge_pre_decel_dist_cm = value;
        break;
    case Nag_Launch_Field_Bump_Dur:
        if (value < Nag_Bump_Duration_Sec_Min)
        {
            value = Nag_Bump_Duration_Sec_Min;
        }
        else if (value > Nag_Bump_Duration_Sec_Max)
        {
            value = Nag_Bump_Duration_Sec_Max;
        }
        nag_bump_duration_sec = value;
        break;
    case Nag_Launch_Field_Bump_Spd:
        nag_enter_bump_target_speed = value;
        break;
    case Nag_Launch_Field_Stair2_Spd:
        nag_enter_stair2_target_speed = value;
        break;
    default:
        break;
    }
}

void Nag_LaunchParamAdjust(uint8 field_index, float delta)
{
    Nag_LaunchParamSet(field_index, Nag_LaunchParamGet(field_index) + delta);
}

static bool Nag_GetHeadingHoldConfig(uint8 event_type)
{
    switch (event_type)
    {
        case NAG_EVENT_TYPE_SPIN: return (Nag_HeadingHold_Spin_Enable != 0u);
        case NAG_EVENT_TYPE_ENTER_TURNAROUND: return (Nag_HeadingHold_EnterTurn_Enable != 0u);
        case NAG_EVENT_TYPE_ENTER_BUMP: return (Nag_HeadingHold_EnterBump_Enable != 0u);
        case NAG_EVENT_TYPE_ENTER_STAIR: return (Nag_HeadingHold_EnterStair_Enable != 0u);
        case NAG_EVENT_TYPE_EXIT_STAIR: return (Nag_HeadingHold_ExitStair_Enable != 0u);
        default: return false;
    }
}

static void Nag_HeadingHold_Enable(float target_yaw)
{
    N.HeadingHold_Target_Yaw = target_yaw;
    N.HeadingHold_Target_Latched = 1u;
    N.HeadingHold_Enable = 1u;
    /* 不每 ms 直接调 steer_set_target_yaw()（会 reset turn_angle/turn_gyro）。
     * 首次由 ISR ShouldRequest -> steer_request -> steer_set_target_yaw 引导；
     * 仅 ENTER_BUMP 在 continuous 期间由 pid_ctrl_Run 每 ms 刷新目标并跑双环（见 Nag_HeadingHold_IsContinuousActive）。
     */
    N.HeadingHold_Request_Armed = 1u;
}

static void Nag_HeadingHold_Disable(void)
{
    N.HeadingHold_Enable = 0u;
    N.HeadingHold_Request_Armed = 0u;
    N.HeadingHold_Target_Latched = 0u;
    N.HeadingHold_Target_Yaw = 0.0f;
    steer_yaw_request_pending = 0;
    steer_yaw_delayed_by_spin = 0;
}

static void Nag_HeadingHold_OnEventEnter(uint8 event_type)
{
    /* event_type：切入时刻的 N.Event_Active_Type（与 flash 表中 type 一致）。 */
    N.HeadingHold_Event_Allowed = Nag_GetHeadingHoldConfig(event_type) ? 1u : 0u;
    if (!N.HeadingHold_Event_Allowed)
    {
        Nag_HeadingHold_Disable();
        return;
    }

    /* ENTER_STAIR / ENTER_BUMP：锁航向目标在对应 Start 内按 Nav_read[enter_index] 单点设置。 */
    if (event_type == NAG_EVENT_TYPE_ENTER_STAIR ||
        event_type == NAG_EVENT_TYPE_ENTER_BUMP)
    {
        return;
    }

    /* 其它元素：锁定进入瞬间的实测 yaw。 */
    Nag_HeadingHold_Enable((float)euler_angle.yaw);
}

/*
 * Spin 等待期（已 Event_Active 但尚未 spin_task_start）：继续惯导路径 yaw 跟踪。
 * 仅 spin_enable==1 起转后由 spin_cmd 接管，避免减速/刹停阶段无航向闭环而乱走。
 * 航向闭环校正不在本 Hook：由 control.c spin_finish(1) + Yaw_AlignDisplayDeg 完成。
 */
static uint8 Nag_Spin_ShouldTrackInsYaw(void)
{
    return (uint8)(N.Event_Active &&
                   (N.Event_Active_Type == NAG_EVENT_TYPE_SPIN) &&
                   (N.Spin_Task_Started == 0u) &&
                   (spin_enable == 0u));
}

static uint8 Nag_Stair2_ShouldTrackInsYaw(void)
{
    /* SEEK_FIRST 阶段仍沿惯导路径推进 Run_index 并跟踪 Angle_Run。 */
    return (uint8)(N.Event_Active &&
                   (N.Event_Active_Type == NAG_EVENT_TYPE_ENTER_STAIR2) &&
                   (N.Stair2_Phase == NAG_STAIR2_PHASE_SEEK_FIRST));
}

void Nag_EventPrepareEnter(uint8 event_type)
{
    N.Target_Request_Valid = 0u;
    steer_yaw_request_pending = 0u;
    steer_yaw_delayed_by_spin = 0u;
    /* 折返/锥桶：沿路惯导，不 steer_task_stop。Spin 等待期需继续跟踪 Angle_Run，也不 stop。
     * ENTER_STAIR 等接管元素仍 stop，避免与锁航向抢转向。
     */
    if (event_type != NAG_EVENT_TYPE_ENTER_TURNAROUND &&
        event_type != NAG_EVENT_TYPE_EXIT_TURNAROUND &&
        event_type != NAG_EVENT_TYPE_ENTER_CONES &&
        event_type != NAG_EVENT_TYPE_EXIT_CONES &&
        event_type != NAG_EVENT_TYPE_ENTER_SINGLE_BRIDGE &&
        event_type != NAG_EVENT_TYPE_EXIT_SINGLE_BRIDGE &&
        event_type != NAG_EVENT_TYPE_EXIT_BUMP &&
        event_type != NAG_EVENT_TYPE_EXIT_STAIR &&
        event_type != NAG_EVENT_TYPE_ENTER_STAIR2 &&
        event_type != NAG_EVENT_TYPE_EXIT_STAIR2 &&
        event_type != NAG_EVENT_TYPE_SPIN)
    {
        steer_task_stop();
    }
    /* Spin 等待期仍推进 Run_index；融合里程快照改在起转瞬间同步，见 Nag_Hook_Spin_Run()。 */
    Nag_HeadingHold_OnEventEnter(event_type);
#if Nag_Element_Enter_Beep_Enable
    buzzer_beep_request(NAG_ELEMENT_ENTER_BEEP_MS);
#endif
}

static void Nag_HeadingHold_OnEventExit(void)
{
    N.HeadingHold_Event_Allowed = 0u;
    Nag_HeadingHold_Disable();
}

static void Nag_Spin_RestoreSetSpeed(void)
{
    if (N.Spin_Speed_Latched)
    {
        motor_user_speed_cmd = N.Spin_Saved_SetSpeed;
        N.Spin_Speed_Latched = 0;
    }
    N.Spin_Saved_SetSpeed = 0.0f;
    N.Spin_Stop_Stable_Count = 0;
    N.Spin_Task_Started = 0;
}

/* 下面这一组 Nag_Hook_* 默认都提供“空实现”：
 * 1. Start 默认返回 false，表示该元素还没有真正接入具体动作；
 * 2. Run/Stop 默认空操作；
 * 3. IsDone 默认 false，表示不会自动完成。
 *
 * 这样当前版本可以先把统一元素状态机、录点、切入、恢复、调试量全部跑通，
 * 后续你只需要把对应元素的 Start/Run/IsDone/Stop 改成自己的逻辑即可。
 */

/* 折返进/出口：单点路径标记；区段调速由 Nag_ApplyTurnaroundZoneSpeed() 处理。 */
bool Nag_Hook_EnterTurn_Start(void) { return true; }
void Nag_Hook_EnterTurn_Run(void) {}
bool Nag_Hook_EnterTurn_IsDone(void) { return true; }
void Nag_Hook_EnterTurn_Stop(void) {}

bool Nag_Hook_ExitTurn_Start(void) { return true; }
void Nag_Hook_ExitTurn_Run(void) {}
bool Nag_Hook_ExitTurn_IsDone(void) { return true; }
void Nag_Hook_ExitTurn_Stop(void) {}

/* 自转元素接法：
 * 1. Start：先接管全局速度档位，把 motor_user_speed_cmd 清零，给车一个“先刹停”的阶段；
 * 2. Run：速度连续多拍低于 Nag_Spin_Stop_Speed_Threshold 后 spin_task_start()（起转前低速区）；
 * 3. IsDone：自旋已启动且 spin_done==1（control 严格角度门限成功后 spin_finish(1) 才做航向校正）；
 * 4. Stop：恢复 Spin_Saved_SetSpeed；航向校正结果由 control 层保留在 euler_angle.yaw。
 *    失败时 Nag_Spin_HandleFailure() 恢复路径索引且不消费该元素。
 */
bool Nag_Hook_Spin_Start(void)
{
    N.Spin_Saved_SetSpeed = motor_user_speed_cmd;
    N.Spin_Stop_Stable_Count = 0;
    N.Spin_Task_Started = 0;
    N.Spin_Resume_RunIndex = 0;
    N.Spin_Speed_Latched = 1;
    motor_user_speed_cmd = 0.0f;
    return true;
}
void Nag_Hook_Spin_Run(void)
{
    /* 等待期 Run_index 仍推进；用 Nag_Spin_Stop_* 判定停稳后再起转（与 control SPIN_ANGLE_SETTLE_DEG 收刹无关）。 */
    float abs_speed = fabsf((float)Nag_Speed_Source);

    if (N.Spin_Task_Started)
    {
        if (spin_failed != 0u)
        {
            Nag_Spin_HandleFailure();
        }
        return;
    }

    if (abs_speed <= Nag_Spin_Stop_Speed_Threshold)
    {
        if (N.Spin_Stop_Stable_Count < 0xFFFFu)
        {
            N.Spin_Stop_Stable_Count++;
        }
    }
    else
    {
        N.Spin_Stop_Stable_Count = 0;
        return;
    }

    if (N.Spin_Stop_Stable_Count < Nag_Spin_Stop_Stable_Count)
    {
        return;
    }

    /* 自旋真正启动前先解除“保持进入元素航向”。
     * 否则普通转向可能继续把车头往锁定方向拉，和 spin_task_start() 抢同一套差速控制。
     */
    Nag_HeadingHold_Disable();
    N.Spin_Resume_RunIndex = N.Run_index;
#if NAV_FUSION_ENABLE && NAG_USE_FUSION_MILEAGE
    /* 起转瞬间刷新融合里程快照，自旋期间位移不计入路径索引。 */
    NavFusion_SyncMileageSnapshot();
#endif
    spin_task_start(Nag_Spin_Demo_Turns, Nag_Spin_Demo_Dir);
    N.Spin_Task_Started = 1;
}
/* spin_done==1 由 control spin_finish(1) 置位；失败走 Nag_Spin_HandleFailure()，不进入 DONE */
bool Nag_Hook_Spin_IsDone(void)
{
    return (N.Spin_Task_Started && (spin_done == 1u));
}
void Nag_Hook_Spin_Stop(void)
{
    if (N.Spin_Task_Started)
    {
        spin_task_stop();
    }
    Nag_Spin_RestoreSetSpeed();
}

/* 自旋失败：恢复起转前路径索引与速度，不消费该元素，便于后续重试。 */
static void Nag_Spin_HandleFailure(void)
{
    if ((N.Event_Active_Type != NAG_EVENT_TYPE_SPIN) || (N.Spin_Task_Started == 0u))
    {
        return;
    }

    steer_yaw_request_pending = 0u;
    steer_yaw_delayed_by_spin = 0u;
    N.Run_index = N.Spin_Resume_RunIndex;
#if NAV_FUSION_ENABLE && NAG_USE_FUSION_MILEAGE
    NavFusion_SyncMileageSnapshot();
#endif
    Nag_Hook_Spin_Stop();
    Nag_ClearEventRuntimeState();
    Nag_UpdatePreviewAndSpeedTarget();
    N.Angle_Run = (float)(Nav_read[N.Prospect_index] / 100.0f);
}

/*
 * 进入颠簸元素：
 * - 锁 enter_index 录制点 Nav_read[enter_index] 单点 yaw；
 * - 经 Nag_HeadingHold continuous + steer_request_target_yaw() 持续锁绝对航向（仅颠簸，1ms ISR 消费）；
 * - 固定速度 nag_enter_bump_target_speed、腿长 Nag_EnterBump_Leg_Long；
 * - 开启横滚平衡 roll_balance_en=1，Run 每拍强制保持；
 * - 腿俯仰倾角由 control.c 经 Nag_GetEffectiveLegTiltMax() 限制为 25°；
 * - 融合里程快照同步；Run_index 在 Event_Active 期间冻结（Run_Nag_GPS）；
 * - nag_bump_duration_sec 计时到后链式切入 EXIT_BUMP。
 */
float Nag_GetEffectiveLegTiltMax(void)
{
    if (Nag_IsEnterBumpActive() != 0u)
    {
        return Nag_EnterBump_Leg_Tilt_Max;
    }
    return VMC_A_EXT_MAX;
}

bool Nag_Hook_EnterBump_Start(void)
{
    uint16 enter_index = 0u;
    float locked_yaw = 0.0f;
    float speed_sign = 1.0f;

    N.Bump_Zone_Active = 1u;
    N.Bump_Chain_To_Exit = 0u;
    N.Bump_Elapsed_Ms = 0u;

    if (N.Event_Active_Index < N.Event_Count &&
        Nag_Event_Table[N.Event_Active_Index].valid)
    {
        enter_index = Nag_Event_Table[N.Event_Active_Index].enter_index;
    }
    else
    {
        enter_index = N.Run_index;
    }

    N.Bump_Saved_SetSpeed = motor_user_speed_cmd;
    N.Bump_Saved_Leg_Long = leg_long;

    locked_yaw = (float)euler_angle.yaw;
    if (enter_index < Read_MaxSize && N.Save_index > 0u && enter_index < N.Save_index)
    {
        locked_yaw = (float)(Nav_read[enter_index] / 100.0f);
    }
    N.Bump_Locked_Yaw = locked_yaw;
    Nag_HeadingHold_Enable(locked_yaw);
    /* 进入瞬间登记一次绝对航向；持续纠偏由 pid_ctrl_Run continuous 双环承担，不在 Run 里每 ms 重复登记。 */
    steer_request_target_yaw(locked_yaw);

#if NAV_FUSION_ENABLE && NAG_USE_FUSION_MILEAGE
    NavFusion_SyncMileageSnapshot();
#endif

    if ((float)motor_user_speed_cmd < 0.0f)
    {
        speed_sign = -1.0f;
    }
    motor_user_speed_cmd = speed_sign * nag_enter_bump_target_speed;
    leg_long = Nag_EnterBump_Leg_Long;
    roll_balance_en = 1u;
    return true;
}

void Nag_Hook_EnterBump_Run(void)
{
    N.Bump_Elapsed_Ms++;
    roll_balance_en = 1u;
}

bool Nag_Hook_EnterBump_IsDone(void)
{
    uint32 duration_ms = (uint32)(nag_bump_duration_sec * 1000.0f);

    if (duration_ms == 0u)
    {
        duration_ms = 1u;
    }
    return (N.Bump_Elapsed_Ms >= duration_ms);
}

void Nag_Hook_EnterBump_Stop(void)
{
    if (N.Bump_Chain_To_Exit != 0u)
    {
        N.Bump_Locked_Yaw = 0.0f;
        N.Bump_Elapsed_Ms = 0u;
        return;
    }

    motor_user_speed_cmd = N.Bump_Saved_SetSpeed;
    leg_long = N.Bump_Saved_Leg_Long;
    roll_balance_en = 0u;
    N.Bump_Zone_Active = 0u;
    N.Bump_Saved_SetSpeed = 0.0f;
    N.Bump_Saved_Leg_Long = 0.0f;
    N.Bump_Locked_Yaw = 0.0f;
    N.Bump_Elapsed_Ms = 0u;
}

/*
 * 退出颠簸元素：ENTER_BUMP 计时完成后软件链式切入。
 * 恢复进入前备份速度与腿长，强制关闭横滚平衡；首拍 IsDone 接回惯导前瞻。
 */
bool Nag_Hook_ExitBump_Start(void)
{
#if NAV_FUSION_ENABLE && NAG_USE_FUSION_MILEAGE
    NavFusion_SyncMileageSnapshot();
#endif
    motor_user_speed_cmd = N.Bump_Saved_SetSpeed;
    leg_long = N.Bump_Saved_Leg_Long;
    roll_balance_en = 0u;
    return true;
}

void Nag_Hook_ExitBump_Run(void) {}

bool Nag_Hook_ExitBump_IsDone(void)
{
    return true;
}

void Nag_Hook_ExitBump_Stop(void)
{
    N.Bump_Saved_SetSpeed = 0.0f;
    N.Bump_Saved_Leg_Long = 0.0f;
    N.Bump_Locked_Yaw = 0.0f;
    N.Bump_Elapsed_Ms = 0u;
    N.Bump_Zone_Active = 0u;
}

/* 锥桶进/出口：单点路径标记；区段调速由 Nag_ApplyConeZoneSpeed() 按 Run_index 与事件表配对处理，
 * 不依赖 Event_Active 窗口。Start 立刻 true，首拍 IsDone 即 true，尽快恢复惯导前瞻。
 */
bool Nag_Hook_EnterCones_Start(void) { return true; }
void Nag_Hook_EnterCones_Run(void) {}
bool Nag_Hook_EnterCones_IsDone(void) { return true; }
void Nag_Hook_EnterCones_Stop(void) {}

bool Nag_Hook_ExitCones_Start(void) { return true; }
void Nag_Hook_ExitCones_Run(void) {}
bool Nag_Hook_ExitCones_IsDone(void) { return true; }
void Nag_Hook_ExitCones_Stop(void) {}

/*
 * 单边桥进/出（白块引导）：
 * - BridgeIn：首拍立即进桥（蜂鸣、横滚、冻结里程、CM7_1 白块寻迹）
 * - BridgeOut：仅录制锚点；回放由白块丢失在 Nag_BridgeConfirmExit 接回 Run_index
 */
bool Nag_Hook_EnterBridge_Start(void)
{
    N.Bridge_Saved_Leg_Long = leg_long;
    N.Bridge_Saved_RollBalance = roll_balance_en;
    Nag_BridgeConfirmEnter();
    return true;
}

void Nag_Hook_EnterBridge_Run(void) {}

bool Nag_Hook_EnterBridge_IsDone(void) { return true; }

void Nag_Hook_EnterBridge_Stop(void)
{
    /* 标记元素首拍即 IsDone；leg/roll 在 Nag_BridgeConfirmEnter 设置，Stop 不恢复。
     * 仅 Nag_Element_Abort() 在桥区中途恢复备份值。
     */
}

bool Nag_Hook_ExitBridge_Start(void)
{
    /* 回放时 Run_index 不会自然到达 BridgeOut（桥区冻结里程）；接回已在 Nag_BridgeConfirmExit 完成 */
    return true;
}

void Nag_Hook_ExitBridge_Run(void) {}

bool Nag_Hook_ExitBridge_IsDone(void) { return true; }

void Nag_Hook_ExitBridge_Stop(void) {}

static void Nag_BridgeConfirmEnter(void)
{
    uint16 enter_index = 0u;
    uint16 exit_index = NAG_BRIDGE_EXIT_INDEX_INVALID;
    uint8 exit_event_index = 0xFFu;

    if (N.Bridge_Zone_Active != 0u)
    {
        return;
    }

    if (N.Bridge_Saved_Leg_Long <= 0.01f)
    {
        N.Bridge_Saved_Leg_Long = leg_long;
        N.Bridge_Saved_RollBalance = roll_balance_en;
    }

    N.Bridge_Zone_Active = 1u;
    N.Bridge_Expected = 0u;
    N.Bridge_Detect_Arm = 1u;
    leg_long = Nag_EnterBridge_Leg_Long;
    roll_balance_en = 1u;
    N.Bridge_Exit_Beep_Done = 0u;
    N.Bridge_Exit_Run_Index = NAG_BRIDGE_EXIT_INDEX_INVALID;
    if (Nag_GetActiveBridgeZone(N.Run_index, &enter_index, &exit_index, &exit_event_index))
    {
        N.Bridge_Exit_Run_Index = exit_index;
    }
    s_bridge_blob_lost_ms = 0u;
    s_bridge_enter_grace_ms = 0u;
    s_bridge_blob_no_frame_ms = 0u;
    N.Target_Request_Valid = 0u;
}

static void Nag_BridgeConfirmExit(uint8 allow_beep)
{
    if (N.Bridge_Zone_Active == 0u)
    {
        return;
    }

    if (N.Bridge_Saved_Leg_Long > 0.01f)
    {
        leg_long = N.Bridge_Saved_Leg_Long; /* 恢复进桥前初始腿长 */
    }
    else
    {
        leg_long = Menu_GetInitLegLong();
    }
    roll_balance_en = N.Bridge_Saved_RollBalance;

    /* 里程接回：跳到 BridgeOut 录制点，清零段内 Mileage_All，再接惯导前瞻 yaw */
    if ((N.Bridge_Exit_Run_Index != NAG_BRIDGE_EXIT_INDEX_INVALID) &&
        (N.Bridge_Exit_Run_Index > N.Run_index))
    {
        N.Run_index = N.Bridge_Exit_Run_Index;
    }
    N.Mileage_All = 0.0f;

    N.Bridge_Zone_Active = 0u;
    N.Bridge_Heading_Lock = 0u;
    N.Bridge_Expected = 0u;
    N.Bridge_Detect_Arm = 0u;
    N.Bridge_Saved_Leg_Long = 0.0f;
    N.Bridge_Saved_RollBalance = 0u;
    N.Bridge_Exit_Run_Index = NAG_BRIDGE_EXIT_INDEX_INVALID;
    s_bridge_blob_lost_ms = 0u;
    s_bridge_enter_grace_ms = 0u;
    s_bridge_blob_no_frame_ms = 0u;

    Nag_UpdatePreviewAndSpeedTarget();
    N.Angle_Run = (float)(Nav_read[N.Prospect_index] / 100.0f);
    steer_request_target_yaw(N.Angle_Run);
    N.Requested_Target_Yaw = N.Angle_Run;
    N.Target_Request_Valid = 1u;

    if ((allow_beep != 0u) && (N.Bridge_Exit_Beep_Done == 0u))
    {
        buzzer_beep_request(BRIDGE_BEEP_MS);
        N.Bridge_Exit_Beep_Done = 1u;
    }
}

uint8 Nag_BridgeDetectShouldArm(void)
{
    return (N.Bridge_Zone_Active != 0u) ? 1u : 0u;
}

static float Nag_BridgeClipf(float value, float lo, float hi)
{
    if (value < lo)
    {
        return lo;
    }
    if (value > hi)
    {
        return hi;
    }
    return value;
}

static void Nag_BridgeApplyBlobYaw(float center_err, uint8 track_valid)
{
    float target_offset_deg = 0.0f;
    float target_yaw_deg = 0.0f;

    if (track_valid == 0u)
    {
        return;
    }
    if (spin_enable != 0u || Motor_Runaway_Latch != 0u)
    {
        return;
    }

    target_offset_deg = center_err * Nag_BridgeBlob_Yaw_K_Pixel;
    target_offset_deg = Nag_BridgeClipf(target_offset_deg,
                                        -Nag_BridgeBlob_Yaw_Max_Offset,
                                        Nag_BridgeBlob_Yaw_Max_Offset);
    if (fabsf(target_offset_deg) < Nag_BridgeBlob_Yaw_Deadband)
    {
        return;
    }

    target_yaw_deg = (float)euler_angle.yaw + target_offset_deg;
    steer_request_target_yaw(target_yaw_deg);
    N.Requested_Target_Yaw = target_yaw_deg;
    N.Target_Request_Valid = 1u;
}

void Nag_BridgeTimeoutTick1ms(void)
{
    if (N.Bridge_Zone_Active == 0u)
    {
        s_bridge_enter_grace_ms = 0u;
        s_bridge_blob_lost_ms = 0u;
        s_bridge_blob_no_frame_ms = 0u;
        return;
    }

    /* 桥区每 ms 锁定腿长与横滚，防止主循环误出桥或遥控键短暂改写 */
    leg_long = Nag_EnterBridge_Leg_Long;
    roll_balance_en = 1u;

    if (s_bridge_enter_grace_ms < BRIDGE_ENTER_GRACE_MS)
    {
        s_bridge_enter_grace_ms++;
    }
    else
    {
        uint8 track_valid = dualcore_white_blob_read_track_valid();

        if (track_valid != 0u)
        {
            s_bridge_blob_lost_ms = 0u;
        }
        else if (s_bridge_blob_lost_ms < 0xFFFFu)
        {
            s_bridge_blob_lost_ms++;
        }

        if (s_bridge_blob_lost_ms >= BRIDGE_BLOB_LOST_EXIT_MS)
        {
            Nag_BridgeConfirmExit(1u);
            return;
        }

        if (s_bridge_blob_no_frame_ms < BRIDGE_BLOB_NO_FRAME_EXIT_MS)
        {
            s_bridge_blob_no_frame_ms++;
        }
        else
        {
            Nag_BridgeConfirmExit(1u);
        }
    }
}

void Nag_BridgeDetectUpdate(void)
{
    float center_err = 0.0f;
    uint8 track_valid = 0u;
    uint8 fresh = 0u;

    if (nav_heading_mode != NAV_HEADING_MODE_INS)
    {
        return;
    }

    N.Bridge_Detect_Arm = Nag_BridgeDetectShouldArm();

    if (N.Bridge_Zone_Active == 0u)
    {
        s_bridge_blob_lost_ms = 0u;
        return;
    }

    dualcore_white_blob_pull(&center_err, &track_valid, &fresh);

    /* 桥区方向控制只看当前白块快照；fresh 仅用于无帧超时复位。 */
    Nag_BridgeApplyBlobYaw(center_err, track_valid);

    if (fresh == 0u)
    {
        return;
    }
    s_bridge_blob_no_frame_ms = 0u;
}

/*
 * 台阶2白块引导（ENTER_STAIR2）：
 * - SEEK_FIRST：沿惯导寻第一白块，Run_index 仍推进；
 * - TRACK_FIRST：冻结里程，按白块修正 yaw；
 * - DARK_HOLD：第一白块丢失 250ms 后锁消失时 yaw，不退出元素；
 * - TRACK_SECOND：第二白块出现后继续白块引导；
 * - 第二白块丢失 250ms 后 IsDone，链式切 EXIT_STAIR2，从 S2out 接回惯导。
 */
static void Nag_Stair2ApplyBlobYaw(float center_err, uint8 track_valid)
{
    float target_offset_deg = 0.0f;
    float target_yaw_deg = 0.0f;

    if (track_valid == 0u)
    {
        return;
    }
    if (spin_enable != 0u || Motor_Runaway_Latch != 0u)
    {
        return;
    }

    target_offset_deg = center_err * Nag_Stair2Blob_Yaw_K_Pixel;
    target_offset_deg = Nag_BridgeClipf(target_offset_deg,
                                        -Nag_Stair2Blob_Yaw_Max_Offset,
                                        Nag_Stair2Blob_Yaw_Max_Offset);
    if (fabsf(target_offset_deg) < Nag_Stair2Blob_Yaw_Deadband)
    {
        return;
    }

    target_yaw_deg = (float)euler_angle.yaw + target_offset_deg;
    steer_request_target_yaw(target_yaw_deg);
    N.Requested_Target_Yaw = target_yaw_deg;
    N.Target_Request_Valid = 1u;
}

static void Nag_Stair2ApplyHoldYaw(void)
{
    if (spin_enable != 0u || Motor_Runaway_Latch != 0u)
    {
        return;
    }

    steer_request_target_yaw(N.Stair2_Hold_Yaw);
    N.Requested_Target_Yaw = N.Stair2_Hold_Yaw;
    N.Target_Request_Valid = 1u;
}

static void Nag_Stair2OnFirstBlobFound(void)
{
    if (N.Stair2_First_Blob_Latched != 0u)
    {
        return;
    }

    N.Stair2_First_Blob_Latched = 1u;
    N.Stair2_Phase = NAG_STAIR2_PHASE_TRACK_FIRST;
#if NAV_FUSION_ENABLE && NAG_USE_FUSION_MILEAGE
    NavFusion_SyncMileageSnapshot();
#endif
    s_stair2_blob_lost_ms = 0u;
}

static void Nag_Stair2ConfirmEnter(void)
{
    uint16 enter_index = 0u;
    uint16 exit_index = NAG_STAIR2_EXIT_INDEX_INVALID;
    uint8 exit_event_index = 0xFFu;
    float speed_sign = 1.0f;

    if (N.Stair2_Zone_Active != 0u)
    {
        return;
    }

    N.Stair2_Zone_Active = 1u;
    N.Stair2_Detect_Arm = 1u;
    N.Stair2_Phase = NAG_STAIR2_PHASE_SEEK_FIRST;
    N.Stair2_First_Blob_Latched = 0u;
    N.Stair2_Second_Blob_Seen = 0u;
    N.Stair2_Chain_To_Exit = 0u;
    N.Stair2_Hold_Yaw = (float)euler_angle.yaw;
    N.Stair2_Exit_Run_Index = NAG_STAIR2_EXIT_INDEX_INVALID;
    if (Nag_GetActiveStair2Zone(N.Run_index, &enter_index, &exit_index, &exit_event_index))
    {
        N.Stair2_Exit_Run_Index = exit_index;
    }

    N.Stair2_Saved_SetSpeed = motor_user_speed_cmd;
    speed_sign = ((float)motor_user_speed_cmd < 0.0f) ? -1.0f : 1.0f;
    motor_user_speed_cmd = speed_sign * nag_enter_stair2_target_speed;

    s_stair2_blob_lost_ms = 0u;
    s_stair2_enter_grace_ms = 0u;
    s_stair2_blob_no_frame_ms = 0u;
    N.Target_Request_Valid = 0u;
}

static void Nag_Stair2ConfirmExit(void)
{
    if (N.Stair2_Zone_Active == 0u)
    {
        return;
    }

#if NAV_FUSION_ENABLE && NAG_USE_FUSION_MILEAGE
    NavFusion_SyncMileageSnapshot();
#endif

    /* 里程接回：跳到 Stair2Out 录制点，清零段内 Mileage_All，再接惯导前瞻 yaw */
    if ((N.Stair2_Exit_Run_Index != NAG_STAIR2_EXIT_INDEX_INVALID) &&
        (N.Stair2_Exit_Run_Index > N.Run_index))
    {
        N.Run_index = N.Stair2_Exit_Run_Index;
    }
    N.Mileage_All = 0.0f;

    N.Stair2_Zone_Active = 0u;
    N.Stair2_Detect_Arm = 0u;
    N.Stair2_Phase = NAG_STAIR2_PHASE_SEEK_FIRST;
    N.Stair2_First_Blob_Latched = 0u;
    N.Stair2_Second_Blob_Seen = 0u;
    N.Stair2_Exit_Run_Index = NAG_STAIR2_EXIT_INDEX_INVALID;
    s_stair2_blob_lost_ms = 0u;
    s_stair2_enter_grace_ms = 0u;
    s_stair2_blob_no_frame_ms = 0u;

    Nag_UpdatePreviewAndSpeedTarget();
    N.Angle_Run = (float)(Nav_read[N.Prospect_index] / 100.0f);
    steer_request_target_yaw(N.Angle_Run);
    N.Requested_Target_Yaw = N.Angle_Run;
    N.Target_Request_Valid = 1u;
}

uint8 Nag_Stair2DetectShouldArm(void)
{
    return (N.Stair2_Zone_Active != 0u) ? 1u : 0u;
}

void Nag_Stair2TimeoutTick1ms(void)
{
    if (N.Stair2_Zone_Active == 0u)
    {
        s_stair2_enter_grace_ms = 0u;
        s_stair2_blob_lost_ms = 0u;
        s_stair2_blob_no_frame_ms = 0u;
        return;
    }

    if (s_stair2_enter_grace_ms < STAIR2_ENTER_GRACE_MS)
    {
        s_stair2_enter_grace_ms++;
        return;
    }

    if (N.Stair2_Phase == NAG_STAIR2_PHASE_SEEK_FIRST ||
        N.Stair2_Phase == NAG_STAIR2_PHASE_TRACK_FIRST ||
        N.Stair2_Phase == NAG_STAIR2_PHASE_TRACK_SECOND)
    {
        uint8 track_valid = dualcore_white_blob_read_track_valid();

        if (track_valid != 0u)
        {
            s_stair2_blob_lost_ms = 0u;
        }
        else if (s_stair2_blob_lost_ms < 0xFFFFu)
        {
            s_stair2_blob_lost_ms++;
        }

        if (N.Stair2_Phase == NAG_STAIR2_PHASE_TRACK_FIRST &&
            s_stair2_blob_lost_ms >= STAIR2_BLOB_LOST_CONFIRM_MS)
        {
            N.Stair2_Phase = NAG_STAIR2_PHASE_DARK_HOLD;
            N.Stair2_Hold_Yaw = (float)euler_angle.yaw;
            s_stair2_blob_lost_ms = 0u;
        }
        else if (N.Stair2_Phase == NAG_STAIR2_PHASE_TRACK_SECOND &&
                 s_stair2_blob_lost_ms >= STAIR2_BLOB_LOST_CONFIRM_MS)
        {
            /* 第二白块丢失：由 IsDone 链式切 EXIT_STAIR2，此处仅停止继续跟白块 */
            s_stair2_blob_lost_ms = STAIR2_BLOB_LOST_CONFIRM_MS;
        }
    }

    if (s_stair2_blob_no_frame_ms < STAIR2_BLOB_NO_FRAME_EXIT_MS)
    {
        s_stair2_blob_no_frame_ms++;
    }
}

void Nag_Stair2DetectUpdate(void)
{
    float center_err = 0.0f;
    uint8 track_valid = 0u;
    uint8 fresh = 0u;

    if (nav_heading_mode != NAV_HEADING_MODE_INS)
    {
        return;
    }

    N.Stair2_Detect_Arm = Nag_Stair2DetectShouldArm();

    if (N.Stair2_Zone_Active == 0u)
    {
        s_stair2_blob_lost_ms = 0u;
        return;
    }

    dualcore_white_blob_pull(&center_err, &track_valid, &fresh);

    if (N.Stair2_Phase == NAG_STAIR2_PHASE_SEEK_FIRST)
    {
        if (track_valid != 0u)
        {
            Nag_Stair2OnFirstBlobFound();
            Nag_Stair2ApplyBlobYaw(center_err, track_valid);
        }
    }
    else if (N.Stair2_Phase == NAG_STAIR2_PHASE_TRACK_FIRST)
    {
        Nag_Stair2ApplyBlobYaw(center_err, track_valid);
    }
    else if (N.Stair2_Phase == NAG_STAIR2_PHASE_DARK_HOLD)
    {
        Nag_Stair2ApplyHoldYaw();
        if (track_valid != 0u)
        {
            N.Stair2_Phase = NAG_STAIR2_PHASE_TRACK_SECOND;
            N.Stair2_Second_Blob_Seen = 1u;
            s_stair2_blob_lost_ms = 0u;
            Nag_Stair2ApplyBlobYaw(center_err, track_valid);
        }
    }
    else if (N.Stair2_Phase == NAG_STAIR2_PHASE_TRACK_SECOND)
    {
        Nag_Stair2ApplyBlobYaw(center_err, track_valid);
    }

    if (fresh != 0u)
    {
        s_stair2_blob_no_frame_ms = 0u;
    }
}

bool Nag_Hook_EnterStair2_Start(void)
{
    Nag_Stair2ConfirmEnter();
    return true;
}

void Nag_Hook_EnterStair2_Run(void) {}

bool Nag_Hook_EnterStair2_IsDone(void)
{
    return (N.Stair2_Phase == NAG_STAIR2_PHASE_TRACK_SECOND &&
            s_stair2_blob_lost_ms >= STAIR2_BLOB_LOST_CONFIRM_MS);
}

void Nag_Hook_EnterStair2_Stop(void)
{
    if (N.Stair2_Chain_To_Exit != 0u)
    {
        return;
    }

    motor_user_speed_cmd = N.Stair2_Saved_SetSpeed;
    Nag_Stair2ConfirmExit();
    N.Stair2_Saved_SetSpeed = 0.0f;
}

bool Nag_Hook_ExitStair2_Start(void)
{
#if NAV_FUSION_ENABLE && NAG_USE_FUSION_MILEAGE
    NavFusion_SyncMileageSnapshot();
#endif
    motor_user_speed_cmd = N.Stair2_Saved_SetSpeed;
    N.Stair2_Zone_Active = 0u;
    N.Stair2_Detect_Arm = 0u;
    s_stair2_blob_lost_ms = 0u;
    s_stair2_enter_grace_ms = 0u;
    s_stair2_blob_no_frame_ms = 0u;
    return true;
}

void Nag_Hook_ExitStair2_Run(void) {}

bool Nag_Hook_ExitStair2_IsDone(void)
{
    return true;
}

void Nag_Hook_ExitStair2_Stop(void)
{
    N.Stair2_Saved_SetSpeed = 0.0f;
    N.Stair2_Paired_Enter_Index = NAG_STAIR2_PAIRED_ENTER_INVALID;
}

static void Nag_ConsumeTableExitStair2Events(void)
{
    uint8 event_index = 0u;

    for (event_index = 0u; event_index < N.Event_Count; event_index++)
    {
        if (Nag_Event_Table[event_index].valid &&
            (N.Event_Consumed[event_index] == 0u) &&
            Nag_Event_Table[event_index].type == NAG_EVENT_TYPE_EXIT_STAIR2)
        {
            N.Event_Consumed[event_index] = 1u;
        }
    }
}

static void Nag_ActivateChainedExitStair2(void)
{
    N.Event_Active = 1u;
    N.Event_Active_Index = 0xFFu;
    N.Active_Event_Enter = N.Run_index;
    N.Active_Event_Exit = N.Run_index;
    N.Event_Active_Type = NAG_EVENT_TYPE_EXIT_STAIR2;
    N.Event_Start_RunIndex = N.Run_index;
    N.Event_Trigger_RunIndex = N.Run_index;
    N.Event_Triggered_In_Window = 0u;
    N.Event_State = NAG_EVENT_STATE_ENTERED;
    N.Event_Start_Latched = 0u;
    N.Event_Done_Latched = 0u;
    N.Stair2_Chain_To_Exit = 0u;
    Nag_EventPrepareEnter(NAG_EVENT_TYPE_EXIT_STAIR2);
}

/*
 * 进入台阶元素：
 * - 锁 enter_index 录制点 Nav_read[enter_index] 单点 yaw（普通锁航，非 continuous）；
 * - 固定速度 nag_enter_stair_target_speed、腿长 Nag_EnterStair_Leg_Long；
 * - 融合里程快照同步；Run_index 在 Event_Active 期间冻结（Run_Nag_GPS）；
 * - CM7_1 经 stair_enter_active 门控 step_detect / 视觉自动跳。
 * IsDone：三次 jump_control 正常结束后链式切入 EXIT_STAIR。
 */
bool Nag_Hook_EnterStair_Start(void)
{
    uint16 enter_index = 0u;
    float locked_yaw = 0.0f;
    float speed_sign = 1.0f;

    N.Stair_Jump_Completed_Count = 0u;
    N.Stair_Chain_To_Exit = 0u;

    if (N.Event_Active_Index < N.Event_Count &&
        Nag_Event_Table[N.Event_Active_Index].valid)
    {
        enter_index = Nag_Event_Table[N.Event_Active_Index].enter_index;
    }
    else
    {
        enter_index = N.Run_index;
    }

    N.Stair_Saved_SetSpeed = motor_user_speed_cmd;
    N.Stair_Saved_Leg_Long = leg_long;

    locked_yaw = (float)euler_angle.yaw;
    if (enter_index < Read_MaxSize && N.Save_index > 0u && enter_index < N.Save_index)
    {
        locked_yaw = (float)(Nav_read[enter_index] / 100.0f);
    }
    N.Stair_Lookback_Yaw = locked_yaw;
    Nag_HeadingHold_Enable(locked_yaw);

#if NAV_FUSION_ENABLE && NAG_USE_FUSION_MILEAGE
    NavFusion_SyncMileageSnapshot();
#endif

    if ((float)motor_user_speed_cmd < 0.0f)
    {
        speed_sign = -1.0f;
    }
    motor_user_speed_cmd = speed_sign * nag_enter_stair_target_speed;
    leg_long = Nag_EnterStair_Leg_Long;
    return true;
}

void Nag_Hook_EnterStair_Run(void) {}

bool Nag_Hook_EnterStair_IsDone(void)
{
    return (N.Stair_Jump_Completed_Count >= Nag_Stair_Jump_Exit_Count);
}

void Nag_Hook_EnterStair_Stop(void)
{
    if (N.Stair_Chain_To_Exit != 0u)
    {
        N.Stair_Lookback_Yaw = 0.0f;
        N.Stair_Jump_Completed_Count = 0u;
        return;
    }

    motor_user_speed_cmd = N.Stair_Saved_SetSpeed;
    leg_long = N.Stair_Saved_Leg_Long;
    N.Stair_Saved_SetSpeed = 0.0f;
    N.Stair_Saved_Leg_Long = 0.0f;
    N.Stair_Lookback_Yaw = 0.0f;
    N.Stair_Jump_Completed_Count = 0u;
}

void Nag_NotifyStairJumpDone(void)
{
    if (N.Event_Active != 0u &&
        N.Event_Active_Type == NAG_EVENT_TYPE_ENTER_STAIR &&
        N.Stair_Jump_Completed_Count < 255u)
    {
        N.Stair_Jump_Completed_Count++;
    }
}

static void Nag_ConsumeTableExitStairEvents(void)
{
    uint8 event_index = 0u;

    for (event_index = 0u; event_index < N.Event_Count; event_index++)
    {
        if (Nag_Event_Table[event_index].valid &&
            (N.Event_Consumed[event_index] == 0u) &&
            Nag_Event_Table[event_index].type == NAG_EVENT_TYPE_EXIT_STAIR)
        {
            N.Event_Consumed[event_index] = 1u;
        }
    }
}

static void Nag_ActivateChainedExitStair(void)
{
    N.Event_Active = 1u;
    N.Event_Active_Index = 0xFFu;
    N.Active_Event_Enter = N.Run_index;
    N.Active_Event_Exit = N.Run_index;
    N.Event_Active_Type = NAG_EVENT_TYPE_EXIT_STAIR;
    N.Event_Start_RunIndex = N.Run_index;
    N.Event_Trigger_RunIndex = N.Run_index;
    N.Event_Triggered_In_Window = 0u;
    N.Event_State = NAG_EVENT_STATE_ENTERED;
    N.Event_Start_Latched = 0u;
    N.Event_Done_Latched = 0u;
    N.Stair_Jump_Completed_Count = 0u;
    N.Stair_Chain_To_Exit = 0u;
    Nag_EventPrepareEnter(NAG_EVENT_TYPE_EXIT_STAIR);
}

static void Nag_ConsumeTableExitBumpEvents(void)
{
    uint8 event_index = 0u;

    for (event_index = 0u; event_index < N.Event_Count; event_index++)
    {
        if (Nag_Event_Table[event_index].valid &&
            (N.Event_Consumed[event_index] == 0u) &&
            Nag_Event_Table[event_index].type == NAG_EVENT_TYPE_EXIT_BUMP)
        {
            N.Event_Consumed[event_index] = 1u;
        }
    }
}

static void Nag_ActivateChainedExitBump(void)
{
    N.Event_Active = 1u;
    N.Event_Active_Index = 0xFFu;
    N.Active_Event_Enter = N.Run_index;
    N.Active_Event_Exit = N.Run_index;
    N.Event_Active_Type = NAG_EVENT_TYPE_EXIT_BUMP;
    N.Event_Start_RunIndex = N.Run_index;
    N.Event_Trigger_RunIndex = N.Run_index;
    N.Event_Triggered_In_Window = 0u;
    N.Event_State = NAG_EVENT_STATE_ENTERED;
    N.Event_Start_Latched = 0u;
    N.Event_Done_Latched = 0u;
    N.Bump_Elapsed_Ms = 0u;
    N.Bump_Chain_To_Exit = 0u;
    Nag_EventPrepareEnter(NAG_EVENT_TYPE_EXIT_BUMP);
}

/*
 * 退出台阶元素：ENTER_STAIR 三次跳跃完成后软件链式切入。
 * 恢复进入前基准速度与初始腿长；首拍 IsDone 接回惯导前瞻。
 */
bool Nag_Hook_ExitStair_Start(void)
{
#if NAV_FUSION_ENABLE && NAG_USE_FUSION_MILEAGE
    NavFusion_SyncMileageSnapshot();
#endif
    motor_user_speed_cmd = N.Stair_Saved_SetSpeed;
    if (N.Stair_Saved_Leg_Long > 0.01f)
    {
        leg_long = N.Stair_Saved_Leg_Long;
    }
    else
    {
        leg_long = Menu_GetInitLegLong();
    }
    stair_jump_reset_boost_phase();
    return true;
}

void Nag_Hook_ExitStair_Run(void) {}

bool Nag_Hook_ExitStair_IsDone(void)
{
    return true;
}

void Nag_Hook_ExitStair_Stop(void)
{
    N.Stair_Saved_SetSpeed = 0.0f;
    N.Stair_Saved_Leg_Long = 0.0f;
    N.Stair_Lookback_Yaw = 0.0f;
}

bool Nag_Element_Start(uint8 event_type)
{
    /* 统一 Start 分发：
     * event_type：N.Event_Active_Type，与 Nag_Event_Type / 事件表 flash 的 type 一致。
     * - 返回 true：已进入 RUNNING；
     * - 返回 false：停留在 ENTERED（可 KEY4 / 串口 v 手动恢复）。
     */
    switch (event_type)
    {
        case NAG_EVENT_TYPE_ENTER_TURNAROUND: return Nag_Hook_EnterTurn_Start();
        case NAG_EVENT_TYPE_EXIT_TURNAROUND: return Nag_Hook_ExitTurn_Start();
        case NAG_EVENT_TYPE_SPIN: return Nag_Hook_Spin_Start();
        case NAG_EVENT_TYPE_ENTER_CONES: return Nag_Hook_EnterCones_Start();
        case NAG_EVENT_TYPE_EXIT_CONES: return Nag_Hook_ExitCones_Start();
        case NAG_EVENT_TYPE_ENTER_SINGLE_BRIDGE: return Nag_Hook_EnterBridge_Start();
        case NAG_EVENT_TYPE_EXIT_SINGLE_BRIDGE: return Nag_Hook_ExitBridge_Start();
        case NAG_EVENT_TYPE_ENTER_BUMP: return Nag_Hook_EnterBump_Start();
        case NAG_EVENT_TYPE_EXIT_BUMP: return Nag_Hook_ExitBump_Start();
        case NAG_EVENT_TYPE_ENTER_STAIR: return Nag_Hook_EnterStair_Start();
        case NAG_EVENT_TYPE_EXIT_STAIR: return Nag_Hook_ExitStair_Start();
        case NAG_EVENT_TYPE_ENTER_STAIR2: return Nag_Hook_EnterStair2_Start();
        case NAG_EVENT_TYPE_EXIT_STAIR2: return Nag_Hook_ExitStair2_Start();
        default: return false;
    }
}

void Nag_Element_Run(uint8 event_type)
{
    /* 统一 Run 分发：元素处于 RUNNING 态时，每拍都会调到这里。 */
    switch (event_type)
    {
        case NAG_EVENT_TYPE_ENTER_TURNAROUND: Nag_Hook_EnterTurn_Run(); break;
        case NAG_EVENT_TYPE_EXIT_TURNAROUND: Nag_Hook_ExitTurn_Run(); break;
        case NAG_EVENT_TYPE_SPIN: Nag_Hook_Spin_Run(); break;
        case NAG_EVENT_TYPE_ENTER_CONES: Nag_Hook_EnterCones_Run(); break;
        case NAG_EVENT_TYPE_EXIT_CONES: Nag_Hook_ExitCones_Run(); break;
        case NAG_EVENT_TYPE_ENTER_SINGLE_BRIDGE: Nag_Hook_EnterBridge_Run(); break;
        case NAG_EVENT_TYPE_EXIT_SINGLE_BRIDGE: Nag_Hook_ExitBridge_Run(); break;
        case NAG_EVENT_TYPE_ENTER_BUMP: Nag_Hook_EnterBump_Run(); break;
        case NAG_EVENT_TYPE_EXIT_BUMP: Nag_Hook_ExitBump_Run(); break;
        case NAG_EVENT_TYPE_ENTER_STAIR: Nag_Hook_EnterStair_Run(); break;
        case NAG_EVENT_TYPE_EXIT_STAIR: Nag_Hook_ExitStair_Run(); break;
        case NAG_EVENT_TYPE_ENTER_STAIR2: Nag_Hook_EnterStair2_Run(); break;
        case NAG_EVENT_TYPE_EXIT_STAIR2: Nag_Hook_ExitStair2_Run(); break;
        default: break;
    }
}

bool Nag_Element_IsDone(uint8 event_type)
{
    /* 统一完成判定：
     * 建议由你自己的元素逻辑在合适时机置位完成条件，
     * 状态机一旦检测到 true，就会自动恢复惯导。
     */
    switch (event_type)
    {
        case NAG_EVENT_TYPE_ENTER_TURNAROUND: return Nag_Hook_EnterTurn_IsDone();
        case NAG_EVENT_TYPE_EXIT_TURNAROUND: return Nag_Hook_ExitTurn_IsDone();
        case NAG_EVENT_TYPE_SPIN: return Nag_Hook_Spin_IsDone();
        case NAG_EVENT_TYPE_ENTER_CONES: return Nag_Hook_EnterCones_IsDone();
        case NAG_EVENT_TYPE_EXIT_CONES: return Nag_Hook_ExitCones_IsDone();
        case NAG_EVENT_TYPE_ENTER_SINGLE_BRIDGE: return Nag_Hook_EnterBridge_IsDone();
        case NAG_EVENT_TYPE_EXIT_SINGLE_BRIDGE: return Nag_Hook_ExitBridge_IsDone();
        case NAG_EVENT_TYPE_ENTER_BUMP: return Nag_Hook_EnterBump_IsDone();
        case NAG_EVENT_TYPE_EXIT_BUMP: return Nag_Hook_ExitBump_IsDone();
        case NAG_EVENT_TYPE_ENTER_STAIR: return Nag_Hook_EnterStair_IsDone();
        case NAG_EVENT_TYPE_EXIT_STAIR: return Nag_Hook_ExitStair_IsDone();
        case NAG_EVENT_TYPE_ENTER_STAIR2: return Nag_Hook_EnterStair2_IsDone();
        case NAG_EVENT_TYPE_EXIT_STAIR2: return Nag_Hook_ExitStair2_IsDone();
        default: return false;
    }
}

void Nag_Element_Stop(uint8 event_type)
{
    /* 统一 Stop 分发：
     * 用于异常/人工中止元素，给每个元素一个清理现场的出口。
     */
    Nag_HeadingHold_OnEventExit();
    switch (event_type)
    {
        case NAG_EVENT_TYPE_ENTER_TURNAROUND: Nag_Hook_EnterTurn_Stop(); break;
        case NAG_EVENT_TYPE_EXIT_TURNAROUND: Nag_Hook_ExitTurn_Stop(); break;
        case NAG_EVENT_TYPE_SPIN: Nag_Hook_Spin_Stop(); break;
        case NAG_EVENT_TYPE_ENTER_CONES: Nag_Hook_EnterCones_Stop(); break;
        case NAG_EVENT_TYPE_EXIT_CONES: Nag_Hook_ExitCones_Stop(); break;
        case NAG_EVENT_TYPE_ENTER_SINGLE_BRIDGE: Nag_Hook_EnterBridge_Stop(); break;
        case NAG_EVENT_TYPE_EXIT_SINGLE_BRIDGE: Nag_Hook_ExitBridge_Stop(); break;
        case NAG_EVENT_TYPE_ENTER_BUMP: Nag_Hook_EnterBump_Stop(); break;
        case NAG_EVENT_TYPE_EXIT_BUMP: Nag_Hook_ExitBump_Stop(); break;
        case NAG_EVENT_TYPE_ENTER_STAIR: Nag_Hook_EnterStair_Stop(); break;
        case NAG_EVENT_TYPE_EXIT_STAIR: Nag_Hook_ExitStair_Stop(); break;
        case NAG_EVENT_TYPE_ENTER_STAIR2: Nag_Hook_EnterStair2_Stop(); break;
        case NAG_EVENT_TYPE_EXIT_STAIR2: Nag_Hook_ExitStair2_Stop(); break;
        default: break;
    }
}

static uint16 Nag_ClampIndex(uint16 index, uint16 max_index)
{
    return (index > max_index) ? max_index : index;
}

#if Nag_AdaptiveLookahead_Enable

static uint16 Nag_GetLookaheadPoints(void)
{
    float abs_speed = fabsf(N.Speed_Forward);
    uint16 dynamic_points = (uint16)(Nag_Lookahead_Speed_Gain * abs_speed);
    uint16 lookahead = (uint16)(Nag_Lookahead_Base_Points + dynamic_points);

    if (lookahead > Nag_Lookahead_Max_Points)
    {
        lookahead = Nag_Lookahead_Max_Points;
    }
    return lookahead;
}

static float Nag_GetCurveStrengthAt(uint16 base_index)
{
    uint16 max_index = 0;
    uint16 far_index = 0;
    float yaw_now = 0.0f;
    float yaw_far = 0.0f;

    if (N.Save_index < 2)
    {
        return 0.0f;
    }

    max_index = (uint16)(N.Save_index - 1);
    far_index = Nag_ClampIndex((uint16)(base_index + Nag_Curve_Lookahead_Extra), max_index);
    yaw_now = (float)(Nav_read[base_index] / 100.0f);
    yaw_far = (float)(Nav_read[far_index] / 100.0f);
    return fabsf((float)ange_deviation1(yaw_far, yaw_now));
}

#endif /* Nag_AdaptiveLookahead_Enable */

static void Nag_UpdatePreviewAndSpeedTarget(void)
{
    uint16 max_index = 0;
    float base_speed = fabsf((float)motor_user_speed_cmd);

    if (N.Save_index < 2)
    {
        N.Prospect_index = 0;
        N.Curve_Strength = 0.0f;
        N.Target_Speed = base_speed;
        return;
    }

    max_index = (uint16)(N.Save_index - 1);
#if Nag_AdaptiveLookahead_Enable
    {
        uint16 lookahead = Nag_GetLookaheadPoints();

        N.Prospect_index = Nag_ClampIndex((uint16)(N.Run_index + lookahead), max_index);
        N.Curve_Strength = Nag_GetCurveStrengthAt(N.Run_index);

        if (N.Curve_Strength >= Nag_Curve_Threshold_Sharp)
        {
            N.Target_Speed = base_speed * Nag_Speed_Ratio_Sharp;
        }
        else if (N.Curve_Strength >= Nag_Curve_Threshold_Straight)
        {
            N.Target_Speed = base_speed * Nag_Speed_Ratio_Curve;
        }
        else
        {
            N.Target_Speed = base_speed;
        }
    }
#else
    N.Prospect_index = Nag_ClampIndex(N.Run_index, max_index);
    N.Curve_Strength = 0.0f;
    N.Target_Speed = base_speed;
#endif
}

static void Nag_ClearEventConsumed(void)
{
    memset(N.Event_Consumed, 0, sizeof(N.Event_Consumed));
}

static void Nag_ClearEventRuntimeState(void)
{
    Nag_HeadingHold_OnEventExit();
    N.Event_Active = 0;
    N.Event_Active_Index = 0xFFu;
    N.Active_Event_Enter = 0;
    N.Active_Event_Exit = 0;
    N.Event_State = NAG_EVENT_STATE_IDLE;
    N.Event_Start_Latched = 0;
    N.Event_Done_Latched = 0;
    N.Event_Active_Type = NAG_EVENT_TYPE_SPIN;
    N.Event_Start_RunIndex = 0;
    N.Event_Trigger_RunIndex = 0;
    N.Event_Triggered_In_Window = 0;
    N.Spin_Saved_SetSpeed = 0.0f;
    N.Spin_Stop_Stable_Count = 0;
    N.Spin_Task_Started = 0;
    N.Spin_Resume_RunIndex = 0;
    N.Spin_Speed_Latched = 0;
    N.Stair_Saved_SetSpeed = 0.0f;
    N.Stair_Saved_Leg_Long = 0.0f;
    N.Stair_Lookback_Yaw = 0.0f;
    N.Stair_Jump_Completed_Count = 0u;
    N.Stair_Chain_To_Exit = 0u;
    N.Stair_Paired_Enter_Index = NAG_STAIR_PAIRED_ENTER_INVALID;
    N.Bump_Saved_SetSpeed = 0.0f;
    N.Bump_Saved_Leg_Long = 0.0f;
    N.Bump_Locked_Yaw = 0.0f;
    N.Bump_Elapsed_Ms = 0u;
    N.Bump_Chain_To_Exit = 0u;
    N.Bump_Zone_Active = 0u;
    N.Bump_Paired_Enter_Index = NAG_BUMP_PAIRED_ENTER_INVALID;
    N.Stair2_Saved_SetSpeed = 0.0f;
    N.Stair2_Hold_Yaw = 0.0f;
    N.Stair2_Phase = NAG_STAIR2_PHASE_SEEK_FIRST;
    N.Stair2_First_Blob_Latched = 0u;
    N.Stair2_Second_Blob_Seen = 0u;
    N.Stair2_Chain_To_Exit = 0u;
    N.Stair2_Zone_Active = 0u;
    N.Stair2_Detect_Arm = 0u;
    N.Stair2_Exit_Run_Index = NAG_STAIR2_EXIT_INDEX_INVALID;
    N.Stair2_Paired_Enter_Index = NAG_STAIR2_PAIRED_ENTER_INVALID;
}

void Nag_EventForceReset(void)
{
    if (N.Event_Active)
    {
        Nag_Element_Stop(N.Event_Active_Type);
    }
    Nag_ClearEventRuntimeState();
}

static void Nag_Element_StateMachine(void)
{
    /* 回放中 Event_Active=1 时每 1ms 由 Nag_System() 调用。
     * ENTERED：首开 Event_Start_Latched 后调 Nag_Element_Start；true→RUNNING。
     * RUNNING：Nag_Element_Run + IsDone；true→DONE。
     * DONE：Nag_Notify_Event_Done() 恢复 Run_index 并清 Active。
     */
    uint8 event_index = N.Event_Active_Index;
    uint8 event_type = N.Event_Active_Type;

    if (!N.Event_Active)
    {
        return;
    }

    if (nav_heading_mode == NAV_HEADING_MODE_GPS)
    {
        if (event_index >= gps_point_count)
        {
            return;
        }
    }
    else if (event_index >= N.Event_Count &&
             !(event_index == 0xFFu && event_type == NAG_EVENT_TYPE_EXIT_STAIR) &&
             !(event_index == 0xFFu && event_type == NAG_EVENT_TYPE_EXIT_BUMP) &&
             !(event_index == 0xFFu && event_type == NAG_EVENT_TYPE_EXIT_STAIR2))
    {
        return;
    }

    switch (N.Event_State)
    {
        case NAG_EVENT_STATE_ENTERED:
            if (!N.Event_Start_Latched)
            {
                N.Event_Start_Latched = 1;
                if (Nag_Element_Start(event_type))
                {
                    N.Event_State = NAG_EVENT_STATE_RUNNING;
                }
                else
                {
                    /* 默认空钩子返回 false，表示该元素尚未接入具体逻辑。
                     * 这时保留在 ENTERED，等待用户后续补钩子或用 KEY4 / 串口 v 手动恢复。
                     */
                }
            }
            break;

        case NAG_EVENT_STATE_RUNNING:
            Nag_Element_Run(event_type);
            if (Nag_Element_IsDone(event_type))
            {
                N.Event_Done_Latched = 1;
                N.Event_State = NAG_EVENT_STATE_DONE;
            }
            break;

        case NAG_EVENT_STATE_DONE:
            N.Final_Out = 0.0f;
            Nag_Notify_Event_Done();
            break;

        case NAG_EVENT_STATE_ABORT:
            Nag_Element_Stop(event_type);
            N.Final_Out = 0.0f;
            break;

        case NAG_EVENT_STATE_IDLE:
        default:
            break;
    }
}

static uint8 Nag_FindEventByEnterIndex(uint16 run_index)
{
    uint8 event_index = 0;

    for (event_index = 0; event_index < N.Event_Count; event_index++)
    {
        if (Nag_Event_Table[event_index].valid &&
            (N.Event_Consumed[event_index] == 0u) &&
            Nag_Event_Table[event_index].enter_index == run_index)
        {
            return event_index;
        }
    }
    return 0xFFu;
}

/*
 * 查找前方最近、尚未消费的 Spin，且 Run_index 已进入其触发区域（距 enter_index <= 窗口点数）。
 * 仅用于 Spin；折返/锥桶等标记元素仍要求精确命中 enter_index。
 */
static uint8 Nag_FindSpinEventInTriggerWindow(uint16 run_index, uint16 *dist_points)
{
    uint8 event_index = 0;
    uint8 best_index = 0xFFu;
    uint16 best_dist = 0xFFFFu;
    uint16 window_points = Nag_DistanceToPoints(Nag_Spin_Trigger_Window_cm);

    if (window_points == 0u)
    {
        if (dist_points != NULL)
        {
            *dist_points = 0xFFFFu;
        }
        return 0xFFu;
    }

    for (event_index = 0; event_index < N.Event_Count; event_index++)
    {
        uint16 enter_index = 0;
        uint16 curr_dist = 0;

        if (!Nag_Event_Table[event_index].valid ||
            (N.Event_Consumed[event_index] != 0u) ||
            (Nag_Event_Table[event_index].type != NAG_EVENT_TYPE_SPIN))
        {
            continue;
        }

        enter_index = Nag_Event_Table[event_index].enter_index;
        if (enter_index < run_index)
        {
            continue;
        }

        curr_dist = (uint16)(enter_index - run_index);
        if (curr_dist <= window_points && curr_dist < best_dist)
        {
            best_dist = curr_dist;
            best_index = event_index;
        }
    }

    if (dist_points != NULL)
    {
        *dist_points = best_dist;
    }
    return best_index;
}

static void Nag_ActivateEvent(uint8 event_index, uint8 triggered_in_window)
{
    N.Event_Active = 1;
    N.Event_Active_Index = event_index;
    N.Active_Event_Enter = Nag_Event_Table[event_index].enter_index;
    N.Active_Event_Exit = Nag_Event_Table[event_index].exit_index;
    N.Event_Active_Type = Nag_Event_Table[event_index].type;
    N.Event_Start_RunIndex = N.Run_index;
    N.Event_Trigger_RunIndex = N.Run_index;
    N.Event_Triggered_In_Window = triggered_in_window;
    N.Event_State = NAG_EVENT_STATE_ENTERED;
    N.Event_Start_Latched = 0;
    N.Event_Done_Latched = 0;
    Nag_EventPrepareEnter(N.Event_Active_Type);
}

static uint8 Nag_FindNextEventAhead(uint16 run_index, uint16 *dist_points)
{
    uint8 event_index = 0;
    uint8 best_index = 0xFFu;
    uint16 best_dist = 0xFFFFu;

    for (event_index = 0; event_index < N.Event_Count; event_index++)
    {
        uint16 enter_index = 0;
        uint16 curr_dist = 0;

        if (!Nag_Event_Table[event_index].valid ||
            (N.Event_Consumed[event_index] != 0u))
        {
            continue;
        }

        enter_index = Nag_Event_Table[event_index].enter_index;
        if (enter_index < run_index)
        {
            continue;
        }

        curr_dist = (uint16)(enter_index - run_index);
        if (curr_dist < best_dist)
        {
            best_dist = curr_dist;
            best_index = event_index;
        }
    }

    if (dist_points != NULL)
    {
        *dist_points = best_dist;
    }
    return best_index;
}

/* 将物理距离（cm）换算为导航点数；distance<=0 返回 0，否则至少 1 点。 */
static uint16 Nag_DistanceToPoints(float distance_cm)
{
    float points_f = 0.0f;
    uint16 points = 0u;

    if (distance_cm <= 0.0f || Nag_Set_mileage <= 0.0f)
    {
        return 0u;
    }

    points_f = distance_cm / Nag_Set_mileage;
    points = (uint16)ceilf(points_f);
    if (points == 0u)
    {
        points = 1u;
    }
    return points;
}

/*
 * 双点录制：找 Ein 之后最近的 EXIT_STAIR 标记索引 Eout（不依赖 Event_Consumed）。
 */
static uint16 Nag_FindPairedExitStairMarker(uint16 enter_stair_marker)
{
    uint8 event_index = 0u;
    uint16 best_marker = NAG_STAIR_PAIRED_ENTER_INVALID;

    if (enter_stair_marker == NAG_STAIR_PAIRED_ENTER_INVALID)
    {
        return NAG_STAIR_PAIRED_ENTER_INVALID;
    }

    for (event_index = 0u; event_index < N.Event_Count; event_index++)
    {
        uint16 exit_marker;

        if (!Nag_Event_Table[event_index].valid ||
            Nag_Event_Table[event_index].type != NAG_EVENT_TYPE_EXIT_STAIR)
        {
            continue;
        }

        exit_marker = Nag_Event_Table[event_index].enter_index;
        if (exit_marker > enter_stair_marker &&
            exit_marker < best_marker)
        {
            best_marker = exit_marker;
        }
    }

    return best_marker;
}

/*
 * 双点录制：找 Bin 之后最近的 EXIT_BUMP 标记索引 Bout（不依赖 Event_Consumed）。
 */
static uint16 Nag_FindPairedExitBumpMarker(uint16 bump_in_marker)
{
    uint8 event_index = 0u;
    uint16 best_marker = NAG_BUMP_PAIRED_ENTER_INVALID;

    if (bump_in_marker == NAG_BUMP_PAIRED_ENTER_INVALID)
    {
        return NAG_BUMP_PAIRED_ENTER_INVALID;
    }

    for (event_index = 0u; event_index < N.Event_Count; event_index++)
    {
        uint16 exit_marker;

        if (!Nag_Event_Table[event_index].valid ||
            Nag_Event_Table[event_index].type != NAG_EVENT_TYPE_EXIT_BUMP)
        {
            continue;
        }

        exit_marker = Nag_Event_Table[event_index].enter_index;
        if (exit_marker > bump_in_marker &&
            exit_marker < best_marker)
        {
            best_marker = exit_marker;
        }
    }

    return best_marker;
}

/*
 * 双点录制：找 S2in 之后最近的 EXIT_STAIR2 标记索引 S2out（不依赖 Event_Consumed）。
 */
static uint16 Nag_FindPairedExitStair2Marker(uint16 enter_stair2_marker)
{
    uint8 event_index = 0u;
    uint16 best_marker = NAG_STAIR2_PAIRED_ENTER_INVALID;

    if (enter_stair2_marker == NAG_STAIR2_PAIRED_ENTER_INVALID)
    {
        return NAG_STAIR2_PAIRED_ENTER_INVALID;
    }

    for (event_index = 0u; event_index < N.Event_Count; event_index++)
    {
        uint16 exit_marker;

        if (!Nag_Event_Table[event_index].valid ||
            Nag_Event_Table[event_index].type != NAG_EVENT_TYPE_EXIT_STAIR2)
        {
            continue;
        }

        exit_marker = Nag_Event_Table[event_index].enter_index;
        if (exit_marker > enter_stair2_marker &&
            exit_marker < best_marker)
        {
            best_marker = exit_marker;
        }
    }

    return best_marker;
}

/* EXIT_STAIR 完成后：从 Eout+1 接回；链式 EXIT 用锁存的 Ein 查表；无 EXIT 标记则 fallback Ein+1。 */
static uint16 Nag_ComputeStairResumeIndex(uint8 event_index, uint16 event_enter_index)
{
    uint16 exit_marker;
    uint16 resume_index;
    uint16 max_run_index;

    if (event_index != 0xFFu)
    {
        resume_index = (uint16)(event_enter_index + 1u);
    }
    else if (N.Stair_Paired_Enter_Index != NAG_STAIR_PAIRED_ENTER_INVALID)
    {
        exit_marker = Nag_FindPairedExitStairMarker(N.Stair_Paired_Enter_Index);
        if (exit_marker != NAG_STAIR_PAIRED_ENTER_INVALID)
        {
            resume_index = (uint16)(exit_marker + 1u);
        }
        else
        {
            resume_index = (uint16)(N.Stair_Paired_Enter_Index + 1u);
        }
    }
    else
    {
        resume_index = N.Run_index;
    }

    if (N.Save_index >= 2u)
    {
        max_run_index = (uint16)(N.Save_index - 2u);
        if (resume_index > max_run_index)
        {
            resume_index = max_run_index;
        }
    }

    return resume_index;
}

/* EXIT_BUMP 完成后：从 Bout+1 接回；链式 EXIT 用锁存的 Bin 查表；无 EXIT 标记则 fallback Bin+1。 */
static uint16 Nag_ComputeBumpResumeIndex(uint8 event_index, uint16 event_enter_index)
{
    uint16 exit_marker;
    uint16 resume_index;
    uint16 max_run_index;

    if (event_index != 0xFFu)
    {
        resume_index = (uint16)(event_enter_index + 1u);
    }
    else if (N.Bump_Paired_Enter_Index != NAG_BUMP_PAIRED_ENTER_INVALID)
    {
        exit_marker = Nag_FindPairedExitBumpMarker(N.Bump_Paired_Enter_Index);
        if (exit_marker != NAG_BUMP_PAIRED_ENTER_INVALID)
        {
            resume_index = (uint16)(exit_marker + 1u);
        }
        else
        {
            resume_index = (uint16)(N.Bump_Paired_Enter_Index + 1u);
        }
    }
    else
    {
        resume_index = N.Run_index;
    }

    if (N.Save_index >= 2u)
    {
        max_run_index = (uint16)(N.Save_index - 2u);
        if (resume_index > max_run_index)
        {
            resume_index = max_run_index;
        }
    }

    return resume_index;
}

/* EXIT_STAIR2 完成后：从 S2out 录制点接回（非 +1）；链式 EXIT 用锁存的 S2in 查表。 */
static uint16 Nag_ComputeStair2ResumeIndex(uint8 event_index, uint16 event_enter_index)
{
    uint16 exit_marker;
    uint16 resume_index;
    uint16 max_run_index;

    if (event_index != 0xFFu)
    {
        resume_index = event_enter_index;
    }
    else if (N.Stair2_Paired_Enter_Index != NAG_STAIR2_PAIRED_ENTER_INVALID)
    {
        exit_marker = Nag_FindPairedExitStair2Marker(N.Stair2_Paired_Enter_Index);
        if (exit_marker != NAG_STAIR2_PAIRED_ENTER_INVALID)
        {
            resume_index = exit_marker;
        }
        else
        {
            resume_index = N.Run_index;
        }
    }
    else
    {
        resume_index = N.Run_index;
    }

    if (N.Save_index >= 2u)
    {
        max_run_index = (uint16)(N.Save_index - 2u);
        if (resume_index > max_run_index)
        {
            resume_index = max_run_index;
        }
    }

    return resume_index;
}

#if NAV_FUSION_ENABLE && NAG_USE_FUSION_MILEAGE && NAV_FUSION_HEADING_CALIB_ENABLE
/*
 * 融合回放：录制时 index=1 会把北向标定直行段写入 flash，回放标定在 index=2 不推进 Run_index。
 * 进入 index=3 前按标定距离预跳索引，与录制路径起点对齐；纯惯导/未标定不调用。
 */
static void Nag_SkipReplayRunIndexForFusionHeadingCalib(void)
{
    uint16 skip_points;
    uint16 max_run_index;

    if (NavFusion_IsRuntimeEnabled() == 0u)
    {
        return;
    }
    if (NavFusion_IsHeadingCalibSessionActive() == 0u ||
        NavFusion_IsHeadingCalibReady() == 0u)
    {
        return;
    }
    if (N.Save_index < 2u)
    {
        return;
    }

    skip_points = Nag_DistanceToPoints(NAV_FUSION_HEADING_CALIB_DISTANCE_M * 100.0f);
    if (skip_points == 0u)
    {
        return;
    }

    max_run_index = (uint16)(N.Save_index - 2u);
    if (skip_points > max_run_index)
    {
        N.Run_index = max_run_index;
    }
    else
    {
        N.Run_index = skip_points;
    }
    N.Mileage_All = 0.0f;
}
#endif

/* 事件调速配置：target_speed=元素目标速度；pre_decel/pre_accel 为 cm，0 表示关闭。 */
bool Nav_GetEventSpeedProfileConfig(uint8 event_type,
                                    float *target_speed,
                                    float *pre_decel_dist_cm,
                                    float *pre_accel_dist_cm)
{
    if (target_speed == NULL || pre_decel_dist_cm == NULL || pre_accel_dist_cm == NULL)
    {
        return false;
    }

    *target_speed = 0.0f;
    *pre_decel_dist_cm = 0.0f;
    *pre_accel_dist_cm = 0.0f;

    switch (event_type)
    {
        case NAG_EVENT_TYPE_SPIN:
            *target_speed = nag_spin_target_speed;
            *pre_decel_dist_cm = nag_spin_pre_decel_dist_cm;
            *pre_accel_dist_cm = Nag_Spin_PreAccel_Dist_cm;
            return true;
        case NAG_EVENT_TYPE_ENTER_TURNAROUND:
            *target_speed = nag_enter_turn_target_speed;
            *pre_decel_dist_cm = nag_enter_turn_pre_decel_dist_cm;
            *pre_accel_dist_cm = 0.0f;
            return true;
        case NAG_EVENT_TYPE_EXIT_TURNAROUND:
            *target_speed = nag_exit_turn_recovery_speed;
            *pre_decel_dist_cm = 0.0f;
            *pre_accel_dist_cm = nag_exit_turn_pre_accel_dist_cm;
            return true;
        case NAG_EVENT_TYPE_ENTER_CONES:
            *target_speed = nag_enter_cones_target_speed;
            *pre_decel_dist_cm = nag_enter_cones_pre_decel_dist_cm;
            *pre_accel_dist_cm = 0.0f;
            return true;
        case NAG_EVENT_TYPE_EXIT_CONES:
            *target_speed = Nag_ExitCones_Recovery_Speed;
            *pre_decel_dist_cm = 0.0f;
            *pre_accel_dist_cm = Nag_ExitCones_PreAccel_Dist_cm;
            return true;
        case NAG_EVENT_TYPE_ENTER_SINGLE_BRIDGE:
            *target_speed = nag_enter_bridge_target_speed;
            *pre_decel_dist_cm = nag_enter_bridge_pre_decel_dist_cm;
            *pre_accel_dist_cm = 0.0f;
            return true;
        case NAG_EVENT_TYPE_ENTER_BUMP:
            *target_speed = nag_enter_bump_target_speed;
            *pre_decel_dist_cm = nag_enter_bump_pre_decel_dist_cm;
            return true;
        case NAG_EVENT_TYPE_ENTER_STAIR:
            *target_speed = nag_enter_stair_target_speed;
            *pre_decel_dist_cm = nag_enter_stair_pre_decel_dist_cm;
            return true;
        case NAG_EVENT_TYPE_ENTER_STAIR2:
            *target_speed = nag_enter_stair2_target_speed;
            *pre_decel_dist_cm = 0.0f;
            return true;
        default:
            return false;
    }
}

/* 查找当前 Run_index 所处锥桶区间：最近已过的 ENTER_CONES 与之后第一个 EXIT_CONES 配对。 */
static bool Nag_GetActiveConeZone(uint16 run_index,
                                  uint16 *enter_index,
                                  uint16 *exit_index,
                                  uint8 *exit_event_index)
{
    uint8 event_index = 0;
    uint8 enter_event_index = 0xFFu;
    uint16 best_enter = 0u;

    if (enter_index == NULL || exit_index == NULL || exit_event_index == NULL)
    {
        return false;
    }

    *enter_index = 0u;
    *exit_index = 0xFFFFu;
    *exit_event_index = 0xFFu;

    for (event_index = 0; event_index < N.Event_Count; event_index++)
    {
        if (!Nag_Event_Table[event_index].valid ||
            Nag_Event_Table[event_index].type != NAG_EVENT_TYPE_ENTER_CONES)
        {
            continue;
        }

        if (Nag_Event_Table[event_index].enter_index <= run_index &&
            Nag_Event_Table[event_index].enter_index >= best_enter)
        {
            best_enter = Nag_Event_Table[event_index].enter_index;
            enter_event_index = event_index;
        }
    }

    if (enter_event_index == 0xFFu)
    {
        return false;
    }

    *enter_index = best_enter;

    for (event_index = (uint8)(enter_event_index + 1u); event_index < N.Event_Count; event_index++)
    {
        if (!Nag_Event_Table[event_index].valid ||
            Nag_Event_Table[event_index].type != NAG_EVENT_TYPE_EXIT_CONES)
        {
            continue;
        }

        if (Nag_Event_Table[event_index].enter_index > best_enter)
        {
            *exit_index = Nag_Event_Table[event_index].enter_index;
            *exit_event_index = event_index;
            return true;
        }
    }

    /* 无配对 EXIT：从 ENTER 起至路径末端视为锥桶区间。 */
    return true;
}

/* 查找当前 Run_index 所处折返区间：最近已过的 ENTER_TURNAROUND 与之后第一个 EXIT_TURNAROUND 配对。 */
static bool Nag_GetActiveTurnaroundZone(uint16 run_index,
                                        uint16 *enter_index,
                                        uint16 *exit_index,
                                        uint8 *exit_event_index)
{
    uint8 event_index = 0;
    uint8 enter_event_index = 0xFFu;
    uint16 best_enter = 0u;

    if (enter_index == NULL || exit_index == NULL || exit_event_index == NULL)
    {
        return false;
    }

    *enter_index = 0u;
    *exit_index = 0xFFFFu;
    *exit_event_index = 0xFFu;

    for (event_index = 0; event_index < N.Event_Count; event_index++)
    {
        if (!Nag_Event_Table[event_index].valid ||
            Nag_Event_Table[event_index].type != NAG_EVENT_TYPE_ENTER_TURNAROUND)
        {
            continue;
        }

        if (Nag_Event_Table[event_index].enter_index <= run_index &&
            Nag_Event_Table[event_index].enter_index >= best_enter)
        {
            best_enter = Nag_Event_Table[event_index].enter_index;
            enter_event_index = event_index;
        }
    }

    if (enter_event_index == 0xFFu)
    {
        return false;
    }

    *enter_index = best_enter;

    for (event_index = (uint8)(enter_event_index + 1u); event_index < N.Event_Count; event_index++)
    {
        if (!Nag_Event_Table[event_index].valid ||
            Nag_Event_Table[event_index].type != NAG_EVENT_TYPE_EXIT_TURNAROUND)
        {
            continue;
        }

        if (Nag_Event_Table[event_index].enter_index > best_enter)
        {
            *exit_index = Nag_Event_Table[event_index].enter_index;
            *exit_event_index = event_index;
            return true;
        }
    }

    return true;
}

/* 查找当前 Run_index 所处单边桥区间：最近已过的 ENTER_SINGLE_BRIDGE 与之后第一个 EXIT_SINGLE_BRIDGE 配对。 */
static bool Nag_GetActiveBridgeZone(uint16 run_index,
                                    uint16 *enter_index,
                                    uint16 *exit_index,
                                    uint8 *exit_event_index)
{
    uint8 event_index = 0;
    uint8 enter_event_index = 0xFFu;
    uint16 best_enter = 0u;

    if (enter_index == NULL || exit_index == NULL || exit_event_index == NULL)
    {
        return false;
    }

    *enter_index = 0u;
    *exit_index = 0xFFFFu;
    *exit_event_index = 0xFFu;

    for (event_index = 0; event_index < N.Event_Count; event_index++)
    {
        if (!Nag_Event_Table[event_index].valid ||
            Nag_Event_Table[event_index].type != NAG_EVENT_TYPE_ENTER_SINGLE_BRIDGE)
        {
            continue;
        }

        if (Nag_Event_Table[event_index].enter_index <= run_index &&
            Nag_Event_Table[event_index].enter_index >= best_enter)
        {
            best_enter = Nag_Event_Table[event_index].enter_index;
            enter_event_index = event_index;
        }
    }

    if (enter_event_index == 0xFFu)
    {
        return false;
    }

    *enter_index = best_enter;

    for (event_index = (uint8)(enter_event_index + 1u); event_index < N.Event_Count; event_index++)
    {
        if (!Nag_Event_Table[event_index].valid ||
            Nag_Event_Table[event_index].type != NAG_EVENT_TYPE_EXIT_SINGLE_BRIDGE)
        {
            continue;
        }

        if (Nag_Event_Table[event_index].enter_index > best_enter)
        {
            *exit_index = Nag_Event_Table[event_index].enter_index;
            *exit_event_index = event_index;
            return true;
        }
    }

    return true;
}

/* 查找当前 Run_index 所处台阶2区间：最近已过的 ENTER_STAIR2 与之后第一个 EXIT_STAIR2 配对。 */
static bool Nag_GetActiveStair2Zone(uint16 run_index,
                                    uint16 *enter_index,
                                    uint16 *exit_index,
                                    uint8 *exit_event_index)
{
    uint8 event_index = 0;
    uint8 enter_event_index = 0xFFu;
    uint16 best_enter = 0u;

    if (enter_index == NULL || exit_index == NULL || exit_event_index == NULL)
    {
        return false;
    }

    *enter_index = 0u;
    *exit_index = NAG_STAIR2_EXIT_INDEX_INVALID;
    *exit_event_index = 0xFFu;

    for (event_index = 0; event_index < N.Event_Count; event_index++)
    {
        if (!Nag_Event_Table[event_index].valid ||
            Nag_Event_Table[event_index].type != NAG_EVENT_TYPE_ENTER_STAIR2)
        {
            continue;
        }

        if (Nag_Event_Table[event_index].enter_index <= run_index &&
            Nag_Event_Table[event_index].enter_index >= best_enter)
        {
            best_enter = Nag_Event_Table[event_index].enter_index;
            enter_event_index = event_index;
        }
    }

    if (enter_event_index == 0xFFu)
    {
        return false;
    }

    *enter_index = best_enter;

    for (event_index = (uint8)(enter_event_index + 1u); event_index < N.Event_Count; event_index++)
    {
        if (!Nag_Event_Table[event_index].valid ||
            Nag_Event_Table[event_index].type != NAG_EVENT_TYPE_EXIT_STAIR2)
        {
            continue;
        }

        if (Nag_Event_Table[event_index].enter_index > best_enter)
        {
            *exit_index = Nag_Event_Table[event_index].enter_index;
            *exit_event_index = event_index;
            return true;
        }
    }

    return true;
}

/* 查找前方指定类型最近事件；dist_points 为 enter_index - run_index。 */
static uint8 Nag_FindNextEventOfType(uint16 run_index, uint8 event_type, uint16 *dist_points)
{
    uint8 event_index = 0;
    uint8 best_index = 0xFFu;
    uint16 best_dist = 0xFFFFu;

    for (event_index = 0; event_index < N.Event_Count; event_index++)
    {
        uint16 enter_index = 0;
        uint16 curr_dist = 0;

        if (!Nag_Event_Table[event_index].valid ||
            Nag_Event_Table[event_index].type != event_type)
        {
            continue;
        }

        enter_index = Nag_Event_Table[event_index].enter_index;
        if (enter_index < run_index)
        {
            continue;
        }

        curr_dist = (uint16)(enter_index - run_index);
        if (curr_dist < best_dist)
        {
            best_dist = curr_dist;
            best_index = event_index;
        }
    }

    if (dist_points != NULL)
    {
        *dist_points = best_dist;
    }
    return best_index;
}

/* 查找刚经过且仍在 post-accel 窗口内的 SPIN 等事件（折返/锥桶标记由区段逻辑处理）。 */
static uint8 Nag_FindRecentPassedEventForPostAccel(uint16 run_index,
                                                   uint16 *dist_since_pass,
                                                   float *target_speed,
                                                   uint16 *pre_accel_points)
{
    uint8 event_index = 0;
    uint8 best_index = 0xFFu;
    uint16 best_enter = 0u;
    uint16 best_dist = 0xFFFFu;
    float profile_target = 0.0f;
    float pre_decel_dist = 0.0f;
    float pre_accel_dist = 0.0f;
    uint16 accel_points = 0u;

    for (event_index = 0; event_index < N.Event_Count; event_index++)
    {
        uint16 enter_index = 0;
        uint16 since_pass = 0;

        if (!Nag_Event_Table[event_index].valid)
        {
            continue;
        }

        if (!Nav_GetEventSpeedProfileConfig(Nag_Event_Table[event_index].type,
                                            &profile_target,
                                            &pre_decel_dist,
                                            &pre_accel_dist))
        {
            continue;
        }

        accel_points = Nag_DistanceToPoints(pre_accel_dist);
        if (accel_points == 0u)
        {
            continue;
        }

        /* 折返/锥桶/单边桥/颠簸标记由区段或链式逻辑处理，不走元素后恢复。 */
        if (Nag_Event_Table[event_index].type == NAG_EVENT_TYPE_ENTER_TURNAROUND ||
            Nag_Event_Table[event_index].type == NAG_EVENT_TYPE_EXIT_TURNAROUND ||
            Nag_Event_Table[event_index].type == NAG_EVENT_TYPE_ENTER_CONES ||
            Nag_Event_Table[event_index].type == NAG_EVENT_TYPE_EXIT_CONES ||
            Nag_Event_Table[event_index].type == NAG_EVENT_TYPE_ENTER_SINGLE_BRIDGE ||
            Nag_Event_Table[event_index].type == NAG_EVENT_TYPE_EXIT_SINGLE_BRIDGE ||
            Nag_Event_Table[event_index].type == NAG_EVENT_TYPE_ENTER_BUMP ||
            Nag_Event_Table[event_index].type == NAG_EVENT_TYPE_EXIT_BUMP)
        {
            continue;
        }

        enter_index = Nag_Event_Table[event_index].enter_index;
        if (enter_index >= run_index)
        {
            continue;
        }

        since_pass = (uint16)(run_index - enter_index);
        if (since_pass > accel_points)
        {
            continue;
        }

        if (enter_index >= best_enter)
        {
            best_enter = enter_index;
            best_dist = since_pass;
            best_index = event_index;
        }
    }

    if (best_index == 0xFFu)
    {
        return 0xFFu;
    }

    if (dist_since_pass != NULL)
    {
        *dist_since_pass = best_dist;
    }
    if (target_speed != NULL)
    {
        (void)Nav_GetEventSpeedProfileConfig(Nag_Event_Table[best_index].type,
                                             target_speed,
                                             &pre_decel_dist,
                                             &pre_accel_dist);
    }
    if (pre_accel_points != NULL)
    {
        *pre_accel_points = Nag_DistanceToPoints(pre_accel_dist);
    }
    return best_index;
}

/* 将 nav_speed 限制在 [0, cap] 范围内。 */
static float Nag_ClampSpeedCap(float nav_speed, float cap)
{
    if (cap <= 0.0f)
    {
        return nav_speed;
    }
    if (nav_speed > cap)
    {
        return cap;
    }
    return nav_speed;
}

/* 锥桶区段调速：入口预减速、区间内维持、出口预加速恢复（进入距离窗口后立即设目标速度）。 */
static float Nag_ApplyConeZoneSpeed(float nav_speed)
{
    uint16 enter_index = 0;
    uint16 exit_index = 0;
    uint8 exit_event_index = 0xFFu;
    uint16 dist_to_enter = 0;
    uint16 dist_to_exit = 0;
    uint16 pre_decel_points = 0u;
    uint16 pre_accel_points = 0u;
    uint8 enter_evt = 0xFFu;
    float cone_target = nag_enter_cones_target_speed;
    float recovery_speed = 0.0f;

    pre_decel_points = Nag_DistanceToPoints(nag_enter_cones_pre_decel_dist_cm);
    pre_accel_points = Nag_DistanceToPoints(Nag_ExitCones_PreAccel_Dist_cm);
    recovery_speed = Nag_ExitCones_Recovery_Speed;
    if (recovery_speed <= 0.0f)
    {
        recovery_speed = nav_speed;
    }

    if (!Nag_GetActiveConeZone(N.Run_index, &enter_index, &exit_index, &exit_event_index))
    {
        /* 尚未进入锥桶区：进入预减速距离后立即限速到锥桶目标速度。 */
        enter_evt = Nag_FindNextEventOfType(N.Run_index,
                                            NAG_EVENT_TYPE_ENTER_CONES,
                                            &dist_to_enter);
        if (enter_evt == 0xFFu || pre_decel_points == 0u || dist_to_enter > pre_decel_points)
        {
            return nav_speed;
        }

        return Nag_ClampSpeedCap(nav_speed, cone_target);
    }

    if (N.Run_index < enter_index)
    {
        return nav_speed;
    }

    /* 已过出口：立即恢复到 nav_speed。 */
    if (exit_index != 0xFFFFu && N.Run_index >= exit_index)
    {
        return nav_speed;
    }

    /* 无配对 EXIT：从 ENTER 起至路径末端维持锥桶速度。 */
    if (exit_index == 0xFFFFu)
    {
        return Nag_ClampSpeedCap(nav_speed, cone_target);
    }

    dist_to_exit = (uint16)(exit_index - N.Run_index);

    /* 出口前 pre_accel 窗口：进入距离后立即恢复到 recovery_speed。 */
    if (pre_accel_points > 0u && dist_to_exit <= pre_accel_points)
    {
        return recovery_speed;
    }

    /* 锥桶区间内：维持锥桶目标速度。 */
    return Nag_ClampSpeedCap(nav_speed, cone_target);
}

/* 折返区段调速：入弯前预减速、区间内维持、出弯前预加速恢复。 */
static float Nag_ApplyTurnaroundZoneSpeed(float nav_speed)
{
    uint16 enter_index = 0;
    uint16 exit_index = 0;
    uint8 exit_event_index = 0xFFu;
    uint16 dist_to_enter = 0;
    uint16 dist_to_exit = 0;
    uint16 pre_decel_points = 0u;
    uint16 pre_accel_points = 0u;
    uint8 enter_evt = 0xFFu;
    float turn_target = nag_enter_turn_target_speed;
    float recovery_speed = 0.0f;

    pre_decel_points = Nag_DistanceToPoints(nag_enter_turn_pre_decel_dist_cm);
    pre_accel_points = Nag_DistanceToPoints(nag_exit_turn_pre_accel_dist_cm);
    recovery_speed = nag_exit_turn_recovery_speed;
    if (recovery_speed <= 0.0f)
    {
        recovery_speed = nav_speed;
    }

    if (!Nag_GetActiveTurnaroundZone(N.Run_index, &enter_index, &exit_index, &exit_event_index))
    {
        enter_evt = Nag_FindNextEventOfType(N.Run_index,
                                            NAG_EVENT_TYPE_ENTER_TURNAROUND,
                                            &dist_to_enter);
        if (enter_evt == 0xFFu || pre_decel_points == 0u || dist_to_enter > pre_decel_points)
        {
            return nav_speed;
        }

        return Nag_ClampSpeedCap(nav_speed, turn_target);
    }

    if (N.Run_index < enter_index)
    {
        return nav_speed;
    }

    if (exit_index != 0xFFFFu && N.Run_index >= exit_index)
    {
        return nav_speed;
    }

    if (exit_index == 0xFFFFu)
    {
        return Nag_ClampSpeedCap(nav_speed, turn_target);
    }

    dist_to_exit = (uint16)(exit_index - N.Run_index);

    if (pre_accel_points > 0u && dist_to_exit <= pre_accel_points)
    {
        return recovery_speed;
    }

    return Nag_ClampSpeedCap(nav_speed, turn_target);
}

/*
 * 单边桥区段调速（惯导）：桥进前预减速、桥进～桥出区间内维持 Launch 目标速度；
 * 过 exit_index 后立即恢复 nav_speed（基准速度，无出口 pre_accel）。
 */
static float Nag_ApplyBridgeZoneSpeed(float nav_speed)
{
    uint16 enter_index = 0;
    uint16 exit_index = 0;
    uint8 exit_event_index = 0xFFu;
    uint16 dist_to_enter = 0;
    uint16 pre_decel_points = 0u;
    uint8 enter_evt = 0xFFu;
    float bridge_target = nag_enter_bridge_target_speed;

    pre_decel_points = Nag_DistanceToPoints(nag_enter_bridge_pre_decel_dist_cm);

    if (!Nag_GetActiveBridgeZone(N.Run_index, &enter_index, &exit_index, &exit_event_index))
    {
        enter_evt = Nag_FindNextEventOfType(N.Run_index,
                                            NAG_EVENT_TYPE_ENTER_SINGLE_BRIDGE,
                                            &dist_to_enter);
        if (enter_evt == 0xFFu || pre_decel_points == 0u || dist_to_enter > pre_decel_points)
        {
            return nav_speed;
        }

        return Nag_ClampSpeedCap(nav_speed, bridge_target);
    }

    if (N.Run_index < enter_index)
    {
        return nav_speed;
    }

    if (exit_index != 0xFFFFu && N.Run_index >= exit_index)
    {
        return nav_speed;
    }

    if (exit_index == 0xFFFFu)
    {
        return Nag_ClampSpeedCap(nav_speed, bridge_target);
    }

    return Nag_ClampSpeedCap(nav_speed, bridge_target);
}

/* 非区段标记元素：元素前预减速 + 元素后预加速恢复（进入距离窗口后立即设目标速度）。 */
static float Nag_ApplyGenericEventSpeed(float nav_speed)
{
    uint16 dist_points = 0;
    uint16 pre_decel_points = 0u;
    uint16 pre_accel_points = 0u;
    uint8 next_event = 0xFFu;
    uint8 passed_event = 0xFFu;
    float target_speed = 0.0f;
    float pre_decel_dist = 0.0f;
    float pre_accel_dist = 0.0f;
    float adjusted = nav_speed;

    /* 元素后预加速：进入 pre_accel 窗口后立即恢复到 nav_speed。 */
    passed_event = Nag_FindRecentPassedEventForPostAccel(N.Run_index,
                                                         NULL,
                                                         &target_speed,
                                                         &pre_accel_points);
    if (passed_event != 0xFFu && pre_accel_points > 0u && target_speed >= 0.0f)
    {
        adjusted = nav_speed;
    }

    /* 元素前预减速：进入 pre_decel 窗口后立即设为目标速度。 */
    next_event = Nag_FindNextEventAhead(N.Run_index, &dist_points);
    if (next_event == 0xFFu || next_event >= N.Event_Count)
    {
        return adjusted;
    }

    if (Nag_Event_Table[next_event].type == NAG_EVENT_TYPE_ENTER_TURNAROUND ||
        Nag_Event_Table[next_event].type == NAG_EVENT_TYPE_EXIT_TURNAROUND ||
        Nag_Event_Table[next_event].type == NAG_EVENT_TYPE_ENTER_CONES ||
        Nag_Event_Table[next_event].type == NAG_EVENT_TYPE_EXIT_CONES ||
        Nag_Event_Table[next_event].type == NAG_EVENT_TYPE_ENTER_SINGLE_BRIDGE ||
        Nag_Event_Table[next_event].type == NAG_EVENT_TYPE_EXIT_SINGLE_BRIDGE ||
        Nag_Event_Table[next_event].type == NAG_EVENT_TYPE_ENTER_BUMP ||
        Nag_Event_Table[next_event].type == NAG_EVENT_TYPE_ENTER_STAIR2 ||
        Nag_Event_Table[next_event].type == NAG_EVENT_TYPE_EXIT_BUMP)
    {
        return adjusted;
    }

    if (!Nav_GetEventSpeedProfileConfig(Nag_Event_Table[next_event].type,
                                        &target_speed,
                                        &pre_decel_dist,
                                        &pre_accel_dist))
    {
        return adjusted;
    }

    pre_decel_points = Nag_DistanceToPoints(pre_decel_dist);
    if (pre_decel_points == 0u)
    {
        return adjusted;
    }
    if (target_speed < 0.0f)
    {
        target_speed = 0.0f;
    }

    if (dist_points > pre_decel_points)
    {
        return adjusted;
    }

    return Nag_ClampSpeedCap(adjusted, target_speed);
}

/* 统一事件调速入口：锥桶区段优先，再叠加通用元素前/后调速。 */
static float Nag_ApplyEventSpeedAdjustments(float nav_speed)
{
#if !Nag_EventSpeed_Enable
    return nav_speed;
#else
    float cone_adjusted = 0.0f;
    float turn_adjusted = 0.0f;
    float bridge_adjusted = 0.0f;

    if (nav_speed <= 0.0f || N.Event_Count == 0u)
    {
        return nav_speed;
    }

    cone_adjusted = Nag_ApplyConeZoneSpeed(nav_speed);
    turn_adjusted = Nag_ApplyTurnaroundZoneSpeed(cone_adjusted);
    bridge_adjusted = Nag_ApplyBridgeZoneSpeed(turn_adjusted);
    return Nag_ApplyGenericEventSpeed(bridge_adjusted);
#endif
}

static void Nag_TryEnterEvent(void)
{
    /* 惯导回放：精确命中 enter_index，或 Spin 进入触发区域时切入元素态。
     * 等待减速期 Run_index 仍推进；自旋完成后从 Spin_Resume_RunIndex 恢复，并置 Event_Consumed
     * 防止提前/滞后再次经过同一录制点时重复触发。
     */
    uint8 event_index = 0;

    if (N.Event_Active)
    {
        return;
    }

    event_index = Nag_FindEventByEnterIndex(N.Run_index);
    if (event_index != 0xFFu)
    {
        Nag_ActivateEvent(event_index, 0u);
        return;
    }

    event_index = Nag_FindSpinEventInTriggerWindow(N.Run_index, NULL);
    if (event_index != 0xFFu)
    {
        Nag_ActivateEvent(event_index, 1u);
    }
}

#if Nag_OdoSlip_Enable

/*
 * 编码器速度 -> 前向线速度（cm/s）。
 * 符号与 speed 环一致：左轮取 -receive_left，右轮取 +receive_right。
 */
static float Nag_OdoEncToCmps(float enc_speed)
{
    return enc_speed * Nag_Speed_To_Mileage_Scale;
}

static float Nag_OdoCmpsToStep(float speed_cmps)
{
    return fabsf(speed_cmps) * Nag_Sample_Dt;
}

static void Nag_OdoSlip_PushHistory(float raw_step_cm, float prot_step_cm)
{
    N.Odo_History_Raw[N.Odo_History_Idx] = raw_step_cm;
    N.Odo_History_Prot[N.Odo_History_Idx] = prot_step_cm;
    N.Odo_History_Idx = (uint8)((N.Odo_History_Idx + 1u) % Nag_OdoSlip_History_Len);
}

static float Nag_OdoSlip_SumHistoryExcess(void)
{
    uint8 i = 0u;
    float excess = 0.0f;
    float delta = 0.0f;

    for (i = 0u; i < Nag_OdoSlip_History_Len; i++)
    {
        delta = N.Odo_History_Raw[i] - N.Odo_History_Prot[i];
        if (delta > 0.0f)
        {
            excess += delta;
        }
    }
    return excess;
}

static float Nag_OdoSlip_ClampSlew(float target_cmps, float reference_cmps)
{
    float delta = target_cmps - reference_cmps;

    if (delta > Nag_OdoSlip_Instant_Slew_Max_Cmps)
    {
        return reference_cmps + Nag_OdoSlip_Instant_Slew_Max_Cmps;
    }
    if (delta < -Nag_OdoSlip_Instant_Slew_Max_Cmps)
    {
        return reference_cmps - Nag_OdoSlip_Instant_Slew_Max_Cmps;
    }
    return target_cmps;
}

/*
 * 第一层：瞬时保护。不一致时优先信更稳定的一侧，并对步长做 slew 限幅。
 */
static float Nag_OdoSlip_InstantProtectSpeed(float vc_l, float vc_r, uint8 *suspect_left, uint8 *suspect_right)
{
    float diff = 0.0f;
    float err_l = 0.0f;
    float err_r = 0.0f;
    float v_corr = 0.0f;
    float trust_ref = N.Odo_Last_Trust_Speed_Cmps;

    if (suspect_left != NULL)
    {
        *suspect_left = 0u;
    }
    if (suspect_right != NULL)
    {
        *suspect_right = 0u;
    }

    diff = fabsf(vc_l - vc_r);
    if (diff < Nag_OdoSlip_Consistency_Th_Cmps)
    {
        v_corr = 0.5f * (vc_l + vc_r);
        N.Odo_Last_Trust_Speed_Cmps = v_corr;
        return v_corr;
    }

    err_l = fabsf(vc_l - trust_ref);
    err_r = fabsf(vc_r - trust_ref);

    if (err_l + 20.0f < err_r)
    {
        if (suspect_left != NULL)
        {
            *suspect_left = 1u;
        }
        v_corr = vc_r;
    }
    else if (err_r + 20.0f < err_l)
    {
        if (suspect_right != NULL)
        {
            *suspect_right = 1u;
        }
        v_corr = vc_l;
    }
    else
    {
        v_corr = (fabsf(vc_l) < fabsf(vc_r)) ? vc_l : vc_r;
        if (suspect_left != NULL && suspect_right != NULL)
        {
            *suspect_left = 1u;
            *suspect_right = 1u;
        }
    }

    v_corr = Nag_OdoSlip_ClampSlew(v_corr, trust_ref);
    return v_corr;
}

/*
 * 第二层：状态确认。连续若干 ms 异常后进入左/右/双侧打滑态。
 */
static void Nag_OdoSlip_UpdateState(uint8 suspect_left, uint8 suspect_right, float vc_l, float vc_r)
{
    uint8 prev_state = N.Odo_Slip_State;
    uint8 target_state = NAG_ODO_SLIP_NORMAL;

    if (suspect_left != 0u && suspect_right == 0u)
    {
        target_state = NAG_ODO_SLIP_LEFT;
    }
    else if (suspect_right != 0u && suspect_left == 0u)
    {
        target_state = NAG_ODO_SLIP_RIGHT;
    }
    else if (suspect_left != 0u && suspect_right != 0u)
    {
        target_state = NAG_ODO_SLIP_BOTH;
    }

    if (target_state == NAG_ODO_SLIP_NORMAL)
    {
        N.Odo_Slip_Enter_Count = 0u;
        if (N.Odo_Slip_State != NAG_ODO_SLIP_NORMAL)
        {
            if (++N.Odo_Slip_Exit_Count >= Nag_OdoSlip_Exit_Count)
            {
                N.Odo_Slip_State = NAG_ODO_SLIP_NORMAL;
                N.Odo_Slip_Exit_Count = 0u;
            }
        }
        else
        {
            N.Odo_Slip_Exit_Count = 0u;
        }
        return;
    }

    N.Odo_Slip_Exit_Count = 0u;
    if (N.Odo_Slip_State == target_state)
    {
        N.Odo_Slip_Enter_Count = Nag_OdoSlip_Enter_Count;
        return;
    }

    if (++N.Odo_Slip_Enter_Count >= Nag_OdoSlip_Enter_Count)
    {
        if (prev_state == NAG_ODO_SLIP_NORMAL)
        {
            /* 第三层：刚确认打滑，对短窗内多推进的里程排队补扣 */
            N.Odo_Rollback_Pending_Cm += Nag_OdoSlip_SumHistoryExcess();
        }
        N.Odo_Slip_State = target_state;
        N.Odo_Slip_Enter_Count = Nag_OdoSlip_Enter_Count;
    }

    (void)vc_l;
    (void)vc_r;
}

static float Nag_OdoSlip_StateSpeed(float instant_cmps)
{
    switch (N.Odo_Slip_State)
    {
        case NAG_ODO_SLIP_LEFT:
            return N.Odo_Vc_From_R_Cmps;
        case NAG_ODO_SLIP_RIGHT:
            return N.Odo_Vc_From_L_Cmps;
        case NAG_ODO_SLIP_BOTH:
            return Nag_OdoSlip_ClampSlew(instant_cmps, N.Odo_Last_Trust_Speed_Cmps);
        default:
            return instant_cmps;
    }
}

/*
 * 里程纠偏主入口：更新检测态并返回本拍应积分的步长（cm）。
 */
static float Nag_GetCorrectedMileageStepCm(float raw_speed_enc)
{
    float w_radps = 0.0f;
    float half_track_cm = 0.5f * Nag_Wheel_Track_Cm;
    float vc_l = 0.0f;
    float vc_r = 0.0f;
    float raw_step = 0.0f;
    float prot_step = 0.0f;
    float final_step = 0.0f;
    float instant_cmps = 0.0f;
    float state_cmps = 0.0f;
    uint8 suspect_left = 0u;
    uint8 suspect_right = 0u;

    N.Odo_Wheel_Left_Cmps = Nag_OdoEncToCmps((float)(-motor_value.receive_left_speed_data));
    N.Odo_Wheel_Right_Cmps = Nag_OdoEncToCmps((float)motor_value.receive_right_speed_data);
    N.Odo_Gyro_Z_Dps = imu_data.gyro_z * 57.2957795f;

    w_radps = imu_data.gyro_z;
    vc_l = N.Odo_Wheel_Left_Cmps + w_radps * half_track_cm;
    vc_r = N.Odo_Wheel_Right_Cmps - w_radps * half_track_cm;
    N.Odo_Vc_From_L_Cmps = vc_l;
    N.Odo_Vc_From_R_Cmps = vc_r;

    if (fabsf(raw_speed_enc) < Nag_Speed_Deadband)
    {
        N.Odo_Corrected_Speed_Cmps = 0.0f;
        N.Odo_Raw_Step_Cm = 0.0f;
        N.Odo_Protected_Step_Cm = 0.0f;
        Nag_OdoSlip_PushHistory(0.0f, 0.0f);
        Nag_OdoSlip_UpdateState(0u, 0u, vc_l, vc_r);
        return 0.0f;
    }

    raw_step = Nag_OdoCmpsToStep(Nag_OdoEncToCmps(raw_speed_enc));
    instant_cmps = Nag_OdoSlip_InstantProtectSpeed(vc_l, vc_r, &suspect_left, &suspect_right);
    Nag_OdoSlip_UpdateState(suspect_left, suspect_right, vc_l, vc_r);
    state_cmps = Nag_OdoSlip_StateSpeed(instant_cmps);
    N.Odo_Corrected_Speed_Cmps = state_cmps;

    prot_step = Nag_OdoCmpsToStep(instant_cmps);
    final_step = Nag_OdoCmpsToStep(state_cmps);
    if (final_step > raw_step)
    {
        final_step = raw_step;
    }

    N.Odo_Raw_Step_Cm = raw_step;
    N.Odo_Protected_Step_Cm = final_step;
    Nag_OdoSlip_PushHistory(raw_step, final_step);
    return final_step;
}

void Nag_OdoSlip_ResetState(void)
{
    memset(N.Odo_History_Raw, 0, sizeof(N.Odo_History_Raw));
    memset(N.Odo_History_Prot, 0, sizeof(N.Odo_History_Prot));
    N.Odo_Wheel_Left_Cmps = 0.0f;
    N.Odo_Wheel_Right_Cmps = 0.0f;
    N.Odo_Gyro_Z_Dps = 0.0f;
    N.Odo_Vc_From_L_Cmps = 0.0f;
    N.Odo_Vc_From_R_Cmps = 0.0f;
    N.Odo_Corrected_Speed_Cmps = 0.0f;
    N.Odo_Raw_Step_Cm = 0.0f;
    N.Odo_Protected_Step_Cm = 0.0f;
    N.Odo_Last_Trust_Speed_Cmps = 0.0f;
    N.Odo_Rollback_Pending_Cm = 0.0f;
    N.Odo_Rollback_Applied_Cm = 0.0f;
    N.Odo_Slip_State = NAG_ODO_SLIP_NORMAL;
    N.Odo_Slip_Enter_Count = 0u;
    N.Odo_Slip_Exit_Count = 0u;
    N.Odo_History_Idx = 0u;
}

void Nag_OdoSlip_ApplyPendingRollback(void)
{
    float apply_cm = 0.0f;

    if (N.Odo_Rollback_Pending_Cm <= 0.001f)
    {
        return;
    }

    apply_cm = N.Odo_Rollback_Pending_Cm;
    if (apply_cm > Nag_OdoSlip_Rollback_Max_Cm)
    {
        apply_cm = Nag_OdoSlip_Rollback_Max_Cm;
    }

    N.Mileage_All -= apply_cm;
    N.Odo_Rollback_Applied_Cm += apply_cm;
    N.Odo_Rollback_Pending_Cm -= apply_cm;

    while (N.Mileage_All < 0.0f && N.Run_index > 0u)
    {
        N.Run_index--;
        N.Mileage_All += Nag_Set_mileage;
    }
}

#endif /* Nag_OdoSlip_Enable */

#if !Nag_OdoSlip_Enable
void Nag_OdoSlip_ResetState(void)
{
}

void Nag_OdoSlip_ApplyPendingRollback(void)
{
}
#endif

static float Nag_GetMileageStep(void)
{
    float speed_forward = (float)Nag_Speed_Source;

#if NAV_FUSION_ENABLE && NAG_USE_FUSION_MILEAGE
    {
        float fusion_step_cm = NavFusion_GetMileageStepCm();
        if (fusion_step_cm >= 0.0f)
        {
            N.Speed_Forward = speed_forward;
            if (fusion_step_cm < 0.001f)
            {
                N.Mileage_Step = 0.0f;
                return 0.0f;
            }
            N.Mileage_Step = fusion_step_cm;
            return fusion_step_cm;
        }
    }
#endif

    /* 这里仍然采用 car_speed 积分距离，先在不引入编码器增量的前提下把导航打通。
     * 关键约束：
     * 1. Nag_Sample_Dt 必须与 Nag_System() 的真实调用周期一致；
     * 2. Nag_Speed_To_Mileage_Scale 表示“当前速度单位 -> cm/s”的换算系数；
     * 3. 只要改了导航周期、速度单位或轮径减速比，就必须重标定这个系数。
     */
    N.Speed_Forward = speed_forward;
    if (fabsf(speed_forward) < Nag_Speed_Deadband)
    {
        N.Mileage_Step = 0.0f;
        return 0.0f;
    }

#if Nag_OdoSlip_Enable
    if (Nag_OdoSlip_IsRuntimeEnabled() != 0u)
    {
        float corrected_step_cm = Nag_GetCorrectedMileageStepCm(speed_forward);
        N.Mileage_Step = corrected_step_cm;
        return corrected_step_cm;
    }
#endif

    N.Mileage_Step = fabsf(speed_forward) * Nag_Speed_To_Mileage_Scale * Nag_Sample_Dt;
    return N.Mileage_Step;
}

float Nag_GetDebugReadYaw(void)
{
    uint16 read_index = N.Prospect_index;

    if (0 == N.Save_index)
    {
        return 0.0f;
    }

    if (read_index >= N.Save_index)
    {
        read_index = N.Save_index - 1;
    }
    if (read_index >= Read_MaxSize)
    {
        return 0.0f;
    }
    return (float)(Nav_read[read_index] / 100.0f);
}

float Nag_HeadingHold_GetTargetYaw(void)
{
    if (!N.HeadingHold_Target_Latched)
    {
        return (float)euler_angle.yaw;
    }
    return N.HeadingHold_Target_Yaw;
}

bool Nag_HeadingHold_IsContinuousActive(void)
{
    if (!N.HeadingHold_Enable ||
        !N.HeadingHold_Target_Latched ||
        !N.HeadingHold_Event_Allowed)
    {
        return false;
    }
    if (spin_enable)
    {
        return false;
    }
    /* 持续绝对航向维护仅用于颠簸段；台阶等仍走普通锁航（一次引导 + 收敛/补发）。 */
    if (!N.Event_Active ||
        N.Event_Active_Type != NAG_EVENT_TYPE_ENTER_BUMP)
    {
        return false;
    }
    return true;
}

bool Nag_HeadingHold_ShouldRequest(void)
{
    float yaw_err = 0.0f;

    if (!N.HeadingHold_Enable ||
        !N.HeadingHold_Target_Latched ||
        !N.HeadingHold_Event_Allowed)
    {
        return false;
    }

    if (spin_enable)
    {
        return false;
    }

    if (N.HeadingHold_Request_Armed)
    {
        N.HeadingHold_Request_Armed = 0u;
        return true;
    }

    /* 颠簸 continuous 期间 pid_ctrl_Run 每 ms 维持双环，勿再 steer_set_target_yaw 重置 PID。 */
    if (Nag_HeadingHold_IsContinuousActive())
    {
        return false;
    }

    if (steer_yaw_request_pending)
    {
        return false;
    }

    if (!steer_enable)
    {
        return true;
    }

    if (fabsf((float)ange_deviation1(N.HeadingHold_Target_Yaw, steer_target_yaw_deg)) > 0.01f)
    {
        return true;
    }

    yaw_err = fabsf((float)ange_deviation1(N.HeadingHold_Target_Yaw, euler_angle.yaw));
    return (yaw_err > Nag_HeadingHold_Reissue_Error);
}

static uint8 GPS_FindPairedZoneExit(uint8 enter_idx, uint8 exit_unified)
{
    uint8 i = 0u;

    for (i = (uint8)(enter_idx + 1u); i < gps_point_count; i++)
    {
        if (u32yuansu[i] == exit_unified)
        {
            return i;
        }
    }
    return 0xFFu;
}

static float GPS_ApplyTurnaroundZoneSpeed(float nav_speed)
{
    uint8 i = 0u;
    float pre_decel_m = nag_enter_turn_pre_decel_dist_cm * 0.01f;
    float pre_accel_m = nag_exit_turn_pre_accel_dist_cm * 0.01f;
    float turn_target = nag_enter_turn_target_speed;
    float recovery_speed = nag_exit_turn_recovery_speed;

    if (recovery_speed <= 0.0f)
    {
        recovery_speed = nav_speed;
    }

    for (i = 0u; i < gps_point_count; i++)
    {
        uint8 exit_idx = 0xFFu;
        float dist_enter_m = 0.0f;
        float dist_exit_m = 0.0f;

        if (u32yuansu[i] != NAV_ELEM_TURN_IN)
        {
            continue;
        }

        exit_idx = GPS_FindPairedZoneExit(i, NAV_ELEM_TURN_OUT);
        if (tagert_point >= i && (exit_idx == 0xFFu || tagert_point <= exit_idx))
        {
            if (exit_idx != 0xFFu && tagert_point == exit_idx)
            {
                dist_exit_m = GPS_NavDistanceToPointM(exit_idx);
                if (pre_accel_m > 0.0f && dist_exit_m <= pre_accel_m)
                {
                    return recovery_speed;
                }
            }
            return Nag_ClampSpeedCap(nav_speed, turn_target);
        }

        dist_enter_m = GPS_NavDistanceToPointM(i);
        if (pre_decel_m > 0.0f && dist_enter_m <= pre_decel_m)
        {
            return Nag_ClampSpeedCap(nav_speed, turn_target);
        }
    }

    return nav_speed;
}

static float GPS_ApplyConeZoneSpeed(float nav_speed)
{
    uint8 i = 0u;
    float pre_decel_m = nag_enter_cones_pre_decel_dist_cm * 0.01f;
    float pre_accel_m = Nag_ExitCones_PreAccel_Dist_cm * 0.01f;
    float cone_target = nag_enter_cones_target_speed;
    float recovery_speed = Nag_ExitCones_Recovery_Speed;

    if (recovery_speed <= 0.0f)
    {
        recovery_speed = nav_speed;
    }

    for (i = 0u; i < gps_point_count; i++)
    {
        uint8 exit_idx = 0xFFu;
        float dist_enter_m = 0.0f;
        float dist_exit_m = 0.0f;

        if (u32yuansu[i] != NAV_ELEM_CONE_IN)
        {
            continue;
        }

        exit_idx = GPS_FindPairedZoneExit(i, NAV_ELEM_CONE_OUT);
        if (tagert_point >= i && (exit_idx == 0xFFu || tagert_point <= exit_idx))
        {
            if (exit_idx != 0xFFu && tagert_point == exit_idx)
            {
                dist_exit_m = GPS_NavDistanceToPointM(exit_idx);
                if (pre_accel_m > 0.0f && dist_exit_m <= pre_accel_m)
                {
                    return recovery_speed;
                }
            }
            return Nag_ClampSpeedCap(nav_speed, cone_target);
        }

        dist_enter_m = GPS_NavDistanceToPointM(i);
        if (pre_decel_m > 0.0f && dist_enter_m <= pre_decel_m)
        {
            return Nag_ClampSpeedCap(nav_speed, cone_target);
        }
    }

    return nav_speed;
}

/* GPS 单边桥区段调速：语义同 Nag_ApplyBridgeZoneSpeed()，按路点 u32yuansu 配对。 */
static float GPS_ApplyBridgeZoneSpeed(float nav_speed)
{
    uint8 i = 0u;
    float pre_decel_m = nag_enter_bridge_pre_decel_dist_cm * 0.01f;
    float bridge_target = nag_enter_bridge_target_speed;

    for (i = 0u; i < gps_point_count; i++)
    {
        uint8 exit_idx = 0xFFu;
        float dist_enter_m = 0.0f;

        if (u32yuansu[i] != NAV_ELEM_BRIDGE_IN)
        {
            continue;
        }

        exit_idx = GPS_FindPairedZoneExit(i, NAV_ELEM_BRIDGE_OUT);
        if (tagert_point >= i && (exit_idx == 0xFFu || tagert_point < exit_idx))
        {
            return Nag_ClampSpeedCap(nav_speed, bridge_target);
        }

        if (exit_idx != 0xFFu && tagert_point >= exit_idx)
        {
            continue;
        }

        dist_enter_m = GPS_NavDistanceToPointM(i);
        if (pre_decel_m > 0.0f && dist_enter_m <= pre_decel_m)
        {
            return Nag_ClampSpeedCap(nav_speed, bridge_target);
        }
    }

    return nav_speed;
}

static float GPS_ApplyGenericEventSpeed(float nav_speed)
{
    uint8 i = 0u;
    float target_speed = 0.0f;
    float pre_decel_dist = 0.0f;
    float pre_accel_dist = 0.0f;
    float pre_decel_m = 0.0f;
    float dist_m = 0.0f;
    uint8 ins_type = 0u;

    for (i = tagert_point; i < gps_point_count; i++)
    {
        uint8 unified = (uint8)u32yuansu[i];

        if (Nav_UnifiedIsPassThrough(unified) || unified == NAV_ELEM_END || Nav_UnifiedIsMarker(unified))
        {
            continue;
        }

        ins_type = Nav_UnifiedToInsEvent(unified);
        if (ins_type == 0xFFu)
        {
            continue;
        }

        if (!Nav_GetEventSpeedProfileConfig(ins_type,
                                            &target_speed,
                                            &pre_decel_dist,
                                            &pre_accel_dist))
        {
            continue;
        }

        pre_decel_m = pre_decel_dist * 0.01f;
        dist_m = GPS_NavDistanceToPointM(i);
        if (pre_decel_m > 0.0f && dist_m <= pre_decel_m)
        {
            if (target_speed < 0.0f)
            {
                target_speed = 0.0f;
            }
            return Nag_ClampSpeedCap(nav_speed, target_speed);
        }
        break;
    }

    return nav_speed;
}

float GPS_ApplyEventSpeedAdjustments(float nav_speed)
{
#if Nag_EventSpeed_Enable
    float cone_adjusted = 0.0f;
    float turn_adjusted = 0.0f;
    float bridge_adjusted = 0.0f;

    if (nav_heading_mode != NAV_HEADING_MODE_GPS)
    {
        return nav_speed;
    }

    cone_adjusted = GPS_ApplyConeZoneSpeed(nav_speed);
    turn_adjusted = GPS_ApplyTurnaroundZoneSpeed(cone_adjusted);
    bridge_adjusted = GPS_ApplyBridgeZoneSpeed(turn_adjusted);
    return GPS_ApplyGenericEventSpeed(bridge_adjusted);
#else
    return nav_speed;
#endif
}

uint16 Nag_GetDebugProspectIndex(void)
{
    return N.Prospect_index;
}

/* 速度目标合成（由 pid_ctrl_Run 每 20ms 读取一次）：
 * - motor_user_speed_cmd：用户层基准（串口 V、菜单/遥控/双核命令等）；
 * - N.Target_Speed：导航前瞻 + 弯道强度算出的建议上限；
 * - Nag_ApplyEventSpeedAdjustments()：按事件表与 Run_index 叠加区段调速、提前加减速；
 * - 元素激活期（Event_Active）：SPIN 等使用配置目标速度，折返/锥桶标记仍走区段逻辑。
 */
float Nag_GetControlSpeedTarget(void)
{
    float nav_speed = N.Target_Speed;
    float abs_user_speed = fabsf((float)motor_user_speed_cmd);
    float event_target = 0.0f;
    float pre_decel_dist = 0.0f;
    float pre_accel_dist = 0.0f;

    /* GPS 点导航：叠加元素区段调速与预减速，到点/保护停车仍由 GPS_PointNav_Run 负责。 */
    if (nav_heading_mode == NAV_HEADING_MODE_GPS)
    {
        if (gps_nav_state == GPS_NAV_STATE_FINISHED || gps_nav_state == GPS_NAV_STATE_PROTECT)
        {
            return 0.0f;
        }

        nav_speed = abs_user_speed;
        nav_speed = GPS_ApplyEventSpeedAdjustments(nav_speed);

        if (N.Event_Active)
        {
            switch (N.Event_Active_Type)
            {
                case NAG_EVENT_TYPE_ENTER_STAIR:
                    nav_speed = nag_enter_stair_target_speed;
                    break;
                case NAG_EVENT_TYPE_ENTER_BUMP:
                    nav_speed = nag_enter_bump_target_speed;
                    break;
                case NAG_EVENT_TYPE_ENTER_STAIR2:
                    nav_speed = nag_enter_stair2_target_speed;
                    break;
                case NAG_EVENT_TYPE_EXIT_STAIR:
                    nav_speed = fabsf(N.Stair_Saved_SetSpeed);
                    if (nav_speed <= 0.0f)
                    {
                        nav_speed = abs_user_speed;
                    }
                    break;
                case NAG_EVENT_TYPE_EXIT_BUMP:
                    nav_speed = fabsf(N.Bump_Saved_SetSpeed);
                    if (nav_speed <= 0.0f)
                    {
                        nav_speed = abs_user_speed;
                    }
                    break;
                case NAG_EVENT_TYPE_EXIT_STAIR2:
                    nav_speed = fabsf(N.Stair2_Saved_SetSpeed);
                    if (nav_speed <= 0.0f)
                    {
                        nav_speed = abs_user_speed;
                    }
                    break;
                case NAG_EVENT_TYPE_ENTER_TURNAROUND:
                case NAG_EVENT_TYPE_EXIT_TURNAROUND:
                case NAG_EVENT_TYPE_ENTER_CONES:
                case NAG_EVENT_TYPE_EXIT_CONES:
                case NAG_EVENT_TYPE_ENTER_SINGLE_BRIDGE:
                case NAG_EVENT_TYPE_EXIT_SINGLE_BRIDGE:
                    nav_speed = GPS_ApplyEventSpeedAdjustments(nav_speed);
                    break;
                case NAG_EVENT_TYPE_SPIN:
                    if (Nav_GetEventSpeedProfileConfig(N.Event_Active_Type,
                                                       &event_target,
                                                       &pre_decel_dist,
                                                       &pre_accel_dist) &&
                        event_target >= 0.0f)
                    {
                        nav_speed = Nag_ClampSpeedCap(nav_speed, event_target);
                    }
                    else
                    {
                        nav_speed = MIN(nav_speed, abs_user_speed * Nag_Event_Speed_Ratio);
                    }
                    break;
                default:
                    nav_speed = MIN(nav_speed, abs_user_speed * Nag_Event_Speed_Ratio);
                    break;
            }
        }

        if ((float)motor_user_speed_cmd < 0.0f)
        {
            return -nav_speed;
        }
        return nav_speed;
    }

    if (N.Nag_Stop_f)
    {
        return 0.0f;
    }

    /* 惯导录制 + 遥控在线：速度遥控映射的 环直接使用motor_user_speed_cmd；掉线立即停车 */
    if (g_menu_input_remote_first != 0u &&
        nav_heading_mode == NAV_HEADING_MODE_INS &&
        N.Nag_SystemRun_Index == 1u && N.End_f == 0u &&
        remote_lora_steer_snapshot_valid != 0u)
    {
        return ((float)motor_user_speed_cmd < 0.0f) ? -abs_user_speed : abs_user_speed;
    }

#if NAV_FUSION_ENABLE && NAG_USE_FUSION_MILEAGE && NAV_FUSION_HEADING_CALIB_ENABLE
    /* 融合回放准备态：北向标定直行 5m 期间用 Launch 速度，航向由 NavFusion 锁定；纯惯导不进入 */
    if (NavFusion_IsRuntimeEnabled() != 0u &&
        NavFusion_IsHeadingCalibrating() != 0u &&
        N.Nag_SystemRun_Index == 2u)
    {
        return fabsf(run_launch_speed);
    }
#endif

#if Nag_Debug_Speed_Bypass_Enable
    return ((float)motor_user_speed_cmd < 0.0f) ? -abs_user_speed : abs_user_speed;
#endif

    if (N.Nag_SystemRun_Index != 3)
    {
        return 0.0f;
    }

    if (nav_speed <= 0.0f)
    {
        nav_speed = abs_user_speed;
    }

    if (N.Event_Active)
    {
        switch (N.Event_Active_Type)
        {
            case NAG_EVENT_TYPE_ENTER_STAIR:
                nav_speed = nag_enter_stair_target_speed;
                break;
            case NAG_EVENT_TYPE_ENTER_BUMP:
                nav_speed = nag_enter_bump_target_speed;
                break;
            case NAG_EVENT_TYPE_ENTER_STAIR2:
                nav_speed = nag_enter_stair2_target_speed;
                break;
            case NAG_EVENT_TYPE_EXIT_STAIR:
                nav_speed = fabsf(N.Stair_Saved_SetSpeed);
                if (nav_speed <= 0.0f)
                {
                    nav_speed = abs_user_speed;
                }
                break;
            case NAG_EVENT_TYPE_EXIT_BUMP:
                nav_speed = fabsf(N.Bump_Saved_SetSpeed);
                if (nav_speed <= 0.0f)
                {
                    nav_speed = abs_user_speed;
                }
                break;
            case NAG_EVENT_TYPE_EXIT_STAIR2:
                nav_speed = fabsf(N.Stair2_Saved_SetSpeed);
                if (nav_speed <= 0.0f)
                {
                    nav_speed = abs_user_speed;
                }
                break;
            case NAG_EVENT_TYPE_ENTER_TURNAROUND:
            case NAG_EVENT_TYPE_EXIT_TURNAROUND:
            case NAG_EVENT_TYPE_ENTER_CONES:
            case NAG_EVENT_TYPE_EXIT_CONES:
            case NAG_EVENT_TYPE_ENTER_SINGLE_BRIDGE:
            case NAG_EVENT_TYPE_EXIT_SINGLE_BRIDGE:
                /* 路径标记瞬时完成，区段调速由 Nag_ApplyEventSpeedAdjustments 按 Run_index 处理。 */
                nav_speed = Nag_ApplyEventSpeedAdjustments(nav_speed);
                break;
            case NAG_EVENT_TYPE_SPIN:
                if (Nav_GetEventSpeedProfileConfig(N.Event_Active_Type,
                                                   &event_target,
                                                   &pre_decel_dist,
                                                   &pre_accel_dist) &&
                    event_target >= 0.0f)
                {
                    if (event_target < 0.0f)
                    {
                        event_target = 0.0f;
                    }
                    nav_speed = Nag_ClampSpeedCap(nav_speed, event_target);
                }
                else
                {
                    nav_speed = MIN(nav_speed, abs_user_speed * Nag_Event_Speed_Ratio);
                }
                break;
            default:
                nav_speed = MIN(nav_speed, abs_user_speed * Nag_Event_Speed_Ratio);
                break;
        }
    }
    else
    {
        nav_speed = Nag_ApplyEventSpeedAdjustments(nav_speed);
    }

    if ((float)motor_user_speed_cmd < 0.0f)
    {
        return -nav_speed;
    }
    return nav_speed;
}
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读取偏航角的里程函数
// 参数说明     读取偏航角的里程函数，通过切换N.End_f来切换里程
// 返回参数     void
// 使用示例     用户自行调用
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void Nag_Read()
{
        switch(N.End_f)
        {
            case 0:Run_Nag_Save();  //默认执行保存
                break;
            case 1:;
                    flash_Nag_Write();  //写入最后一页，确保falsh存储
                    N.End_f++;
                    break;
            case 2://Buzzer_check(500);   //蜂鸣器确认执行
                    N.End_f++;  //结束里程
                    break;
        }
}
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     计算导航偏航角
// 参数说明     N.Final_Out为最终生成的偏航角
// 返回参数     void
// 使用示例     用户自行调用
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void Nag_Run()
{
    float yaw_err = 0.0f;

    if (nav_heading_mode != NAV_HEADING_MODE_INS)
    {
        N.Final_Out = 0.0f;
        N.Target_Request_Valid = 0u;
        return;
    }

    /* 回放态总流程：
     * 1. 先根据里程推进 Run_index；
     * 2. 再按速度得到前瞻点 Prospect_index，控制目标使用前瞻点 yaw；
     * 3. 非 Spin 元素 active 后冻结 Run_index；Spin 等待期仍推进索引与前瞻；
     * 4. Spin 起转后冻结索引；等待期仍跟踪 Angle_Run，spin_enable 期间释放航向；
     * 5. Spin 完成后从 Spin_Resume_RunIndex 接回；其他元素由 Nag_Notify_Event_Done() 恢复。
     */
    Run_Nag_GPS();  //偏航角读取函数
    Nag_BridgeDetectUpdate();
    Nag_Stair2DetectUpdate();
    if(N.Nag_Stop_f) //终点停止
    {
        N.Final_Out = 0;
        N.Target_Request_Valid = 0;
        steer_yaw_request_pending = 0;
        steer_yaw_delayed_by_spin = 0;
        steer_task_stop();
        return;
    }

    if (N.Event_Active && !Nag_Spin_ShouldTrackInsYaw() && !Nag_Stair2_ShouldTrackInsYaw() &&
        N.Event_Active_Type != NAG_EVENT_TYPE_EXIT_STAIR &&
        N.Event_Active_Type != NAG_EVENT_TYPE_EXIT_BUMP &&
        N.Event_Active_Type != NAG_EVENT_TYPE_EXIT_STAIR2)
    {
        /* 非 Spin/台阶2寻第一白块等待态 / 非 EXIT：ENTER_STAIR 等不再发惯导 yaw。 */
        N.Final_Out = 0.0f;
        return;
    }

    if (N.Bridge_Zone_Active != 0u)
    {
        /* 桥区航向由 Nag_BridgeDetectUpdate 的白块修正接管，惯导不发 steer */
        N.Final_Out = 0.0f;
        N.Target_Request_Valid = 0u;
        return;
    }

    if (N.Stair2_Zone_Active != 0u && N.Stair2_Phase != NAG_STAIR2_PHASE_SEEK_FIRST)
    {
        /* 台阶2：白块/锁 yaw 由 Nag_Stair2DetectUpdate 接管；SEEK_FIRST 仍走下方惯导 yaw */
        N.Final_Out = 0.0f;
        N.Target_Request_Valid = 0u;
        return;
    }

    yaw_err = (float)ange_deviation1(N.Angle_Run, euler_angle.yaw);
    N.Final_Out = yaw_err;

    if (!N.Target_Request_Valid ||
        fabsf((float)ange_deviation1(N.Angle_Run, N.Requested_Target_Yaw)) > 0.01f)
    {
        steer_request_target_yaw(N.Angle_Run);
        N.Requested_Target_Yaw = N.Angle_Run;
        N.Target_Request_Valid = 1;
    }
    else if (!steer_enable &&
             !steer_yaw_request_pending &&
             fabsf(yaw_err) > Nag_Reissue_Error)
    {
        steer_request_target_yaw(N.Angle_Run);
    }
    
}
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     偏航角保存
// 参数说明     读取YAW存储flash写入
// 返回参数     void
// 使用示例     用户自行调用
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void Run_Nag_Save()
{
    N.Mileage_All += Nag_GetMileageStep();
    N.Mileage_Debug_Total += N.Mileage_Step;
    if (Nag_OdoSlip_IsRuntimeEnabled() != 0u)
    {
        Nag_OdoSlip_ApplyPendingRollback();
    }

    while(N.Mileage_All >= Nag_Set_mileage)    //当里程超过设定值时
    {
       if(N.size >= MaxSize)//当数组超过一页的flash大小时，写入一次，防止重复写入
       {
           flash_Nag_Write();
           N.size=0;   //将数组大小为0，下一次从新开始取
           N.Flash_page_index--;
           zf_assert(N.Flash_page_index > Nag_GetSubjectYawEndPage(g_nag_record_subject));
       }
       int32 Save=(int32)(Nag_Yaw*100); //取偏航角放大100倍，避免使用Float类型存储
       flash_union_buffer[N.size++].int32_type = Save;  //将偏航角写入缓冲区

       N.Save_index++;
       N.Mileage_All -= Nag_Set_mileage;//里程计累加//保存到flash
    }

}
// 偏航角读取
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     偏航角读取
// 参数说明     读取flash存储YAW
// 返回参数     void
// 使用示例     用户自行调用
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void Run_Nag_GPS()
{
    uint16 max_run_index = 0;

    if (N.Save_index < 2)
    {
        N.Angle_Run = (N.Save_index > 0) ? (float)(Nav_read[0] / 100.0f) : (float)Nag_Yaw;
        N.Nag_Stop_f = 1;
        return;
    }

    if (N.Event_Active)
    {
        if (Nag_Spin_ShouldTrackInsYaw())
        {
            /* Spin 等待期：仍按里程推进 Run_index。 */
        }
        else if (N.Event_Active_Type == NAG_EVENT_TYPE_ENTER_STAIR2 &&
                 N.Stair2_First_Blob_Latched == 0u)
        {
            /* 台阶2寻第一白块：仍推进 Run_index 并跟踪惯导前瞻 yaw。 */
        }
        else
        {
            /* 非 Spin 等待态 / 台阶2已锁第一白块：冻结 Run_index。 */
            N.Angle_Run = (float)(Nav_read[N.Prospect_index] / 100.0f);
            return;
        }
    }

    if (N.Bridge_Zone_Active != 0u)
    {
        /* 桥区白块引导：冻结 Run_index；出桥时在 Nag_BridgeConfirmExit 对齐 BridgeOut 索引 */
        Nag_UpdatePreviewAndSpeedTarget();
        N.Angle_Run = (float)(Nav_read[N.Prospect_index] / 100.0f);
        return;
    }

    N.Mileage_All += Nag_GetMileageStep();
    N.Mileage_Debug_Total += N.Mileage_Step;
    if (Nag_OdoSlip_IsRuntimeEnabled() != 0u)
    {
        Nag_OdoSlip_ApplyPendingRollback();
    }
    max_run_index = (uint16)(N.Save_index - 2);
    while(N.Mileage_All >= Nag_Set_mileage)
    {
        if(N.Run_index >= max_run_index)
        {
            N.Nag_Stop_f = 1;
            return;
        }
        N.Run_index++;//增加需要跑的圈数，直接从后往前，值为0.
        N.Mileage_All -= Nag_Set_mileage;//里程计累加//保存到flash
    }

    if (!N.Event_Active)
    {
        Nag_TryEnterEvent();
    }
    Nag_UpdatePreviewAndSpeedTarget();
    N.Angle_Run = (float)(Nav_read[N.Prospect_index] / 100.0f);
}
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     导航参数初始化
// 返回参数     void
// 使用示例     导航执行前开始
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void Init_Nag()
{
    memset(&N, 0, sizeof(N));
    N.Stair_Paired_Enter_Index = NAG_STAIR_PAIRED_ENTER_INVALID;
    N.Bump_Paired_Enter_Index = NAG_BUMP_PAIRED_ENTER_INVALID;
    N.Bridge_Exit_Run_Index = NAG_BRIDGE_EXIT_INDEX_INVALID;
    N.Stair2_Exit_Run_Index = NAG_STAIR2_EXIT_INDEX_INVALID;
    N.Stair2_Paired_Enter_Index = NAG_STAIR2_PAIRED_ENTER_INVALID;
    memset(Nag_Event_Table, 0, sizeof(Nag_Event_Table));
    N.Flash_page_index = Nag_GetSubjectYawStartPage(g_nag_record_subject);
    N.Event_Active_Index = 0xFFu;
    N.Event_Record_Type = NAG_EVENT_TYPE_SPIN;
    flash_Nag_ResetReadState();
    flash_buffer_clear();
    Nag_OdoSlip_ResetState();
}

void Nag_Begin_Record(void)
{
    /* 按 Run→RecSubj 所选科目绑定 Flash 页区后进入录制态（index=1） */
    Init_Nag();
    steer_yaw_request_pending = 0;
    steer_yaw_delayed_by_spin = 0;
    steer_task_stop();
#if NAV_FUSION_ENABLE && NAG_USE_FUSION_MILEAGE && NAV_FUSION_ORIGIN_ENABLE
    /* 融合惯导录制：先静止 GPS 原点平均，再锁定发车 yaw 直行 5m 标定北向偏角；纯惯导开关关闭时不进入 */
    if (NavFusion_IsRuntimeEnabled() != 0u)
    {
        NavFusion_BeginOriginAverage((float)euler_angle.yaw);
#if NAV_FUSION_HEADING_CALIB_ENABLE
        NavFusion_BeginHeadingCalibSession((float)euler_angle.yaw);
#endif
    }
#endif
    N.Nag_SystemRun_Index = 1;
}

/* 进入回放准备态：索引置 2，待 NagFlashRead() 读完 flash 且（若启用）原点平均完成后进入 3。 */
#if NAV_FUSION_ENABLE && NAG_USE_FUSION_MILEAGE && NAV_FUSION_ORIGIN_ENABLE
static void Nag_TryEnterReplayRun(void)
{
    if (N.Nag_SystemRun_Index != 2u || g_nag_replay_flash_ready == 0u)
    {
        return;
    }

    /* 纯惯导（融合运行时关）：不等待 GPS 原点/北向标定，flash 读完即进入 index=3 */
    if (NavFusion_IsRuntimeEnabled() == 0u)
    {
        motor_user_speed_cmd = run_launch_speed;
        N.Target_Speed = fabsf((float)motor_user_speed_cmd);
        Nag_UpdatePreviewAndSpeedTarget();
        N.Nag_SystemRun_Index = 3u;
        return;
    }

    if (NavFusion_IsOriginCalibrating() != 0u)
    {
        return;
    }
#if NAV_FUSION_HEADING_CALIB_ENABLE
    if (NavFusion_IsHeadingCalibSessionActive() != 0u &&
        NavFusion_IsHeadingCalibReady() == 0u)
    {
        /* 融合回放：北向标定直行 5m 期间用 Launch 速度，标定完成后再进 index=3 */
        motor_user_speed_cmd = run_launch_speed;
        N.Target_Speed = fabsf((float)motor_user_speed_cmd);
        Nag_UpdatePreviewAndSpeedTarget();
        return;
    }
#endif
    if (NavFusion_IsValid() == 0u)
    {
        return;
    }

    motor_user_speed_cmd = run_launch_speed;
    N.Target_Speed = fabsf((float)motor_user_speed_cmd);
    Nag_UpdatePreviewAndSpeedTarget();
#if NAV_FUSION_ENABLE && NAG_USE_FUSION_MILEAGE
    NavFusion_SyncMileageSnapshot();
#if NAV_FUSION_HEADING_CALIB_ENABLE
    Nag_SkipReplayRunIndexForFusionHeadingCalib();
#endif
    Nag_UpdatePreviewAndSpeedTarget();
#endif
    N.Nag_SystemRun_Index = 3u;
}

void Nag_CompleteReplayAfterOrigin(void)
{
    Nag_TryEnterReplayRun();
}
#else
void Nag_CompleteReplayAfterOrigin(void)
{
}
#endif

void Nag_Begin_Replay(void)
{
    /* 按 Run→PlaySubj 所选科目从 Flash 装载轨迹（index=2→NagFlashRead→3） */
    nav_heading_mode = NAV_HEADING_MODE_INS;
    N.Mileage_All = 0;
    N.Mileage_Step = 0;
    N.Mileage_Debug_Total = 0;
    N.Speed_Forward = 0;
    N.Curve_Strength = 0;
    N.Target_Speed = 0.0f;
    N.Angle_Run = 0;
    N.Run_index = 0;
    N.Prospect_index = 0;
    N.Nag_Stop_f = 0;
    N.Save_state = 0;
    N.End_f = 0;
    N.Flash_page_index = Nag_GetSubjectYawStartPage(g_nag_replay_subject);
    N.Requested_Target_Yaw = 0;
    N.Target_Request_Valid = 0;
    Nag_ClearEventConsumed();
    Nag_ClearEventRuntimeState();
    steer_yaw_request_pending = 0;
    steer_yaw_delayed_by_spin = 0;
    steer_task_stop();
    flash_Nag_ResetReadState();
#if NAV_FUSION_ENABLE && NAG_USE_FUSION_MILEAGE
    g_nag_replay_flash_ready = 0u;
    if (NavFusion_IsRuntimeEnabled() != 0u)
    {
        NavFusion_Reset();
#if NAV_FUSION_ORIGIN_ENABLE
        NavFusion_BeginOriginAverage((float)euler_angle.yaw);
#if NAV_FUSION_HEADING_CALIB_ENABLE
        NavFusion_BeginHeadingCalibSession((float)euler_angle.yaw);
#endif
#else
        {
            uint8 gnss_live = (uint8)((gnss.time.year != 0u) || (gnss.state != 0u) || (gnss.satellite_used != 0u));
            if (gnss_live && (gnss.latitude != 0.0) && (gnss.longitude != 0.0))
            {
                NavFusion_InitFromGps(gnss.latitude, gnss.longitude, (float)euler_angle.yaw);
            }
        }
#endif
    }
#endif
    N.Nag_SystemRun_Index = 2;
}

void Nag_Request_Stop_Record(void)
{
    if (N.Nag_SystemRun_Index == 1 && N.End_f == 0)
    {
        N.End_f = 1;
    }
}

void Nag_Request_Event_Mark(void)
{
    if (N.Nag_SystemRun_Index != 1 || N.Event_Count >= Nag_Event_Max)
    {
        return;
    }

    Nag_Event_Table[N.Event_Count].enter_index = N.Save_index;
    Nag_Event_Table[N.Event_Count].exit_index = N.Save_index;
    Nag_Event_Table[N.Event_Count].type = N.Event_Record_Type;
    Nag_Event_Table[N.Event_Count].valid = 1;
    N.Event_Count++;
    N.Event_Record_Pending = 0;
}

void Nag_Cycle_Record_Event_Type(void)
{
    N.Event_Record_Type = (uint8)((N.Event_Record_Type + 1u) % NAG_EVENT_TYPE_COUNT);
}

void Nag_Notify_Event_Done(void)
{
    uint8 event_index = N.Event_Active_Index;
    uint8 event_type = N.Event_Active_Type;
    uint8 chain_exit_stair = 0u;
    uint8 chain_exit_bump = 0u;
    uint8 chain_exit_stair2 = 0u;
    float preserved_saved_speed = 0.0f;
    float preserved_saved_leg = 0.0f;
    float preserved_bump_saved_speed = 0.0f;
    float preserved_bump_saved_leg = 0.0f;
    uint16 preserved_stair_enter_index = NAG_STAIR_PAIRED_ENTER_INVALID;
    uint16 preserved_bump_enter_index = NAG_BUMP_PAIRED_ENTER_INVALID;
    float preserved_stair2_saved_speed = 0.0f;
    uint16 preserved_stair2_enter_index = NAG_STAIR2_PAIRED_ENTER_INVALID;
    uint16 enter_index = 0;
    uint16 exit_index = 0;
    uint16 resume_index = 0;
    uint16 max_run_index = 0;

    if (!N.Event_Active)
    {
        return;
    }

    if (nav_heading_mode == NAV_HEADING_MODE_GPS)
    {
        if (event_index >= gps_point_count)
        {
            return;
        }
        Nag_Element_Stop(event_type);
        Nag_ClearEventRuntimeState();
        GPS_NavOnElementDone();
        return;
    }

    if (event_type == NAG_EVENT_TYPE_ENTER_STAIR)
    {
        chain_exit_stair = 1u;
        preserved_saved_speed = N.Stair_Saved_SetSpeed;
        preserved_saved_leg = N.Stair_Saved_Leg_Long;
        N.Stair_Chain_To_Exit = 1u;
    }

    if (event_type == NAG_EVENT_TYPE_ENTER_BUMP)
    {
        chain_exit_bump = 1u;
        preserved_bump_saved_speed = N.Bump_Saved_SetSpeed;
        preserved_bump_saved_leg = N.Bump_Saved_Leg_Long;
        N.Bump_Chain_To_Exit = 1u;
    }

    if (event_type == NAG_EVENT_TYPE_ENTER_STAIR2)
    {
        chain_exit_stair2 = 1u;
        preserved_stair2_saved_speed = N.Stair2_Saved_SetSpeed;
        N.Stair2_Chain_To_Exit = 1u;
    }

    if (event_type == NAG_EVENT_TYPE_ENTER_STAIR &&
        event_index != 0xFFu && event_index < N.Event_Count)
    {
        enter_index = Nag_Event_Table[event_index].enter_index;
        N.Stair_Paired_Enter_Index = enter_index;
        preserved_stair_enter_index = enter_index;
        /* 链式切 EXIT 前保持 Ein，不推到 Ein+1。 */
        resume_index = N.Run_index;
    }
    else if (event_type == NAG_EVENT_TYPE_ENTER_BUMP &&
             event_index != 0xFFu && event_index < N.Event_Count)
    {
        enter_index = Nag_Event_Table[event_index].enter_index;
        N.Bump_Paired_Enter_Index = enter_index;
        preserved_bump_enter_index = enter_index;
        /* 链式切 EXIT 前保持 Bin，不推到 Bin+1。 */
        resume_index = N.Run_index;
    }
    else if (event_type == NAG_EVENT_TYPE_ENTER_STAIR2 &&
             event_index != 0xFFu && event_index < N.Event_Count)
    {
        enter_index = Nag_Event_Table[event_index].enter_index;
        N.Stair2_Paired_Enter_Index = enter_index;
        preserved_stair2_enter_index = enter_index;
        resume_index = N.Run_index;
    }
    else if (event_type == NAG_EVENT_TYPE_EXIT_STAIR)
    {
        if (event_index != 0xFFu && event_index < N.Event_Count)
        {
            enter_index = Nag_Event_Table[event_index].enter_index;
        }
        resume_index = Nag_ComputeStairResumeIndex(event_index, enter_index);
        N.Stair_Paired_Enter_Index = NAG_STAIR_PAIRED_ENTER_INVALID;
    }
    else if (event_type == NAG_EVENT_TYPE_EXIT_BUMP)
    {
        if (event_index != 0xFFu && event_index < N.Event_Count)
        {
            enter_index = Nag_Event_Table[event_index].enter_index;
        }
        resume_index = Nag_ComputeBumpResumeIndex(event_index, enter_index);
        N.Bump_Paired_Enter_Index = NAG_BUMP_PAIRED_ENTER_INVALID;
    }
    else if (event_type == NAG_EVENT_TYPE_EXIT_STAIR2)
    {
        if (event_index != 0xFFu && event_index < N.Event_Count)
        {
            enter_index = Nag_Event_Table[event_index].enter_index;
        }
        resume_index = Nag_ComputeStair2ResumeIndex(event_index, enter_index);
        N.Stair2_Paired_Enter_Index = NAG_STAIR2_PAIRED_ENTER_INVALID;
    }
    else if (event_index != 0xFFu && event_index < N.Event_Count)
    {
        enter_index = Nag_Event_Table[event_index].enter_index;
        exit_index = Nag_Event_Table[event_index].exit_index;

        if (event_type == NAG_EVENT_TYPE_SPIN)
        {
            /* 从起转前锁存的实际路径索引恢复，不按录制 enter_index 或 enter_index+1 跳点。 */
            resume_index = (N.Spin_Task_Started != 0u) ? N.Spin_Resume_RunIndex : N.Run_index;
        }
        else if (exit_index > enter_index)
        {
            /* 旧双点录制：从 exit 索引接回惯导。 */
            resume_index = exit_index;
        }
        else
        {
            /* 单点精确触发：推进到触发点之后，避免再次命中同一 enter_index。 */
            resume_index = (uint16)(enter_index + 1u);
            if (N.Save_index >= 2u)
            {
                max_run_index = (uint16)(N.Save_index - 2u);
                if (resume_index > max_run_index)
                {
                    resume_index = max_run_index;
                }
            }
        }
    }
    else if (event_index != 0xFFu)
    {
        return;
    }

    if (event_type == NAG_EVENT_TYPE_SPIN ||
        event_type == NAG_EVENT_TYPE_EXIT_STAIR ||
        event_type == NAG_EVENT_TYPE_EXIT_BUMP ||
        event_type == NAG_EVENT_TYPE_EXIT_STAIR2)
    {
#if NAV_FUSION_ENABLE && NAG_USE_FUSION_MILEAGE
        /* 丢弃元素期融合位移增量，防止恢复后第一拍 Run_index 异常跳点。 */
        NavFusion_SyncMileageSnapshot();
#endif
    }

    if (event_index != 0xFFu)
    {
        N.Event_Consumed[event_index] = 1u;
    }

    N.Run_index = resume_index;
    N.Mileage_All = 0.0f;
    Nag_OdoSlip_ResetState();
    N.Target_Request_Valid = 0;
    steer_yaw_request_pending = 0u;
    steer_yaw_delayed_by_spin = 0u;
    Nag_Element_Stop(event_type);
    Nag_ClearEventRuntimeState();

    if (chain_exit_stair != 0u)
    {
        N.Stair_Saved_SetSpeed = preserved_saved_speed;
        N.Stair_Saved_Leg_Long = preserved_saved_leg;
        N.Stair_Paired_Enter_Index = preserved_stair_enter_index;
        Nag_ConsumeTableExitStairEvents();
        Nag_ActivateChainedExitStair();
    }
    else if (chain_exit_bump != 0u)
    {
        N.Bump_Saved_SetSpeed = preserved_bump_saved_speed;
        N.Bump_Saved_Leg_Long = preserved_bump_saved_leg;
        N.Bump_Paired_Enter_Index = preserved_bump_enter_index;
        Nag_ConsumeTableExitBumpEvents();
        Nag_ActivateChainedExitBump();
    }
    else if (chain_exit_stair2 != 0u)
    {
        N.Stair2_Saved_SetSpeed = preserved_stair2_saved_speed;
        N.Stair2_Paired_Enter_Index = preserved_stair2_enter_index;
        Nag_ConsumeTableExitStair2Events();
        Nag_ActivateChainedExitStair2();
    }

    Nag_UpdatePreviewAndSpeedTarget();
    N.Angle_Run = (float)(Nav_read[N.Prospect_index] / 100.0f);
}

void Nag_Element_Abort(void)
{
    if (!N.Event_Active)
    {
        return;
    }

    /* 颠簸区中途 Abort：恢复进入前备份的速度/腿长，强制关闭横滚平衡。 */
    if (N.Event_Active_Type == NAG_EVENT_TYPE_ENTER_STAIR2 &&
        N.Stair2_Chain_To_Exit == 0u)
    {
        motor_user_speed_cmd = N.Stair2_Saved_SetSpeed;
        N.Stair2_Saved_SetSpeed = 0.0f;
        N.Stair2_Zone_Active = 0u;
        N.Stair2_Detect_Arm = 0u;
        N.Stair2_Phase = NAG_STAIR2_PHASE_SEEK_FIRST;
        N.Stair2_First_Blob_Latched = 0u;
        N.Stair2_Second_Blob_Seen = 0u;
        N.Stair2_Exit_Run_Index = NAG_STAIR2_EXIT_INDEX_INVALID;
        s_stair2_blob_lost_ms = 0u;
        s_stair2_enter_grace_ms = 0u;
        s_stair2_blob_no_frame_ms = 0u;
    }

    /* 颠簸区中途 Abort：恢复进入前备份的速度/腿长，强制关闭横滚平衡。 */
    if (N.Event_Active_Type == NAG_EVENT_TYPE_ENTER_BUMP &&
        N.Bump_Chain_To_Exit == 0u)
    {
        if (N.Bump_Saved_Leg_Long > 0.01f)
        {
            leg_long = N.Bump_Saved_Leg_Long;
        }
        motor_user_speed_cmd = N.Bump_Saved_SetSpeed;
        roll_balance_en = 0u;
        N.Bump_Zone_Active = 0u;
        N.Bump_Saved_Leg_Long = 0.0f;
        N.Bump_Saved_SetSpeed = 0.0f;
        N.Bump_Locked_Yaw = 0.0f;
        N.Bump_Elapsed_Ms = 0u;
    }

    /* 桥区中途 Abort：恢复进入桥前备份并清桥区状态。 */
    if (N.Bridge_Zone_Active != 0u || N.Bridge_Heading_Lock != 0u || N.Bridge_Expected != 0u)
    {
        if (N.Bridge_Saved_Leg_Long > 0.01f)
        {
            leg_long = N.Bridge_Saved_Leg_Long;
        }
        roll_balance_en = N.Bridge_Saved_RollBalance;
        N.Bridge_Zone_Active = 0u;
        N.Bridge_Heading_Lock = 0u;
        N.Bridge_Expected = 0u;
        N.Bridge_Detect_Arm = 0u;
        N.Bridge_Saved_Leg_Long = 0.0f;
        N.Bridge_Saved_RollBalance = 0u;
        N.Bridge_Exit_Run_Index = NAG_BRIDGE_EXIT_INDEX_INVALID;
        s_bridge_blob_lost_ms = 0u;
        s_bridge_enter_grace_ms = 0u;
        s_bridge_blob_no_frame_ms = 0u;
    }

    if (N.Event_Active_Type == NAG_EVENT_TYPE_SPIN)
    {
        Nag_Spin_RestoreSetSpeed();
        if (N.Spin_Task_Started)
        {
            spin_task_stop();
        }
        steer_yaw_request_pending = 0u;
        steer_yaw_delayed_by_spin = 0u;
    }

    N.Event_State = NAG_EVENT_STATE_ABORT;
    Nag_Element_Stop(N.Event_Active_Type);
    N.Target_Request_Valid = 0;
    steer_task_stop();
}
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     封装执行函数
// 参数说明     index           索引
// 参数说明     type            类型值
// 返回参数     void
// 使用示例     在中断里调用
// 备注信息     当前固定在 pit0_ch0_isr 的 1ms 中断里调用一次；请勿同时在主循环/5ms 软任务里重复调用。
//             Nag_SystemRun_Index==2 时 switch 无分支，不执行 Nag_Run；速度门控由 Nag_GetControlSpeedTarget 在索引!=3 时返回 0。
//-------------------------------------------------------------------------------------------------------------------
void Nag_System(){
    Nag_BridgeTimeoutTick1ms();
    Nag_Stair2TimeoutTick1ms();

    if (N.Event_Active)
    {
        Nag_Element_StateMachine();
        if (nav_heading_mode == NAV_HEADING_MODE_GPS)
        {
            return;
        }
    }

    if(!N.Nag_SystemRun_Index || N.Nag_Stop_f )
    {
        return;
    }

    switch(N.Nag_SystemRun_Index)
    {
       case 1 : Nag_Read();    //1是读取
            break;
      case 3: Nag_Run();
            break;
    }
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     第一次自动读取，只读取一次
// 参数说明     index           索引
// 参数说明     type            类型值
// 返回参数     void
// 使用示例     在主循环直接调用，demo中直接显示
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void NagFlashRead(){
  if(N.Save_state) return;
  N.Flash_page_index = Nag_GetSubjectYawStartPage(g_nag_replay_subject);
  flash_Nag_Read();
  uint8 page_trun=0;
  
  for(int index=0;index <= N.Save_index;index++)
  {
    if(index >= N.Save_index)
    {
        N.Save_state=1;
        break;
    }
    int temp_index=index-(MaxSize*page_trun);
    if(temp_index >= MaxSize)    //当超过设定的flsh大小时
    {
        N.Flash_page_index--;   //页索引减小
        page_trun++;
        flash_Nag_Read(); //重新读取
        temp_index=index-(MaxSize*page_trun);
    }
     Nav_read[index]= flash_union_buffer[temp_index].int32_type;
  }
  N.Mileage_All = 0;
  N.Mileage_Step = 0;
  N.Mileage_Debug_Total = 0;
  Nag_OdoSlip_ResetState();
  N.Run_index = 0;
  N.Prospect_index = 0;
  N.Nag_Stop_f = 0;
  N.Curve_Strength = 0;
  if (N.Nag_SystemRun_Index == 2u)
  {
#if NAV_FUSION_ENABLE && NAG_USE_FUSION_MILEAGE && NAV_FUSION_ORIGIN_ENABLE
    /* 融合模式：原点/北向标定完成前不赋速；纯惯导仍用 Launch 速度 */
    if (NavFusion_IsRuntimeEnabled() != 0u)
    {
        motor_user_speed_cmd = 0.0f;
    }
    else
    {
        motor_user_speed_cmd = run_launch_speed;
    }
#else
    motor_user_speed_cmd = run_launch_speed;
#endif
  }
  N.Target_Speed = fabsf((float)motor_user_speed_cmd);
  N.Angle_Run = (N.Save_index > 0) ? (float)(Nav_read[0] / 100.0f) : (float)Nag_Yaw;
  N.Requested_Target_Yaw = 0;
  N.Target_Request_Valid = 0;
  Nag_ClearEventConsumed();
  Nag_ClearEventRuntimeState();
  Nag_UpdatePreviewAndSpeedTarget();
  N.Angle_Run = (N.Save_index > 0) ? (float)(Nav_read[N.Prospect_index] / 100.0f) : (float)Nag_Yaw;
#if NAV_FUSION_ENABLE && NAG_USE_FUSION_MILEAGE && NAV_FUSION_ORIGIN_ENABLE
  g_nag_replay_flash_ready = 1u;
  Nag_TryEnterReplayRun();
#else
  N.Nag_SystemRun_Index++;
#endif
}

/* -------------------------------------------------------------------------- */
/* 惯导路径修正（Debug → PathFix 2.6 → KEY3 进入 2.6.1 功能页）                  */
/* -------------------------------------------------------------------------- */

NagPathFixState g_nag_pathfix;

/* 折线屏幕坐标缓存：避免每帧 O(N) 甚至旧实现 O(N^2) 重算 */
static int16 s_pathfix_cache_x[Nag_PathFix_Draw_Max];
static int16 s_pathfix_cache_y[Nag_PathFix_Draw_Max];
static uint16 s_pathfix_cache_count = 0u;
static uint16 s_pathfix_cache_sel = 0u;
static uint8 s_pathfix_cache_valid = 0u;
static uint16 s_pathfix_cache_x_off = 0u;
static uint16 s_pathfix_cache_y_off = 0u;
static uint16 s_pathfix_cache_w = 0u;
static uint16 s_pathfix_cache_h = 0u;
/* 元素点十字（统一颜色，不区分类型） */
static int16 s_pathfix_elem_x[Nag_PathFix_Elem_Max];
static int16 s_pathfix_elem_y[Nag_PathFix_Elem_Max];
static uint16 s_pathfix_elem_count = 0u;

static float Nag_PathFix_WrapYawDeg(float yaw_deg)
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

static int32 Nag_PathFix_YawToStorage(float yaw_deg)
{
    return (int32)(Nag_PathFix_WrapYawDeg(yaw_deg) * 100.0f);
}

static uint8 Nag_PathFix_CanEnter(void)
{
    /* 录制/回放/元素接管期间禁止进入，避免与 1ms 导航状态机冲突 */
    if ((N.Nag_SystemRun_Index == 1u) && (N.End_f == 0u))
    {
        return 0u;
    }
    if (N.Nag_SystemRun_Index >= 2u)
    {
        return 0u;
    }
    if (N.Event_Active != 0u)
    {
        return 0u;
    }
    return 1u;
}

static uint16 Nag_PathFix_ClampU16(int32 value, uint16 min_value, uint16 max_value)
{
    if (value < (int32)min_value)
    {
        return min_value;
    }
    if (value > (int32)max_value)
    {
        return max_value;
    }
    return (uint16)value;
}

static void Nag_PathFix_FillViewportGrid(uint16 x_offset, uint16 y_offset, uint16 width, uint16 height)
{
    uint16 y = 0;
    uint16 x_end = (uint16)(x_offset + width);
    uint16 y_end = (uint16)(y_offset + height);

    for (y = y_offset; y <= y_end; y++)
    {
        ips200_draw_line(x_offset, y, x_end, y, RGB565_WHITE);
    }
}

static void Nag_PathFix_DrawPoint(uint16 x,
                                  uint16 y,
                                  uint16 x_offset,
                                  uint16 y_offset,
                                  uint16 width,
                                  uint16 height,
                                  uint16 color)
{
    uint16 x_max = (uint16)(x_offset + width);
    uint16 y_max = (uint16)(y_offset + height);

    ips200_draw_point(x, y, color);
    ips200_draw_point(Nag_PathFix_ClampU16((int32)x + 1, x_offset, x_max), y, color);
    ips200_draw_point(Nag_PathFix_ClampU16((int32)x - 1, x_offset, x_max), y, color);
    ips200_draw_point(x, Nag_PathFix_ClampU16((int32)y + 1, y_offset, y_max), color);
    ips200_draw_point(x, Nag_PathFix_ClampU16((int32)y - 1, y_offset, y_max), color);
}

/* 通用十字标记（元素点/选中点共用几何，颜色由调用方指定） */
static void Nag_PathFix_DrawCrossMarker(uint16 x,
                                        uint16 y,
                                        uint16 x_offset,
                                        uint16 y_offset,
                                        uint16 width,
                                        uint16 height,
                                        uint16 color,
                                        uint16 span)
{
    uint16 x_max = (uint16)(x_offset + width);
    uint16 y_max = (uint16)(y_offset + height);
    uint16 x0 = Nag_PathFix_ClampU16((int32)x, x_offset, x_max);
    uint16 y0 = Nag_PathFix_ClampU16((int32)y, y_offset, y_max);

    ips200_draw_line(Nag_PathFix_ClampU16((int32)x0 - (int32)span, x_offset, x_max), y0,
                     Nag_PathFix_ClampU16((int32)x0 + (int32)span, x_offset, x_max), y0,
                     color);
    ips200_draw_line(x0, Nag_PathFix_ClampU16((int32)y0 - (int32)span, y_offset, y_max),
                     x0, Nag_PathFix_ClampU16((int32)y0 + (int32)span, y_offset, y_max),
                     color);
}

/* 当前选中点：黑色加粗十字，最后绘制以免被轨迹/端点覆盖 */
static void Nag_PathFix_DrawSelectedMarker(uint16 x,
                                           uint16 y,
                                           uint16 x_offset,
                                           uint16 y_offset,
                                           uint16 width,
                                           uint16 height)
{
    uint16 x_max = (uint16)(x_offset + width);
    uint16 y_max = (uint16)(y_offset + height);
    uint16 x0 = Nag_PathFix_ClampU16((int32)x, x_offset, x_max);
    uint16 y0 = Nag_PathFix_ClampU16((int32)y, y_offset, y_max);
    uint16 span = 4u;

    Nag_PathFix_DrawCrossMarker(x0, y0, x_offset, y_offset, width, height, RGB565_BLACK, span);
    ips200_draw_line(Nag_PathFix_ClampU16((int32)x0 - 3, x_offset, x_max),
                     Nag_PathFix_ClampU16((int32)y0 - 3, y_offset, y_max),
                     Nag_PathFix_ClampU16((int32)x0 + 3, x_offset, x_max),
                     Nag_PathFix_ClampU16((int32)y0 + 3, y_offset, y_max),
                     RGB565_BLACK);
    ips200_draw_line(Nag_PathFix_ClampU16((int32)x0 - 3, x_offset, x_max),
                     Nag_PathFix_ClampU16((int32)y0 + 3, y_offset, y_max),
                     Nag_PathFix_ClampU16((int32)x0 + 3, x_offset, x_max),
                     Nag_PathFix_ClampU16((int32)y0 - 3, y_offset, y_max),
                     RGB565_BLACK);
}

/* 收集有效元素 enter/exit 索引（去重），全部用同一颜色显示 */
static uint16 Nag_PathFix_CollectElemIndices(uint16 *out_idx, uint16 out_max)
{
    uint16 n = 0u;
    uint8 ei = 0u;
    uint8 event_count = N.Event_Count;

    if ((out_idx == NULL) || (out_max == 0u))
    {
        return 0u;
    }
    if (event_count > Nag_Event_Max)
    {
        event_count = Nag_Event_Max;
    }

    for (ei = 0u; ei < event_count; ei++)
    {
        uint16 idxs[2];
        uint8 k = 0u;

        if (Nag_Event_Table[ei].valid == 0u)
        {
            continue;
        }

        idxs[0] = Nag_Event_Table[ei].enter_index;
        idxs[1] = Nag_Event_Table[ei].exit_index;
        for (k = 0u; k < 2u; k++)
        {
            uint16 j = 0u;
            uint8 dup = 0u;

            if (idxs[k] >= g_nag_pathfix.point_count)
            {
                continue;
            }
            /* 单点元素 enter==exit，只记一次 */
            if ((k == 1u) && (idxs[1] == idxs[0]))
            {
                continue;
            }
            for (j = 0u; j < n; j++)
            {
                if (out_idx[j] == idxs[k])
                {
                    dup = 1u;
                    break;
                }
            }
            if ((dup == 0u) && (n < out_max))
            {
                out_idx[n++] = idxs[k];
            }
        }
    }
    return n;
}

/* 由原始点索引映射到抽稀后的显示索引（O(1)） */
static uint16 Nag_PathFix_SrcToDrawIdx(uint16 src_idx, uint16 draw_count, uint16 point_count)
{
    if ((draw_count == 0u) || (point_count == 0u))
    {
        return 0u;
    }
    if (draw_count == 1u)
    {
        return 0u;
    }
    if (src_idx >= point_count)
    {
        src_idx = (uint16)(point_count - 1u);
    }
    return (uint16)((uint32)src_idx * (uint32)(draw_count - 1u) / (uint32)(point_count - 1u));
}

/* O(N) 单次累计：先求包围盒，再抽稀映射到屏幕；替代旧版对每个点 WorldCmAt 的 O(N^2) */
static void Nag_PathFix_RebuildDrawMap(uint16 x_offset,
                                       uint16 y_offset,
                                       uint16 width,
                                       uint16 height,
                                       uint16 out_max)
{
    float x_min = 0.0f;
    float x_max = 0.0f;
    float y_min = 0.0f;
    float y_max = 0.0f;
    float x_range = 0.0f;
    float y_range = 0.0f;
    float wx = 0.0f;
    float wy = 0.0f;
    uint16 point_count = g_nag_pathfix.point_count;
    uint16 draw_count = 0u;
    uint16 i = 0u;
    uint16 next_src = 0u;
    uint16 emit_i = 0u;
    uint16 elem_idx[Nag_PathFix_Elem_Max];
    uint16 elem_n = 0u;

    s_pathfix_cache_valid = 0u;
    s_pathfix_cache_count = 0u;
    s_pathfix_cache_sel = 0u;
    s_pathfix_elem_count = 0u;

    if ((point_count == 0u) || (out_max == 0u) || (width == 0u) || (height == 0u))
    {
        return;
    }

    elem_n = Nag_PathFix_CollectElemIndices(elem_idx, Nag_PathFix_Elem_Max);

    /* Pass1：O(N) 累计求包围盒 */
    wx = 0.0f;
    wy = 0.0f;
    x_min = 0.0f;
    x_max = 0.0f;
    y_min = 0.0f;
    y_max = 0.0f;
    for (i = 1u; i < point_count; i++)
    {
        float yaw_deg = (float)Nav_read[i - 1u] / 100.0f;
        float yaw_rad = yaw_deg * 3.1415926f / 180.0f;

        wx += Nag_Set_mileage * sinf(yaw_rad);
        wy += Nag_Set_mileage * cosf(yaw_rad);
        if (wx < x_min)
        {
            x_min = wx;
        }
        if (wx > x_max)
        {
            x_max = wx;
        }
        if (wy < y_min)
        {
            y_min = wy;
        }
        if (wy > y_max)
        {
            y_max = wy;
        }
    }

    x_range = x_max - x_min;
    y_range = y_max - y_min;
    if (fabsf(x_range) < 1.0f)
    {
        x_range = 1.0f;
    }
    if (fabsf(y_range) < 1.0f)
    {
        y_range = 1.0f;
    }

    draw_count = point_count;
    if (draw_count > out_max)
    {
        draw_count = out_max;
    }
    if (draw_count > Nag_PathFix_Draw_Max)
    {
        draw_count = Nag_PathFix_Draw_Max;
    }

    /* Pass2：O(N) 再扫一遍，按均匀抽稀写入屏幕坐标 */
    wx = 0.0f;
    wy = 0.0f;
    emit_i = 0u;
    next_src = 0u;
    for (i = 0u; i < point_count; i++)
    {
        if (i > 0u)
        {
            float yaw_deg = (float)Nav_read[i - 1u] / 100.0f;
            float yaw_rad = yaw_deg * 3.1415926f / 180.0f;

            wx += Nag_Set_mileage * sinf(yaw_rad);
            wy += Nag_Set_mileage * cosf(yaw_rad);
        }

        {
            float x_ratio = 0.0f;
            float y_ratio = 0.0f;
            int32 mapped_x = 0;
            int32 mapped_y = 0;
            uint16 ei = 0u;
            uint8 is_sample = (uint8)((i == next_src) && (emit_i < draw_count));
            uint8 is_elem = 0u;

            for (ei = 0u; ei < elem_n; ei++)
            {
                if (elem_idx[ei] == i)
                {
                    is_elem = 1u;
                    break;
                }
            }

            if ((is_sample != 0u) || (is_elem != 0u))
            {
                /* 横向镜像：车体左转对应屏幕左侧（与实车左右一致；wy 仍北向上） */
                x_ratio = (x_max - wx) / x_range;
                y_ratio = (y_max - wy) / y_range;
                mapped_x = (int32)x_offset + (int32)(x_ratio * (float)width);
                mapped_y = (int32)y_offset + (int32)(y_ratio * (float)height);
            }

            if (is_sample != 0u)
            {
                s_pathfix_cache_x[emit_i] =
                    (int16)Nag_PathFix_ClampU16(mapped_x, x_offset, (uint16)(x_offset + width));
                s_pathfix_cache_y[emit_i] =
                    (int16)Nag_PathFix_ClampU16(mapped_y, y_offset, (uint16)(y_offset + height));
                emit_i++;

                if (emit_i < draw_count)
                {
                    next_src = (uint16)((uint32)emit_i * (uint32)(point_count - 1u) /
                                        (uint32)(draw_count - 1u));
                }
            }

            if ((is_elem != 0u) && (s_pathfix_elem_count < Nag_PathFix_Elem_Max))
            {
                s_pathfix_elem_x[s_pathfix_elem_count] =
                    (int16)Nag_PathFix_ClampU16(mapped_x, x_offset, (uint16)(x_offset + width));
                s_pathfix_elem_y[s_pathfix_elem_count] =
                    (int16)Nag_PathFix_ClampU16(mapped_y, y_offset, (uint16)(y_offset + height));
                s_pathfix_elem_count++;
            }
        }
    }

    s_pathfix_cache_count = emit_i;
    s_pathfix_cache_sel = Nag_PathFix_SrcToDrawIdx(g_nag_pathfix.select_index,
                                                    s_pathfix_cache_count,
                                                    point_count);
    s_pathfix_cache_x_off = x_offset;
    s_pathfix_cache_y_off = y_offset;
    s_pathfix_cache_w = width;
    s_pathfix_cache_h = height;
    s_pathfix_cache_valid = 1u;
    g_nag_pathfix.map_dirty = 0u;
}

void Nag_PathFix_PublishDrawMap(uint16 x_offset,
                              uint16 y_offset,
                              uint16 width,
                              uint16 height,
                              int16 *out_x,
                              int16 *out_y,
                              uint16 out_max,
                              uint16 *out_count,
                              uint16 *out_sel_draw)
{
    uint16 i = 0u;
    uint8 need_rebuild = 0u;

    if ((out_x == NULL) || (out_y == NULL) || (out_count == NULL) || (out_max == 0u))
    {
        return;
    }

    *out_count = 0u;
    if (out_sel_draw != NULL)
    {
        *out_sel_draw = 0u;
    }

    if ((g_nag_pathfix.loaded == 0u) || (g_nag_pathfix.point_count == 0u))
    {
        s_pathfix_cache_valid = 0u;
        s_pathfix_elem_count = 0u;
        return;
    }

    need_rebuild = (uint8)((g_nag_pathfix.map_dirty != 0u) ||
                           (s_pathfix_cache_valid == 0u) ||
                           (s_pathfix_cache_x_off != x_offset) ||
                           (s_pathfix_cache_y_off != y_offset) ||
                           (s_pathfix_cache_w != width) ||
                           (s_pathfix_cache_h != height));

    if (need_rebuild != 0u)
    {
        Nag_PathFix_RebuildDrawMap(x_offset, y_offset, width, height, out_max);
    }
    else
    {
        /* 仅选点变化：O(1) 更新高亮索引，不重算几何 */
        s_pathfix_cache_sel = Nag_PathFix_SrcToDrawIdx(g_nag_pathfix.select_index,
                                                        s_pathfix_cache_count,
                                                        g_nag_pathfix.point_count);
    }

    if (s_pathfix_cache_valid == 0u)
    {
        return;
    }

    *out_count = s_pathfix_cache_count;
    if (out_sel_draw != NULL)
    {
        *out_sel_draw = s_pathfix_cache_sel;
    }

    for (i = 0u; i < s_pathfix_cache_count; i++)
    {
        out_x[i] = s_pathfix_cache_x[i];
        out_y[i] = s_pathfix_cache_y[i];
    }
}

void Nag_PathFix_DrawViewport(uint16 x_offset, uint16 y_offset, uint16 width, uint16 height)
{
    int16 map_x[Nag_PathFix_Draw_Max];
    int16 map_y[Nag_PathFix_Draw_Max];
    uint16 draw_count = 0;
    uint16 sel_draw = 0;
    uint16 i = 0;

    if ((width == 0u) || (height == 0u))
    {
        return;
    }

    Nag_PathFix_FillViewportGrid(x_offset, y_offset, width, height);
    Nag_PathFix_PublishDrawMap(x_offset, y_offset, width, height,
                              map_x, map_y, Nag_PathFix_Draw_Max,
                              &draw_count, &sel_draw);

    /* 折线 → 起终点 → 元素十字(蓝) → 选中高亮(黑) */
    for (i = 1u; i < draw_count; i++)
    {
        ips200_draw_line((uint16)map_x[i - 1u], (uint16)map_y[i - 1u],
                         (uint16)map_x[i], (uint16)map_y[i], RGB565_RED);
    }

    if (draw_count > 0u)
    {
        Nag_PathFix_DrawPoint((uint16)map_x[0], (uint16)map_y[0],
                              x_offset, y_offset, width, height, RGB565_GREEN);
        if (draw_count > 1u)
        {
            Nag_PathFix_DrawPoint((uint16)map_x[draw_count - 1u], (uint16)map_y[draw_count - 1u],
                                  x_offset, y_offset, width, height, RGB565_PURPLE);
        }
    }

    for (i = 0u; i < s_pathfix_elem_count; i++)
    {
        Nag_PathFix_DrawCrossMarker((uint16)s_pathfix_elem_x[i],
                                    (uint16)s_pathfix_elem_y[i],
                                    x_offset, y_offset, width, height,
                                    RGB565_BLUE, 3u);
    }

    if ((draw_count > 0u) && (sel_draw < draw_count))
    {
        Nag_PathFix_DrawSelectedMarker((uint16)map_x[sel_draw], (uint16)map_y[sel_draw],
                                       x_offset, y_offset, width, height);
    }
}

#if defined(CY_CORE_CM7_0)
void Nag_PathFix_SyncToShared(void *ctrl_snapshot,
                              uint16 x_offset,
                              uint16 y_offset,
                              uint16 width,
                              uint16 height)
{
    dualcore_ctrl_to_ui_t *c = (dualcore_ctrl_to_ui_t *)ctrl_snapshot;

    if (c == NULL)
    {
        return;
    }

    c->pathfix_active = g_nag_pathfix.active;
    c->pathfix_loaded = g_nag_pathfix.loaded;
    c->pathfix_dirty = g_nag_pathfix.dirty;
    c->pathfix_replay_subject = g_nag_pathfix.replay_subject;
    c->pathfix_select_index = g_nag_pathfix.select_index;
    c->pathfix_point_count = (uint32)g_nag_pathfix.point_count;

    if ((g_nag_pathfix.loaded != 0u) &&
        (g_nag_pathfix.select_index < g_nag_pathfix.point_count))
    {
        c->pathfix_select_yaw_x100 = Nav_read[g_nag_pathfix.select_index];
    }
    else
    {
        c->pathfix_select_yaw_x100 = 0;
    }

    Nag_PathFix_PublishDrawMap(x_offset, y_offset, width, height,
                               c->pathfix_draw_x, c->pathfix_draw_y,
                               DUALCORE_PATHFIX_DRAW_MAX,
                               &c->pathfix_draw_count,
                               &c->pathfix_draw_sel_idx);

    c->pathfix_elem_count = 0u;
    if (s_pathfix_elem_count > 0u)
    {
        uint16 copy_n = s_pathfix_elem_count;
        uint16 ei = 0u;

        if (copy_n > DUALCORE_PATHFIX_ELEM_MAX)
        {
            copy_n = DUALCORE_PATHFIX_ELEM_MAX;
        }
        for (ei = 0u; ei < copy_n; ei++)
        {
            c->pathfix_elem_x[ei] = s_pathfix_elem_x[ei];
            c->pathfix_elem_y[ei] = s_pathfix_elem_y[ei];
        }
        c->pathfix_elem_count = (uint8)copy_n;
    }
}
#endif

uint8 Nag_PathFix_Enter(void)
{
    uint16 save_index = 0u;
    uint8 replay_subject = 0u;

    if (g_nag_pathfix.active != 0u)
    {
        return 1u;
    }

    if (Nag_PathFix_CanEnter() == 0u)
    {
        s_pathfix_cache_valid = 0u;
        memset(&g_nag_pathfix, 0, sizeof(g_nag_pathfix));
        return 0u;
    }

    replay_subject = Nag_ClampSubject(g_nag_replay_subject);
    s_pathfix_cache_valid = 0u;
    memset(&g_nag_pathfix, 0, sizeof(g_nag_pathfix));
    flash_Nag_BindSubjectSlot(replay_subject);
    if (flash_Nag_LoadTrajectoryOnly(&save_index) == 0u)
    {
        g_nag_pathfix.replay_subject = replay_subject;
        return 0u;
    }

    g_nag_pathfix.active = 1u;
    g_nag_pathfix.loaded = 1u;
    g_nag_pathfix.replay_subject = replay_subject;
    g_nag_pathfix.point_count = save_index;
    g_nag_pathfix.select_index = 0u;
    g_nag_pathfix.map_dirty = 1u;
    s_pathfix_cache_valid = 0u;
    return 1u;
}

void Nag_PathFix_Leave(uint8 save_if_dirty)
{
    if ((save_if_dirty != 0u) && (g_nag_pathfix.dirty != 0u) && (g_nag_pathfix.loaded != 0u))
    {
        flash_Nag_BindSubjectSlot(g_nag_pathfix.replay_subject);
        (void)flash_Nag_WriteFullPath(g_nag_pathfix.point_count);
    }
    s_pathfix_cache_valid = 0u;
    s_pathfix_cache_count = 0u;
    s_pathfix_elem_count = 0u;
    memset(&g_nag_pathfix, 0, sizeof(g_nag_pathfix));
}

uint8 Nag_PathFix_CycleSelect(void)
{
    if ((g_nag_pathfix.active == 0u) || (g_nag_pathfix.loaded == 0u) ||
        (g_nag_pathfix.point_count == 0u))
    {
        return 0u;
    }

    g_nag_pathfix.select_index =
        (uint16)((g_nag_pathfix.select_index + Nag_PathFix_Select_Step) % g_nag_pathfix.point_count);
    /* 选点不改几何，仅刷新高亮索引 */
    if (s_pathfix_cache_valid != 0u)
    {
        s_pathfix_cache_sel = Nag_PathFix_SrcToDrawIdx(g_nag_pathfix.select_index,
                                                        s_pathfix_cache_count,
                                                        g_nag_pathfix.point_count);
    }
    return 1u;
}

/* 两航向最短弧插值，t∈[0,1] */
static float Nag_PathFix_LerpYawDeg(float yaw_a_deg, float yaw_b_deg, float t)
{
    float diff = yaw_b_deg - yaw_a_deg;

    while (diff > 180.0f)
    {
        diff -= 360.0f;
    }
    while (diff < -180.0f)
    {
        diff += 360.0f;
    }
    return Nag_PathFix_WrapYawDeg(yaw_a_deg + diff * t);
}

/* 用两端点当前 yaw 线性填充 (i0, i1) 之间的中间点，使整段斜线一起变向 */
static void Nag_PathFix_LerpSegmentYaw(uint16 i0, uint16 i1)
{
    float y0 = 0.0f;
    float y1 = 0.0f;
    uint16 i = 0u;
    uint16 span = 0u;

    if ((i1 <= i0) || (i1 >= g_nag_pathfix.point_count))
    {
        return;
    }

    y0 = (float)Nav_read[i0] / 100.0f;
    y1 = (float)Nav_read[i1] / 100.0f;
    span = (uint16)(i1 - i0);
    for (i = (uint16)(i0 + 1u); i < i1; i++)
    {
        float t = (float)(i - i0) / (float)span;
        Nav_read[i] = Nag_PathFix_YawToStorage(Nag_PathFix_LerpYawDeg(y0, y1, t));
    }
}

/*
 * KEY2/3：改当前选中点 yaw，并按 Select_Step 锚点网格重插相邻段。
 * 例：先改锚点 A、再改锚点 B，则 [A,B] 整段被拉成新斜线，而不是只动两个端点。
 */
uint8 Nag_PathFix_AdjustYaw(float delta_deg)
{
    uint16 idx = 0;
    uint16 step = Nag_PathFix_Select_Step;
    uint16 left = 0u;
    uint16 right = 0u;
    float yaw_deg = 0.0f;

    if ((g_nag_pathfix.active == 0u) || (g_nag_pathfix.loaded == 0u) ||
        (g_nag_pathfix.point_count == 0u))
    {
        return 0u;
    }

    idx = g_nag_pathfix.select_index;
    if (idx >= g_nag_pathfix.point_count)
    {
        return 0u;
    }

    if (step == 0u)
    {
        step = 1u;
    }

    yaw_deg = (float)Nav_read[idx] / 100.0f + delta_deg;
    Nav_read[idx] = Nag_PathFix_YawToStorage(yaw_deg);

    left = (uint16)((idx / step) * step);
    right = (uint16)(left + step);
    if (right >= g_nag_pathfix.point_count)
    {
        right = (uint16)(g_nag_pathfix.point_count - 1u);
    }

    if (idx == left)
    {
        /* 落在锚点：重插前一段 + 后一段 */
        if (left >= step)
        {
            Nag_PathFix_LerpSegmentYaw((uint16)(left - step), left);
        }
        if (right > left)
        {
            Nag_PathFix_LerpSegmentYaw(left, right);
        }
    }
    else
    {
        /* 落在段内：以当前点为折点，分别插值到左右锚点 */
        Nag_PathFix_LerpSegmentYaw(left, idx);
        if (right > idx)
        {
            Nag_PathFix_LerpSegmentYaw(idx, right);
        }
    }

    g_nag_pathfix.dirty = 1u;
    g_nag_pathfix.map_dirty = 1u;
    return 1u;
}

uint8 Nag_PathFix_ExitSave(void)
{
    if (g_nag_pathfix.active == 0u)
    {
        return 0u;
    }

    if ((g_nag_pathfix.dirty != 0u) && (g_nag_pathfix.loaded != 0u))
    {
        flash_Nag_BindSubjectSlot(g_nag_pathfix.replay_subject);
        if (flash_Nag_WriteFullPath(g_nag_pathfix.point_count) == 0u)
        {
            return 0u;
        }
    }

    s_pathfix_cache_valid = 0u;
    s_pathfix_cache_count = 0u;
    s_pathfix_elem_count = 0u;
    memset(&g_nag_pathfix, 0, sizeof(g_nag_pathfix));
    return 1u;
}
