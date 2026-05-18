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
#include "my_gps.h"

int32 Nav_read[Read_MaxSize];//每5cm的点，1000个点50m
Nag N;
uint8 nav_heading_mode = NAV_HEADING_MODE_INS;
NagEvent Nag_Event_Table[Nag_Event_Max];
uint8 Nag_Vofa_Group = 0;

static bool Nag_GetHeadingHoldConfig(uint8 event_type)
{
    switch (event_type)
    {
        case NAG_EVENT_TYPE_SPIN: return (Nag_HeadingHold_Spin_Enable != 0u);
        case NAG_EVENT_TYPE_TURNAROUND: return (Nag_HeadingHold_Turnaround_Enable != 0u);
        case NAG_EVENT_TYPE_SINGLE_BRIDGE: return (Nag_HeadingHold_SingleBridge_Enable != 0u);
        case NAG_EVENT_TYPE_BUMP: return (Nag_HeadingHold_Bump_Enable != 0u);
        case NAG_EVENT_TYPE_JUMP: return (Nag_HeadingHold_Jump_Enable != 0u);
        default: return false;
    }
}

static void Nag_HeadingHold_Enable(float target_yaw)
{
    N.HeadingHold_Target_Yaw = target_yaw;
    N.HeadingHold_Target_Latched = 1u;
    N.HeadingHold_Enable = 1u;
    /* 这里不直接反复调用 steer_set_target_yaw()。
     * 统一通过 ISR 前置登记请求，再走现有 pending -> consume 链路，
     * 可以继续复用 spin_enable 的互斥保护，避免普通转向与自旋直接抢控制权。
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

    /* 当前先统一锁定“进入元素瞬间的实测 yaw”，这样元素里即使暂停导航前瞻推进，
     * 也仍能把车头稳在切入该元素前的方向。
     */
    Nag_HeadingHold_Enable((float)euler_angle.yaw);
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

/* 折返：占位不接 spin；与单边桥同属“未接入”模板，回放将停在 ENTERED。 */
bool Nag_Hook_Turnaround_Start(void) { return false; }
void Nag_Hook_Turnaround_Run(void) {}
bool Nag_Hook_Turnaround_IsDone(void) { return false; }
void Nag_Hook_Turnaround_Stop(void) {}

/* 自转元素接法：
 * 1. Start：先接管全局速度档位，把 motor_user_speed_cmd 清零，给车一个“先刹停”的阶段；
 * 2. Run：只有当前速度连续多拍低于阈值后，才真正启动 spin_task_start()；
 * 3. IsDone：只有自旋真正启动后才读取 spin_done，避免等待阶段误判完成；
 * 4. Stop：无论正常结束还是异常中止，都统一恢复被接管前的速度档位。
 */
bool Nag_Hook_Spin_Start(void)
{
    N.Spin_Saved_SetSpeed = motor_user_speed_cmd;
    N.Spin_Stop_Stable_Count = 0;
    N.Spin_Task_Started = 0;
    N.Spin_Speed_Latched = 1;
    motor_user_speed_cmd = 0.0f;
    return true;
}
void Nag_Hook_Spin_Run(void)
{
    /* 元素接管后导航里程更新会暂停，因此这里直接读取实时速度源做停稳判定。 */
    float abs_speed = fabsf((float)Nag_Speed_Source);

    if (N.Spin_Task_Started)
    {
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
    spin_task_start(Nag_Spin_Demo_Turns, Nag_Spin_Demo_Dir);
    N.Spin_Task_Started = 1;
}
bool Nag_Hook_Spin_IsDone(void)
{
    return (N.Spin_Task_Started && (spin_done != 0));
}
void Nag_Hook_Spin_Stop(void)
{
    if (N.Spin_Task_Started)
    {
        spin_task_stop();
    }
    Nag_Spin_RestoreSetSpeed();
}

bool Nag_Hook_SingleBridge_Start(void) { return false; }
void Nag_Hook_SingleBridge_Run(void) {}
bool Nag_Hook_SingleBridge_IsDone(void) { return false; }
void Nag_Hook_SingleBridge_Stop(void) {}

bool Nag_Hook_Bump_Start(void) { return false; }
void Nag_Hook_Bump_Run(void) {}
bool Nag_Hook_Bump_IsDone(void) { return false; }
void Nag_Hook_Bump_Stop(void) {}

/* 锥桶进/出口：与事件表 enter/exit 对齐的路径标记；不配预减速/锁航（见 navigation.h）。
 * Start 立刻 true，首个 RUNNING 周期 IsDone 即 true，减少 Event_Active 窗口、尽快恢复惯导前瞻。
 */
bool Nag_Hook_EnterCones_Start(void) { return true; }
void Nag_Hook_EnterCones_Run(void) {}
bool Nag_Hook_EnterCones_IsDone(void) { return true; }
void Nag_Hook_EnterCones_Stop(void) {}

bool Nag_Hook_ExitCones_Start(void) { return true; }
void Nag_Hook_ExitCones_Run(void) {}
bool Nag_Hook_ExitCones_IsDone(void) { return true; }
void Nag_Hook_ExitCones_Stop(void) {}

/* 跳跃元素：与串口调试 'i' 相同，置 jump_flag=1，由 control.c 的 jump_control() 在 ISR 内推进并在结束时清零。
 * Jump_Element_Armed 防止 IsDone 在 Start 前因 jump_flag 初值为 0 而误判完成。
 */
bool Nag_Hook_Jump_Start(void)
{
    if (jump_flag != 0u || jump_is_allowed() == 0u)
    {
        return false;
    }
    N.Jump_Element_Armed = 1u;
    jump_flag = 1u;
    return true;
}
void Nag_Hook_Jump_Run(void) {}
bool Nag_Hook_Jump_IsDone(void)
{
    return (N.Jump_Element_Armed != 0u) && (jump_flag == 0u);
}
void Nag_Hook_Jump_Stop(void)
{
    jump_stop();
    N.Jump_Element_Armed = 0u;
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
        case NAG_EVENT_TYPE_TURNAROUND: return Nag_Hook_Turnaround_Start();
        case NAG_EVENT_TYPE_SPIN: return Nag_Hook_Spin_Start();
        case NAG_EVENT_TYPE_ENTER_CONES: return Nag_Hook_EnterCones_Start();
        case NAG_EVENT_TYPE_EXIT_CONES: return Nag_Hook_ExitCones_Start();
        case NAG_EVENT_TYPE_SINGLE_BRIDGE: return Nag_Hook_SingleBridge_Start();
        case NAG_EVENT_TYPE_BUMP: return Nag_Hook_Bump_Start();
        case NAG_EVENT_TYPE_JUMP: return Nag_Hook_Jump_Start();
        default: return false;
    }
}

void Nag_Element_Run(uint8 event_type)
{
    /* 统一 Run 分发：元素处于 RUNNING 态时，每拍都会调到这里。 */
    switch (event_type)
    {
        case NAG_EVENT_TYPE_TURNAROUND: Nag_Hook_Turnaround_Run(); break;
        case NAG_EVENT_TYPE_SPIN: Nag_Hook_Spin_Run(); break;
        case NAG_EVENT_TYPE_ENTER_CONES: Nag_Hook_EnterCones_Run(); break;
        case NAG_EVENT_TYPE_EXIT_CONES: Nag_Hook_ExitCones_Run(); break;
        case NAG_EVENT_TYPE_SINGLE_BRIDGE: Nag_Hook_SingleBridge_Run(); break;
        case NAG_EVENT_TYPE_BUMP: Nag_Hook_Bump_Run(); break;
        case NAG_EVENT_TYPE_JUMP: Nag_Hook_Jump_Run(); break;
        default: break;
    }
}

bool Nag_Element_IsDone(uint8 event_type)
{
    /* 统一完成判定：
     * 建议由你自己的元素逻辑在合适时机置位完成条件，
     * 状态机一旦检测到 true，就会自动从 exit_index 接回导航。
     */
    switch (event_type)
    {
        case NAG_EVENT_TYPE_TURNAROUND: return Nag_Hook_Turnaround_IsDone();
        case NAG_EVENT_TYPE_SPIN: return Nag_Hook_Spin_IsDone();
        case NAG_EVENT_TYPE_ENTER_CONES: return Nag_Hook_EnterCones_IsDone();
        case NAG_EVENT_TYPE_EXIT_CONES: return Nag_Hook_ExitCones_IsDone();
        case NAG_EVENT_TYPE_SINGLE_BRIDGE: return Nag_Hook_SingleBridge_IsDone();
        case NAG_EVENT_TYPE_BUMP: return Nag_Hook_Bump_IsDone();
        case NAG_EVENT_TYPE_JUMP: return Nag_Hook_Jump_IsDone();
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
        case NAG_EVENT_TYPE_TURNAROUND: Nag_Hook_Turnaround_Stop(); break;
        case NAG_EVENT_TYPE_SPIN: Nag_Hook_Spin_Stop(); break;
        case NAG_EVENT_TYPE_ENTER_CONES: Nag_Hook_EnterCones_Stop(); break;
        case NAG_EVENT_TYPE_EXIT_CONES: Nag_Hook_ExitCones_Stop(); break;
        case NAG_EVENT_TYPE_SINGLE_BRIDGE: Nag_Hook_SingleBridge_Stop(); break;
        case NAG_EVENT_TYPE_BUMP: Nag_Hook_Bump_Stop(); break;
        case NAG_EVENT_TYPE_JUMP: Nag_Hook_Jump_Stop(); break;
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
    N.Spin_Saved_SetSpeed = 0.0f;
    N.Spin_Stop_Stable_Count = 0;
    N.Spin_Task_Started = 0;
    N.Spin_Speed_Latched = 0;
    if (N.Jump_Element_Armed != 0u)
    {
        jump_flag = 0u;
    }
    N.Jump_Element_Armed = 0u;
}

static void Nag_Element_StateMachine(void)
{
    /* 回放中 Event_Active=1 时每 1ms 由 Nag_System() 调用。
     * ENTERED：首开 Event_Start_Latched 后调 Nag_Element_Start；true→RUNNING。
     * RUNNING：Nag_Element_Run + IsDone；true→DONE。
     * DONE：Nag_Notify_Event_Done() 将 Run_index 置 exit 并清 Active。
     */
    uint8 event_index = N.Event_Active_Index;
    uint8 event_type = N.Event_Active_Type;

    if (!N.Event_Active || event_index >= N.Event_Count)
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
            Nag_Event_Table[event_index].enter_index == run_index)
        {
            return event_index;
        }
    }
    return 0xFFu;
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

        if (!Nag_Event_Table[event_index].valid)
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

static bool Nag_GetPreDecelConfig(uint8 event_type, uint16 *pre_points, float *min_speed)
{
    if (pre_points == NULL || min_speed == NULL)
    {
        return false;
    }

    switch (event_type)
    {
        case NAG_EVENT_TYPE_SPIN:
            *pre_points = Nag_Spin_PreDecel_Points;
            *min_speed = Nag_Spin_PreDecel_MinSpeed;
            break;
        case NAG_EVENT_TYPE_TURNAROUND:
            *pre_points = Nag_Turnaround_PreDecel_Points;
            *min_speed = Nag_Turnaround_PreDecel_MinSpeed;
            break;
        case NAG_EVENT_TYPE_SINGLE_BRIDGE:
            *pre_points = Nag_SingleBridge_PreDecel_Points;
            *min_speed = Nag_SingleBridge_PreDecel_MinSpeed;
            break;
        case NAG_EVENT_TYPE_BUMP:
            *pre_points = Nag_Bump_PreDecel_Points;
            *min_speed = Nag_Bump_PreDecel_MinSpeed;
            break;
        case NAG_EVENT_TYPE_JUMP:
            *pre_points = Nag_Jump_PreDecel_Points;
            *min_speed = Nag_Jump_PreDecel_MinSpeed;
            break;
        default:
            *pre_points = 0u;
            *min_speed = 0.0f;
            break;
    }

    return (*pre_points > 0u);
}

/* 元素前预减速：
 * 1. 仅在尚未真正切入元素前生效；一旦 N.Event_Active=1，就交回现有元素内限速逻辑；
 * 2. 距离元素越近，输出速度越接近该元素配置的 min_speed；
 * 3. 当前采用线性斜坡：dist=K 时不减速，dist=0 时压到 min_speed。
 */
static float Nag_ApplyPreEventDecel(float nav_speed)
{
#if !Nag_PreEventDecel_Enable
    return nav_speed;
#else
    uint8 event_index = 0xFFu;
    uint16 dist_points = 0xFFFFu;
    uint16 pre_points = 0u;
    float min_speed = 0.0f;
    float ratio = 1.0f;
    float decel_speed = nav_speed;

    if (nav_speed <= 0.0f || N.Event_Active || N.Event_Count == 0u)
    {
        return nav_speed;
    }

    event_index = Nag_FindNextEventAhead(N.Run_index, &dist_points);
    if (event_index == 0xFFu || event_index >= N.Event_Count)
    {
        return nav_speed;
    }

    if (!Nag_GetPreDecelConfig(Nag_Event_Table[event_index].type, &pre_points, &min_speed))
    {
        return nav_speed;
    }

    if (dist_points > pre_points)
    {
        return nav_speed;
    }

    if (min_speed < 0.0f)
    {
        min_speed = 0.0f;
    }
    if (min_speed > nav_speed)
    {
        min_speed = nav_speed;
    }

    ratio = (float)dist_points / (float)pre_points;
    decel_speed = min_speed + (nav_speed - min_speed) * ratio;

    if (decel_speed < min_speed)
    {
        decel_speed = min_speed;
    }
    if (decel_speed > nav_speed)
    {
        decel_speed = nav_speed;
    }
    return decel_speed;
#endif
}

static void Nag_TryEnterEvent(void)
{
    /* Run_Nag_GPS() 在里程推进到某条事件的 enter_index 时切入：置 Event_Active、
     * 清 pending、锥桶标记类不 steer_task_stop() 以免打断沿路惯导转向。
     */
    uint8 event_index = 0;

    if (N.Event_Active)
    {
        return;
    }

    event_index = Nag_FindEventByEnterIndex(N.Run_index);
    if (event_index == 0xFFu)
    {
        return;
    }

    N.Event_Active = 1;
    N.Event_Active_Index = event_index;
    N.Active_Event_Enter = Nag_Event_Table[event_index].enter_index;
    N.Active_Event_Exit = Nag_Event_Table[event_index].exit_index;
    N.Event_Active_Type = Nag_Event_Table[event_index].type;
    N.Event_Start_RunIndex = N.Run_index;
    N.Event_State = NAG_EVENT_STATE_ENTERED;
    N.Event_Start_Latched = 0;
    N.Event_Done_Latched = 0;
    N.Target_Request_Valid = 0;
    steer_yaw_request_pending = 0;
    steer_yaw_delayed_by_spin = 0;
    if (N.Event_Active_Type != NAG_EVENT_TYPE_ENTER_CONES &&
        N.Event_Active_Type != NAG_EVENT_TYPE_EXIT_CONES)
    {
        steer_task_stop();
    }
    Nag_HeadingHold_OnEventEnter(N.Event_Active_Type);
}

static float Nag_GetMileageStep(void)
{
    float speed_forward = (float)Nag_Speed_Source;

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

uint16 Nag_GetDebugProspectIndex(void)
{
    return N.Prospect_index;
}

/* 速度目标合成（由 pid_ctrl_Run 每 20ms 读取一次）：
 * - motor_user_speed_cmd：用户层基准（串口 V、菜单/遥控/双核命令、串口 q/r/s 等），正号前进、负号后退；
 * - N.Target_Speed：导航前瞻 + 弯道强度算出的“建议上限”，再与用户基准取 MIN/比例；
 * - 本函数在非回放执行态（Nag_SystemRun_Index!=3）强制返回 0，避免待机误跑。
 */
float Nag_GetControlSpeedTarget(void)
{
    float nav_speed = N.Target_Speed;
    float abs_user_speed = fabsf((float)motor_user_speed_cmd);

    /* GPS 点导航由 GPS_PointNav_Run() 自己负责保护停车和到点停车。
     * 这里直接放行 motor_user_speed_cmd，避免 GPS 模式被惯导回放态或 N.Nag_Stop_f 误门控。
     */
    if (nav_heading_mode == NAV_HEADING_MODE_GPS)
    {
        if (gps_nav_state == GPS_NAV_STATE_FINISHED || gps_nav_state == GPS_NAV_STATE_PROTECT)
        {
            return 0.0f;
        }
        return motor_user_speed_cmd;
    }

    /* 安全门控：正常模式下只有导航真正进入回放执行态(case 3)后，速度目标才允许生效。
     * 这样用户设定的 motor_user_speed_cmd 在上电、待机、录制、以及回放准备阶段
     * （Nag_SystemRun_Index==2, 仍在读 flash）时不会直接驱动速度环（本函数返回 0）。
     * 调试时若发现 motor_user_speed_cmd 非零但车还没动，优先看两件事：
     * 1. N.Nag_SystemRun_Index 是否已经到 3；
     * 2. Motor_Switch 是否为 ON（由 SWITCH1 决定，失控锁存时强制关）。
     */
    if (N.Nag_Stop_f)
    {
        return 0.0f;
    }

#if Nag_Debug_Speed_Bypass_Enable
    /* PID 调试旁路：
     * 1. 旁路只保留终点停车保护，允许非回放态也直接输出固定速度目标；
     * 2. 目标直接跟随 motor_user_speed_cmd，便于直道阶跃调试速度环；
     * 3. 正式跑导航前请把 Nag_Debug_Speed_Bypass_Enable 改回 0。
     */
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
        /* JUMP：冲击不限速；锥桶标记：瞬时元素窗口内不按 Nag_Event_Speed_Ratio 压车速 */
        float ratio =
            (N.Event_Active_Type == NAG_EVENT_TYPE_JUMP ||
             N.Event_Active_Type == NAG_EVENT_TYPE_ENTER_CONES ||
             N.Event_Active_Type == NAG_EVENT_TYPE_EXIT_CONES)
                ? 1.0f
                : Nag_Event_Speed_Ratio;
        nav_speed = MIN(nav_speed, abs_user_speed * ratio);
    }
    else
    {
        nav_speed = Nag_ApplyPreEventDecel(nav_speed);
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
     * 3. 如果跑到元素 enter_index，则冻结导航索引推进，把控制权让给元素逻辑；
     * 4. 元素完成后通过 Nag_Notify_Event_Done() 从 exit_index 接回。
     */
    Run_Nag_GPS();  //偏航角读取函数
    if(N.Nag_Stop_f) //终点停止
    {
        N.Final_Out = 0;
        N.Target_Request_Valid = 0;
        steer_yaw_request_pending = 0;
        steer_yaw_delayed_by_spin = 0;
        steer_task_stop();
        return;
    }

    if (N.Event_Active)
    {
        /* 元素期间导航不再用前瞻点刷新目标航向；
         * 若该元素配置了“保持航向”，则改由 1ms ISR 在 steer_yaw_request_pending 消费前，
         * 按锁存的 HeadingHold_Target_Yaw 重新登记请求。
         */
        N.Final_Out = 0.0f;
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

    while(N.Mileage_All >= Nag_Set_mileage)    //当里程超过设定值时
    {
       if(N.size >= MaxSize)//当数组超过一页的flash大小时，写入一次，防止重复写入
       {
           flash_Nag_Write();
           N.size=0;   //将数组大小为0，下一次从新开始取
           N.Flash_page_index--;   //flash页索引减小
           zf_assert(N.Flash_page_index > Nag_End_Page);//防止越界保护
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
        N.Angle_Run = (float)(Nav_read[N.Prospect_index] / 100.0f);
        return;
    }

    N.Mileage_All += Nag_GetMileageStep();
    N.Mileage_Debug_Total += N.Mileage_Step;
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

    Nag_TryEnterEvent();
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
    memset(Nag_Event_Table, 0, sizeof(Nag_Event_Table));
    N.Flash_page_index=Nag_Start_Page;
    N.Event_Active_Index = 0xFFu;
    N.Event_Record_Type = NAG_EVENT_TYPE_SPIN;
    flash_Nag_ResetReadState();
    flash_buffer_clear();
}

void Nag_Begin_Record(void)
{
    Init_Nag();
    steer_yaw_request_pending = 0;
    steer_yaw_delayed_by_spin = 0;
    steer_task_stop();
    N.Nag_SystemRun_Index = 1;
}

/* 进入回放准备态：索引置 2，待 NagFlashRead() 读完 flash 后进入 3，才装载 run_launch_speed 并放行速度环。 */
void Nag_Begin_Replay(void)
{
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
    N.Flash_page_index = Nag_Start_Page;
    N.Requested_Target_Yaw = 0;
    N.Target_Request_Valid = 0;
    Nag_ClearEventRuntimeState();
    steer_yaw_request_pending = 0;
    steer_yaw_delayed_by_spin = 0;
    steer_task_stop();
    flash_Nag_ResetReadState();
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

    if (!N.Event_Record_Pending)
    {
        Nag_Event_Table[N.Event_Count].enter_index = N.Save_index;
        Nag_Event_Table[N.Event_Count].exit_index = N.Save_index;
        Nag_Event_Table[N.Event_Count].type = N.Event_Record_Type;
        Nag_Event_Table[N.Event_Count].valid = 0;
        N.Event_Record_Pending = 1;
    }
    else
    {
        Nag_Event_Table[N.Event_Count].exit_index = N.Save_index;
        if (Nag_Event_Table[N.Event_Count].exit_index < Nag_Event_Table[N.Event_Count].enter_index)
        {
            Nag_Event_Table[N.Event_Count].exit_index = Nag_Event_Table[N.Event_Count].enter_index;
        }
        Nag_Event_Table[N.Event_Count].valid = 1;
        N.Event_Count++;
        N.Event_Record_Pending = 0;
    }
}

void Nag_Cycle_Record_Event_Type(void)
{
    N.Event_Record_Type = (uint8)((N.Event_Record_Type + 1u) % NAG_EVENT_TYPE_COUNT);
}

void Nag_Notify_Event_Done(void)
{
    uint8 event_index = N.Event_Active_Index;

    if (!N.Event_Active || event_index >= N.Event_Count)
    {
        return;
    }

    N.Run_index = Nag_Event_Table[event_index].exit_index;
    N.Mileage_All = 0.0f;
    N.Target_Request_Valid = 0;
    Nag_Element_Stop(N.Event_Active_Type);
    Nag_ClearEventRuntimeState();
    Nag_UpdatePreviewAndSpeedTarget();
    N.Angle_Run = (float)(Nav_read[N.Prospect_index] / 100.0f);
}

void Nag_Element_Abort(void)
{
    if (!N.Event_Active)
    {
        return;
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
    //偏航角
    if(!N.Nag_SystemRun_Index || N.Nag_Stop_f )  return;

    if (N.Event_Active)
    {
        /* 元素接管期间不再直接 return，而是进入统一元素状态机：
         * 1. ENTERED：刚切入元素，等待 Start 钩子真正接管；
         * 2. RUNNING：周期执行 Run 钩子，并检测完成标志；
         * 3. DONE：自动从 exit_index 恢复导航；
         * 4. 若当前元素尚未补具体动作，默认空钩子会停留在 ENTERED，便于继续用 KEY4 / 串口 v 手动恢复。
         */
        Nag_Element_StateMachine();
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
  N.Flash_page_index = Nag_Start_Page;
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
  N.Run_index = 0;
  N.Prospect_index = 0;
  N.Nag_Stop_f = 0;
  N.Curve_Strength = 0;
  if (N.Nag_SystemRun_Index == 2u)
  {
    /* 发车设定值只在惯导回放真正进入执行态前装载，避免待机/读 flash 阶段改动实际速度基准。 */
    motor_user_speed_cmd = run_launch_speed;
  }
  N.Target_Speed = fabsf((float)motor_user_speed_cmd);
  N.Angle_Run = (N.Save_index > 0) ? (float)(Nav_read[0] / 100.0f) : (float)Nag_Yaw;
  N.Requested_Target_Yaw = 0;
  N.Target_Request_Valid = 0;
  Nag_ClearEventRuntimeState();
  Nag_UpdatePreviewAndSpeedTarget();
  N.Angle_Run = (N.Save_index > 0) ? (float)(Nav_read[N.Prospect_index] / 100.0f) : (float)Nag_Yaw;
  N.Nag_SystemRun_Index++;
}
