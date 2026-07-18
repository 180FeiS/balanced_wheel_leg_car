/*
 * nagivation.h
 *
 *  Created on: 2024年10月16日
 *      Author: Monst
 */

#ifndef _NAVIGATION_H_
#define _NAVIGATION_H_


//*********************用户宏定义****************************//
#define MaxSize 500    //flash存储数组页数

#define Read_MaxSize 10000//导航读取数组，1w个应该是够了

//存储范围 <0 - 47>
#define Nag_End_Page 1      //flash结束页数
#define Nag_Start_Page 45   //flash开始页数

/* 速度积分式里程计参数：
 * 1. Nag_Set_mileage 表示每隔多少“距离单位”记录/回放一次 yaw，当前按 cm 理解；
 * 2. Nag_Speed_Source 为当前选用的前向速度源，先继续使用 car_speed（左右轮 RPM 平均）；
 * 3. Nag_Speed_To_Mileage_Scale = 2π·WHEEL_RADIUS_CM/60，将 RPM 换算为 cm/s；换轮改 WHEEL_RADIUS_CM；
 * 4. Nag_Sample_Dt 必须与 Nag_System() 的真实调用周期严格一致。
 *    当前 Nag_System() 固定在 pit0_ch0_isr 的 1ms 中断里跑，因此这里必须是 0.001s。
 *    若 car_speed 非 RPM 或有减速比，改半径后仍建议卷尺短距离实测微调。
 */
#define WHEEL_RADIUS_CM 4.325f            //驱动轮半径（cm）；直径 8.65cm 卡尺值，实车可卷尺短距微调
#define Nag_Set_mileage 2.0f              //每隔 2cm 记录一次 yaw
#define Nag_Prev 200                      //保留的历史/预读缓存长度
#define Nag_Yaw euler_angle.yaw           //航向角度取偏航角
#define Nag_Sample_Dt 0.001f              //Nag_System 当前固定 1ms 运行一次
#define Nag_Speed_Source car_speed        //默认优先使用车体平均速度
#define Nag_Speed_To_Mileage_Scale (2.0f * 3.1415926f * WHEEL_RADIUS_CM / 60.0f)  //≈0.453，RPM→cm/s
#define Nag_Speed_Deadband 3.0f           //速度死区，抑制静止噪声
#define Nag_Reissue_Error 0.2f            //转向收敛后若再次偏离该角度，则重新下发目标 yaw

/*
 * 纯惯导里程纠偏（打滑/单轮空转）：
 * 1. 仅在 Nag_GetMileageStep() 走 car_speed 积分路径时启用；融合位移有效时不介入；
 * 2. 策略：瞬时保护 -> 状态确认 -> 回溯纠偏；不处理台阶开环跳跃腾空；
 * 3. 判据：左右轮速度 + gyro_z 重建中心速度一致性；轮距为左右驱动轮中心横向间距（cm）。
 * 编译期门控：Nag_OdoSlip_Enable=0 时不链接算法；运行时开关见 g_menu_odo_slip_enable（Run→Config OdoSlipEn，Flash V13）。
 * 调试：VOFA 组 3 观察 odo_slip_state；关闭时状态恒为 0。
 */
#define Nag_OdoSlip_Enable 1u

/* 轮距（cm）：左右驱动轮滚动中心线间距；宽胎以轮中面测量，外八/内八明显时用圆弧实车微调 */
#define Nag_Wheel_Track_Cm 16.0f

/*
 * 左右反推中心速度一致性阈值（cm/s）。
 * 调大：更少触发保护/打滑，但短时空转可能漏检；调小：更敏感，正常弯道易误报。
 */
#define Nag_OdoSlip_Consistency_Th_Cmps 60.0f

/* 单拍瞬时保护：候选中心速度相对上一拍可信速度的最大允许跳变（cm/s） */
#define Nag_OdoSlip_Instant_Slew_Max_Cmps 120.0f

/* 进入/退出打滑态连续计数（1ms/次）；调大进入更稳、退出更慢 */
#define Nag_OdoSlip_Enter_Count 10u
#define Nag_OdoSlip_Exit_Count 25u

/* 短历史窗口（ms），用于确认后回溯扣账 */
#define Nag_OdoSlip_History_Len 15u

/* 单次回溯补扣上限（cm），防止异常数据一次拉回过多 Run_index */
#define Nag_OdoSlip_Rollback_Max_Cm 6.0f

/*
 * 惯导回放融合里程辅助（nav_fusion）：
 * 1=Run_index 推进优先用融合位移(cm)，长距离/元素停车后漂移更小；0=纯 car_speed 积分。
 * 录制/回放 KEY 流程不变；融合模式下先 GPS 原点平均，再锁定发车 yaw 直行 5m 标定北向偏角。
 * 纯惯导（g_menu_nav_fusion_enable=0 或 NAG_USE_FUSION_MILEAGE=0）不受影响。
 */
#define NAG_USE_FUSION_MILEAGE 1u

/* 速度调试旁路：
 * 1. 置 1 后，即使导航未进入回放态，也允许速度环直接使用 motor_user_speed_cmd；
 * 2. 仅用于直道阶跃调 PID，比赛/正式回放前务必改回 0；
 * 3. 打开后 Nag_GetControlSpeedTarget() 会绕过 Nag_SystemRun_Index==3 的门控。
 */
#define Nag_Debug_Speed_Bypass_Enable 0u

/* 惯导录制态遥控速度（Nag_GetControlSpeedTarget 内联门控，非全局旁路）：
 * 1. 需 MENU_INPUT_REMOTE_MENU_FIRST==1、Nag_SystemRun_Index==1 且 End_f==0、LORA 在线（remote_lora_steer_snapshot_valid）；
 * 2. 速度环目标直接跟随遥控 left_y 映射的 motor_user_speed_cmd，掉线立即停车；
 * 3. 与 Nag_Debug_Speed_Bypass_Enable 独立，正式录制可用、调试旁路仍保持关闭。
 */

#define Nag_AdaptiveLookahead_Enable 1u  /* 0=关闭下列速度自适应前瞻与弯道限速；1=启用 Nag_Lookahead_* / Nag_Curve_* */

/* 速度自适应前瞻参数（仅当 Nag_AdaptiveLookahead_Enable==1 时参与 Nag_UpdatePreviewAndSpeedTarget 计算）：
 * 1. Run_index 代表“已经沿轨迹推进到的里程点”；
 * 2. Prospect_index 代表“真正用于转向控制的预瞄点”；
 * 3. 低速时前瞻短，高速时前瞻长；若前瞻过大容易切弯过早，过小则高速左右摆头明显。
 */
#define Nag_Lookahead_Base_Points 3u
#define Nag_Lookahead_Speed_Gain 0.010f     // 前向速度 -> 额外前瞻点增益（速度越快前瞻越远）
#define Nag_Lookahead_Max_Points 24u        // 前瞻点上限，避免高速时前瞻过大导致切弯过早
#define Nag_Curve_Lookahead_Extra 10u       // 弯道强度估计时额外向前看的点数（启用自适应前瞻时有效）

/* 基于“前方 yaw 变化量”的简单弯道强度估计（仅当 Nag_AdaptiveLookahead_Enable==1 时使用）。
 * 当前先不把 curvature 持久化到 flash，而是直接用 Nav_read[] 前后点的 yaw 差来限速。
 */
#define Nag_Curve_Threshold_Straight 8.0f   // 进入“普通弯道”判定阈值（deg）
#define Nag_Curve_Threshold_Sharp 16.0f     // 进入“急弯”判定阈值（deg）
#define Nag_Speed_Ratio_Curve 0.9f         // 普通弯道目标速度倍率（基于 motor_user_speed_cmd）
#define Nag_Speed_Ratio_Sharp 0.8f         // 急弯目标速度倍率（基于 motor_user_speed_cmd）
#define Nag_Event_Speed_Ratio 0.35f         // 元素执行期间速度倍率上限（未切入自定义元素逻辑时的保护）

/* 元素调速总开关与距离换算：
 * 1. 提前加/减速距离均以 cm 配置，运行时由 Nag_DistanceToPoints() 按 Nag_Set_mileage 换算成导航点数；
 * 2. 实际比较仍使用 enter_index - Run_index 这类索引差，标定时只需关心物理距离；
 * 3. 元素前预减速、元素后预加速：进入 PreDecel / PreAccel 距离窗口后立即设为目标速度（非线性渐变）；
 * 4. 调速链挂在 Nag_GetControlSpeedTarget() / Nag_ApplyEventSpeedAdjustments()；
 * 5. 调试建议观察 VOFA 组 1：speed_target_effective / car_speed；组 3：里程纠偏/打滑（菜单 n 切组）。
 */
#define Nag_EventSpeed_Enable 1u               // 元素调速总开关：1=开启，0=关闭

/* SPIN / 折返进出口 / ENTER_CONES：Launch 页可调的运行时参数（默认值见 *_DEFAULT） */
#define Nag_Spin_Target_Speed_Default 1.0f
#define Nag_Spin_PreDecel_Dist_cm_Default 150.0f
#define Nag_Spin_Rate_Max_Dps_Default 200.0f
extern float nag_spin_target_speed;
extern float nag_spin_pre_decel_dist_cm;
#define Nag_Spin_PreAccel_Dist_cm 0.0f        // 自旋完成后恢复速度的提前加速距离（Launch 页不调节）

/*
 * Spin 触发区域（cm）：不是预减速距离，仅决定“允许提前触发自旋元素”的物理范围。
 * 当 Run_index 距前方未消费 Spin 的 enter_index <= 该距离时，可进入 Spin 元素态；
 * 等待减速期 Run_index 仍按里程推进；自旋完成后从起转前锁存的 Spin_Resume_RunIndex 恢复，
 * 不按录制 enter_index 或 enter_index+1 跳点。Event_Consumed 防止提前/滞后后再次触发同一点。
 * 停稳判定仍由 Nag_Hook_Spin_Run() 在元素内完成；预减速继续用 nag_spin_pre_decel_dist_cm。
 */
#define Nag_Spin_Trigger_Window_cm 30.0f

/* 折返入弯：Launch 可调 */
#define Nag_EnterTurn_Target_Speed_Default 220.0f
#define Nag_EnterTurn_PreDecel_Dist_cm_Default 0.0f
extern float nag_enter_turn_target_speed;
extern float nag_enter_turn_pre_decel_dist_cm;

/* 折返出弯：Launch 可调（语义同 EXIT_CONES） */
#define Nag_ExitTurn_Recovery_Speed_Default 0.0f   /* 0=恢复基础导航速度 */
#define Nag_ExitTurn_PreAccel_Dist_cm_Default 0.0f
extern float nag_exit_turn_recovery_speed;
extern float nag_exit_turn_pre_accel_dist_cm;

#define Nag_EnterCones_Target_Speed_Default 1000.0f
#define Nag_EnterCones_PreDecel_Dist_cm_Default 50.0f
extern float nag_enter_cones_target_speed;
extern float nag_enter_cones_pre_decel_dist_cm;

/* EXIT_CONES：锥桶出口恢复速度与提前加速距离（cm） */
#define Nag_ExitCones_Recovery_Speed 0.0f       // 0=恢复到基础导航速度；>0 则恢复到该固定速度
#define Nag_ExitCones_PreAccel_Dist_cm 20.0f    // 接近锥桶出口前开始恢复/加速的距离

/* 单边桥进/出（ENTER/EXIT_SINGLE_BRIDGE）：BridgeIn 立即白块进桥；BridgeOut 仅里程接回锚点 */
#define Nag_EnterBridge_Target_Speed_Default 500.0f   // 桥进目标速度（Launch/Flash 可调）
#define Nag_EnterBridge_PreDecel_Dist_cm_Default 0.0f // 桥进前预减速距离（cm）
#define Nag_EnterBridge_Leg_Long 5.5f                 // 桥进标记：目标腿长
#define Nag_ExitBridge_Leg_Long 3.5f                  // 桥出兜底腿长（已废弃：请用 Menu_GetInitLegLong()）
#define Nag_ExitBridge_Recovery_Speed 0.0f            // 0=过桥出后恢复基准速度；无出口 pre_accel
/** 宽限后 track_valid=0 连续该时间（ms）确认出桥；1ms ISR 读快照，不依赖 fresh */
#define BRIDGE_BLOB_LOST_EXIT_MS        250u
/** 进桥后该时间内禁止出桥判定（ms），给 CM7_1 启动 blob 与腿高步进留时间 */
#define BRIDGE_ENTER_GRACE_MS           400u
/** 进桥宽限后该时间仍没有 CM7_1 白块新帧，则退出桥区，避免视觉任务关闭时卡死 */
#define BRIDGE_BLOB_NO_FRAME_EXIT_MS    1000u
/** 单边桥白块中心误差 -> yaw 偏移，放在导航元素内消费，避免主循环抢 fresh */
#define Nag_BridgeBlob_Yaw_K_Pixel      0.22f
#define Nag_BridgeBlob_Yaw_Max_Offset   30.0f
#define Nag_BridgeBlob_Yaw_Deadband     0.1f
extern float nag_enter_bridge_target_speed;
extern float nag_enter_bridge_pre_decel_dist_cm;

/*
 * 颠簸路段双点录制（惯导回放）：
 * 1) KEY4 切 BumpIn，颠簸段前打点（Bin）；
 * 2) KEY4 切 BumpOut，在期望接回路径处打点（Bout，仅作里程锚点）；
 * 3) 回放：Bin 触发后 leg=6.5、固定速度、锁 Bin 点 yaw、开启横滚平衡、腿倾角限 25°、冻结 Run_index；
 *    nag_bump_duration_sec 计时到后链式切 EXIT_BUMP，从 Bout+1 接回惯导。
 * 见 Nag_FindPairedExitBumpMarker / Nag_ComputeBumpResumeIndex（navigation.c）。
 */
#define Nag_EnterBump_Target_Speed_Default 500.0f  // 颠簸段目标速度（motor_user_speed_cmd 档位）
#define Nag_EnterBump_Leg_Long 5.5f                // 元素期内 leg_long
#define Nag_EnterBump_Leg_Tilt_Max  25.0f          // 颠簸元素内腿俯仰倾角上限（°），仅 control 层限幅，勿改 VMC_A_TABLE_MAX
#define Nag_Bump_Duration_Sec_Default 5.0f           // 颠簸接管时长（秒），仅 ENTER_BUMP 计时
#define Nag_EnterBump_PreDecel_Dist_cm_Default 0.0f  // 进入颠簸前预减速距离（cm）
extern float nag_enter_bump_target_speed;
extern float nag_bump_duration_sec;
extern float nag_enter_bump_pre_decel_dist_cm;
#define Nag_HeadingHold_EnterBump_Enable 1u          // 1=颠簸期间锁 Nav_read[Bin] 单点 yaw + continuous 绝对航向维护

/*
 * 台阶双点录制（惯导回放）：
 * 1) KEY4 切 EnterStair，上台阶前打点（Ein）；
 * 2) 三级跳稳定后 KEY4 切 ExitStair 打点（Eout）；
 * 3) 回放：ENTER~EXIT 间 flash 路径不推进 Run_index；EXIT 完成后从 Eout+1 接回惯导。
 * 见 Nag_FindPairedExitStairMarker / Nag_ComputeStairResumeIndex（navigation.c）。
 */
/* 进入台阶元素（ENTER_STAIR）：接管后固定速度与腿长，锁 enter_index 录制点单点 yaw；见 Nag_Hook_EnterStair_* */
#define Nag_EnterStair_Target_Speed_Default 300.0f // 进入台阶目标速度默认；Launch/Flash 可调
#define Nag_EnterStair_Leg_Long 5.5f             // 元素期内 leg_long（非 jump_flag 跳跃时序）
#define Nag_EnterStair_PreDecel_Dist_cm_Default 0.0f  // 进入台阶预减速默认（cm）；Launch/Flash 可调
extern float nag_enter_stair_target_speed;
extern float nag_enter_stair_pre_decel_dist_cm;
#define Nag_HeadingHold_EnterStair_Enable 1u       // 1=台阶期间普通锁航（目标 Nav_read[enter_index]，非 continuous）

/* 退出台阶元素（EXIT_STAIR）：三次跳跃完成后软件链式切入；见 Nag_Hook_ExitStair_* */
#define Nag_ExitStair_Leg_Long 3.5f              // 台阶出兜底腿长（已废弃：请用 Menu_GetInitLegLong()）
#define Nag_Stair_Jump_Exit_Count 3u             // 进入台阶内完成该次数跳跃后切 EXIT
#define Nag_HeadingHold_ExitStair_Enable 0u      // 退出后立即交还惯导 yaw

/*
 * 台阶2双点录制（白块引导，惯导回放）：
 * 1) KEY4 切 Stair2In，上台阶2前打点（S2in）；
 * 2) KEY4 切 Stair2Out，期望接回路径处打点（S2out，仅作里程锚点）；
 * 3) 回放：S2in 触发后先沿惯导寻第一白块；找到后冻结 Run_index 并按白块修正 yaw；
 *    第一白块连续丢失 STAIR2_BLOB_LOST_CONFIRM_MS 后锁消失时 yaw 过深色路面（不退出元素）；
 *    第二白块出现后继续白块引导；第二白块连续丢失 STAIR2_BLOB_LOST_CONFIRM_MS 后链式切 EXIT_STAIR2，
 *    从 S2out 录制点接回惯导。
 * 见 Nag_FindPairedExitStair2Marker / Nag_ComputeStair2ResumeIndex（navigation.c）。
 */
#define Nag_EnterStair2_Target_Speed_Default 500.0f  // 台阶2元素期目标速度（Launch/Flash 可调）
extern float nag_enter_stair2_target_speed;
/** 白块 track_valid=0 连续该时间（ms）确认丢失；第一块→锁 yaw，第二块→链式 EXIT */
#define STAIR2_BLOB_LOST_CONFIRM_MS     250u
/** 元素激活后该时间内禁止丢失判定（ms），给 CM7_1 启动 blob 留时间 */
#define STAIR2_ENTER_GRACE_MS           400u
/** 激活后该时间仍无 CM7_1 白块新帧，则中止元素，避免视觉任务关闭时卡死 */
#define STAIR2_BLOB_NO_FRAME_EXIT_MS    1000u
/** 白块 center_err → yaw 偏移系数（与单边桥同量级） */
#define Nag_Stair2Blob_Yaw_K_Pixel      0.22f
#define Nag_Stair2Blob_Yaw_Max_Offset   30.0f
#define Nag_Stair2Blob_Yaw_Deadband      0.1f

/* 元素段数量先固定为少量结构，并写入单独的 flash 专用页：
 * 1. yaw 轨迹仍放在页 2~45；
 * 2. Save_index 仍放在页 1；
 * 3. 元素表单独放在 Nag_Event_Page，避免和导航元数据页混在一起；
 * 4. 事件页头的 Nag_Event_Version 与固件不一致时整块表不装载（枚举 type 语义变更时需升版并重录）。
 */
#define Nag_Event_Max 8u
#define Nag_Event_Page 46u
#define Nag_Event_Magic 0x4E414745u     // "NAGE"
#define Nag_Event_Version 7u            // v7：新增 ENTER/EXIT_STAIR2 type 11/12；旧事件表需重录

/* Run Launch 参数页（页 47）：
 * v1：仅 run_launch_speed；v2：7 个 float；v3：9 个 float（折返进/出口各两项）；v4：10 个 float（含自旋角速度）；
 * v5：v4 + menu_input_remote_first；v6：v5 + menu_vofa_enable；v7：v6 + enter_stair pre_decel；
 * v8：v7 + bridge_in speed/decel；v9：v8 + Nag_Vofa_Group @ [18]；
 * v10：v9 + g_menu_nav_fusion_enable @ [19]；
 * v11：v10 布局在 [12] 插入 enter_stair target speed，[13..20] 顺延；
 * v12：v11 + nag_bump_duration_sec @ [21]；
 * v13：v12 + g_menu_odo_slip_enable @ [22]；
 * v14：v13 + nag_enter_bump_target_speed @ [23]；
 * v15：v14 + nag_enter_stair2_target_speed @ [24]；
 * v16：v15 + g_menu_init_leg_long_sel @ [25]。
 */
#define Nag_Run_Launch_Speed_Page 47u
#define Nag_Run_Launch_Speed_Magic 0x524C5350u   // "RLSP"
#define Nag_Run_Launch_Speed_Version 1u          /* 旧版：仅 speed */
#define Nag_Run_Launch_Params_Version 2u         /* v2：7 个 float */
#define Nag_Run_Launch_Params_Version_V3 3u      /* v3：9 个 float */
#define Nag_Run_Launch_Params_Version_V4 4u      /* v4：10 个 float */
#define Nag_Run_Launch_Params_Version_V5 5u      /* v5：10 float + input mode */
#define Nag_Run_Launch_Params_Version_V6 6u      /* v6：v5 + vofa enable */
#define Nag_Run_Launch_Params_Version_V7 7u      /* v7：v6 + enter_stair pre_decel */
#define Nag_Run_Launch_Params_Version_V8 8u      /* v8：v7 + bridge_in speed/decel */
#define Nag_Run_Launch_Params_Version_V9 9u      /* v9：v8 + vofa debug group */
#define Nag_Run_Launch_Params_Version_V10 10u    /* v10：v9 + nav fusion enable */
#define Nag_Run_Launch_Params_Version_V11 11u    /* v11：v10 + enter_stair target speed @ [12] */
#define Nag_Run_Launch_Params_Version_V12 12u    /* v12：v11 + nag_bump_duration_sec @ [21] */
#define Nag_Run_Launch_Params_Version_V13 13u    /* v13：v12 + g_menu_odo_slip_enable @ [22] */
#define Nag_Run_Launch_Params_Version_V14 14u    /* v14：v13 + nag_enter_bump_target_speed @ [23] */
#define Nag_Run_Launch_Params_Version_V15 15u    /* v15：v14 + nag_enter_stair2_target_speed @ [24] */
#define Nag_Run_Launch_Params_Version_V16 16u    /* v16：v15 + g_menu_init_leg_long_sel @ [25] */
#define Nag_Run_Launch_Param_Count 17u
#define Nag_Run_Launch_Config_Word_Count_V5 11u  /* v5：10 float + menu_input_remote_first @ [13] */
#define Nag_Run_Launch_Config_Word_Count_V6 12u  /* v6：v5 + menu_vofa_enable @ [14] */
#define Nag_Run_Launch_Config_Word_Count_V7 13u  /* v7：11 float + 2 config word */
#define Nag_Run_Launch_Config_Word_Count_V8 15u  /* v8：13 float + 2 config word */
#define Nag_Run_Launch_Config_Word_Count_V9 16u  /* v9：v8 + Nag_Vofa_Group @ [18] */
#define Nag_Run_Launch_Config_Word_Count_V10 17u /* v10：v9 + g_menu_nav_fusion_enable @ [19] */
#define Nag_Run_Launch_Config_Word_Count_V11 18u /* v11：v10 顺延 + enter_stair target @ [12] */
#define Nag_Run_Launch_Config_Word_Count_V12 19u /* v12：v11 + bump_duration @ [21] */
#define Nag_Run_Launch_Config_Word_Count_V13 20u /* v13：v12 + odo_slip_enable @ [22] */
#define Nag_Run_Launch_Config_Word_Count_V14 21u /* v14：v13 + bump_target_speed @ [23] */
#define Nag_Run_Launch_Config_Word_Count_V15 22u /* v15：v14 + stair2_target_speed @ [24] */
#define Nag_Run_Launch_Config_Word_Count 23u     /* v16：v15 + init_leg_long_sel @ [25] */

/* Launch 页字段索引（与 flash 顺序一致） */
#define Nag_Launch_Field_Base_Spd 0u
#define Nag_Launch_Field_Spin_Spd 1u
#define Nag_Launch_Field_Spin_Dec 2u
#define Nag_Launch_Field_TurnIn_Spd 3u
#define Nag_Launch_Field_TurnIn_Dec 4u
#define Nag_Launch_Field_TurnOut_Spd 5u
#define Nag_Launch_Field_TurnOut_Acc 6u
#define Nag_Launch_Field_Cone_Spd 7u
#define Nag_Launch_Field_Cone_Dec 8u
#define Nag_Launch_Field_Spin_Rate 9u
#define Nag_Launch_Field_Stair_Spd 10u      /* 进入台阶目标速度 */
#define Nag_Launch_Field_Stair_Dec 11u      /* 进入台阶预减速距离（cm） */
#define Nag_Launch_Field_BridgeIn_Spd 12u   /* 单边桥进目标速度 */
#define Nag_Launch_Field_BridgeIn_Dec 13u   /* 单边桥进预减速距离（cm） */
#define Nag_Launch_Field_Bump_Dur 14u       /* 颠簸接管时长（秒），Launch KEY2/3 步进 1s */
#define Nag_Launch_Field_Bump_Spd 15u       /* 颠簸段目标速度（motor_user_speed_cmd 档位） */
#define Nag_Launch_Field_Stair2_Spd 16u     /* 台阶2白块引导目标速度（motor_user_speed_cmd 档位） */

#define Nag_Bump_Duration_Sec_Min 1.0f      /* Launch/Set 下限，避免 0s 立即链式退出 */
#define Nag_Bump_Duration_Sec_Max 60.0f     /* Launch/Set 上限（秒） */

float Nag_LaunchParamGet(uint8 field_index);
void Nag_LaunchParamSet(uint8 field_index, float value);
void Nag_LaunchParamAdjust(uint8 field_index, float delta);
void Nag_LaunchParamApplyDefaults(void);

/* CM7_1 不链接 navigation.c，Launch 菜单步进判断放头文件内联 */
static inline uint8 Nag_LaunchParamIsSpeed(uint8 field_index)
{
    return (uint8)((field_index == Nag_Launch_Field_Base_Spd) ||
                   (field_index == Nag_Launch_Field_Spin_Spd) ||
                   (field_index == Nag_Launch_Field_TurnIn_Spd) ||
                   (field_index == Nag_Launch_Field_TurnOut_Spd) ||
                   (field_index == Nag_Launch_Field_Cone_Spd) ||
                   (field_index == Nag_Launch_Field_Stair_Spd) ||
                   (field_index == Nag_Launch_Field_BridgeIn_Spd) ||
                   (field_index == Nag_Launch_Field_Bump_Spd) ||
                   (field_index == Nag_Launch_Field_Stair2_Spd));
}

static inline uint8 Nag_LaunchParamIsSpinRate(uint8 field_index)
{
    return (uint8)(field_index == Nag_Launch_Field_Spin_Rate);
}

static inline uint8 Nag_LaunchParamIsBumpDuration(uint8 field_index)
{
    return (uint8)(field_index == Nag_Launch_Field_Bump_Dur);
}

static inline float Nag_LaunchParamGetStep(uint8 field_index)
{
    if (Nag_LaunchParamIsSpinRate(field_index))
    {
        return 10.0f;
    }
    if (Nag_LaunchParamIsBumpDuration(field_index))
    {
        return 1.0f;
    }
    if (Nag_LaunchParamIsSpeed(field_index))
    {
        return 100.0f;
    }
    return 10.0f;
}

/* 自转元素示范参数：
 * 这组参数只是给默认的 Nag_Hook_Spin_* 一个“能跑通模板”的最小接法，
 * 后续你可以把它改成来自菜单、事件参数表，或完全替换成自己的实现。
 */
#define Nag_Spin_Demo_Turns 2.0f
#define Nag_Spin_Demo_Dir 1
/*
 * 起转前“低速停稳”判定（navigation 层，与 control 层收刹无关）：
 * Nag_Spin_Stop_Speed_Threshold — 当前速度低于该值视为进入低速区；
 * Nag_Spin_Stop_Stable_Count   — 连续满足上述条件的 1ms 拍数，达到后才 spin_task_start()。
 * 收刹结束区在 control.c：SPIN_ANGLE_SETTLE_DEG / SPIN_RATE_SETTLE_DPS / spin_finish(1)。
 * 航向漂移闭环校正在 control spin_finish(1) 内（Yaw_AlignDisplayDeg），本层不参与。
 */
#define Nag_Spin_Stop_Speed_Threshold 30.0f
#define Nag_Spin_Stop_Stable_Count 15u

/*
 * Spin 分阶段策略（实现见 Nag_Spin_ShouldTrackInsYaw / Run_Nag_GPS / Nag_Hook_Spin_Run）：
 * - 等待减速期：Run_index 与 Angle_Run 仍按里程/前瞻推进；
 * - spin_task_start 起转后：冻结 Run_index，锁存 Spin_Resume_RunIndex，融合里程快照同步；
 * - 自旋成功（spin_done==1，control 严格角度门限 + spin_finish(1) 航向校正）：从 Spin_Resume_RunIndex 接回惯导；
 * - 自旋失败（spin_failed==1）：恢复 Spin_Resume_RunIndex，不消费该元素；
 * - 起转后（spin_enable==1）：释放惯导航向，由 spin_cmd 控制。
 */

/* 元素航向保持配置：
 * 1. 这里的“保持航向”指元素接管后，锁定进入元素瞬间的实测 yaw（或 Bin/enter_index 单点 yaw）；
 * 2. ISR 首次 arm 后 steer_request → steer_set_target_yaw 引导；仅 ENTER_BUMP 在 continuous 期间由 pid_ctrl_Run 每 ms 双环纠偏（见 Nag_HeadingHold_IsContinuousActive）；
 * 3. ENTER_STAIR 等其它锁航元素走普通模式：一次引导、收敛后可 steer_finish，偏离超阈值再补发；
 * 4. 自旋元素：等待期由 Nag_Run 继续跟踪 Angle_Run；起转后 spin_enable 接管，见 Nag_HeadingHold_Spin_Enable 说明；
 * 5. 其它元素可按需要独立开关，后续新增元素时优先在这里配策略，不要把判断散到 ISR。
 */
#define Nag_HeadingHold_Reissue_Error 2.0f      // 普通锁航兜底：steer_finish 后偏离超过该阈值才重新登记；颠簸 continuous 期间不依赖
#define Nag_HeadingHold_Spin_Enable 0u          // 自旋元素在减速等待阶段保持进入元素时的航向
#define Nag_HeadingHold_EnterTurn_Enable 0u     // 折返入弯若需主动改航向则不保持锁定，默认关闭
#define Nag_Element_Enter_Beep_Enable 1u        // 1=回放/GPS 切入任意元素时蜂鸣一声
#ifndef BRIDGE_BEEP_MS
#define BRIDGE_BEEP_MS 100u                   // 与 init.h 一致；未包含 init.h 时的兜底
#endif
#ifndef NAG_ELEMENT_ENTER_BEEP_MS
#define NAG_ELEMENT_ENTER_BEEP_MS BRIDGE_BEEP_MS
#endif
//********************************************************//

/* 元素类型枚举：
 * 与 flash 事件表每条记录的 type 字节一致；Nag_Cycle_Record_Event_Type() 在 0..COUNT-1 间循环。
 * 折返/锥桶进/出口：惯导路径上的分段标记；区段调速见对应 Launch 参数或宏。
 */
/* GPS 路点统一元素（Normal=0，惯导 Spin~StairOut 后移一位；END 仅末点自动标记）：
 * u32yuansu[] / gps_current_yuansu 存此枚举；执行时 Nav_UnifiedToInsEvent() 映射到 Nag_Event_Type。
 */
typedef enum
{
    NAV_ELEM_NORMAL = 0,
    NAV_ELEM_SPIN = 1,
    NAV_ELEM_TURN_IN = 2,
    NAV_ELEM_TURN_OUT = 3,
    NAV_ELEM_CONE_IN = 4,
    NAV_ELEM_CONE_OUT = 5,
    NAV_ELEM_BRIDGE_IN = 6,
    NAV_ELEM_BRIDGE_OUT = 7,
    NAV_ELEM_BUMP_IN = 8,
    NAV_ELEM_BUMP_OUT = 9,
    NAV_ELEM_STAIR_IN = 10,
    NAV_ELEM_STAIR_OUT = 11,
    NAV_ELEM_STAIR2_IN = 12,
    NAV_ELEM_STAIR2_OUT = 13,
    NAV_ELEM_END = 14,
    NAV_ELEM_COUNT = 14,
} Nav_Unified_Element;

#define NAV_ELEM_RECORD_CYCLE_MAX NAV_ELEM_STAIR2_OUT

static inline uint8 Nav_UnifiedToInsEvent(uint8 unified)
{
    if (unified <= NAV_ELEM_NORMAL || unified >= NAV_ELEM_END)
    {
        return 0xFFu;
    }
    return (uint8)(unified - 1u);
}

static inline uint8 Nav_CycleUnifiedElement(uint8 current)
{
    current++;
    if (current > NAV_ELEM_RECORD_CYCLE_MAX)
    {
        current = NAV_ELEM_NORMAL;
    }
    return current;
}

static inline uint8 Nav_UnifiedIsMarker(uint8 unified)
{
    return (uint8)((unified == NAV_ELEM_TURN_IN) || (unified == NAV_ELEM_TURN_OUT) ||
                   (unified == NAV_ELEM_CONE_IN) || (unified == NAV_ELEM_CONE_OUT) ||
                   (unified == NAV_ELEM_BRIDGE_IN) || (unified == NAV_ELEM_BRIDGE_OUT) ||
                   (unified == NAV_ELEM_BUMP_OUT) ||
                   (unified == NAV_ELEM_STAIR2_OUT));
}

static inline uint8 Nav_UnifiedIsTakeover(uint8 unified)
{
    return (uint8)((unified == NAV_ELEM_SPIN) || (unified == NAV_ELEM_STAIR_IN) ||
                   (unified == NAV_ELEM_BUMP_IN) || (unified == NAV_ELEM_STAIR2_IN));
}

static inline uint8 Nav_UnifiedIsPassThrough(uint8 unified)
{
    return (uint8)(unified == NAV_ELEM_NORMAL);
}

static inline const char *Nav_GetUnifiedElementName(uint8 unified)
{
    switch (unified)
    {
        case NAV_ELEM_NORMAL: return "Normal";
        case NAV_ELEM_SPIN: return "Spin";
        case NAV_ELEM_TURN_IN: return "TurnIn";
        case NAV_ELEM_TURN_OUT: return "TurnOut";
        case NAV_ELEM_CONE_IN: return "ConeIn";
        case NAV_ELEM_CONE_OUT: return "ConeOut";
        case NAV_ELEM_BRIDGE_IN: return "BridgeIn";
        case NAV_ELEM_BRIDGE_OUT: return "BridgeOut";
        case NAV_ELEM_BUMP_IN: return "BumpIn";
        case NAV_ELEM_BUMP_OUT: return "BumpOut";
        case NAV_ELEM_STAIR_IN: return "StairIn";
        case NAV_ELEM_STAIR_OUT: return "StairOut";
        case NAV_ELEM_STAIR2_IN: return "Stair2In";
        case NAV_ELEM_STAIR2_OUT: return "Stair2Out";
        case NAV_ELEM_END: return "End";
        default: return "Unknown";
    }
}

typedef enum
{
       NAG_EVENT_TYPE_SPIN = 0,              // 原地自旋元素
       NAG_EVENT_TYPE_ENTER_TURNAROUND = 1,  // 折返入弯标记（沿路惯导，瞬时完成钩子）
       NAG_EVENT_TYPE_EXIT_TURNAROUND = 2,   // 折返出弯标记（沿路惯导，瞬时完成钩子）
       NAG_EVENT_TYPE_ENTER_CONES = 3,       // 进入锥桶标记（沿路惯导，瞬时完成钩子）
       NAG_EVENT_TYPE_EXIT_CONES = 4,        // 退出锥桶标记（沿路惯导，瞬时完成钩子）
       NAG_EVENT_TYPE_ENTER_SINGLE_BRIDGE = 5, // 单边桥进：惯导路径标记，leg=5.5，开横滚平衡
       NAG_EVENT_TYPE_EXIT_SINGLE_BRIDGE = 6,  // 单边桥出：惯导路径标记，leg=3.5，关横滚，恢复基准速度
       NAG_EVENT_TYPE_ENTER_BUMP = 7,        // 进入颠簸：锁 yaw+固定速度/腿长+横滚，计时后链式 EXIT
       NAG_EVENT_TYPE_EXIT_BUMP = 8,         // 退出颠簸：恢复 speed/leg、关横滚，首拍完成并接回 Bout+1
       NAG_EVENT_TYPE_ENTER_STAIR = 9,       // 进入台阶：锁航向+固定速度/腿长+台阶视觉；3 跳后链式 EXIT
       NAG_EVENT_TYPE_EXIT_STAIR = 10,       // 退出台阶：恢复基准速度与腿长 3.5，首拍完成
       NAG_EVENT_TYPE_ENTER_STAIR2 = 11,    // 进入台阶2：白块引导；第二白块丢失后链式 EXIT
       NAG_EVENT_TYPE_EXIT_STAIR2 = 12,      // 退出台阶2：从 S2out 里程点接回惯导，首拍完成
       NAG_EVENT_TYPE_COUNT = 13,            // 元素类型数量，录制时用于循环切换
} Nag_Event_Type;

/** ENTER_STAIR2 内部阶段（N.Stair2_Phase） */
typedef enum
{
       NAG_STAIR2_PHASE_SEEK_FIRST = 0,   /* 沿惯导寻第一白块，Run_index 仍推进 */
       NAG_STAIR2_PHASE_TRACK_FIRST = 1,  /* 跟第一白块，里程已冻结 */
       NAG_STAIR2_PHASE_DARK_HOLD = 2,    /* 第一白块丢失，锁 yaw 过深色路面 */
       NAG_STAIR2_PHASE_TRACK_SECOND = 3, /* 跟第二白块 */
} Nag_Stair2_Phase;

/* CM7_1 不链接 navigation.c，元素类型名映射放头文件内联 */
static inline const char *Nag_GetEventTypeName(uint8 event_type)
{
    switch (event_type)
    {
        case NAG_EVENT_TYPE_SPIN: return "Spin";
        case NAG_EVENT_TYPE_ENTER_TURNAROUND: return "TurnIn";
        case NAG_EVENT_TYPE_EXIT_TURNAROUND: return "TurnOut";
        case NAG_EVENT_TYPE_ENTER_CONES: return "ConeIn";
        case NAG_EVENT_TYPE_EXIT_CONES: return "ConeOut";
        case NAG_EVENT_TYPE_ENTER_SINGLE_BRIDGE: return "BridgeIn";
        case NAG_EVENT_TYPE_EXIT_SINGLE_BRIDGE: return "BridgeOut";
        case NAG_EVENT_TYPE_ENTER_BUMP: return "EnterBump";
        case NAG_EVENT_TYPE_EXIT_BUMP: return "ExitBump";
        case NAG_EVENT_TYPE_ENTER_STAIR: return "EnterStair";
        case NAG_EVENT_TYPE_EXIT_STAIR: return "ExitStair";
        case NAG_EVENT_TYPE_ENTER_STAIR2: return "Stair2In";
        case NAG_EVENT_TYPE_EXIT_STAIR2: return "Stair2Out";
        default: return "Unknown";
    }
}

/* 元素状态机枚举（全局一条状态机）：
 * 由各元素共用的 Nag_Element_StateMachine() 在回放、Event_Active 期间每周期调用；
 * Start 返回 false 时可长期停在 ENTERED，直至人工恢复或重写钩子。
 */
typedef enum
{
       NAG_EVENT_STATE_IDLE = 0,      // 空闲态：当前没有元素接管（Event_Active=0）
       NAG_EVENT_STATE_ENTERED = 1,    // 已到达 enter_index，已执行一次 Nag_Element_Start
       NAG_EVENT_STATE_RUNNING = 2,   // Start 已为 true：周期 Nag_Element_Run，直到 IsDone
       NAG_EVENT_STATE_DONE = 3,      // IsDone：下一拍 Nag_Notify_Event_Done() 恢复惯导
       NAG_EVENT_STATE_ABORT = 4,     // 中止：Nag_Element_Stop 清理后由调用方收尾
} Nag_Event_State;

typedef struct
{
       uint16 enter_index;   //回放触发点；单点录制时与 exit_index 相同
       uint16 exit_index;    //恢复索引；单点录制时与 enter_index 相同，旧双点录制可大于 enter_index
       uint8 type;           //元素类型，当前主要用于调试显示/人工分辨
       uint8 valid;          //1 表示这一条元素段有效
} NagEvent;

typedef struct{
       float Final_Out; //导航输出
       float Mileage_All;   //里程计累加
       float Mileage_Step;  //本周期位移增量
       float Mileage_Debug_Total; //调试用累计总路程
       float Speed_Forward; //当前用于积分的前向速度
       float Curve_Strength; //前方弯道强度（由前方 yaw 变化量估算）
       float Target_Speed; //导航根据路况计算出的原始目标速度
       float Angle_Run; //取偏航角
       float Requested_Target_Yaw; //最近一次下发给 steer 的目标航向
       bool Nag_Stop_f; //走到终点flag
       uint8 Target_Request_Valid; //目标航向是否已下发
       uint8 Flash_read_f;//走到终点读取flag
       uint16 size; //导航数组大小，通过计数
       uint16 Run_index;
       uint16 Save_count;
       uint16 Save_index;//保存flag
       uint16 Prospect_index; //当前真正用于控制的前瞻点索引
       uint16 Active_Event_Enter; //当前元素触发点，仅供调试查看
       uint16 Active_Event_Exit; //当前元素恢复索引，仅供调试查看（单点与 enter 相同）
       uint8 Save_state;
       uint8 End_f;//终点flag
       //flash相关参数
       uint8 Flash_page_index;//flash页索引
       uint8 Flash_Save_Page_Index;//flash保存页索引
       uint8 Nag_SystemRun_Index;   //导航执行索引
       uint8 Event_Active; //1表示当前已切出惯导，元素逻辑正在接管
       uint8 Event_Count; //当前已经录到多少个元素段
       uint8 Event_Record_Pending; //保留字段，单点录制不再使用，始终为 0
       uint8 Event_Record_Type; //录制阶段当前准备写入的元素类型
       uint8 Event_Active_Index; //回放阶段当前正在执行的元素编号
       uint8 Event_State; //元素状态机当前状态
       uint8 Event_Start_Latched; //1表示当前元素的 Start 钩子已经执行过
       uint8 Event_Done_Latched; //1表示当前元素报告完成，等待导航恢复
       uint8 Event_Active_Type; //回放阶段当前元素类型，供调试观察
       uint16 Event_Start_RunIndex; //当前元素开始接管时对应的 Run_index
       uint16 Event_Trigger_RunIndex; //本次元素进入时的 Run_index（调试用；Spin 恢复不依赖此字段）
       uint8 Event_Triggered_In_Window; //1=在 Spin 触发区域内提前触发；0=精确命中 enter_index
       uint8 Event_Consumed[Nag_Event_Max]; //本轮回放各事件是否已执行，防止窗口触发后再次命中同一点
       float Spin_Saved_SetSpeed; // 自旋元素接管前保存的全局速度档位
       uint16 Spin_Stop_Stable_Count; // 起转前：已连续低于 Nag_Spin_Stop_Speed_Threshold 的 1ms 拍数
       uint8 Spin_Task_Started; // 1=已调用 spin_task_start，控制层自旋进行中或刚结束
       uint16 Spin_Resume_RunIndex; // 起转前锁存 Run_index；自旋完成后从此索引接回惯导（非 enter_index 跳点）
       uint8 Spin_Speed_Latched; // 1=元素已清零 motor_user_speed_cmd，退出时需 Nag_Spin_RestoreSetSpeed
       float Stair_Saved_Leg_Long; //进入台阶前备份的 leg_long，Stop/Done 时恢复
       float Stair_Saved_SetSpeed; //进入台阶前备份的 motor_user_speed_cmd，Stop/Done 时恢复
       float Stair_Lookback_Yaw;   //进入台阶时锁定的航向目标（deg，Nav_read[enter_index]），供 VOFA/调试
       uint8 Stair_Jump_Completed_Count; //ENTER_STAIR 内 jump_control 正常结束次数
       uint8 Stair_Chain_To_Exit;  //1=ENTER 完成链式切 EXIT，EnterStair_Stop 跳过恢复速度/腿长
       uint16 Stair_Paired_Enter_Index; /* ENTER 完成链式 EXIT 时锁存 Ein；0xFFFF=无效 */
       float Bump_Saved_Leg_Long;     // 进入颠簸前备份 leg_long，Stop/Done 时恢复
       float Bump_Saved_SetSpeed;     // 进入颠簸前备份 motor_user_speed_cmd
       float Bump_Locked_Yaw;         // 进入颠簸时锁定的航向（deg），供 VOFA/调试
       uint32 Bump_Elapsed_Ms;        // ENTER_BUMP 元素期 1ms 计数
       uint8 Bump_Chain_To_Exit;      // 1=链式切 EXIT_BUMP，EnterBump_Stop 跳过恢复
       uint8 Bump_Zone_Active;        // 1=颠簸区腿倾角限 25°（EnterBump_Start～ExitBump_Stop），速度环仍闭环
       uint16 Bump_Paired_Enter_Index; /* ENTER 完成链式 EXIT 时锁存 Bin；0xFFFF=无效 */
       float Bridge_Saved_Leg_Long;   // 桥进前备份 leg_long，仅人工 Abort 时恢复
       uint8 Bridge_Saved_RollBalance; // 桥进前备份 roll_balance_en，仅人工 Abort 时恢复
       uint8 Bridge_Zone_Active;      // 1=桥上白块引导中（进桥确认～出桥确认）
       uint8 Bridge_Detect_Arm;       // 1=桥上，CM7_1 跑白块 detect（同 Zone_Active）
       uint8 Bridge_Heading_Lock;     // 保留字段；新方案不再置位，Abort 时仍清理
       uint8 Bridge_Expected;         // 保留字段；新方案不再置位，Abort 时仍清理
       uint8 Bridge_Exit_Beep_Done;   // 本段桥已响出桥蜂鸣
       float Bridge_Locked_Yaw;       // 保留字段；新方案不再使用 IMU 锁航
       uint16 Bridge_Exit_Run_Index;  // 进桥时缓存配对 BridgeOut enter_index；0xFFFF=未配对
       /* 台阶2白块引导（ENTER_STAIR2～EXIT_STAIR2） */
       uint8 Stair2_Zone_Active;       // 1=台阶2元素期白块引导中
       uint8 Stair2_Detect_Arm;        // 1=CM7_1 跑白块 detect（同 Zone_Active）
       uint8 Stair2_Phase;             // Nag_Stair2_Phase：寻第一白块/跟第一/锁 yaw/跟第二
       uint8 Stair2_First_Blob_Latched;// 1=已找到第一白块并冻结 Run_index
       uint8 Stair2_Second_Blob_Seen;  // 1=深色段后已重新捕获第二白块
       uint8 Stair2_Chain_To_Exit;    // 1=链式切 EXIT_STAIR2，EnterStair2_Stop 跳过恢复
       float Stair2_Saved_SetSpeed;    // 进入台阶2前备份 motor_user_speed_cmd
       float Stair2_Hold_Yaw;          // DARK_HOLD 阶段锁定的航向（deg）
       uint16 Stair2_Exit_Run_Index;   // 进元素时缓存配对 Stair2Out enter_index；0xFFFF=未配对
       uint16 Stair2_Paired_Enter_Index; /* 链式 EXIT 时锁存 S2in；0xFFFF=无效 */
       uint8 HeadingHold_Enable; //1表示当前元素期间已启用“锁定固定航向”模块
       uint8 HeadingHold_Request_Armed; //1表示 ISR 下一次应优先登记一次锁航向请求
       uint8 HeadingHold_Target_Latched; //1表示 HeadingHold_Target_Yaw 已锁存有效目标
       uint8 HeadingHold_Event_Allowed; //1表示当前元素类型配置允许启用航向保持
       float HeadingHold_Target_Yaw; //当前锁定的绝对航向目标，默认取进入元素瞬间的 euler_angle.yaw
       /* 里程纠偏：瞬时保护 + 打滑确认 + 回溯扣账（详见 Nag_OdoSlip_* 宏） */
       float Odo_Wheel_Left_Cmps;    //左轮前向线速度（cm/s），供 VOFA/调试
       float Odo_Wheel_Right_Cmps;   //右轮前向线速度（cm/s）
       float Odo_Gyro_Z_Dps;         //IMU Z 轴角速度（deg/s）
       float Odo_Vc_From_L_Cmps;     //由左轮+gyro_z 反推的中心速度
       float Odo_Vc_From_R_Cmps;     //由右轮+gyro_z 反推的中心速度
       float Odo_Corrected_Speed_Cmps; //本拍纠偏后中心速度
       float Odo_Raw_Step_Cm;        //未保护的本拍原始步长
       float Odo_Protected_Step_Cm;  //瞬时保护后的本拍步长
       float Odo_Last_Trust_Speed_Cmps; //最近可信中心速度
       float Odo_Rollback_Pending_Cm;   //待补扣里程（cm）
       float Odo_Rollback_Applied_Cm;   //累计已补扣（cm），调试用
       float Odo_History_Raw[Nag_OdoSlip_History_Len];  //短窗原始步长
       float Odo_History_Prot[Nag_OdoSlip_History_Len]; //短窗保护步长
       uint8 Odo_Slip_State;         //Nag_OdoSlip_State 枚举值
       uint8 Odo_Slip_Enter_Count;     //进入打滑计数
       uint8 Odo_Slip_Exit_Count;      //退出打滑计数
       uint8 Odo_History_Idx;          //历史环写指针
       //临时未使用参数
       int Prev_mile[Nag_Prev]; //前包
}Nag;

/* 里程纠偏打滑状态 */
typedef enum
{
       NAG_ODO_SLIP_NORMAL = 0u,
       NAG_ODO_SLIP_LEFT = 1u,   /* 左轮疑似空转，优先信右轮重建速度 */
       NAG_ODO_SLIP_RIGHT = 2u,  /* 右轮疑似空转，优先信左轮重建速度 */
       NAG_ODO_SLIP_BOTH = 3u,   /* 双侧不可信，保守限速/冻结 */
} Nag_OdoSlip_State;

extern Nag N;   //导航相关的结构体，用户开放参数
extern int32 Nav_read[Read_MaxSize];//每5cm的点，1000个点50m
extern NagEvent Nag_Event_Table[Nag_Event_Max];
extern uint8 Nag_Vofa_Group; // VOFA 调试组切换（菜单 n / 上位机命令循环）
/* 0=IMU 姿态；1=速度目标/实测；2=GPS+惯导融合；3=里程纠偏/打滑 */
#define NAG_VOFA_GROUP_COUNT (5u)

void Nag_OdoSlip_ResetState(void); /* 清零里程纠偏运行时态；Init_Nag/回放开始时调用 */
void Nag_OdoSlip_ApplyPendingRollback(void); /* 将待补扣里程写回 Mileage_All/Run_index */

extern uint8 g_menu_odo_slip_enable; /* 0=关 1=开；Run→Config / Flash V13，见 control.h */

static inline uint8 Nag_OdoSlip_IsRuntimeEnabled(void)
{
#if Nag_OdoSlip_Enable
    return (uint8)(g_menu_odo_slip_enable != 0u);
#else
    return 0u;
#endif
}

static inline uint8 Nag_IsEnterBumpActive(void)
{
    return N.Bump_Zone_Active;
}

float Nag_GetEffectiveLegTiltMax(void); /* 颠簸内 25°，否则 VMC_A_EXT_MAX */

typedef enum {
    NAV_HEADING_MODE_INS = 0u,
    NAV_HEADING_MODE_GPS = 1u,
} NavHeadingMode;
extern uint8 nav_heading_mode;
void Nag_Run(); //偏航角控制总函数
uint8 Nag_BridgeDetectShouldArm(void); /* 1=Bridge_Zone_Active，供 CM7_1 白块门控 */
void Nag_BridgeDetectUpdate(void);     /* 桥区白块 yaw + 新帧复位无帧超时 */
void Nag_BridgeTimeoutTick1ms(void);
uint8 Nag_Stair2DetectShouldArm(void); /* 1=Stair2_Zone_Active，供 CM7_1 白块门控 */
void Nag_Stair2DetectUpdate(void);     /* 台阶2白块 yaw / 锁 yaw / 阶段推进 */
void Nag_Stair2TimeoutTick1ms(void);
void Run_Nag_GPS();//偏航角读取

void NagFlashRead();   //Flash读取目标点数组
void Run_Nag_Save();    //偏航角读取保存
void Nag_Read();    //偏航角读取总函数
//****************************//
void Init_Nag();    //偏航角初始化，flash缓冲区初始化，索引初始化
void Nag_Begin_Record(void); //开始录制前复位运行态
void Nag_Begin_Replay(void); //开始复现前复位运行态
/* 原点平均采集完成后调用：flash 已就绪且融合 valid 时进入 index=3 并赋速 */
void Nag_CompleteReplayAfterOrigin(void);
void Nag_Request_Stop_Record(void); //录制结束请求
void Nag_Request_Event_Mark(void); /* 录制：单击在当前 Save_index 保存一条有效元素事件 */
void Nag_Cycle_Record_Event_Type(void); /* 录制：N.Event_Record_Type 加一模 NAG_EVENT_TYPE_COUNT */
void Nag_Notify_Event_Done(void); /* 元素完成：恢复 Run_index 并清 Event_Active，继续惯导前瞻 */
void Nag_NotifyStairJumpDone(void); /* jump_control 时序正常结束时调用；ENTER_STAIR 内计数 */
void Nag_Element_Abort(void); //异常/手动中止当前元素，清理状态并停留在当前元素态
float Nag_GetDebugReadYaw(void); //安全读取当前回放目标 yaw
float Nag_GetControlSpeedTarget(void); //给速度环的目标速度，叠加弯道限速、元素区段调速与提前加减速
bool Nav_GetEventSpeedProfileConfig(uint8 event_type,
                                    float *target_speed,
                                    float *pre_decel_dist_cm,
                                    float *pre_accel_dist_cm);
float GPS_ApplyEventSpeedAdjustments(float nav_speed);
void Nag_EventPrepareEnter(uint8 event_type);
void Nag_EventForceReset(void);
uint16 Nag_GetDebugProspectIndex(void); //安全读取当前前瞻索引
bool Nag_HeadingHold_ShouldRequest(void); //供 1ms ISR 查询：当前元素是否需要在消费 pending 前补登一次锁航向请求
float Nag_HeadingHold_GetTargetYaw(void); //安全读取当前锁定的元素航向保持目标
bool Nag_HeadingHold_IsContinuousActive(void); //1=仅 ENTER_BUMP 持续双环纠偏（pid_ctrl_Run 不 steer_finish）
bool Nag_Element_Start(uint8 event_type); /* event_type：N.Event_Active_Type；true 则进入 RUNNING */
void Nag_Element_Run(uint8 event_type);
bool Nag_Element_IsDone(uint8 event_type); /* true：本周期转入 DONE 并随后 Nag_Notify_Event_Done */
void Nag_Element_Stop(uint8 event_type);   /* 中止/完成清理：先退出锁航再调具体 Stop */

/* 折返进/出口：仅路径标记，Start 置真、首拍 IsDone 真以尽快接回惯导 */
bool Nag_Hook_EnterTurn_Start(void);
void Nag_Hook_EnterTurn_Run(void);
bool Nag_Hook_EnterTurn_IsDone(void);
void Nag_Hook_EnterTurn_Stop(void);

bool Nag_Hook_ExitTurn_Start(void);
void Nag_Hook_ExitTurn_Run(void);
bool Nag_Hook_ExitTurn_IsDone(void);
void Nag_Hook_ExitTurn_Stop(void);

bool Nag_Hook_Spin_Start(void);
void Nag_Hook_Spin_Run(void);
bool Nag_Hook_Spin_IsDone(void);
void Nag_Hook_Spin_Stop(void);

/* 单边桥进/出：惯导路径标记（同锥桶），Start 置真、首拍 IsDone 真；桥进设腿长/横滚，区段调速见 Nag_ApplyBridgeZoneSpeed */
bool Nag_Hook_EnterBridge_Start(void);
void Nag_Hook_EnterBridge_Run(void);
bool Nag_Hook_EnterBridge_IsDone(void);
void Nag_Hook_EnterBridge_Stop(void);

bool Nag_Hook_ExitBridge_Start(void);
void Nag_Hook_ExitBridge_Run(void);
bool Nag_Hook_ExitBridge_IsDone(void);
void Nag_Hook_ExitBridge_Stop(void);

/* 颠簸进/出：ENTER 开横滚、腿倾角限 25°、计时接管后链式 EXIT 关横滚；见 Nag_Hook_EnterBump_* / Nag_Hook_ExitBump_* */
bool Nag_Hook_EnterBump_Start(void);
void Nag_Hook_EnterBump_Run(void);
bool Nag_Hook_EnterBump_IsDone(void);
void Nag_Hook_EnterBump_Stop(void);

bool Nag_Hook_ExitBump_Start(void);
void Nag_Hook_ExitBump_Run(void);
bool Nag_Hook_ExitBump_IsDone(void);
void Nag_Hook_ExitBump_Stop(void);

bool Nag_Hook_EnterStair_Start(void);
void Nag_Hook_EnterStair_Run(void);
bool Nag_Hook_EnterStair_IsDone(void);
void Nag_Hook_EnterStair_Stop(void);

bool Nag_Hook_ExitStair_Start(void);
void Nag_Hook_ExitStair_Run(void);
bool Nag_Hook_ExitStair_IsDone(void);
void Nag_Hook_ExitStair_Stop(void);

/* 台阶2进/出：白块双段引导；S2out 为里程锚点，第二白块丢失后链式 EXIT 接回 */
bool Nag_Hook_EnterStair2_Start(void);
void Nag_Hook_EnterStair2_Run(void);
bool Nag_Hook_EnterStair2_IsDone(void);
void Nag_Hook_EnterStair2_Stop(void);

bool Nag_Hook_ExitStair2_Start(void);
void Nag_Hook_ExitStair2_Run(void);
bool Nag_Hook_ExitStair2_IsDone(void);
void Nag_Hook_ExitStair2_Stop(void);

/* 锥桶：仅路径标记，Start 置真、首拍 IsDone 真以尽快接回惯导，供后续按 Run_index/事件表做区段限速 */
bool Nag_Hook_EnterCones_Start(void);
void Nag_Hook_EnterCones_Run(void);
bool Nag_Hook_EnterCones_IsDone(void);
void Nag_Hook_EnterCones_Stop(void);

bool Nag_Hook_ExitCones_Start(void);
void Nag_Hook_ExitCones_Run(void);
bool Nag_Hook_ExitCones_IsDone(void);
void Nag_Hook_ExitCones_Stop(void);

void Nag_System();  //偏航角函数的封装，包装进中断小

/*
 * 惯导路径修正界面（Debug → PathFix → 2.6.1 功能页）：
 * 1. 从 Flash 载入 Nav_read[]（yaw×100，每 Nag_Set_mileage cm 一点），在 LCD 上重建二维折线；
 * 2. KEY1 每次前进 Nag_PathFix_Select_Step 个点（循环）；KEY2/KEY3 对选中点 yaw ±Nag_PathFix_Yaw_Step_Deg；KEY4 有修改时写回 Flash 并退出；
 * 3. 不修改 Nag_SystemRun_Index，录制/回放进行中禁止进入；修正后的 Nav_read[] 直接用于下次惯导回放。
 */
#define Nag_PathFix_Yaw_Step_Deg   2.0f
#define Nag_PathFix_Select_Step    10u    /* KEY1 每次切换的路径点数 */
#define Nag_PathFix_Draw_Max       180u   /* 与 dualcore_shared.h DUALCORE_PATHFIX_DRAW_MAX 保持一致 */

typedef struct
{
    uint8 active;        /* 1=PathFix 会话进行中 */
    uint8 loaded;        /* 1=已从 Flash 载入有效轨迹到 Nav_read[] */
    uint8 dirty;         /* 1=自上次 Flash 保存后有 yaw 修改 */
    uint16 select_index; /* 当前选中点 [0, point_count) */
    uint16 point_count;  /* 轨迹点数（等于载入时的 Save_index） */
} NagPathFixState;

extern NagPathFixState g_nag_pathfix;

uint8 Nag_PathFix_Enter(void);
void Nag_PathFix_Leave(uint8 save_if_dirty);
uint8 Nag_PathFix_CycleSelect(void);
uint8 Nag_PathFix_AdjustYaw(float delta_deg);
uint8 Nag_PathFix_ExitSave(void);
void Nag_PathFix_PublishDrawMap(uint16 x_offset, uint16 y_offset, uint16 width, uint16 height,
                                int16 *out_x, int16 *out_y, uint16 out_max,
                                uint16 *out_count, uint16 *out_sel_draw);
void Nag_PathFix_SyncToShared(void *ctrl_snapshot,
                              uint16 x_offset,
                              uint16 y_offset,
                              uint16 width,
                              uint16 height);
void Nag_PathFix_DrawViewport(uint16 x_offset, uint16 y_offset, uint16 width, uint16 height);

#endif /* _NAVIGATION_H_ */
