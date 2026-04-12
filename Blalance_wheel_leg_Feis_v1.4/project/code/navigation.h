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
 * 2. Nag_Speed_Source 为当前选用的前向速度源，先继续使用 car_speed；
 * 3. Nag_Speed_To_Mileage_Scale 用来把速度源换算成 cm/s，必须实车标定；
 * 4. Nag_Sample_Dt 必须与 Nag_System() 的真实调用周期严格一致。
 *    当前 Nag_System() 固定在 pit0_ch0_isr 的 1ms 中断里跑，因此这里必须是 0.001s。
 *    一旦修改导航调用周期或速度单位，Nag_Speed_To_Mileage_Scale 必须重新标定。
 */
#define Nag_Set_mileage 5.0f              //每隔 5cm 记录一次 yaw
#define Nag_Prev 200                      //保留的历史/预读缓存长度
#define Nag_Yaw euler_angle.yaw           //航向角度取偏航角
#define Nag_Sample_Dt 0.001f              //Nag_System 当前固定 1ms 运行一次
#define Nag_Speed_Source car_speed        //默认优先使用车体平均速度
#define Nag_Speed_To_Mileage_Scale 0.37f  //速度单位到 cm/s 的换算系数，需标定
#define Nag_Speed_Deadband 1.0f           //速度死区，抑制静止噪声
#define Nag_Reissue_Error 3.5f            //转向收敛后若再次偏离该角度，则重新下发目标 yaw
/* 速度调试旁路：
 * 1. 置 1 后，即使导航未进入回放态，也允许速度环直接使用 motor_user_speed_cmd；
 * 2. 仅用于直道阶跃调 PID，比赛/正式回放前务必改回 0；
 * 3. 打开后 Nag_GetControlSpeedTarget() 会绕过 Nag_SystemRun_Index==3 的门控。
 */
#define Nag_Debug_Speed_Bypass_Enable 1u

/* 速度自适应前瞻参数：
 * 1. Run_index 代表“已经沿轨迹推进到的里程点”；
 * 2. Prospect_index 代表“真正用于转向控制的预瞄点”；
 * 3. 低速时前瞻短，高速时前瞻长；若前瞻过大容易切弯过早，过小则高速左右摆头明显。
 */
#define Nag_Lookahead_Base_Points 3u
#define Nag_Lookahead_Speed_Gain 0.010f     // 前向速度 -> 额外前瞻点增益（速度越快前瞻越远）
#define Nag_Lookahead_Max_Points 24u        // 前瞻点上限，避免高速时前瞻过大导致切弯过早
#define Nag_Curve_Lookahead_Extra 10u       // 弯道强度估计时额外向前看的点数（用于比较 yaw 变化）

/* 基于“前方 yaw 变化量”的简单弯道强度估计。
 * 当前先不把 curvature 持久化到 flash，而是直接用 Nav_read[] 前后点的 yaw 差来限速。
 */
#define Nag_Curve_Threshold_Straight 6.0f   // 进入“普通弯道”判定阈值（deg）
#define Nag_Curve_Threshold_Sharp 16.0f     // 进入“急弯”判定阈值（deg）
#define Nag_Speed_Ratio_Curve 0.85f         // 普通弯道目标速度倍率（基于 motor_user_speed_cmd）
#define Nag_Speed_Ratio_Sharp 0.75f         // 急弯目标速度倍率（基于 motor_user_speed_cmd）
#define Nag_Event_Speed_Ratio 0.35f         // 元素执行期间速度倍率上限（未切入自定义元素逻辑时的保护）

/* 元素前预减速：
 * 1. 这层逻辑挂在 Nag_GetControlSpeedTarget() 里，只在“尚未真正切入元素前”生效；
 * 2. K 表示提前多少个导航点开始减速。当前 Nag_Set_mileage=5cm，因此 K 点约等于 K*5cm；
 * 3. 标定建议：K = ceil(D_stop / Nag_Set_mileage) + 2~4 点安全裕量；
 * 4. 初次实车建议重点观察 VOFA 里的 speed_target_effective / car_speed / Run_index / Event_Active_Type。
 */
#define Nag_PreEventDecel_Enable 1u            // 元素前预减速总开关：1=开启，0=关闭
#define Nag_Spin_PreDecel_Points 20u           // 自旋元素提前 K 点开始减速（K*5cm）
#define Nag_Spin_PreDecel_MinSpeed 100.0f      // 自旋元素预进入最小速度下限
#define Nag_Turnaround_PreDecel_Points 0u      // 掉头元素提前 K 点开始减速（0=当前关闭）
#define Nag_Turnaround_PreDecel_MinSpeed 220.0f// 掉头元素预进入最小速度下限
#define Nag_SingleBridge_PreDecel_Points 0u    // 独木桥元素提前 K 点开始减速（0=当前关闭）
#define Nag_SingleBridge_PreDecel_MinSpeed 220.0f // 独木桥元素预进入最小速度下限
#define Nag_Bump_PreDecel_Points 0u            // 减速带元素提前 K 点开始减速（0=当前关闭）
#define Nag_Bump_PreDecel_MinSpeed 220.0f      // 减速带元素预进入最小速度下限
#define Nag_Jump_PreDecel_Points 0u            // 跳跃元素提前 K 点开始减速（0=当前关闭）
#define Nag_Jump_PreDecel_MinSpeed 220.0f      // 跳跃元素预进入最小速度下限

/* 元素段数量先固定为少量结构，并写入单独的 flash 专用页：
 * 1. yaw 轨迹仍放在页 2~45；
 * 2. Save_index 仍放在页 1；
 * 3. 元素表单独放在 Nag_Event_Page，避免和导航元数据页混在一起。
 */
#define Nag_Event_Max 8u
#define Nag_Event_Page 46u
#define Nag_Event_Magic 0x4E414745u     // "NAGE"
#define Nag_Event_Version 1u

/* 自转元素示范参数：
 * 这组参数只是给默认的 Nag_Hook_Spin_* 一个“能跑通模板”的最小接法，
 * 后续你可以把它改成来自菜单、事件参数表，或完全替换成自己的实现。
 */
#define Nag_Spin_Demo_Turns 2.0f
#define Nag_Spin_Demo_Dir 1
#define Nag_Spin_Stop_Speed_Threshold 80.0f // 当前速度低于该值时视为进入低速区
#define Nag_Spin_Stop_Stable_Count 15u      // 连续低于阈值 N 个 1ms 周期后才开始自旋

/* 元素航向保持配置：
 * 1. 这里的“保持航向”指元素接管后，锁定进入元素瞬间的实测 yaw；
 * 2. ISR 会在 steer_yaw_request_pending 消费前，按需重新登记该固定目标；
 * 3. 自旋元素推荐只在“减速等待起转”的阶段保持，真正 spin_task_start() 前解除；
 * 4. 其它元素可按需要独立开关，后续新增元素时优先在这里配策略，不要把判断散到 ISR。
 */
#define Nag_HeadingHold_Reissue_Error 2.0f      // 已解锁普通转向后，实际 yaw 偏离锁定目标超过该阈值才重新登记保持请求
#define Nag_HeadingHold_Spin_Enable 1u          // 自旋元素在减速等待阶段保持进入元素时的航向
#define Nag_HeadingHold_Turnaround_Enable 0u    // 掉头元素需要主动改航向，默认不保持
#define Nag_HeadingHold_SingleBridge_Enable 1u  // 单边桥默认整段保持进入元素时的航向
#define Nag_HeadingHold_Bump_Enable 1u          // 颠簸/减速带默认整段保持进入元素时的航向
#define Nag_HeadingHold_Jump_Enable 1u          // 跳跃元素默认整段保持进入元素时的航向
//********************************************************//

/* 元素类型枚举：
 * 与录制到 flash 的事件表 type 字段一一对应。
 */
typedef enum
{
       NAG_EVENT_TYPE_GENERIC = 0,       // 通用占位类型（默认未接入动作）
       NAG_EVENT_TYPE_TURNAROUND = 1,    // 掉头元素
       NAG_EVENT_TYPE_SPIN = 2,          // 原地自旋元素
       NAG_EVENT_TYPE_SINGLE_BRIDGE = 3, // 单边桥元素
       NAG_EVENT_TYPE_BUMP = 4,          // 减速带/颠簸元素
       NAG_EVENT_TYPE_JUMP = 5,          // 跳跃元素
} Nag_Event_Type;

/* 元素状态机枚举：
 * 由 Nag_Element_StateMachine() 周期驱动。
 */
typedef enum
{
       NAG_EVENT_STATE_IDLE = 0,      // 空闲态：当前没有元素接管
       NAG_EVENT_STATE_ENTERED = 1,   // 刚切入元素，等待 Start 钩子启动
       NAG_EVENT_STATE_RUNNING = 2,   // 元素运行中，周期执行 Run 并检查完成
       NAG_EVENT_STATE_DONE = 3,      // 元素完成，准备从 exit_index 接回导航
       NAG_EVENT_STATE_ABORT = 4,     // 元素中止，执行 Stop 清理现场
} Nag_Event_State;

typedef struct
{
       uint16 enter_index;   //回放时到达该索引后切出惯导
       uint16 exit_index;    //元素完成后从该索引继续接回惯导
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
       uint16 Active_Event_Enter; //当前元素段进入点，仅供调试查看
       uint16 Active_Event_Exit; //当前元素段退出点，仅供调试查看
       uint8 Save_state;
       uint8 End_f;//终点flag
       //flash相关参数
       uint8 Flash_page_index;//flash页索引
       uint8 Flash_Save_Page_Index;//flash保存页索引
       uint8 Nag_SystemRun_Index;   //导航执行索引
       uint8 Event_Active; //1表示当前已切出惯导，元素逻辑正在接管
       uint8 Event_Count; //当前已经录到多少个元素段
       uint8 Event_Record_Pending; //录制阶段：1表示已经记下 enter，等待 exit
       uint8 Event_Record_Type; //录制阶段当前准备写入的元素类型
       uint8 Event_Active_Index; //回放阶段当前正在执行的元素编号
       uint8 Event_State; //元素状态机当前状态
       uint8 Event_Start_Latched; //1表示当前元素的 Start 钩子已经执行过
       uint8 Event_Done_Latched; //1表示当前元素报告完成，等待导航恢复
       uint8 Event_Active_Type; //回放阶段当前元素类型，供调试观察
       uint16 Event_Start_RunIndex; //当前元素开始接管时对应的 Run_index
       float Spin_Saved_SetSpeed; //自旋元素接管前保存的全局速度档位
       uint16 Spin_Stop_Stable_Count; //当前已连续低于速度阈值多少个 1ms 周期
       uint8 Spin_Task_Started; //1表示当前自旋任务已经真正下发给控制层
       uint8 Spin_Speed_Latched; //1表示当前元素已接管并清零 motor_user_speed_cmd，退出时需恢复
       uint8 Jump_Element_Armed; //1表示跳跃元素已置 jump_flag，供 IsDone 防误判（Start 前 jump_flag 可能为 0）
       uint8 HeadingHold_Enable; //1表示当前元素期间已启用“锁定固定航向”模块
       uint8 HeadingHold_Request_Armed; //1表示 ISR 下一次应优先登记一次锁航向请求
       uint8 HeadingHold_Target_Latched; //1表示 HeadingHold_Target_Yaw 已锁存有效目标
       uint8 HeadingHold_Event_Allowed; //1表示当前元素类型配置允许启用航向保持
       float HeadingHold_Target_Yaw; //当前锁定的绝对航向目标，默认取进入元素瞬间的 euler_angle.yaw
       //临时未使用参数
       int Prev_mile[Nag_Prev]; //前包
}Nag;

extern Nag N;   //导航相关的结构体，用户开放参数
extern int32 Nav_read[Read_MaxSize];//每5cm的点，1000个点50m
extern NagEvent Nag_Event_Table[Nag_Event_Max];
extern uint8 Nag_Vofa_Group; //VOFA 调试组切换
void Nag_Run(); //偏航角控制总函数
void Run_Nag_GPS();//偏航角读取

void NagFlashRead();   //Flash读取目标点数组
void Run_Nag_Save();    //偏航角读取保存
void Nag_Read();    //偏航角读取总函数
//****************************//
void Init_Nag();    //偏航角初始化，flash缓冲区初始化，索引初始化
void Nag_Begin_Record(void); //开始录制前复位运行态
void Nag_Begin_Replay(void); //开始复现前复位运行态
void Nag_Request_Stop_Record(void); //录制结束请求
void Nag_Request_Event_Mark(void); //录制阶段登记元素 enter/exit 点；完整录制结束后会随导航元数据一起写入 flash
void Nag_Cycle_Record_Event_Type(void); //切换下一条待录元素类型
void Nag_Notify_Event_Done(void); //元素逻辑完成后通知导航从 exit 点继续
void Nag_Element_Abort(void); //异常/手动中止当前元素，清理状态并停留在当前元素态
float Nag_GetDebugReadYaw(void); //安全读取当前回放目标 yaw
float Nag_GetControlSpeedTarget(void); //给速度环的目标速度，自动叠加弯道限速和元素限速
uint16 Nag_GetDebugProspectIndex(void); //安全读取当前前瞻索引
bool Nag_HeadingHold_ShouldRequest(void); //供 1ms ISR 查询：当前元素是否需要在消费 pending 前补登一次锁航向请求
float Nag_HeadingHold_GetTargetYaw(void); //安全读取当前锁定的元素航向保持目标
bool Nag_Element_Start(uint8 event_type); //统一元素 Start 分发，默认钩子返回 false 表示“未接入具体动作”
void Nag_Element_Run(uint8 event_type); //统一元素 Run 分发
bool Nag_Element_IsDone(uint8 event_type); //统一元素完成判定
void Nag_Element_Stop(uint8 event_type); //统一元素 Stop 分发

bool Nag_Hook_Turnaround_Start(void);
void Nag_Hook_Turnaround_Run(void);
bool Nag_Hook_Turnaround_IsDone(void);
void Nag_Hook_Turnaround_Stop(void);

bool Nag_Hook_Spin_Start(void);
void Nag_Hook_Spin_Run(void);
bool Nag_Hook_Spin_IsDone(void);
void Nag_Hook_Spin_Stop(void);

bool Nag_Hook_SingleBridge_Start(void);
void Nag_Hook_SingleBridge_Run(void);
bool Nag_Hook_SingleBridge_IsDone(void);
void Nag_Hook_SingleBridge_Stop(void);

bool Nag_Hook_Bump_Start(void);
void Nag_Hook_Bump_Run(void);
bool Nag_Hook_Bump_IsDone(void);
void Nag_Hook_Bump_Stop(void);

bool Nag_Hook_Jump_Start(void);
void Nag_Hook_Jump_Run(void);
bool Nag_Hook_Jump_IsDone(void);
void Nag_Hook_Jump_Stop(void);

void Nag_System();  //偏航角函数的封装，包装进中断小
#endif /* _NAVIGATION_H_ */
