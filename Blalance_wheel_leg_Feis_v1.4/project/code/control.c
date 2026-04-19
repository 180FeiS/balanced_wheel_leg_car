#include "zf_common_headfile.h"

ins_struct ins;  //惯性导航结构体
double TempLat_Now=0,TempLon_Now=0;     // 二维坐标系下的实时位置
uint16 pwm_4 = 0;
uint16 pwm_1 = 0;

// 全局PWM参数变量
int16 pwm_ph1 = 0;
int16 pwm_ph2 = 0;
int16 pwm_ph3 = 0;
int16 pwm_ph4 = 0;

// LQR的增益矩阵K, 修改参数使用matlab
const float LQR_K[8] = {
    // -0.0281 ,  -0.8067 , -1.6867 , -0.1745,
    // -0.0281 ,  -0.8067 , -1.6867 , -0.1745
    //        -0.0285  , -0.8754, -1.4840 ,-0.1806,
    //        -0.0285  , -0.8754 ,-1.4840,-0.1806

    -0.0585, -0.6200, -1.5706, -0.1139,
    -0.0585, -0.6200, -1.5706, -0.1139
    //        -0.0431 ,  -0.6339 ,  -1.5234  , -0.1616,
    //        -0.0431 ,  -0.6339  , -1.5234  , -0.1616
};

// 无刷电机极对数, 已确定不可修改
const float Lmoto_K = 4980;
const float Rmoto_K = 4980;

// PID初始化
pid_t leg_hight, turn_angle, turn_gyro, gyro, angle, speed, turn;

float angle_kd = 0;    // 角度环kd
float pitch_mid = 7.5;  // pitch机械中值（俯仰平衡）
float roll_mid = -9.0; // roll机械中值（横滚平衡，leg_hight PID目标）

// 各个环节PID的运算周期
float dt_pid_gyro = 0.002f;
float dt_pid_angle = 0.01f;
float dt_pid_speed = 0.02f;
float dt_pid_turn = 0.01f;
float dt_leg = 0.005f;   // 5ms，横滚leg_hight PID周期，与pit0_ch1 leg_control一致
/* 航向外环（turn_angle）周期：须与 pid_ctrl_Run() 实际调用周期一致。
 * pit0_ch0_isr 为 1ms 一次，普通转向里每拍都跑外环，故 dt=0.001。
 * 若改为“每 3ms 只算一次外环”，应在此处保持 dt=0.003，并在 pid_ctrl_Run 里用计数器包一层，内环仍每 1ms 跑。
 */
float dt_pid_turn_angle = 0.001f;
float dt_pid_turn_gyro = 0.001f;

// 初始腿高（非跳跃时基准）
float leg_long = 5.5f; 
// float leg_high_integral = 0;

/*---------------------------------------------------------------------------
 * 用户速度与拨码（与 Menu.c dip_switch_motor_sync_from_hw 配合）：
 * - motor_user_speed_cmd：导航/速度环的“用户期望基准”，符号表示前进/后退；
 * - motor_poll_switch2_speed_baseline()：周期调用，按 SWITCH2 档位/边沿同步 1000 或 1500；
 * - motor_user_speed_cmd_set_from_pc()：串口 V<数值> 直接写基准（无速度锁；SWITCH2 档位变化时仍会同步为 1000/1500）；
 * - Motor_Switch 仅由 SWITCH1 与 Motor_Runaway_Latch 决定（见 Menu.c）。
 *---------------------------------------------------------------------------*/

/* 用户速度基准：导航弯道限速、元素限速等均以此为上限参考；符号用于倒车方向 */
float motor_user_speed_cmd = 0.0f;
float speed_target_effective = 0.0f; /* 经 Nag_GetControlSpeedTarget() 后的速度环目标，供调试对比 */

/* 拨码 SWITCH2 边沿检测状态，集中在控制层，避免 Menu 与串口解析各写一套 */
static uint8 motor_dip_speed_inited = 0u;
static uint8 motor_dip_last_fast = 0u;

// 跳跃标志位
uint8 jump_flag = 0;
uint8 jump_step_index = 0;  // 当前跳跃步 0起跳 1准备缓冲 2执行缓冲
static int jump_time = 0;   // 跳跃时序计数，单位见 jump_control()

uint8 speed_flag = 0;

/* 轮速失控保护触发后置 1；拨码须先拨到 OFF 再允许恢复使能，避免覆盖 Motor_Switch=0。 */
uint8 Motor_Runaway_Latch = 0;

uint8 jump_is_allowed(void)
{
    return (Motor_Runaway_Latch == 0u);
}

void jump_stop(void)
{
    jump_flag = 0u;
    jump_step_index = 0u;
    jump_time = 0;
    leg_long = 5.5f;
}

void motor_user_speed_cmd_set_from_pc(float cmd)
{
    motor_user_speed_cmd = cmd;
}

void motor_poll_switch2_speed_baseline(void)
{
    uint8 dip_speed_fast = (gpio_get_level(SWITCH2) == GPIO_LOW) ? 1u : 0u;

    if (!motor_dip_speed_inited)
    {
        motor_dip_last_fast = dip_speed_fast;
        motor_dip_speed_inited = 1u;
        motor_user_speed_cmd = dip_speed_fast ? 0.0f : 500.0f;
        return;
    }

    if (dip_speed_fast != motor_dip_last_fast)
    {
        motor_dip_last_fast = dip_speed_fast;
        motor_user_speed_cmd = dip_speed_fast ? 0.0f : 500.0f;
    }
}

// 速度环输出，供腿部倾斜角使用
float speed_loop_leg_tilt = 0.0f;

// 横滚控制调试变量，供VOFA查看
float roll_debug_roll = 0;           // euler_angle.roll
float roll_debug_pid_out = 0;        // leg_hight.out
float roll_debug_pid_err = 0;       // leg_hight.error
float roll_debug_desired_left = 0;  // desired_left_p
float roll_debug_desired_right = 0; // desired_right_p
float roll_debug_out_left = 0;      // out_left_p
float roll_debug_out_right = 0;     // out_right_p
float roll_debug_left_offset = 0;    // left_offset
float roll_debug_right_offset = 0;  // right_offset

// 旧版 LQR 转向实验参数，当前普通转向/自旋互斥链路不依赖这些量
float turn_out = 0;
float KP = 5;  // 25.24f;
float KPP = 0; // 0.3805;
float KD = 0;  // 0.2f;
float KDD = 0; // 0.2f

/* 普通转向/自旋共用的差速输出通道拆分：
 * steer_cmd 由外部模块写入普通转向量；
 * spin_cmd  由自旋任务生成；
 * turn_mix_cmd 是最终实际送往左右轮的差速。
 */
float steer_cmd = 0.0f;
float spin_cmd = 0.0f;
float turn_mix_cmd = 0.0f;

/* 普通转向任务的最小状态：
 * steer_enable         标记当前是否正在执行“转到指定航向”的闭环任务；
 * steer_target_yaw_deg 用绝对航向存目标，后续无论菜单测试还是导航都能复用；
 * steer_angle_err      仅用于观察当前还差多少角度，不需要频繁手调。
 */
uint8 steer_enable = 0;
float steer_target_yaw_deg = 0.0f;
float steer_angle_err = 0.0f;
float steer_rate_target_dps = 0.0f;   // 普通转向外环生成的目标角速度，供 VOFA 观察
float steer_rate_meas_dps = 0.0f;     // 普通转向内环的实际角速度反馈，供 VOFA 观察

/* 1ms 控制中断边沿下发：
 * - steer_yaw_request_deg 始终保存最新绝对航向目标；
 * - steer_yaw_request_pending 表示该目标尚未真正进入 steer_set_target_yaw()；
 * - steer_yaw_delayed_by_spin 表示当前因 spin_enable==1 而暂缓，等自旋结束后下一拍再执行。
 */
vuint8 steer_yaw_request_pending = 0;
vuint8 steer_yaw_delayed_by_spin = 0;
volatile float steer_yaw_request_deg = 0.0f;

volatile float remote_lora_steer_rate_cmd_dps = 0.0f;
volatile uint8 remote_lora_steer_snapshot_valid = 0u;

uint8 remote_lora_nav_allows_heading_override(void)
{
    if (N.Nag_SystemRun_Index == 3)
    {
        return 0u;
    }
    if (N.Event_Active)
    {
        return 0u;
    }
    if (N.Nag_Stop_f)
    {
        return 0u;
    }
    return 1u;
}

uint8 remote_lora_nav_allows_spin_request(void)
{
    if (remote_lora_nav_allows_heading_override() == 0u)
    {
        return 0u;
    }
    if (Motor_Switch != MOTOR_ON)
    {
        return 0u;
    }
    if (Motor_Runaway_Latch != 0u)
    {
        return 0u;
    }
    return 1u;
}

/* 普通转向参数先固定成常量，后续确认效果后再决定是否开放到菜单。
 * 这里故意比自旋保守，避免“点一下转向”变成类似原地甩尾的激烈动作。
 */
#define STEER_ANGLE_SETTLE_DEG       3.0f   // 剩余角度进入该窗口后认为已经基本到位
#define STEER_RATE_SETTLE_DPS        6.0f   // 接近目标时，实测角速度也要足够小才允许结束
#define STEER_RATE_TARGET_MAX_DPS   200.0f   // 外环生成的目标角速度上限，限制普通转向的灵敏度
#define STEER_CMD_MAX              1500.0f   // 最终差速限幅，防止普通转向输出过猛影响平衡
#define STEER_SETTLE_COUNT_MAX      20u     // 连续满足收敛条件若干次再结束，避免边界抖动误判

/* 自旋任务参数与调试变量 */
#define SPIN_ANGLE_OUT_MAX_DPS      250.0f  // 单层匀速方案下的固定巡航角速度
#define SPIN_ANGLE_SETTLE_DEG         10.0f  // 剩余角度进入该窗口后开始收转向并准备结束任务
#define SPIN_RATE_SETTLE_DPS         6.0f  // 收转向后，实测角速度低于该值时认为已经基本停住
#define SPIN_SETTLE_COUNT_MAX        80u
#define SPIN_TIMEOUT_BASE_MS       3000u
#define SPIN_TIMEOUT_PER_TURN_MS   4000u
#define SPIN_PITCH_ABORT_DEG         20.0f

uint8 spin_enable = 0;
uint8 spin_done = 0;
int8 spin_dir = 1;
float spin_target_deg = 0.0f;
float spin_accum_deg = 0.0f;
float spin_angle_err = 0.0f;
float spin_rate_target_dps = 0.0f;
float spin_rate_meas_dps = 0.0f;

static float spin_last_yaw = 0.0f;
static uint8 spin_brake_phase = 0;
static uint8 spin_settle_count = 0;
static uint32 spin_timeout_ms = 0;
static uint32 spin_timeout_limit_ms = 0;

static void spin_reset_pid_state(pid_t *pid)
{
    pid->target = 0;
    pid->observation = 0;
    pid->error = 0;
    pid->last_error = 0;
    pid->prev_error = 0;
    pid->integral = 0;
    pid->differential = 0;
    pid->last_differential = 0;
    pid->out = 0;
}

/* 把任意角度包到 [-180, 180]。
 * 绝对航向目标、相对转角换算后的目标，以及误差计算前都统一走这里，
 * 这样跨越 ±180° 时仍能沿最短方向闭环。
 */
static float wrap_yaw_deg(float yaw_deg)
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

/* 普通转向统一收尾：
 * done=1 表示正常转到位，done=0 表示中途取消。
 * 这里同时清空 turn_angle/turn_gyro，避免上一次任务残留状态影响下一次转向。
 */
static void steer_finish(uint8 done)
{
    (void)done;
    steer_enable = 0;
    steer_angle_err = 0.0f;
    steer_cmd = 0.0f;
    turn_mix_cmd = spin_enable ? spin_cmd : 0.0f;
    spin_reset_pid_state(&turn_angle);
    spin_reset_pid_state(&turn_gyro);
}

/* 统一收尾：结束自旋任务并清空双环内部状态。 */
static void spin_finish(uint8 done)
{
    spin_enable = 0;
    spin_done = done;
    spin_rate_target_dps = 0.0f;
    spin_settle_count = 0;
    spin_timeout_ms = 0;
    spin_brake_phase = 0;
    spin_cmd = 0.0f;
    turn_mix_cmd = 0.0f;
    spin_reset_pid_state(&turn_angle);
    spin_reset_pid_state(&turn_gyro);
}

/* 启动自旋任务：turns 为圈数，dir 为方向（正数=沿 yaw 正方向）。 */
void spin_task_start(float turns, int8 dir)
{
    if (turns <= 0.0f)
    {
        spin_finish(0);
        spin_target_deg = 0.0f;
        spin_accum_deg = 0.0f;
        spin_angle_err = 0.0f;
        return;
    }

    spin_dir = (dir >= 0) ? 1 : -1;
    spin_enable = 1;
    spin_done = 0;
    spin_target_deg = turns * 360.0f * (float)spin_dir;
    spin_accum_deg = 0.0f;
    spin_angle_err = spin_target_deg;
    spin_rate_target_dps = 0.0f;
    spin_rate_meas_dps = 0.0f;
    spin_last_yaw = (float)euler_angle.yaw;
    spin_brake_phase = 0;
    spin_settle_count = 0;
    spin_timeout_ms = 0;
    spin_timeout_limit_ms = SPIN_TIMEOUT_BASE_MS + (uint32)(turns * SPIN_TIMEOUT_PER_TURN_MS);
    spin_reset_pid_state(&turn_angle);
    spin_reset_pid_state(&turn_gyro);
    /* 自旋与普通转向互斥：开始自旋时清空普通转向量。 */
    steer_enable = 0;
    steer_angle_err = 0.0f;
    steer_cmd = 0.0f;
    spin_cmd = 0.0f;
    turn_mix_cmd = 0.0f;
}

/* 手动停止自旋任务，保留平衡控制但清空本次自旋目标。 */
void spin_task_stop(void)
{
    spin_target_deg = 0.0f;
    spin_accum_deg = 0.0f;
    spin_angle_err = 0.0f;
    spin_finish(0);
}

/* 设置绝对航向目标：
 * 1. 这是“立即执行”接口，若当前在自旋，会直接停掉自旋并切入普通转向；
 * 2. 约定 target_yaw_deg 使用 [-180, 180] 度，函数内部仍会做一次包角保护；
 * 3. 不要在周期里重复调用，否则会不断刷新任务状态，影响闭环收敛。
 */
void steer_set_target_yaw(float target_yaw_deg)
{
    float curr_yaw = (float)euler_angle.yaw;
    float target_yaw = wrap_yaw_deg(target_yaw_deg);
    float target_err = (float)ange_deviation1(target_yaw, curr_yaw);

    /* 立即执行接口优先级最高：一旦直接调用，就认为旧的请求式目标已经失效，
     * 统一清掉 pending/delayed，避免自旋结束后又把过期请求重新执行一遍。
     */
    steer_yaw_request_pending = 0;
    steer_yaw_delayed_by_spin = 0;

    if (ABS(target_err) <= 0.001f)
    {
        steer_finish(0);
        steer_target_yaw_deg = target_yaw;
        return;
    }

    /* 普通转向与自旋互斥：绝对航向任务启动前先退出自旋。 */
    if (spin_enable)
    {
        spin_task_stop();
    }

    steer_enable = 1;
    steer_target_yaw_deg = target_yaw;
    steer_angle_err = target_err;
    steer_cmd = 0.0f;
    turn_mix_cmd = 0.0f;
    /* 每次新任务都清空双环内部状态，避免上次积分/微分残留导致一上电就猛打。 */
    spin_reset_pid_state(&turn_angle);
    spin_reset_pid_state(&turn_gyro);
}

/* 登记最新绝对航向请求：
 * 1. steer_yaw_request_deg 始终保留最新目标，新请求会覆盖旧请求，不排队；
 * 2. 若自旋在跑，pit0_ch0_isr 不会立刻调用 steer_set_target_yaw()，而是把它延迟到自旋结束；
 * 3. 一旦请求真正被 1ms ISR 执行，会统一清掉 pending/delayed 标志，避免旧请求残留或重复触发。
 */
void steer_request_target_yaw(float target_yaw_deg)
{
    steer_yaw_request_deg = target_yaw_deg;
    steer_yaw_request_pending = 1;
    steer_yaw_delayed_by_spin = 0;
}

/* 登记相对转角请求：
 * 1. 语义与 steer_task_start(delta_deg) 一致，仍表示“在当前朝向基础上再转多少度”；
 * 2. 区别是这里只登记请求，真正的 steer_set_target_yaw() 由 1ms ISR 在安全时机执行；
 * 3. 当前航向 + 相对角度 的换算统一放在控制层，避免菜单/导航各自重复实现一套。
 */
void steer_request_relative_yaw(float delta_deg)
{
    float curr_yaw = (float)euler_angle.yaw;

    if (delta_deg == 0.0f)
    {
        steer_request_target_yaw(curr_yaw);
        return;
    }

    steer_request_target_yaw(curr_yaw + delta_deg);
}

/* 启动相对转角任务：
 * delta_deg 是“在当前朝向基础上再转多少度”，因此只适合触发一次。
 * 该接口会同步调用 steer_set_target_yaw()，主要保留给旧逻辑兼容；
 * 若希望与 1ms 控制链路时序保持一致，推荐改用 steer_request_relative_yaw()。
 */
void steer_task_start(float delta_deg)
{
    float curr_yaw = (float)euler_angle.yaw;

    if (delta_deg == 0.0f)
    {
        steer_finish(0);
        steer_target_yaw_deg = curr_yaw;
        return;
    }

    steer_set_target_yaw(curr_yaw + delta_deg);
}

/* 手动停止普通转向任务，但不影响平衡控制主链路。 */
void steer_task_stop(void)
{
    steer_finish(0);
}

/* 设置普通转向差速；自旋开启时该值会被暂时忽略。 */
void set_steer_cmd(float cmd)
{
    steer_cmd = cmd;
}

/*=============================================================================
 * 舵机/腿控制参数（leg_control在pit0_ch1 5ms周期执行）
 *
 * 参数分类速查：
 *   横滚角：roll_balance_en, ROLL_LEG_SCALE, ROLL_LEG_OFFSET_MAX, roll_mid, leg_hight, dt_leg
 *   俯仰角：LEG_TILT_K, LEG_TILT_MAX, LEG_SERVO_SPEED_TILT_EN
 *   跳跃：  JUMP_* 系列, jump_control_config
 *   通用：  LEG_STEP_P_MAX, LEG_STEP_ANGLE_MAX, LEG_P_MIN/MAX
 *=============================================================================*/

/*---------- 通用腿/舵机参数 ----------*/
#define LEG_P_MIN           2.4f   // 腿长下限
#define LEG_P_MAX          14.5f   // 腿长上限
#define LEG_STEP_P_MAX      0.2f   // 每5ms腿高最大变化（步进限幅，越大响应越快）
#define LEG_STEP_ANGLE_MAX  0.5f   // 每5ms腿部倾角最大变化(度)
#define LEG_RIGHT_ANGLE_INVERT  1   // 右腿俯仰取反(左右镜像)，若方向反则改0

/*---------- 横滚角参数（只抬腿不收腿，抬腿侧给占空比）----------*/
uint8 roll_balance_en = 0;  // 运行时可改：1开启横滚平衡，0关闭（左右腿保持leg_long）
#define ROLL_LEG_SCALE          1.0f  // 横滚PID输出→腿长增量缩放，越大抬腿越猛
#define ROLL_LEG_OFFSET_MAX      8.5f  // 单侧腿长增量上限，防止过度抬腿
// dt_leg、leg_hight PID 见上方变量及 pid_ctrl_Init()

/*---------- 俯仰角参数（速度环→腿倾角，与横滚并级）----------*/
#define LEG_SERVO_SPEED_TILT_EN  1     // 置0关闭速度环→舵机倾角
#define LEG_TILT_K              0.016f // 速度环输出→腿倾角缩放系数
#define LEG_TILT_MAX             20.0f // 腿倾角限幅±20°

/*---------- 跳跃参数（障碍跨越）----------*/
#define JUMP_PID_SCALE          0.5f  // 跳跃时angle/speed的kp缩放，维持稳定
#define JUMP_TAKEOFF_P          13.0f // 起跳爆发目标腿长（直通伸腿）
#define JUMP_PREPARE_P          10.0f // 准备缓冲目标腿长（起跳后伸腿高度）
#define JUMP_BUFFER_P           5.5f  // 执行缓冲最终腿长（落地收腿高度）
#define JUMP_BUFFER_STEP_P_MAX  0.2f  // 执行缓冲时每5ms腿高最大变化
#define JUMP_BUFFER_STEP_PER_20MS  (JUMP_BUFFER_STEP_P_MAX * 4)  // 每20ms步进（4次5ms）
#define JUMP_BUFFER_MARGIN      2     // 缓冲周期余量
#define JUMP_BUFFER_CYCLES  ((int)(((JUMP_PREPARE_P - JUMP_BUFFER_P) / JUMP_BUFFER_STEP_PER_20MS) + 0.999f) + JUMP_BUFFER_MARGIN)

/* 跳跃时序（jump_control在pit0_ch10 20ms周期，单位=20ms）*/
const jump_control_struct jump_control_config[] =
    {
        {0,  5,  jump_set_step, "起跳"},           //  伸腿爆发
        {5,  9, jump_set_step, "准备缓冲"},       //  过渡姿态
        {9, 9 + JUMP_BUFFER_CYCLES - 1, jump_set_step, "执行缓冲"},  // 落地收腿
};
const uint8 jump_step_num = sizeof(jump_control_config) / sizeof(jump_control_struct);

// 导航相关全局变量
double victual_point_lat[] = {0};           //虚拟点纬度数组
double victual_point_lon[] = {0};           //虚拟点经度数组
uint8 Temp_num = 0;                       //当前目标点索引
double Angle_Z_Quaternions = 0;           //当前航向角（四元数计算）

/*-------------------------------------------------------------------------------------------------------------------
// 函数简介     PID控制初始化
// 参数说明     null
// 返回参数     null
// 使用示例     pid_ctrl_Init();
// 备注信息     总初始化调用
-------------------------------------------------------------------------------------------------------------------*/
void pid_ctrl_Init(void)
{
    // pid_init(&turn, 1.0087, 15, 0, 0.01, 0, 0, 0, 5000, Position_pid);
    /* 横滚角位置式PID：目标roll_mid，反馈euler_angle.roll，输出→只抬腿不收腿 */
    pid_init(&leg_hight, 0.25f, 0.12f, 0.0, dt_leg, 500, 0, 0, 50, Position_pid);
    /* 自旋控制当前采用单层固定巡航角速度；
     * turn_gyro 用于跟踪目标角速度；
     * turn_angle 用于普通转向外环；dt_pid_turn_angle 已与 pit0_ch0 的 1ms 周期对齐。
     */
    pid_init(&turn_angle, 20.0f, 2.0f, 0.0f, dt_pid_turn_angle, 0, 0, 0, SPIN_ANGLE_OUT_MAX_DPS, Position_pid);
    pid_init(&turn_gyro, 30.0f, 2.0f, 0.0f, dt_pid_turn_gyro, 0, 0, 0, 2200, Position_pid);
    // pid_init(&turn, 1.87, 19, 0, 0.01, 0, 0, 0, 5000, Position_pid);
     pid_init(&gyro, 1.1, 0, 0, 0.002, 0, 0, 0, 10000, Position_pid);
     pid_init(&angle, 500.0, 0, 0, 0.01, 0, 0, 0, 10000, Position_pid);
     pid_init(&speed, 3.0, 0.001, 0.01, 0.02, 0, 0, 0, 10000, Position_pid);//3.0
    //pid_init(&turn, 0.01, 0.0000667, 0, 0.02, 0, 0, 0, 10000, Position_pid);
    pid_set_target(&leg_hight, roll_mid);  // 横滚目标=机械零点
    pid_set_target(&speed, 0);
}

/*-------------------------------------------------------------------------------------------------------------------
// 函数简介     调试模式下用 leg_long 初始化 pwm_ph1~4，使腿高从 5.5 起步
// 参数说明     null
// 返回参数     null
// 使用示例     leg_debug_init_pwm();
// 备注信息     servo_init 中调用，仅 LEG_DEBUG_MODE=1 时有效
-------------------------------------------------------------------------------------------------------------------*/
void leg_debug_init_pwm(void)
{
#if LEG_DEBUG_MODE
    servo_control_table(leg_long, 0, &pwm_ph4, &pwm_ph3);
    servo_control_table(leg_long, 0, &pwm_ph1, &pwm_ph2);
#endif
}

/*-------------------------------------------------------------------------------------------------------------------
// 函数简介     LQR控制平衡和行驶
// 参数说明     V_target    设定速度, 真实速度 m/s
              th          机械零点
// 返回参数     null
// 使用示例     LQR_control(2, pitch_mid);
// 备注信息     isr中断调用
-------------------------------------------------------------------------------------------------------------------*/
void LQR_control(float V_target, float th)
{
    // static uint16 pid_time_turn = 0;
    float L_min = 0.0353019121;
    float TL = 0, TR = 0;
    static float x_hat_last = 0;
    static float last_image_error = 0;
    float Tangle = -(euler_angle.pitch - th) / DEG_TO_RAD;
    float gy = -imu660rc_gyro_transition(imu660rc_gyro_y) / DEG_TO_RAD;

    float v_t = (v_hat - V_target);

    // TL = LQR_K[3] * gy + LQR_K[2] * Tangle + LQR_K[1] * v_t;
    // TR = LQR_K[7] * gy + LQR_K[6] * Tangle + LQR_K[5] * v_t;

    TL = LQR_K[3] * gy + LQR_K[2] * Tangle + LQR_K[1] * v_t + LQR_K[0] * ((x_hat + L_min * sin(euler_angle.pitch / DEG_TO_RAD) - (x_hat_last + v_hat)));
    TR = LQR_K[7] * gy + LQR_K[6] * Tangle + LQR_K[5] * v_t + LQR_K[4] * ((x_hat + L_min * sin(euler_angle.pitch / DEG_TO_RAD) - (x_hat_last + v_hat)));

    x_hat_last = x_hat;

    // 下面这段是旧版 LQR 转向实验残留，当前 pid_ctrl_Run() 主链路不使用
    //    pid_set_target(&turn_gyro, 0);
    //
    //    // 设置角速度环观测值(Z轴角速度)
    //    pid_get_observation(&turn_gyro, imu660rc_gyro_transition(imu660rc_gyro_z));
    //    get_timer(TOM0_CH4, &pid_time_turn, &dt_pid_turn_gyro);
    //    pid_set_dt(&turn_gyro, dt_pid_turn_gyro);
    //    pid_run(&turn_gyro);
    //    Err = 0.5;

    // float turn_out = KP * image_error + ABS(image_error) * image_error * KPP + KD * (image_error - last_image_error) - imu660rc_gyro_z * KDD;
    // last_image_error = image_error;

    //   turn_out = 0;

    int16 LO = (int16)(Lmoto_K * TL - turn_out);
    int16 RO = (int16)(Rmoto_K * TR + turn_out);

    Left_Motor_Pwm = LO;
    Right_Motor_Pwm = RO;

    // 死区补偿
    dead_compensate(&LO, &RO);

    if (Motor_Switch)
    {
        if (jump_flag != 1)
        {
            if (30 >= euler_angle.pitch)
            {
                small_driver_set_duty(LO, -RO);
            }
            if (euler_angle.pitch >= -50)
            {
                small_driver_set_duty(LO, -RO);
            }
            else
            {
                small_driver_set_duty(0, 0);
            }
        }
    }
    else
    {
        small_driver_set_duty(0, 0);
    }
}

/*-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取当前普通转向差速指令
// 参数说明     image_error    当前未使用，仅为兼容旧接口保留
// 返回参数     steer_cmd      当前普通转向差速指令
// 使用示例     float cmd = turn_control(0);
// 备注信息     普通转向的实际写入口为 set_steer_cmd()，自旋开启时该返回值不会参与最终差速输出
-------------------------------------------------------------------------------------------------------------------*/
float turn_control(float image_error)
{
    (void)image_error;
    return steer_cmd;
}

/*-------------------------------------------------------------------------------------------------------------------
// 函数简介     PID控制平衡和行驶
// 参数说明     null
// 返回参数     null
// 使用示例     pid_ctrl_Run();
// 备注信息     isr中断调用
-------------------------------------------------------------------------------------------------------------------*/
void pid_ctrl_Run(void)
{
    static uint16 pid_time_gyro = 0;
    static uint16 pid_time_angle = 0;
    static uint16 pid_time_speed = 0;
    static uint16 pid_time_turn = 0;
    static uint32 timer_flag = 0;
    static float Angle_Out = 0;
    imu660rc_get_gyro();

    if (0 == timer_flag) // 速度环（20ms，与 car_speed 更新节拍保持一致）
    {
        /* 速度环入口：motor_user_speed_cmd 为用户基准；speed_target_effective 为导航门控+弯道/元素限速后目标。
         * 调试建议同时观察 motor_user_speed_cmd / speed_target_effective / car_speed。
         */
        speed_target_effective = Nag_GetControlSpeedTarget();
        pid_set_target(&speed, -speed_target_effective);
        pid_get_observation(&speed, -motor_value.receive_left_speed_data + motor_value.receive_right_speed_data);

        pid_set_dt(&speed, dt_pid_speed);
        pid_run(&speed);
        speed_loop_leg_tilt = (jump_flag == 1) ? (speed.out * JUMP_PID_SCALE) : speed.out;
    }

    if (spin_enable)
    {
        /* 自旋时不再使用速度环输出驱动腿部前后倾，避免和原地旋转任务打架。 */
        pid_set_target(&speed, 0);
        speed_loop_leg_tilt = 0.0f;
    }

    if (0 == timer_flag % 5) // 角度环
    {
        pid_set_target(&angle, pitch_mid);  //pitch_mid - speed.out
        pid_get_observation(&angle, euler_angle.pitch);

        pid_set_dt(&angle, dt_pid_angle);
        pid_run(&angle);
        Angle_Out = angle.out + angle_kd * imu660rc_gyro_y * dt_pid_angle;
        if (jump_flag == 1)
            Angle_Out *= JUMP_PID_SCALE;
    }

    // 角速度环
    pid_set_target(&gyro, Angle_Out);
    pid_get_observation(&gyro, imu660rc_gyro_y);
    pid_set_dt(&gyro, dt_pid_gyro);
    pid_run(&gyro);

    /* 自旋控制：
     * 1. 用相邻yaw增量累计总角度，跨 ±180° 时靠 ange_deviation1 解包。
     * 2. 远离目标时固定角速度巡航，接近目标后直接把目标角速度收为 0。
     * 3. 内环只负责把实际 gyro_z 跟踪到目标角速度。
     */
    spin_rate_meas_dps = imu_data.gyro_z * DEG_TO_RAD;
    if (spin_enable)
    {
        if (Motor_Switch)
        {
            float curr_yaw = (float)euler_angle.yaw;
            float delta_yaw = (float)ange_deviation1(curr_yaw, spin_last_yaw);
            float abs_spin_err = 0.0f;
            spin_last_yaw = curr_yaw;
            spin_accum_deg += delta_yaw;
            spin_angle_err = spin_target_deg - spin_accum_deg;
            abs_spin_err = ABS(spin_angle_err);

            if (!spin_brake_phase)
            {
                if ((spin_target_deg >= 0.0f && spin_angle_err <= 0.0f) ||
                    (spin_target_deg < 0.0f && spin_angle_err >= 0.0f) ||
                    abs_spin_err <= SPIN_ANGLE_SETTLE_DEG)
                {
                    spin_brake_phase = 1;
                }
            }

            if (spin_brake_phase)
            {
                spin_rate_target_dps = 0.0f;
            }
            else
            {
                float spin_err_sign = (spin_angle_err >= 0.0f) ? 1.0f : -1.0f;
                spin_rate_target_dps = spin_err_sign * SPIN_ANGLE_OUT_MAX_DPS;
            }

            pid_set_target(&turn_gyro, spin_rate_target_dps);
            pid_get_observation(&turn_gyro, spin_rate_meas_dps);
            pid_set_dt(&turn_gyro, dt_pid_turn_gyro);
            pid_run(&turn_gyro);
            spin_cmd = turn_gyro.out;

            spin_timeout_ms++;
            if (ABS(euler_angle.pitch - pitch_mid) > SPIN_PITCH_ABORT_DEG || spin_timeout_ms > spin_timeout_limit_ms)
            {
                spin_finish(0);
            }
            else if (spin_brake_phase && ABS(spin_rate_meas_dps) < SPIN_RATE_SETTLE_DPS)
            {
                if (++spin_settle_count >= SPIN_SETTLE_COUNT_MAX)
                {
                    spin_finish(1);
                }
            }
            else
            {
                spin_settle_count = 0;
            }
        }
        else
        {
            spin_last_yaw = (float)euler_angle.yaw;
            spin_rate_target_dps = 0.0f;
            spin_cmd = 0.0f;
        }
    }
    else
    {
        spin_angle_err = spin_target_deg - spin_accum_deg;
        spin_rate_target_dps = 0.0f;
        spin_cmd = 0.0f;
    }

    if (!spin_enable &&
        (remote_lora_steer_snapshot_valid != 0u) &&
        (remote_lora_nav_allows_heading_override() != 0u))
    {
        /* LORA 横向：直接给目标偏航角速度，松杆为 0，不维护绝对航向目标 */
        if (steer_enable != 0u)
        {
            steer_task_stop(); /* 仅退出航向闭环时清一次，避免每拍 reset turn_gyro */
        }

        steer_rate_meas_dps = imu_data.gyro_z * DEG_TO_RAD;
        steer_rate_target_dps = clip(
            remote_lora_steer_rate_cmd_dps,
            -STEER_RATE_TARGET_MAX_DPS,
            STEER_RATE_TARGET_MAX_DPS);

        pid_set_target(&turn_gyro, steer_rate_target_dps);
        pid_get_observation(&turn_gyro, steer_rate_meas_dps);
        pid_set_dt(&turn_gyro, dt_pid_turn_gyro);
        pid_run(&turn_gyro);
        set_steer_cmd(clip(turn_gyro.out, -STEER_CMD_MAX, STEER_CMD_MAX));
    }
    else if (!spin_enable && steer_enable)
    {
        static uint8 steer_settle_count = 0;
        steer_rate_meas_dps = imu_data.gyro_z * DEG_TO_RAD;
        steer_rate_target_dps = 0.0f;

        /* 普通转向用“目标航向 - 当前航向”的归一化误差做外环输入。 */
        steer_angle_err = (float)ange_deviation1(steer_target_yaw_deg, euler_angle.yaw);

        /* 外环：航向误差 -> 目标角速度。 */
        pid_set_target(&turn_angle, 0.0f);
        pid_get_observation(&turn_angle, -steer_angle_err);
        pid_set_dt(&turn_angle, dt_pid_turn_angle);
        pid_run(&turn_angle);
        steer_rate_target_dps = clip(turn_angle.out, -STEER_RATE_TARGET_MAX_DPS, STEER_RATE_TARGET_MAX_DPS);

        /* 内环：目标角速度 -> 左右轮差速输出。 */
        pid_set_target(&turn_gyro, steer_rate_target_dps);
        pid_get_observation(&turn_gyro, steer_rate_meas_dps);
        pid_set_dt(&turn_gyro, dt_pid_turn_gyro);
        pid_run(&turn_gyro);
        set_steer_cmd(clip(turn_gyro.out, -STEER_CMD_MAX, STEER_CMD_MAX));

        /* 角度和角速度都进入收敛窗口后，再连续确认若干个周期再结束，
         * 可以避免刚到目标附近时因为摆头/噪声导致“到位-没到位”反复抖动。
         */
        if (ABS(steer_angle_err) <= STEER_ANGLE_SETTLE_DEG &&
            ABS(steer_rate_meas_dps) <= STEER_RATE_SETTLE_DPS)
        {
            if (++steer_settle_count >= STEER_SETTLE_COUNT_MAX)
            {
                steer_finish(1);
                steer_settle_count = 0;
            }
        }
        else
        {
            steer_settle_count = 0;
        }
    }
    else if (!spin_enable)
    {
        steer_rate_meas_dps = imu_data.gyro_z * DEG_TO_RAD;
        steer_rate_target_dps = 0.0f;
        steer_angle_err = (float)ange_deviation1(steer_target_yaw_deg, euler_angle.yaw);
        if (!steer_enable)
        {
            steer_cmd = 0.0f;
        }
    }

    /* 互斥选择最终差速：
     * - 自旋开启时只允许 spin_cmd 生效；
     * - 非自旋时只允许 steer_cmd 生效。
     */
    turn_mix_cmd = spin_enable ? spin_cmd : steer_cmd;

    if(Motor_Switch)
    {
        if ((-motor_value.receive_left_speed_data + motor_value.receive_right_speed_data) / 2 > 3000 || (-motor_value.receive_left_speed_data + motor_value.receive_right_speed_data) / 2 < -3000)
        {
            Motor_Switch = 0;
            Motor_Runaway_Latch = 1;
            jump_stop();
         }
        else
        {
            float scale = (jump_flag == 1) ? JUMP_PID_SCALE : 1.0f;
            small_driver_set_duty((int16)((gyro.out + turn_mix_cmd) * scale), (int16)(-(gyro.out - turn_mix_cmd) * scale));
        }
    }
    else
    {
        small_driver_set_duty(0, 0);
    }

    
    timer_flag = (timer_flag + 1) % 20;
}

/*-------------------------------------------------------------------------------------------------------------------
// 函数简介     舵机步进更新，根据jump_flag步进或直通
// 备注信息     leg_control内部调用
-------------------------------------------------------------------------------------------------------------------*/
static void leg_servo_step_update(float desired_left_p, float desired_right_p, float desired_angle,
                                  float *out_left_p, float *out_right_p,
                                  float *out_left_angle, float *out_right_angle)
{
    static float current_left_p = 0;
    static float current_right_p = 0;
    static float current_left_angle = 0;
    static float current_right_angle = 0;
    static uint8 first_run = 1;

    if (first_run)
    {
        current_left_p = desired_left_p;
        current_right_p = desired_right_p;
        current_left_angle = desired_angle;
        current_right_angle = desired_angle;
        first_run = 0;
    }

    /* use_step: 1=步进逼近, 0=直通。非跳跃或执行缓冲(step2)用步进，起跳/准备缓冲直通 */
    uint8 use_step = 0;
    if (jump_flag == 0)
        use_step = 1;    /* 非跳跃：步进 */
    else if (jump_step_index == 2)
        use_step = 1;    /* 执行缓冲(step2)：步进，实现缓慢收腿 */
    /* else: 起跳(step0)/准备缓冲(step1)：直通 */

    if (use_step)
    {
        float step_p = (jump_step_index == 2) ? JUMP_BUFFER_STEP_P_MAX : LEG_STEP_P_MAX;
        float delta;
        delta = desired_left_p - current_left_p;
        current_left_p += clip2(delta, step_p);
        delta = desired_right_p - current_right_p;
        current_right_p += clip2(delta, step_p);
        delta = desired_angle - current_left_angle;
        current_left_angle += clip2(delta, LEG_STEP_ANGLE_MAX);
        delta = desired_angle - current_right_angle;
        current_right_angle += clip2(delta, LEG_STEP_ANGLE_MAX);
    }
    else
    {
        current_left_p = desired_left_p;
        current_right_p = desired_right_p;
        current_left_angle = desired_angle;
        current_right_angle = desired_angle;
    }

    /* 腿长/倾角限幅后输出 */
    current_left_p = clip(current_left_p, LEG_P_MIN, LEG_P_MAX);
    current_right_p = clip(current_right_p, LEG_P_MIN, LEG_P_MAX);

    *out_left_p = current_left_p;
    *out_right_p = current_right_p;
    *out_left_angle = current_left_angle;
    *out_right_angle = current_right_angle;
}

/*-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取期望腿部倾斜角（速度环输出映射）
// 备注信息     LEG_SERVO_SPEED_TILT_EN=0时恒返回0
-------------------------------------------------------------------------------------------------------------------*/
static float leg_servo_get_desired_tilt_angle(void)
{
#if LEG_SERVO_SPEED_TILT_EN
    // 车向前→腿后倾，取反使极性正确
    float a = LEG_TILT_K * speed_loop_leg_tilt;
    return clip(a, -LEG_TILT_MAX, LEG_TILT_MAX);
#else
    return 0.0f;
#endif
}

/*-------------------------------------------------------------------------------------------------------------------
// 函数简介     控制腿高
// 参数说明     null
// 返回参数     null
// 使用示例     leg_control();
// 备注信息     isr中断调用, 针对单边桥使用
-------------------------------------------------------------------------------------------------------------------*/
void leg_control(void)
{
    float desired_left_p, desired_right_p;
    if (jump_flag == 1)
    {
        desired_left_p = desired_right_p = leg_long;
        roll_debug_left_offset = 0;
        roll_debug_right_offset = 0;
    }
    else if (roll_balance_en)
    {
        /* 横滚平衡开启：只抬腿不收腿，仅抬腿侧有腿长增量 */
        pid_get_observation(&leg_hight, euler_angle.roll);
        pid_set_dt(&leg_hight, dt_leg);
        pid_run(&leg_hight);

        float roll_angle_out = leg_hight.out;
        float left_offset, right_offset;
        if (roll_angle_out > 0)
        {
            left_offset = 0;
            right_offset = clip2(ROLL_LEG_SCALE * roll_angle_out, ROLL_LEG_OFFSET_MAX);
        }
        else
        {
            left_offset = clip2(ROLL_LEG_SCALE * (-roll_angle_out), ROLL_LEG_OFFSET_MAX);
            right_offset = 0;
        }
        desired_left_p = leg_long + left_offset;
        desired_right_p = leg_long + right_offset;
        roll_debug_left_offset = left_offset;
        roll_debug_right_offset = right_offset;
        roll_debug_pid_out = leg_hight.out;
        roll_debug_pid_err = leg_hight.error;
    }
    else
    {
        /* 横滚平衡关闭：左右腿均保持 leg_long */
        desired_left_p = desired_right_p = leg_long;
        roll_debug_left_offset = 0;
        roll_debug_right_offset = 0;
        roll_debug_pid_out = 0;
        roll_debug_pid_err = 0;
    }

    roll_debug_roll = euler_angle.roll;
    roll_debug_desired_left = desired_left_p;
    roll_debug_desired_right = desired_right_p;

    float desired_angle = leg_servo_get_desired_tilt_angle();

    float out_left_p, out_right_p, out_left_angle, out_right_angle;
    leg_servo_step_update(desired_left_p, desired_right_p, desired_angle,
                          &out_left_p, &out_right_p, &out_left_angle, &out_right_angle);

    roll_debug_out_left = out_left_p;
    roll_debug_out_right = out_right_p;

    // 俯仰倾斜时左右腿镜像，需对一侧取反使左右同向
#if LEG_RIGHT_ANGLE_INVERT
    left_leg_control(out_left_p, out_left_angle);
    right_leg_control(out_right_p, -out_right_angle);
#else
    left_leg_control(out_left_p, -out_left_angle);
    right_leg_control(out_right_p, out_right_angle);
#endif
}

/*-------------------------------------------------------------------------------------------------------------------
// 函数简介     执行跳跃内容
// 参数说明     step_num    执行目标
// 返回参数     null
// 使用示例     jump_set_step(step_num);
// 备注信息     jump_control函数中调用, 针对障碍使用
-------------------------------------------------------------------------------------------------------------------*/
void jump_set_step(int step_num)
{
    switch (step_num)
    {
    case 0:
        leg_long = JUMP_TAKEOFF_P;  // 起跳：直接爆发伸腿
        break;
    case 1:
        leg_long = JUMP_PREPARE_P;  // 准备缓冲：直通到中间姿态
        break;
    case 2:
        leg_long = JUMP_BUFFER_P;   // 执行缓冲：步进收腿到落地
        break;
    default:
        break;
    }
}

/*-------------------------------------------------------------------------------------------------------------------
// 函数简介     控制跳跃
// 参数说明     null
// 返回参数     null
// 使用示例     jump_control();
// 备注信息     isr中断调用, 针对障碍使用
// 补充说明     本函数只负责在 jump_flag==1 时按 jump_control_config[] 推进腿姿态与时序；
//              何时置 jump_flag（例如普通模式视觉下沿消失）由其它模块决定，勿在此处做视觉判定。
-------------------------------------------------------------------------------------------------------------------*/
void jump_control(void)
{
    if (jump_flag == 1)
    {
        if (jump_is_allowed() == 0u)
        {
            jump_stop();
            return;
        }

        jump_time++;

        if (jump_time < jump_control_config[jump_step_num - 1].max)
        {
            for (int i = 0; i < jump_step_num; i++)
            {
                if (jump_time >= jump_control_config[i].min && jump_time <= jump_control_config[i].max)
                {
                    jump_control_config[i].handler(i);
                    jump_step_index = (uint8)i; // 更新当前跳跃步

                    // ips200_show_int(0,0,i,3);
                    break;
                }
            }
        }
        else
        {
            jump_stop();
        }
    }
}

/*-------------------------------------------------------------------------------------------------------------------
// 函数简介     死区补偿
// 参数说明     *input_L    左电机
              *input_R    右电机
// 返回参数     null
// 使用示例     dead_compensate(&input_L, &input_R);
// 备注信息     LQR函数中调用
-------------------------------------------------------------------------------------------------------------------*/
void dead_compensate(int16 *input_L, int16 *input_R)
{
    if (*input_L > 0)
    {
        *input_L = clip2(*input_L + L_dead_zone_correct, 10000);
    }
    else if (*input_L < 0)
    {
        *input_L = clip2(*input_L + L_dead_zone_negative, 10000);
    }
    else
    {
        *input_L = 0;
    }
    if (*input_R > 0)
    {
        *input_R = clip2(*input_R + R_dead_zone_correct, 10000);
    }
    else if (*input_R < 0)
    {
        *input_R = clip2(*input_R + R_dead_zone_negative, 10000);
    }
    else
    {
        *input_R = 0;
    }
}

/*-------------------------------------------------------------------------------------------------------------------
// 函数简介     控制左腿
// 参数说明     p       腿高
             angle   角度
// 返回参数     null
// 使用示例     left_leg_control(5.5, 0);
// 备注信息     控制腿高函数调用
-------------------------------------------------------------------------------------------------------------------*/
void left_leg_control(float p, float angle)
{
#if !LEG_DEBUG_MODE
    // 调用五连杆姿态解算函数
    servo_control_table(p, -angle, &pwm_ph4, &pwm_ph3);
#endif
    // 边界检查，确保PWM值在合理范围内
      if(10000 == pwm_ph3 || 10000 == pwm_ph4)
    {
        ASSERT(10000 == pwm_ph4 || 10000 == pwm_ph3);   
        return;
    }

    pwm_set_duty(SERVO_3, SERVO3_MID - pwm_ph3);
    pwm_set_duty(SERVO_4, SERVO4_MID + pwm_ph4);
}

/*-------------------------------------------------------------------------------------------------------------------
// 函数简介     控制右腿
// 参数说明     p       腿高
              angle   角度
// 返回参数     null
// 使用示例     right_leg_control(5.5, 0);
// 备注信息     控制腿高函数调用
-------------------------------------------------------------------------------------------------------------------*/
void right_leg_control(float p, float angle)
{
#if !LEG_DEBUG_MODE
    // 调用五连杆姿态解算函数
    servo_control_table(p, -angle, &pwm_ph1, &pwm_ph2);
#endif
    // 边界检查，确保PWM值在合理范围内
   if(10000 == pwm_ph1 || 10000 == pwm_ph2)
    {
        ASSERT(10000 == pwm_ph2 || 10000 == pwm_ph1);
        return;
    }

    pwm_set_duty(SERVO_1, SERVO1_MID + pwm_ph1);
    pwm_set_duty(SERVO_2, SERVO2_MID - pwm_ph2);

    pwm_4 = pwm_ph2; // 测试中值
    pwm_1 = pwm_ph1;
}

/*-------------------------------------------------------------------------------------------------------------------
// 函数简介     计算方位角
// 参数说明     X_now    当前X坐标
//              Y_now    当前Y坐标
//              X_next   目标X坐标
//              Y_next   目标Y坐标
// 返回参数     方位角（度）
// 使用示例     double angle = get_fang_wei_jiao(0, 0, 1, 1);
// 备注信息     计算从当前点到目标点的方位角
-------------------------------------------------------------------------------------------------------------------*/
double get_fang_wei_jiao(double X_now, double Y_now, double X_next, double Y_next)
{
    double X_err, Y_err, angle;
    
    X_err = X_next - X_now;  // X方向距离
    Y_err = Y_next - Y_now;  // Y方向距离
    
    // 计算方位角（考虑四个象限）
    if(X_err == 0)
    {
        if(Y_err > 0) return 90;
        if(Y_err < 0) return 270;
    }
    
    angle = atan(Y_err / X_err);  // 反正切计算角度
    angle = angle / PI * 180;  // 弧度转角度
    
    // 象限判断
    if(angle > 0)  // 1,3象限
    {
        if(Y_err > 0) return angle;
        else if(Y_err < 0) return angle + 180;
    }
    else if(angle < 0)  // 2,4象限
    {
        if(Y_err > 0) return angle + 180;
        else if(Y_err < 0) return angle + 360;
    }
    
    return 0;
}

/*-------------------------------------------------------------------------------------------------------------------
// 函数简介     航向角偏差归一化
// 参数说明     angel1    参与计算的第一个角度
//              angel2    参与计算的第二个角度
// 返回参数     归一化后的角度偏差（-180°到180°）
// 使用示例     double deviation = ange_deviation1(90, 45);
// 备注信息     返回 wrap(angel1 - angel2)
//              普通转向里使用 ange_deviation1(target, current)，即“目标航向 - 当前航向”的最短路径误差
-------------------------------------------------------------------------------------------------------------------*/
double ange_deviation1(double angel1, double angel2)
{
    double x = angel1 - angel2;

    // 归一化到 [-180°, 180°]
    while (x > 180.0)
    {
        x -= 360.0;
    }
    while (x < -180.0)
    {
        x += 360.0;
    }

    return x;
}

/*-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取最终角度
// 参数说明     null
// 返回参数     最终角度（归一化后的航向偏差）
// 使用示例     float angle = Get_Final_Angle();
// 备注信息     计算当前位置到目标点的航向偏差
-------------------------------------------------------------------------------------------------------------------*/
float Get_Final_Angle(void)
{
    // 步骤1：计算目标航向角
    float direction;
    direction = get_fang_wei_jiao(TempLat_Now, TempLon_Now,
                                  victual_point_lat[Temp_num],
                                  victual_point_lon[Temp_num]);
    
    // 步骤2：计算航向角误差
    float final_angle = 0;
    final_angle = ange_deviation1(euler_angle.yaw, direction);
    
    return final_angle;
}


//*********************************************************************************************************
//惯性导航部分代码
//*********************************************************************************************************
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取车子在xy轴上的位置。（x轴为正北方向）
// 参数说明     void
// 返回参数     void
// 使用示例     get_car_xy（）;
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void get_car_xy(void)
{
        /* 若后续重新启用二维惯导，这里的积分周期和速度单位必须与 navigation.c 保持一致，
         * 不要再单独使用另一套“经验 dt/比例系数”，否则会出现导航里程和 xy 位置各算各的。
         * 当前仍保留旧式速度投影，仅把 dt 改成与 Nag_System() 相同的 1ms 节拍。
         */
        double speed_x = 0 , speed_y = 0;
        speed_x = car_speed * cos(euler_angle.yaw/180.0*3.1415926);
        speed_y = car_speed * sin(euler_angle.yaw/180.0*3.1415926);
        ins.distance_x += speed_x * Nag_Sample_Dt * 1.9;
        ins.distance_y += speed_y * Nag_Sample_Dt * 1.9;
        TempLat_Now=ins.distance_x;                                         //卡尔曼滤波过滤获得的xy轴上的移动距离
        TempLon_Now=ins.distance_y;
    
   
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介    惯性导航参数初始化
// 参数说明    void
// 返回参数     void
// 使用示例
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void ins_init(void)
{
   ins.distance_x = 0;
   ins.distance_y = 0;
   ins.distance = 0;
   car_speed = 0;
}