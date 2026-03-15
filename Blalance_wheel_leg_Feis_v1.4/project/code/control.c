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
    // -0.0281 ,  -0.8067 ,  -1.6867 ,  -0.1745,
    // -0.0281 ,  -0.8067 ,  -1.6867 ,  -0.1745
    //        -0.0285  , -0.8754,   -1.4840 ,  -0.1806,
    //        -0.0285  , -0.8754 ,  -1.4840,   -0.1806

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
float pitch_mid = -5.0;  // pitch机械中值（俯仰平衡）
float roll_mid = -1.866; // roll机械中值（横滚平衡，leg_hight PID目标）

// 各个环节PID的运算周期
float dt_pid_gyro = 0.002f;
float dt_pid_angle = 0.01f;
float dt_pid_speed = 0.02f;
float dt_pid_turn = 0.01f;
float dt_leg = 0.005f;   // 5ms，横滚leg_hight PID周期，与pit0_ch1 leg_control一致
float dt_pid_turn_angle = 0.003f;
float dt_pid_turn_gyro = 0.001f;

// 初始腿高（非跳跃时基准）
float leg_long = 5.5f; 
// float leg_high_integral = 0;

// 跳跃标志位
float set_speed = 0;
uint8 jump_flag = 0;
uint8 jump_step_index = 0;  // 当前跳跃步 0起跳 1准备缓冲 2执行缓冲

uint8 speed_flag = 0;


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

// 转向环参数
float turn_out = 0;
float KP = 5;  // 25.24f;
float KPP = 0; // 0.3805;
float KD = 0;  // 0.2f;
float KDD = 0; // 0.2f

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
uint8 roll_balance_en = 1;  // 运行时可改：1开启横滚平衡，0关闭（左右腿保持leg_long）
#define ROLL_LEG_SCALE          1.0f  // 横滚PID输出→腿长增量缩放，越大抬腿越猛
#define ROLL_LEG_OFFSET_MAX      8.5f  // 单侧腿长增量上限，防止过度抬腿
// dt_leg、leg_hight PID 见上方变量及 pid_ctrl_Init()

/*---------- 俯仰角参数（速度环→腿倾角，与横滚并级）----------*/
#define LEG_SERVO_SPEED_TILT_EN  1     // 置0关闭速度环→舵机倾角
#define LEG_TILT_K              0.02f // 速度环输出→腿倾角缩放系数
#define LEG_TILT_MAX             20.0f // 腿倾角限幅±20°

/*---------- 跳跃参数（障碍跨越）----------*/
#define JUMP_PID_SCALE          0.3f  // 跳跃时angle/speed的kp缩放，维持稳定
#define JUMP_PREPARE_P          10.0f // 准备缓冲目标腿长（起跳后伸腿高度）
#define JUMP_BUFFER_P           5.5f  // 执行缓冲最终腿长（落地收腿高度）
#define JUMP_BUFFER_STEP_P_MAX  0.2f  // 执行缓冲时每5ms腿高最大变化
#define JUMP_BUFFER_STEP_PER_20MS  (JUMP_BUFFER_STEP_P_MAX * 4)  // 每20ms步进（4次5ms）
#define JUMP_BUFFER_MARGIN      2     // 缓冲周期余量
#define JUMP_BUFFER_CYCLES  ((int)(((JUMP_PREPARE_P - JUMP_BUFFER_P) / JUMP_BUFFER_STEP_PER_20MS) + 0.999f) + JUMP_BUFFER_MARGIN)

/* 跳跃时序（jump_control在pit0_ch10 20ms周期，单位=20ms）*/
const jump_control_struct jump_control_config[] =
    {
        {0,  5,  jump_set_step, "起跳"},           // 0~100ms 伸腿爆发
        {5,  8, jump_set_step, "准备缓冲"},       // 100~160ms 过渡姿态
        {8, 8 + JUMP_BUFFER_CYCLES - 1, jump_set_step, "执行缓冲"},  // 落地收腿
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
    // pid_init(&turn_angle, 2.045, 0, 0.15, 0.003, 0, 0, 0, 10000, Position_pid);
    // pid_init(&turn_gyro, 2.087, 15, 0, 0.001, 0, 0, 0, 10000, Position_pid);
    // pid_init(&turn, 1.87, 19, 0, 0.01, 0, 0, 0, 5000, Position_pid);
     pid_init(&gyro, 1.1, 0, 0, 0.002, 0, 0, 0, 10000, Position_pid);
     pid_init(&angle, 500.0, 0, 0, 0.01, 0, 0, 0, 10000, Position_pid);
     pid_init(&speed, 2.8, 0.0000, 0, 0.02, 0, 0, 0, 10000, Position_pid);//3.0
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
    float gy = -imu660ra_gyro_transition(imu660ra_gyro_y) / DEG_TO_RAD;

    float v_t = (v_hat - V_target);

    // TL = LQR_K[3] * gy + LQR_K[2] * Tangle + LQR_K[1] * v_t;
    // TR = LQR_K[7] * gy + LQR_K[6] * Tangle + LQR_K[5] * v_t;

    TL = LQR_K[3] * gy + LQR_K[2] * Tangle + LQR_K[1] * v_t + LQR_K[0] * ((x_hat + L_min * sin(euler_angle.pitch / DEG_TO_RAD) - (x_hat_last + v_hat)));
    TR = LQR_K[7] * gy + LQR_K[6] * Tangle + LQR_K[5] * v_t + LQR_K[4] * ((x_hat + L_min * sin(euler_angle.pitch / DEG_TO_RAD) - (x_hat_last + v_hat)));

    x_hat_last = x_hat;

    // 更新角度环目标时间为角速度环提供设定值
    //    pid_set_target(&turn_gyro, 0);
    //
    //    // 设置角速度环观测值(Z轴角速度)
    //    pid_get_observation(&turn_gyro, imu660ra_gyro_transition(imu660ra_gyro_z));
    //    get_timer(TOM0_CH4, &pid_time_turn, &dt_pid_turn_gyro);
    //    pid_set_dt(&turn_gyro, dt_pid_turn_gyro);
    //    pid_run(&turn_gyro);
    //    Err = 0.5;

    // float turn_out = KP * image_error + ABS(image_error) * image_error * KPP + KD * (image_error - last_image_error) - imu660ra_gyro_z * KDD;
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
// 函数简介     转向环控制
// 参数说明     image_error    图像误差
// 返回参数     null
// 使用示例     turn_control(93 - mid_point);
// 备注信息     isr中断调用
-------------------------------------------------------------------------------------------------------------------*/
float turn_control(float image_error)
{

    return turn_gyro.out;
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
    static float angle_kp_normal = 500.0f;
    static float speed_kp_normal = 0.02f;
    imu660ra_get_gyro();

    if (0 == timer_flag) // 速度环
    {
        pid_get_observation(&speed, -motor_value.receive_left_speed_data + motor_value.receive_right_speed_data);

        pid_set_dt(&speed, dt_pid_speed);
        pid_run(&speed);
        speed_loop_leg_tilt = (jump_flag == 1) ? (speed.out * JUMP_PID_SCALE) : speed.out;
    }

    if (0 == timer_flag % 5) // 角度环
    {
        pid_set_target(&angle, pitch_mid);  //pitch_mid - speed.out
        pid_get_observation(&angle, euler_angle.pitch);

        pid_set_dt(&angle, dt_pid_angle);
        pid_run(&angle);
        Angle_Out = angle.out + angle_kd * imu660ra_gyro_y * dt_pid_angle;
        if (jump_flag == 1)
            Angle_Out *= JUMP_PID_SCALE;
    }

    // 角速度环
    pid_set_target(&gyro, Angle_Out);
    pid_get_observation(&gyro, imu660ra_gyro_y);
    pid_set_dt(&gyro, dt_pid_gyro);
    pid_run(&gyro);

    // // 转向环
    // //    pid_set_target(&turn, mid_point);
    // pid_get_observation(&turn, imu660ra_gyro_transition(imu660ra_gyro_z));
    // pid_set_dt(&turn, dt_pid_turn);
    // pid_run(&turn);

    

    if(Motor_Switch)
    {
        if ((-motor_value.receive_left_speed_data + motor_value.receive_right_speed_data) / 2 > 1500 || (-motor_value.receive_left_speed_data + motor_value.receive_right_speed_data) / 2 < -1500)
        {
            Motor_Switch = 0;
        }
        else
        {
            float scale = (jump_flag == 1) ? JUMP_PID_SCALE : 1.0f;
            small_driver_set_duty((int16)(-(gyro.out + turn.out) * scale), (int16)((gyro.out - turn.out) * scale));
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
    float a = -LEG_TILT_K * speed_loop_leg_tilt;
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
        leg_long = 13.5f;           // 起跳：直接爆发伸腿
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
-------------------------------------------------------------------------------------------------------------------*/
void jump_control(void)
{
    static int jump_time = 0;
    if (jump_flag == 1)
    {
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
            jump_flag = 0;
            jump_time = 0;
            jump_step_index = 0; // 重置当前跳跃步
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
// 参数说明     angel1    当前航向角
//              angel2    目标航向角
// 返回参数     归一化后的角度偏差（-180°到180°）
// 使用示例     double deviation = ange_deviation1(90, 45);
// 备注信息     计算当前航向与目标航向的偏差并归一化
-------------------------------------------------------------------------------------------------------------------*/
double ange_deviation1(double angel1, double angel2)
{
    double x;
    x = angel1 - angel2;  // 当前航向 - 目标航向
    
    // 归一化到 [-180°, 180°]
    if(x >= 180) x = x - 360;
    if(x <= -180) x = x + 360;
    
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

                //获取车子的速度
        double speed_x = 0 , speed_y = 0;
        speed_x = car_speed * cos(euler_angle.yaw/180.0*3.1415926);
        speed_y = car_speed * sin(euler_angle.yaw/180.0*3.1415926);
        ins.distance_x += speed_x * 0.005*1.9;
        ins.distance_y += speed_y * 0.005*1.9;
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