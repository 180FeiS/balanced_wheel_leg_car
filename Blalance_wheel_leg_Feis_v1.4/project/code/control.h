#include "zf_common_headfile.h"
#ifndef CODE_CONTROL_H_
#define CODE_CONTROL_H_


/*********************************************************************参数*********************************************************************/
extern float pitch_mid;                           //pitch机械中值
extern float roll_mid;                            //roll机械中值

#define DEG_TO_RAD      (57.295779513082320876798154814105f)        //角度转弧度
#define K               (1.0f)                                      //加速度低通滤波系数

typedef void (*HandlerFunc)(int value);
typedef struct
{
        int16           min;                //运行时间最小值
        int16           max;                //运行时间最大值
        HandlerFunc     handler;            //执行函数
        const char      *description;       //执行内容
}jump_control_struct;

//各个环节PID的运算周期
extern float dt_pid_gyro;
extern float dt_pid_angle;
extern float dt_pid_speed;
extern float dt_pid_turn;
extern float dt_leg;
extern float dt_pid_turn_angle;
extern float dt_pid_turn_gyro;

/* 用户期望的基准速度（可带符号：负号表示反向），导航弯道限速等在此基础上缩放。
 * 来源：拨码 SWITCH2（1000/1500，边沿刷新）、串口 V<数值>、串口 q/r/s 微调；后两者写入后仍可能被 SWITCH2 拨动覆盖。
 * 旧工程中的 set_speed 已合并为该变量，请勿在模块外随意直接写全局，优先调用 motor_* API。
 */
extern float motor_user_speed_cmd;
extern float speed_target_effective;      //真正送入速度环的目标速度，已叠加导航限速/元素限速

void motor_user_speed_cmd_set_from_pc(float cmd);
void motor_poll_switch2_speed_baseline(void);
extern uint8 jump_flag;                   //跳跃标志位
extern uint8 speed_flag;                  //速度标志位
extern float speed_loop_leg_tilt;         //速度环输出，供腿部倾斜角

#define L_dead_zone_correct           (140)       //左电机正死区
#define L_dead_zone_negative          (-148)      //左电机负死区
#define R_dead_zone_correct           (140)       //右电机正死区
#define R_dead_zone_negative          (-140)      //右电机负死区

/* 旧版 LQR 转向实验参数，当前普通转向/自旋互斥链路不依赖这些量 */
extern float turn_out;
extern float KP;
extern float KPP;
extern float KD;
extern float KDD;

extern uint16 pwm_4;
extern uint16 pwm_1;

extern int16 LO_S;
extern int16 RO_S;

extern uint8 roll_balance_en;  // 1开启横滚平衡，0关闭，运行时可改

extern uint8 Motor_Runaway_Latch;  /* 失控保护最高优先级关断；清除方式见 dip_switch_motor_sync_from_hw */

/* 横滚控制调试变量，供VOFA查看 */
extern float roll_debug_roll;
extern float roll_debug_pid_out;
extern float roll_debug_pid_err;
extern float roll_debug_desired_left;
extern float roll_debug_desired_right;
extern float roll_debug_out_left;
extern float roll_debug_out_right;
extern float roll_debug_left_offset;
extern float roll_debug_right_offset;

/* 转向/自旋差速指令。
 * turn_mix_cmd 是最终真正参与左右轮差速的量：
 * - spin_enable == 1 时，只允许 spin_cmd 生效；
 * - spin_enable == 0 时，只允许 steer_cmd 生效。
 */
extern float steer_cmd;      // 普通转向写入的差速指令，仅在 spin_enable==0 时生效
extern float spin_cmd;       // 自旋任务生成的差速指令，仅在 spin_enable==1 时生效
extern float turn_mix_cmd;   // 最终送往左右轮的差速指令，按互斥规则在 steer_cmd/spin_cmd 间选择
extern uint8 steer_enable;   // 普通转向任务使能，1表示当前正在闭环转向
extern float steer_target_yaw_deg; // 普通转向的绝对航向目标（建议使用 -180~180 度）
extern float steer_angle_err; // 普通转向当前航向误差，主要用于VOFA/调试观察
extern float steer_rate_target_dps; // 普通转向外环生成的目标角速度，主要用于 VOFA/调试观察
extern float steer_rate_meas_dps;   // 普通转向内环使用的实际角速度反馈，主要用于 VOFA/调试观察
extern vuint8 steer_yaw_request_pending;   // 1：存在一条尚未真正进入 steer_set_target_yaw() 的最新航向请求
extern vuint8 steer_yaw_delayed_by_spin;   // 1：该请求因 spin_enable==1 被延迟，等自旋结束后再应用
extern volatile float steer_yaw_request_deg; // 最新待下发的绝对航向目标（度），新请求会覆盖旧目标

/* 自旋任务调试变量 */
extern uint8 spin_enable;
extern uint8 spin_done;
extern int8 spin_dir;
extern float spin_target_deg;
extern float spin_accum_deg;
extern float spin_angle_err;
extern float spin_rate_target_dps;
extern float spin_rate_meas_dps;

/*********************************************************************参数*********************************************************************/

/*********************************************************************函数*********************************************************************/
void pid_ctrl_Init(void);                                   //PID控制初始化

void LQR_control(float V_target, float th);                 //LQR控制平衡和行驶

float turn_control(float image_error);                      // 兼容旧接口，返回当前普通转向差速指令

void set_steer_cmd(float cmd);                              // 设置普通转向差速，自旋开启时该值会被忽略

void steer_set_target_yaw(float target_yaw_deg);            // 立即设置绝对航向目标；若当前在自旋，会直接打断自旋

void steer_request_target_yaw(float target_yaw_deg);        // 登记最新绝对航向；若自旋在跑则延迟到自旋结束后，由 1ms ISR 执行一次

void steer_request_relative_yaw(float delta_deg);           // 登记相对转角请求，由控制层统一换算成绝对航向后再交给 1ms ISR 执行

void steer_task_start(float delta_deg);                     // 启动相对转角任务，同步执行版本，主要保留给旧调用兼容

void steer_task_stop(void);                                 // 停止普通转向任务并清空本次转向输出

void pid_ctrl_Run(void);                                    //PID控制平衡和行驶

void leg_control(void);                                     //控制腿高

void jump_set_step(int step_num);                           //执行跳跃内容

void jump_control(void);                                     // 控制腿高

void dead_compensate(int16 *input_L, int16 *input_R);       //死区补偿

void left_leg_control(float p, float angle);                // 控制左腿

void right_leg_control(float p, float angle);               // 控制右腿

void leg_debug_init_pwm(void);                               // 调试模式腿高初始化

void spin_task_start(float turns, int8 dir);                 // 启动自旋任务，dir>0沿yaw正方向

void spin_task_stop(void);                                   // 停止自旋任务

/*********************************************************************函数*********************************************************************/


typedef struct
{
    double angle;                           //方位角
    double speed;                           //车辆的移动速度(由编码器测得)
    double distance;                        // 移动距离
    double distance_x,distance_y;           // X,Y轴上的移动距离
    double ins_x[400];                      //GPS经度转换成m在x轴上的距离
    double ins_y[400];                      //GPS纬度转换成m在y轴上的距离
}ins_struct;

extern ins_struct ins;                      //惯性导航结构体
extern double TempLat_Now;                   //转化坐标系后实时位置
extern double TempLon_Now;                   //转化坐标系后实时位置
extern double victual_point_lat[];           //虚拟点纬度数组
extern double victual_point_lon[];           //虚拟点经度数组
extern uint8 Temp_num;                       //当前目标点索引
extern double Angle_Z_Quaternions;           //当前航向角（四元数计算）

void get_car_xy(void);
void ins_init(void);
float Get_Final_Angle(void);                 //获取最终角度
double ange_deviation1(double angel1, double angel2); //航向角偏差归一化
#endif
