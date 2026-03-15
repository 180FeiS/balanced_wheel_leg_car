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

extern float set_speed;                   //腿高积分
extern uint8 jump_flag;                   //跳跃标志位
extern uint8 speed_flag;                  //速度标志位
extern float speed_loop_leg_tilt;         //速度环输出，供腿部倾斜角

#define L_dead_zone_correct           (140)       //左电机正死区
#define L_dead_zone_negative          (-148)      //左电机负死区
#define R_dead_zone_correct           (140)       //右电机正死区
#define R_dead_zone_negative          (-140)      //右电机负死区

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

/*********************************************************************参数*********************************************************************/

/*********************************************************************函数*********************************************************************/
void pid_ctrl_Init(void);                                   //PID控制初始化

void LQR_control(float V_target, float th);                 //LQR控制平衡和行驶

float turn_control(float image_error);                      //转向环控制

void pid_ctrl_Run(void);                                    //PID控制平衡和行驶

void leg_control(void);                                     //控制腿高

void jump_set_step(int step_num);                           //执行跳跃内容

void jump_control(void);                                     // 控制腿高

void dead_compensate(int16 *input_L, int16 *input_R);       //死区补偿

void left_leg_control(float p, float angle);                // 控制左腿

void right_leg_control(float p, float angle);               // 控制右腿

void leg_debug_init_pwm(void);                               // 调试模式腿高初始化

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
