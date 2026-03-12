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
#endif
