#include "zf_common_headfile.h"
#ifndef CODE_PID_H_
#define CODE_PID_H_


/*********************************************************************参数*********************************************************************/
typedef enum
{
    Incremental_pid     = 0,        //增量式PID
    Position_pid        = 1         //位置式PID
}pid_type_t;

typedef struct
{
   float target;                //目标值
   float observation;           //实际值
   float error;                 //误差
   float last_error;            //上一次误差
   float prev_error;            //上上次误差
   float kp;                    //KP
   float kpp;                   //2次KP
   float ki;                    //KI
   float kd;                    //KD
   float dt_s;                  //周期
   float integral;              //积分
   float integral_clip;         //积分限幅
   float differential;          //微分
   float last_differential;     //上一次微分
   float lpf_n;                 //低通滤波
   bool use_lpf;                //使用低通滤波
   pid_type_t pid_type;         //使用PID类型
   float out;                   //PID输出
   float out_clip;              //PID输出限幅
}pid_t;

extern pid_t turn_angle;        //转向环
extern pid_t turn_gyro;         //转向环
extern pid_t leg_hight;         //腿高环
//extern pid_t gyro;              //角速度环
//extern pid_t angle;             //角度环
//extern pid_t speed;             //速度环
//extern pid_t leg_balance_L;     //左腿平衡环
//extern pid_t leg_balance_R;     //右腿平衡环
/*********************************************************************参数*********************************************************************/


/*********************************************************************函数*********************************************************************/
void pid_init(pid_t *pid, float kp, float ki, float kd, float dt_s, float integral_clip, float N, bool lpf, float out_clip, pid_type_t pid_type);           //PID初始化

void pid_set_error(pid_t *pid, float error);                        //设置PID的误差

void pid_set_target(pid_t *pid, float target);                      //设置PID的期望

void pid_get_observation(pid_t *pid, float observation);            //设置PID的实际值

void pid_set_dt(pid_t *pid, float d_time);                          //设置PID的周期

float pid_Incremental(pid_t *pid);                                  //增量式PID

float pid_Position(pid_t *pid);                                     //位置式PID

float pid_run(pid_t *pid);                                          //运行PID

const char *pid_type_to_string(pid_type_t pid_type);                //设置PID的类型
/*********************************************************************函数*********************************************************************/


#endif /* CODE_PID_H_ */
