#include "zf_common_headfile.h"
#ifndef CODE_PID_H_
#define CODE_PID_H_
/*********************************************************************����*********************************************************************/
typedef enum
{
    Incremental_pid     = 0,        //����ʽPID
    Position_pid        = 1         //λ��ʽPID
}pid_type_t;
typedef struct
{
   float target;                //Ŀ��ֵ
   float observation;           //ʵ��ֵ
   float error;                 //���
   float last_error;            //��һ�����
   float prev_error;            //���ϴ����
   float kp;                    //KP
   float kpp;                   //2��KP
   float ki;                    //KI
   float kd;                    //KD
   float dt_s;                  //����
   float integral;              //����
   float integral_clip;         //�����޷�
   float differential;          //΢��
   float last_differential;     //��һ��΢��
   float lpf_n;                 //��ͨ�˲�
   bool use_lpf;                //ʹ�õ�ͨ�˲�
   pid_type_t pid_type;         //ʹ��PID����
   float out;                   //PID���
   float out_clip;              //PID����޷�
}pid_t;
extern pid_t turn_angle;        //ת��
extern pid_t turn_gyro;         //ת��
extern pid_t leg_hight;         //�ȸ߻�
//extern pid_t gyro;              //���ٶȻ�
//extern pid_t angle;             //�ǶȻ�
//extern pid_t speed;             //�ٶȻ�
//extern pid_t leg_balance_L;     //����ƽ�⻷
//extern pid_t leg_balance_R;     //����ƽ�⻷
/*********************************************************************����*********************************************************************/
/*********************************************************************����*********************************************************************/
void pid_init(pid_t *pid, float kp, float ki, float kd, float dt_s, float integral_clip, float N, bool lpf, float out_clip, pid_type_t pid_type);           //PID��ʼ��
void pid_set_error(pid_t *pid, float error);                        //����PID�����
void pid_set_target(pid_t *pid, float target);                      //����PID������
void pid_get_observation(pid_t *pid, float observation);            //����PID��ʵ��ֵ
void pid_set_dt(pid_t *pid, float d_time);                          //����PID������
float pid_Incremental(pid_t *pid);                                  //����ʽPID
float pid_Position(pid_t *pid);                                     //λ��ʽPID
float pid_run(pid_t *pid);                                          //����PID
const char *pid_type_to_string(pid_type_t pid_type);                //����PID������
/*********************************************************************����*********************************************************************/
#endif /* CODE_PID_H_ */
