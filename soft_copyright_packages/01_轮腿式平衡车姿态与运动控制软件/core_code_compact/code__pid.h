#include "zf_common_headfile.h"
#ifndef CODE_PID_H_
#define CODE_PID_H_
typedef enum
{
    Incremental_pid     = 0,
    Position_pid        = 1
}pid_type_t;
typedef struct
{
   float target;
   float observation;
   float error;
   float last_error;
   float prev_error;
   float kp;
   float kpp;
   float ki;
   float kd;
   float dt_s;
   float integral;
   float integral_clip;
   float differential;
   float last_differential;
   float lpf_n;
   bool use_lpf;
   pid_type_t pid_type;
   float out;
   float out_clip;
}pid_t;
extern pid_t turn_angle;
extern pid_t turn_gyro;
extern pid_t leg_hight;
void pid_init(pid_t *pid, float kp, float ki, float kd, float dt_s, float integral_clip, float N, bool lpf, float out_clip, pid_type_t pid_type);
void pid_set_error(pid_t *pid, float error);
void pid_set_target(pid_t *pid, float target);
void pid_get_observation(pid_t *pid, float observation);
void pid_set_dt(pid_t *pid, float d_time);
float pid_Incremental(pid_t *pid);
float pid_Position(pid_t *pid);
float pid_run(pid_t *pid);
const char *pid_type_to_string(pid_type_t pid_type);
#endif
