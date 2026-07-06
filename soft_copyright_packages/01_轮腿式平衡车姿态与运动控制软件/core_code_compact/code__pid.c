#include "zf_common_headfile.h"
void pid_init(pid_t *pid, float kp, float ki, float kd, float dt_s, float integral_clip, float N, bool lpf, float out_clip, pid_type_t pid_type)
{
    memset(pid, 0, sizeof(pid_t));
    pid->target = 0;
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->dt_s = dt_s;
    pid->integral_clip = integral_clip > 0 ? integral_clip : 0;
    pid->error = 0;
    pid->integral = 0;
    pid->differential = 0;
    pid->use_lpf = lpf;
    pid->lpf_n = pid->use_lpf ? N : 0;
    pid->pid_type = pid_type;
    pid->out_clip = out_clip > 0 ? out_clip : -out_clip;
}
void pid_set_error(pid_t *pid, float error)
{
    pid->error = error;
    pid->observation = -error;
    pid->target = 0;
}
void pid_set_target(pid_t *pid, float target)
{
    pid->target = target;
}
void pid_get_observation(pid_t *pid, float observation)
{
    pid->observation = observation;
}
void pid_set_dt(pid_t *pid, float d_time)
{
    pid->dt_s = d_time;
}
float pid_Incremental(pid_t *pid)
{
    pid->error = pid->target - pid->observation;
    pid->integral = pid->error;
    pid->differential = pid->error - 2 * pid->last_error + pid->prev_error;
    pid->prev_error = pid->last_error;
    pid->last_error = pid->error;
    float out = pid->kp * (pid->error - pid->last_error) + pid->ki * pid->integral * pid->dt_s + pid->kd * pid->differential / pid->dt_s;
    pid->out = clip2(pid->out + out, pid->out_clip);
    return pid->out;
}
float pid_Position(pid_t *pid)
{
    pid->error = pid->target - pid->observation;
    if(pid->integral_clip > 0)
    {
        pid->integral = clip2(pid->integral + pid->error * pid->dt_s, pid->integral_clip);
    }
    else
    {
        pid->integral += pid->error * pid->dt_s;
    }
    if(pid->use_lpf)
    {
        pid->differential = (pid->error - pid->last_error) / pid->dt_s;
        float A = (pid->dt_s * pid->lpf_n) / (1.0f + pid->dt_s * pid->lpf_n);
        pid->differential = pid->differential * (1.0f - A) + pid->last_differential * A;
        pid->last_differential = pid->differential;
    }
    else
    {
        pid->differential = (pid->error - pid->last_error) / pid->dt_s;
    }
    pid->last_error = pid->error;
    pid->out = pid->kp * pid->error + pid->ki * pid->integral + pid->kd * pid->differential + pid->kpp * pid->error * abs(pid->error)/(15*15);
    if(pid->out_clip > 0)
    {
        pid->out = clip2(pid->out, pid->out_clip);
    }
    return pid->out;
}
float pid_run(pid_t *pid)
{
    float out = 0;
    switch(pid->pid_type)
    {
       case Incremental_pid:
           out = pid_Incremental(pid);
           break;
       case Position_pid:
           out = pid_Position(pid);
           break;
       default:
           break;
    }
    return out;
}
const char *pid_type_to_string(pid_type_t pid_type)
{
    switch(pid_type)
    {
        case Incremental_pid:
            return "Incremental PID";
        case Position_pid:
            return "Position PID";
        default:
            return "Unknown";
    }
}
