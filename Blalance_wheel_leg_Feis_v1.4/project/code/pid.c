#include "zf_common_headfile.h"


/*-------------------------------------------------------------------------------------------------------------------
// 函数简介     PID初始化
// 参数说明     *pid               设置对象
              kp                 PID的P项
              ki                 PID的I项
              kd                 PID的D项
              dt_s               周期
              integral_clip      积分限幅
              N                  低通滤波
              lpf                是否启动低通滤波
              out_clip           输出限幅
              pid_type           PID类型
// 返回参数     null
// 使用示例     pid_init(&pid, kp, ki, kd, 0.001, 0, 0, FALSE, 10000, Position_pid);
// 备注信息     初始化调用
-------------------------------------------------------------------------------------------------------------------*/
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



/*-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置PID的误差
// 参数说明     *pid       设置对象
              error      误差
// 返回参数     null
// 使用示例     pid_set_error(&pid, error);
// 备注信息     循环内部调用
-------------------------------------------------------------------------------------------------------------------*/
void pid_set_error(pid_t *pid, float error)
{
    pid->error = error;
    pid->observation = -error;
    pid->target = 0;
}



/*-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置PID的期望
// 参数说明     *pid       设置对象
              target     期望
// 返回参数     null
// 使用示例     pid_set_target(&pid, target);
// 备注信息     循环内部调用
-------------------------------------------------------------------------------------------------------------------*/
void pid_set_target(pid_t *pid, float target)
{
    pid->target = target;
}



/*-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置PID的实际值
// 参数说明     *pid            设置对象
              observation     实际值
// 返回参数     null
// 使用示例     pid_set_target(&pid, observation);
// 备注信息     循环内部调用
-------------------------------------------------------------------------------------------------------------------*/
void pid_get_observation(pid_t *pid, float observation)
{
    pid->observation = observation;
}



/*-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置PID的周期
// 参数说明     *pid       设置对象
              d_time     周期
// 返回参数     null
// 使用示例     pid_set_dt(&pid, d_time);
// 备注信息     循环内部调用
-------------------------------------------------------------------------------------------------------------------*/
void pid_set_dt(pid_t *pid, float d_time)
{
    pid->dt_s = d_time;
}



/*-------------------------------------------------------------------------------------------------------------------
// 函数简介     增量式PID
// 参数说明     *pid       设置对象
// 返回参数     float      PID输出
// 使用示例     pid_Incremental(&pid);
// 备注信息     循环内部调用
-------------------------------------------------------------------------------------------------------------------*/
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



/*-------------------------------------------------------------------------------------------------------------------
// 函数简介     位置式PID
// 参数说明     *pid       设置对象
// 返回参数     float      PID输出
// 使用示例     pid_Position(&pid);
// 备注信息     循环内部调用
-------------------------------------------------------------------------------------------------------------------*/
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

    //2次p仅让它在误差比较大的时候起作用
    pid->out = pid->kp * pid->error + pid->ki * pid->integral + pid->kd * pid->differential + pid->kpp * pid->error * abs(pid->error)/(15*15);

    if(pid->out_clip > 0)
    {
        pid->out = clip2(pid->out, pid->out_clip);
    }

    return pid->out;
}



/*-------------------------------------------------------------------------------------------------------------------
// 函数简介     运行PID
// 参数说明     *pid       设置对象
// 返回参数     float      PID输出
// 使用示例     pid_run(&pid);
// 备注信息     循环内部调用
-------------------------------------------------------------------------------------------------------------------*/
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



/*-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置PID的类型
// 参数说明     pid_type       设置PID类型
// 返回参数     null
// 使用示例     *pid_type_to_string(pid_type);
// 备注信息     函数内部调用, 0为增量式, 1为位置式
-------------------------------------------------------------------------------------------------------------------*/
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
