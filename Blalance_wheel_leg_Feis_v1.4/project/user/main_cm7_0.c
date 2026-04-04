


#include "zf_common_headfile.h"

// 外部全局PWM参数变量
extern int16 pwm_ph1;
extern int16 pwm_ph2;
extern int16 pwm_ph3;
extern int16 pwm_ph4;

// gyro_z 零偏观测（ekf.h 中声明）
extern float gyro_z_bias_mean;

uint16 jump_test = 1;

/* 主循环侧统一用这个函数“取走一次待执行任务”。
 * 这里短暂关中断是为了避免与 ISR 同时修改 pending 计数。
 * 如果以后增加新的软任务，通常不需要改这个函数，直接在 run_soft_tasks() 里复用即可。
 */
static uint8 task_pending_take(vuint8 *task_pending)
{
  uint8 has_task = 0;
  uint32 interrupt_status = interrupt_global_disable();

  if (*task_pending > 0)
  {
    (*task_pending)--;
    has_task = 1;
  }

  interrupt_global_enable(interrupt_status);
  return has_task;
}

/* 软任务统一入口。
 * 新增任务时，建议按“控制相关优先、UI/显示靠后”的顺序往下添加。
 * 典型新增方法：
 *   1. 在 task_schedule.h 增加 extern pending 变量
 *   2. 在对应 PIT ISR 中调用 task_pending_push()
 *   3. 在这里增加 if (task_pending_take(...)) { your_task(); }
 *
 * 如果某个任务明显变重，可以：
 * - 降低它的周期
 * - 把它拆成多个小步骤分多次执行
 * - 或继续保留在主循环，但放到更靠后的位置
 */
static void run_soft_tasks(void)
{
  if (task_pending_take(&task_5ms_nav_pending))
  {
    Nag_System();
  }

  if (task_pending_take(&task_10ms_menu_key_pending))
  {
    selectMenu_Key();
  }

  if (task_pending_take(&task_20ms_menu_pending))
  {
    selectMenu();
  }

  if (task_pending_take(&task_50ms_step_pending))
  {
    step_detect();
  }
}

int main(void)
{
  clock_init(SYSTEM_CLOCK_250M); // 时钟配置及系统初始化<务必保留>
  debug_init();                  // 调试串口信息初始化
  // 此处编写用户代码 例如外设初始化代码等
  // 此处编写用户代码 例如外设初始化代码等
  all_init(1,  // 是否开启屏幕显示标志位           //0:关闭          1:IPS200显示    （默认开启摄像头初始化）
           0,  // 是否开启逐飞助手标志位           //0:关闭          1:开启
           1,  // 是否开启vofa初始化标志位         //0:关闭          1:开启
           1,  // 是否开启陀螺仪标志位             //0:关闭          1:开启
           1,  // 是否开启舵机初始化标志位          //0:关闭          1:开启
           1,  // 是否开启无刷电机初始化标志位       //0:关闭          1:开启
           1,  // 是否开启PID标志位              //0:关闭          1:开启
           1,  // 是否开启惯导标志位              //0:关闭          1:开启
           1,  // 是否开启姿态解算标志位           //0:关闭          1:开启
           1,  // 是否开启按键初始化标志位           //0:关闭          1:开启
           1,  // 是否开启中断标志位              //0:关闭          1:开启
           1); // 是否开启菜单初始化标志位        //0:关闭          1:开启


  // 此处编写用户代码 例如外设初始化代码等

  while (true)
  {
    /* 先处理由中断挂起的软任务，再做主循环中的显示/调试输出。 */
    run_soft_tasks();
     

#if LEG_DEBUG_MODE
    // 调试模式：发送4路舵机PWM到VOFA
    int16 servo1_value = SERVO1_MID + pwm_ph1;
    int16 servo2_value = SERVO2_MID - pwm_ph2;
    int16 servo3_value = SERVO3_MID - pwm_ph3;
    int16 servo4_value = SERVO4_MID + pwm_ph4;
    SendDataStreamToVOFA(4, (float)servo1_value, (float)servo2_value, (float)servo3_value, (float)servo4_value);
#else
    // 正常模式：6路姿态/速度 + 9路横滚调试（roll_debug_*）
    // SendDataStreamToVOFA(15,
    //                      (float)euler_angle.pitch, (float)euler_angle.roll, (float)car_speed,
    //                      (float)-motor_value.receive_left_speed_data, (float)motor_value.receive_right_speed_data,
    //                      (float)Motor_Switch,
    //                      (float)roll_debug_roll, (float)roll_debug_pid_out, (float)roll_debug_pid_err,
    //                      (float)roll_debug_desired_left, (float)roll_debug_desired_right,
    //                      (float)roll_debug_out_left, (float)roll_debug_out_right,
    //                      (float)roll_debug_left_offset, (float)roll_debug_right_offset);
        // if(!gpio_get_level(KEY_1)) N.Nag_SystemRun_Index=1;//1读取
        // if(!gpio_get_level(KEY_2)) N.Nag_SystemRun_Index=2;//2复现
        // if(!gpio_get_level(KEY_3) && N.Nag_SystemRun_Index == 1) N.End_f=1;//End_f请勿重复赋值
        if(N.Nag_SystemRun_Index == 2) NagFlashRead();//移植的时候这个必须要。直接复制粘贴过去就行
    SendDataStreamToVOFA(4, (float)euler_angle.pitch, (float)euler_angle.roll, (float)euler_angle.yaw, (float)gyro_z_bias_mean);
    //SendDataStreamToVOFA(4, (float)N.Mileage_All,(float)N.Save_index,(float)R_Mileage,(float)L_Mileage);
#endif
   

    

    // 此处编写需要循环执行的代码
  }
}

// **************************** 代码区域 ****************************
