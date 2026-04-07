


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
                                                 // 启动定时
    //Nag_System();

  }

  if (task_pending_take(&task_10ms_menu_key_pending))
  {
    selectMenu_Key();
  }

  if (task_pending_take(&task_20ms_menu_pending))
  {
    selectMenu();                                               // 
  }

  if (task_pending_take(&task_10ms_step_pending))
  {
    step_detect();                                                  // 
  }
  
}

/* 惯导调试输出分组：
 * 通过菜单里的字符 'n' 切换 Nag_Vofa_Group。
 * 建议调试顺序：
 * 0 -> 先确认 yaw 和速度输入是否正常
 * 1 -> 再看录制阶段是否真的在累计路程、写入点位
 * 2 -> 然后看复现阶段是否在推进目标点
 * 3 -> 最后看惯导偏差和实际转向输出是否同向
 */
static void send_nav_debug_to_vofa(void)
{
  switch (Nag_Vofa_Group)
  {
    case 0:
      /* 基础输入组：
       * yaw: 当前姿态解算出来的偏航角
       * car_speed: 当前选作惯导积分的平均车速
       * left/right_speed: 原始左右轮速度，便于检查方向和符号
       */
      SendDataStreamToVOFA(5,
                           (float)euler_angle.yaw,
                           (float)car_speed,
                           (float)motor_value.receive_left_speed_data,
                           (float)motor_value.receive_right_speed_data,
                           (float)Nag_Vofa_Group);
      break;
    case 1:
      /* 录制状态组：
       * Mileage_Debug_Total: 从开始录制到当前累计的总路程
       * Save_index: 已经写入了多少个 yaw 点
       * Flash_page_index: 当前正在写哪一页 Flash
       * End_f: 录制结束标志，便于看是否进入收尾保存
       */
      SendDataStreamToVOFA(5,
                           (float)N.Mileage_Debug_Total,
                           (float)N.Save_index,
                           (float)N.Flash_page_index,
                           (float)N.End_f,
                          (float)Nag_Vofa_Group);
      break;
    case 2:
      /* 复现状态组：
       * Run_index: 当前回放推进到的目标点索引
       * Angle_Run: 当前实际采用的目标 yaw
       * Nag_GetDebugReadYaw(): 安全读取当前目标点对应的 flash yaw
       * Nag_Stop_f: 到达终点后会置位，可用于停车或切逻辑
       */
      SendDataStreamToVOFA(5,
                           (float)N.Run_index,
                           (float)N.Angle_Run,
                           Nag_GetDebugReadYaw(),
                           (float)N.Nag_Stop_f,
                          (float)Nag_Vofa_Group);
      break;
    case 3:
      /* 闭环输出组：
       * Final_Out: 当前 yaw 与目标 yaw 的偏差
       * euler_angle.yaw: 实时航向
       * Angle_Run: 当前目标航向
       * turn_mix_cmd: 最终送到转向差速链路的控制量
       * 如果这一组里偏差方向和 turn_mix_cmd 对不上，优先检查符号方向
       */
      SendDataStreamToVOFA(5,
                           (float)N.Final_Out,
                           (float)euler_angle.yaw,
                           (float)N.Angle_Run,
                           (float)turn_mix_cmd,
                          (float)Nag_Vofa_Group);
      break;
    default:
      break;
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
        //send_nav_debug_to_vofa();
    /* VOFA 调试输出按需要二选一或三选一打开：
     * 1. 姿态/零偏观测：pitch / roll / yaw / gyro_z_bias_mean
     * 2. 单层自旋调试：spin_accum_deg / spin_angle_err / spin_rate_target_dps / spin_rate_meas_dps
     * 3. 普通转向双环调试：steer_angle_err / steer_rate_target_dps / steer_rate_meas_dps / turn_gyro.out
     */
    //  SendDataStreamToVOFA(4, (float)euler_angle.pitch, (float)euler_angle.roll, (float)euler_angle.yaw, (float)gyro_z_bias_mean);
    // SendDataStreamToVOFA(4, (float)spin_accum_deg, (float)spin_angle_err, (float)spin_rate_target_dps, (float)spin_rate_meas_dps);

    // SendDataStreamToVOFA(4, (float)steer_cmd, (float)spin_cmd, (float)turn_mix_cmd, (float)spin_enable);



    //SendDataStreamToVOFA(4, (float)N.Mileage_All,(float)N.Save_index,(float)R_Mileage,(float)L_Mileage);


    // SendDataStreamToVOFA(4, (float)set_speed, (float)car_speed, (float)Left_Motor_Speed, (float)Right_Motor_Speed);

    // SendDataStreamToVOFA(4, (float)steer_angle_err, (float)steer_rate_target_dps, (float)steer_rate_meas_dps, (float)turn_gyro.out);
#endif

    /* 切勿在 while(true) 里每轮调用 steer_request_target_yaw()：
     * 否则会每 1ms 触发一次 steer_set_target_yaw，持续打断自旋并反复重置转向 PID，
     * 表现为菜单里 o/p 调试（spin_task_start / steer_task_start）“失效”。 */
    
    
    
     // timer_init(TC_TIME2_CH0,TIMER_US);                                         // 定时器使用TC_TIME2_CH0 使用微秒级计数
    // timer_start(TC_TIME2_CH0);                                                  // 启动定时
    //                                                // 
    // timer_stop(TC_TIME2_CH0);                                                   // 停止定时器
    // SendDataStreamToVOFA(1, (float)timer_get(TC_TIME2_CH0));
    // timer_clear(TC_TIME2_CH0); 

      
  }
}

// **************************** 代码区域 ****************************
