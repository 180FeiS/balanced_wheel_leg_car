#include "zf_common_headfile.h"
#include "my_gps.h"

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
    /* GPS 点到点导航轻量计算放在 5ms 软任务里跑：
     * 1. 只有 NAV_HEADING_MODE_GPS 才会请求航向；
     * 2. 惯导仍固定在 pit0_ch0_isr 的 1ms 中断里跑；
     * 3. GPS 和惯导通过 nav_heading_mode 仲裁，避免同时写 steer_request_target_yaw()。
     * 4. 模式由 GPS_ApplyLaunchSpeed() / Nag_Begin_Replay() 切换，本任务只按当前模式执行。
     * 5. KEY3 记录起点；直线驶过约 GPS_NAV_GPS_FIRST_DISTANCE_M（默认 3 m）后用 RMC 的 gnss.direction（COG）作 GPS_first 标定 GPS–IMU 偏置，再追路点（不依赖 car_gps_dir）。
     */
    GPS_PointNav_Run();
  }

#if !DUALCORE_UI_ON_CM7_1
  if (task_pending_take(&task_10ms_menu_key_pending))
  {
    selectMenu_Key();
  }

  if (task_pending_take(&task_20ms_menu_pending))
  {
    selectMenu();
  }

  if (task_pending_take(&task_10ms_step_pending))
  {
    step_detect();
  }
#endif
  
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
  switch (0)
  {
    case 0:
      /* 基础输入组：
   
       */
      SendDataStreamToVOFA(4, (float)euler_angle.pitch, (float)euler_angle.roll, (float)euler_angle.yaw, (float)gyro_z_bias_mean);
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
       * Prospect_index: 当前前瞻点索引；高速时应大于 Run_index
       * Angle_Run: 当前实际采用的目标 yaw（来自前瞻点）
       * Nag_GetDebugReadYaw(): 安全读取当前前瞻点对应的 flash yaw
       * Nag_Stop_f: 到达终点后会置位，可用于停车或切逻辑
       */
      SendDataStreamToVOFA(5,
                           (float)N.Run_index,
                           (float)Nag_GetDebugProspectIndex(),
                           (float)N.Angle_Run,
                           Nag_GetDebugReadYaw(),
                           (float)N.Nag_Stop_f,
                          (float)Nag_Vofa_Group);
      break;
    case 3:
      /* 闭环输出组：
       * Final_Out: 当前 yaw 与目标 yaw 的偏差
       * Curve_Strength: 根据前方 yaw 变化量估算的弯道强度
       * nav_speed: 导航最终给速度环的目标速度
       * event_active: 1 表示当前已切出惯导，由元素状态机接管
       * event_state: 当前元素状态机状态（IDLE/ENTERED/RUNNING/DONE/ABORT）
       */
      SendDataStreamToVOFA(5,
                           (float)N.Final_Out,
                           (float)N.Curve_Strength,
                           (float)Nag_GetControlSpeedTarget(),
                           (float)N.Event_Active,
                           (float)N.Event_State,
                          (float)Nag_Vofa_Group);
      break;
    case 4:
      /* 调速/元素组：
       * motor_user_speed_cmd: SWITCH2 / V 命令 / qrs 等用户基准速度；旁路调试时可直接设成 1000 做阶跃
       * speed_target_effective: 真正送给速度环的目标速度，是 PID 调参最该盯住的“目标值”
       * car_speed: 当前实际车速，用来和 speed_target_effective 对比响应快慢、超调和拖尾
       * Nag_SystemRun_Index: 导航状态机。正常回放时 2=正在读 flash，3=正式回放运行
       * 调试顺序建议：
       *   1. 先把 SWITCH1 拨到 OFF，只看 motor_user_speed_cmd 是否能随 SWITCH2 在 1000/1500 间切换；
       *   2. 正常模式下若未回放，speed_target_effective 应保持 0；
       *   3. 若打开速度调试旁路，则不进回放也能直接观察 speed_target_effective 与 car_speed 的阶跃响应；
       *   4. 建议 VOFA 第 4 组按顺序看：motor_user_speed_cmd / speed_target_effective / car_speed / Nag_SystemRun_Index / group。
       */
      SendDataStreamToVOFA(5,
                           (float)motor_user_speed_cmd,
                           (float)speed_target_effective,
                           (float)-car_speed,
                           (float)N.Nag_SystemRun_Index,
                           (float)Nag_Vofa_Group);
      break;
    case 5:
      /* 自旋排障组：
       * spin_enable: 1=自旋任务正在运行
       * spin_done: 1=自旋完成并退出（会触发元素状态机继续）
       * spin_target_deg/spin_accum_deg/spin_angle_err: 目标角度、累计角度、剩余误差
       * spin_rate_target_dps/spin_rate_meas_dps: 自旋内环目标角速度与实测角速度
       */
      SendDataStreamToVOFA(2,
                           (float)spin_enable,
                           (float)spin_done
          );
      break;
    default:
      break;
  }
}

int main(void)
{
  clock_init(SYSTEM_CLOCK_250M); // 时钟配置及系统初始化<务必保留>
  debug_init();                  // 调试串口信息初始化
  gnss_module_init();
#if DUALCORE_UI_ON_CM7_1
  all_init_cm7_0_control();
#else
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
#endif

  while (true)
  {
    if (gnss_flag)
    {
      gnss_flag = 0;
      gnss_data_parse();
    }

    run_soft_tasks();
#if DUALCORE_UI_ON_CM7_1
    /* 先消费 CM7_1 菜单命令，再发布快照；否则 Launch 调速、SaveSpd 保存和导航按键都不会真正落到控制核。 */
    dualcore_ui_cmd_consume_all();
    /* 验证用：左摇杆速度 + 左杆键翻转 Motor_Switch；正式策略可迁到 control/navigation */
    remote_lora_apply_validate_motor();
    dualcore_ctrl_to_ui_publish();
#endif

#if LEG_DEBUG_MODE && !DUALCORE_UI_ON_CM7_1
    // 调试模式：发送4路舵机PWM到VOFA
    int16 servo1_value = SERVO1_MID + pwm_ph1;
    int16 servo2_value = SERVO2_MID - pwm_ph2;
    int16 servo3_value = SERVO3_MID - pwm_ph3;
    int16 servo4_value = SERVO4_MID + pwm_ph4;
    SendDataStreamToVOFA(4, (float)servo1_value, (float)servo2_value, (float)servo3_value, (float)servo4_value);
#elif !LEG_DEBUG_MODE && !DUALCORE_UI_ON_CM7_1
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
        /* 台阶测距 VOFA：在 step_detection.h 里把 STEP_DEBUG_USE_VOFA 置 1，则此处改发 6 路台阶数据，避免与导航帧混在同一串口 */
#if STEP_DEBUG_USE_VOFA
        step_debug_send_to_vofa();
#else
        send_nav_debug_to_vofa();
#endif
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
#else
    if (N.Nag_SystemRun_Index == 2)
    {
      NagFlashRead();
    }
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
