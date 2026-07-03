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

/* VOFA 调试输出分组（菜单 'n' 循环 Nag_Vofa_Group）：
 * 0 -> IMU 姿态 / 零偏
 * 1 -> 速度目标 speed_target_effective vs 实测 car_speed
 * 2 -> GPS+惯导融合 fusion_x/y、gps_residual 等
 */
static void send_nav_debug_to_vofa(void)
{
  switch (Nag_Vofa_Group % NAG_VOFA_GROUP_COUNT)
  {
    case 0:
      SendDataStreamToVOFA(4, (float)euler_angle.pitch, (float)euler_angle.roll, (float)euler_angle.yaw, (float)gyro_z_bias_mean);
      break;
    case 1:
      SendDataStreamToVOFA(2, (float)speed_target_effective, (float)car_speed);
      break;
#if NAV_FUSION_ENABLE
    case 2:
    {
      const NavFusionState *fusion_st = NavFusion_GetState();
      if (fusion_st != NULL)
      {
        SendDataStreamToVOFA(6,
                             fusion_st->x_m,
                             fusion_st->y_m,
                             fusion_st->v_mps,
                             fusion_st->gps_residual_m,
                             NavFusion_GetHoldDistM(),
                             NavFusion_GetHeadingBiasDeg());
      }
      break;
    }
#endif
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
#if NAV_FUSION_ENABLE
#if NAV_FUSION_ORIGIN_ENABLE
      if (NavFusion_IsOriginCalibrating() != 0u)
      {
        uint8 origin_r = NavFusion_FeedOriginSample(gnss.latitude,
                                                    gnss.longitude,
                                                    gnss.state,
                                                    gnss.satellite_used);
        if (origin_r == NAV_FUSION_ORIGIN_FEED_DONE)
        {
          Nag_CompleteReplayAfterOrigin();
          GPS_CompleteLaunchAfterOrigin();
        }
        else if (origin_r == NAV_FUSION_ORIGIN_FEED_FAILED)
        {
          GPS_OnOriginCalibrationFailed();
        }
      }
#if NAV_FUSION_HEADING_CALIB_ENABLE
      else if (NavFusion_IsHeadingAlignPending() != 0u)
      {
        uint8 heading_r = NavFusion_FeedHeadingAlignSample((float)gnss.direction,
                                                           gnss.state,
                                                           (float)euler_angle.yaw);
        if (heading_r == NAV_FUSION_HEADING_FEED_DONE)
        {
          Nag_CompleteReplayAfterOrigin();
        }
      }
#endif
      else if (NavFusion_IsGpsPositionUpdateAllowed() != 0u)
      {
        NavFusion_UpdateGps(gnss.latitude,
                            gnss.longitude,
                            gnss.state,
                            gnss.satellite_used);
      }
#else
      NavFusion_UpdateGps(gnss.latitude,
                          gnss.longitude,
                          gnss.state,
                          gnss.satellite_used);
#endif
#endif
    }

    run_soft_tasks();
#if NAV_FUSION_ENABLE && NAV_FUSION_ORIGIN_ENABLE
    if (NavFusion_ConsumeOriginFailure() != 0u)
    {
      GPS_OnOriginCalibrationFailed();
      if (N.Nag_SystemRun_Index == 1u)
      {
        Nag_Request_Stop_Record();
      }
    }
#endif
#if NAV_FUSION_ENABLE && NAV_FUSION_ORIGIN_ENABLE && NAV_FUSION_HEADING_CALIB_ENABLE
    if (NavFusion_ConsumeOriginDonePulse() != 0u)
    {
      buzzer_check(100u);
      buzzer_check(100u);
    }
#endif
#if NAV_FUSION_ENABLE && NAV_FUSION_HEADING_CALIB_ENABLE
    if (NavFusion_ConsumeHeadingCalibDonePulse() != 0u)
    {
      buzzer_check(100u);
      Nag_CompleteReplayAfterOrigin();
    }
    if (NavFusion_ConsumeHeadingCalibFailure() != 0u)
    {
      buzzer_check(500u);
      if (N.Nag_SystemRun_Index == 1u)
      {
        Nag_Request_Stop_Record();
      }
    }
#endif
#if NAV_FUSION_ENABLE && NAV_FUSION_ORIGIN_ENABLE && !NAV_FUSION_HEADING_CALIB_ENABLE
    if (NavFusion_IsValid() != 0u)
    {
      Nag_CompleteReplayAfterOrigin();
      GPS_CompleteLaunchAfterOrigin();
    }
#endif
#if DUALCORE_UI_ON_CM7_1
    /* 先消费 CM7_1 菜单命令，再发布快照；否则 Launch 调速、SaveSpd 保存和导航按键都不会真正落到控制核。 */
    dualcore_ui_cmd_consume_all();
    /* 验证用：左摇杆速度 + 左杆键翻转 Motor_Switch；正式策略可迁到 control/navigation */
    remote_lora_apply_validate_motor();
    Nag_BridgeDetectUpdate();
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
