#include "zf_common_headfile.h"
#include "image.h"


/*-------------------------------------------------------------------------------------------------------------------
// 函数简介     总初始化
// 参数说明     null
// 返回参数     null
// 使用示例     all_init();
// 备注信息     主函数中调用
-------------------------------------------------------------------------------------------------------------------*/
void all_init(uint8 camera_flag, uint8 seekfree_flag, uint8 vofa_flag,
              uint8 imu_flag, uint8 servo_flag, uint8 foc_flag, uint8 pid_flag,uint8 guandao_flag,
              uint8 ekf_flag, uint8 key_flag, uint8 pit_flag, uint8 menu_flag)
{
  // 屏幕初始化
  if (camera_flag == 1)
  {
    camera_init_ips200();
    step_detection_init(); //台阶检测初始化
  }
  else
  {
    // my_camera_init();
  }

  // 逐飞助手上位机初始化
  if (seekfree_flag == 1)
  {
    // seekfree_uart_init();
  }

  // vofa上位机初始化
  if (vofa_flag == 1)
  {
    // vofa_init();
    // send_enable();
    wireless_uart_init();
  }

  // IMU传感器初始化（660RC：关闭片内四元数输出，姿态由 EKF 解算）
  if (imu_flag == 1)
  {
    imu660rc_init(IMU660RC_QUARTERNION_DISABLE);
  }

  if (servo_flag == 1)
  {
    servo_init();
  }

#if !defined(CY_CORE_CM7_1)
  // 无刷电机初始化
  if (foc_flag == 1)
  {
    small_driver_uart_init();
  }
  // PID控制初始化
  if (pid_flag == 1)
  {
    pid_ctrl_Init();
  }
  if (guandao_flag == 1)
  {
    flash_init();//CYT系列独有的Flash初始化。!!!!!!!!!
    Init_Nag();
    flash_RunLaunchSpeed_Read();
    flash_GpsPoints_Read();
  }
  // 姿态解算初始化
  if (ekf_flag == 1)
  {
    EKF_Init();
  }
#endif /* !CY_CORE_CM7_1 */

  // 按键初始化
  if (key_flag == 1)
  {
    key_init (10);
    gpio_init(LED1, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(SWITCH1, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(SWITCH2, GPI, GPIO_HIGH, GPI_PULL_UP);
  }

  // 中断初始化
  if (pit_flag == 1)
  {
    pit_ms_init(PIT_CH0, 1);
    pit_ms_init(PIT_CH1, 5);
    pit_ms_init(PIT_CH2, 10);
    pit_ms_init(PIT_CH10, 20);
    pit_ms_init(PIT_CH11, 10);

  }
#if defined(CY_CORE_CM7_1)
  else if (key_flag == 1)
  {
    /*
     * CM7_1 UI 核：`all_init_cm7_1_ui` 传 pit=0 时仍须注册本核实际用到的 PIT。
     *
     * 根因（勿再对 PIT_CH1 调 pit_ms_init）：
     * zf_driver_pit.c 中 PIT_CH0~2 均绑定 TCPWM0->GRP[2].CNT[pit_index]，片上只有一份物理实例；
     * CM7_0 已在 pit_flag==1 路径初始化 CNT[1]，供 pit0_ch1_isr -> leg_control()（约 5ms 舵机腿控）。
     * CM7_1 若再 pit_init(PIT_CH1)，会二次配置同一计数器/中断路由，导致腿控节拍异常、舵机似卡死；
     * 混烧两核不同版本时只要 CM7_1 仍初始化 CH1 即可复现，与 CM7_0 单核源码是否回退无关。
     *
     * 避免准则：每个 pit_index 仅允许一个内核调用 pit_init；另一核用共享内存+对方节拍，或本核未占用的通道。
     * 视觉跳跃落地冷却：用 pit0_ch0（1ms）内 step_visual_jump_post_jump_cooldown_on_cm7_1_1ms() 做 ÷5，等效原 5ms 递减。
     *
     * 注：PIT_CH0 仍可能被 CM7_0与CM7_1 各 init 一次；若日后要根除，须在 TRM/驱动层约定 owner 核，本补丁不改动 CM7_0。
     */
    pit_ms_init(PIT_CH0, 1);
    pit_ms_init(PIT_CH2, 10);
  }
#endif
  // 菜单初始化
  if (menu_flag == 1)
  {
    MenuInit();
  }

  if (key_flag == 1 && menu_flag == 1)
  {
    /* g_menu_input_remote_first==1 时此处不读拨码，Motor_Switch 等保持默认直至控制/串口侧更新。 */
    dip_switch_motor_sync_from_hw();
  }
}

/*-------------------------------------------------------------------------------------------------------------------
// 函数简介     CM7_0：运动控制核初始化（无摄像头/无菜单/无无线 VOFA）
-------------------------------------------------------------------------------------------------------------------*/
void all_init_cm7_0_control(void)
{
  all_init(0, 0, 0,
           1, 1, 1, 1, 1, 1,
           1, 1, 0);
}

/*-------------------------------------------------------------------------------------------------------------------
// 函数简介     CM7_1：图像与菜单核初始化（摄像头+IPS+菜单+按键+无线；无 IMU/电机/控制定时器）
-------------------------------------------------------------------------------------------------------------------*/
void all_init_cm7_1_ui(void)
{
  all_init(1, 0, 1,
           0, 0, 0, 0, 0, 0,
           1, 0, 1);
}

/*-------------------------------------------------------------------------------------------------------------------
// 函数简介     TAU1201 GPS初始化
// 备注信息     参照逐飞GNSS例程，CM7_0主程序在debug_init之后调用
-------------------------------------------------------------------------------------------------------------------*/
void gnss_module_init(void)
{
#if GNSS_MODULE_ENABLE
  gnss_init(TAU1201);
#endif
}

/*-------------------------------------------------------------------------------------------------------------------
// 函数简介     蜂鸣器初始化
// 参数说明     null
// 返回参数     null
// 使用示例     buzzer_init();
// 备注信息     总初始化中调用
-------------------------------------------------------------------------------------------------------------------*/
void buzzer_init(void)
{
  gpio_init(BUZZER_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);
}

/*-------------------------------------------------------------------------------------------------------------------
// 函数简介     蜂鸣器检测
// 参数说明     buzzer_time         延时时间ms
// 返回参数     null
// 使用示例     buzzer_check(100);
// 备注信息     无
-------------------------------------------------------------------------------------------------------------------*/
void buzzer_check(uint32 buzzer_time)
{
  gpio_set_level(BUZZER_PIN, GPIO_HIGH);
  system_delay_ms(buzzer_time);
  gpio_set_level(BUZZER_PIN, GPIO_LOW);
}

/*-------------------------------------------------------------------------------------------------------------------
// 函数简介     摄像头初始化
// 参数说明     null
// 返回参数     null
// 使用示例     my_camera_init();
// 备注信息     总初始化中调用
-------------------------------------------------------------------------------------------------------------------*/
void my_camera_init(void)
{
  while (1)
  {
    if (mt9v03x_init())
    {
    }
    else
    {
      break;
    }
  }
}

/*-------------------------------------------------------------------------------------------------------------------
// 函数简介     屏幕和摄像头初始化
// 参数说明     null
// 返回参数     null
// 使用示例     camera_init_ips200();
// 备注信息     总初始化中调用
-------------------------------------------------------------------------------------------------------------------*/
void camera_init_ips200(void)
{
  static uint8 s_camera_flash_inited = 0u;

  if (s_camera_flash_inited == 0u)
  {
    flash_init();
    s_camera_flash_inited = 1u;
  }
  image_camera_exposure_flash_read();

  ips200_show_init();
  if (mt9v03x_init() == 0u)
  {
    (void)mt9v03x_set_exposure_time(image_camera_exposure);
  }
  ips200_show_string(0, 0, "mt9v03x_init...");
  ips200_show_string(0, 16, "init success...");
  ips200_clear();
}

/*-------------------------------------------------------------------------------------------------------------------
// 函数简介     屏幕初始化
// 参数说明     null
// 返回参数     null
// 使用示例     ips200_show_init();
// 备注信息     总初始化中调用
-------------------------------------------------------------------------------------------------------------------*/
void ips200_show_init(void)
{
  ips200_set_color(RGB565_BLACK, RGB565_WHITE); // 设置颜色为彩色
  ips200_set_font(IPS200_8X16_FONT);            // 设置字体大小为8*16像素
  ips200_init(IPS200_TYPE_SPI);

  ips200_set_dir(IPS200_PORTAIT); // 设置显示方向，图像和字体可以在屏幕上横着或竖着显示
}

/*-------------------------------------------------------------------------------------------------------------------
// 函数简介     舵机初始化
// 参数说明     null
// 返回参数     null
// 使用示例     servo_init();
// 备注信息     总初始化中调用
-------------------------------------------------------------------------------------------------------------------*/
void servo_init(void)
{
  pwm_init(SERVO_1, SERVO_FREQ, SERVO1_MID);
  pwm_init(SERVO_2, SERVO_FREQ, SERVO2_MID);
  pwm_init(SERVO_3, SERVO_FREQ, SERVO3_MID);
  pwm_init(SERVO_4, SERVO_FREQ, SERVO4_MID);

#if LEG_DEBUG_MODE
  leg_debug_init_pwm();
#endif
}
