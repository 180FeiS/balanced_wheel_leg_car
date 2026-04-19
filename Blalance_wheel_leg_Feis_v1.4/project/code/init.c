#include "zf_common_headfile.h"


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
  // 菜单初始化
  if (menu_flag == 1)
  {
    MenuInit();
  }

  if (key_flag == 1 && menu_flag == 1)
  {
    /* MENU_INPUT_REMOTE_MENU_FIRST==1 时此处不读拨码，Motor_Switch 等保持默认直至控制/串口侧更新。 */
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
  ips200_show_init();
  mt9v03x_init();
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
