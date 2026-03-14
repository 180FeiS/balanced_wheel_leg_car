#include "zf_common_headfile.h"


/*-------------------------------------------------------------------------------------------------------------------
// 函数简介     总初始化
// 参数说明     null
// 返回参数     null
// 使用示例     all_init();
// 备注信息     主函数中调用
-------------------------------------------------------------------------------------------------------------------*/
void all_init(uint8 camera_flag, uint8 seekfree_flag, uint8 vofa_flag,
              uint8 imu_flag, uint8 servo_flag, uint8 foc_flag, uint8 pid_flag,
              uint8 ekf_flag, uint8 key_flag, uint8 pit_flag, uint8 menu_flag)
{
  // 屏幕初始化
  if (camera_flag == 1)
  {
    camera_init_ips200();
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

  // IMU传感器初始化
  if (imu_flag == 1)
  {
    imu660ra_init();
  }

  if (servo_flag == 1)
  {
    servo_init();
  }

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

  // 姿态解算初始化
  if (ekf_flag == 1)
  {
    EKF_Init();
  }

  // Flash初始化
  flash_init();
  Init_Nag();
  gyroOffset_init();//开机延迟2S    去零飘

  // 按键初始化
  if (key_flag == 1)
  {
    key_init (10);
  }

  // 中断初始化
  if (pit_flag == 1)
  {
    pit_ms_init(PIT_CH0, 1);
    pit_ms_init(PIT_CH1, 5);
    pit_ms_init(PIT_CH2, 10);
    pit_ms_init(PIT_CH10, 20);
    pit_ms_init(PIT_CH11, 50);

  }
  // 菜单初始化
  if (menu_flag == 1)
  {
    MenuInit();
  }
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
}
