


#include "zf_common_headfile.h"

uint16 jump_test = 1;

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
           0,  // 是否开启PID标志位              //0:关闭          1:开启
           1,  // 是否开启姿态解算标志位           //0:关闭          1:开启
           1,  // 是否开启按键初始化标志位           //0:关闭          1:开启
           1,  // 是否开启中断标志位              //0:关闭          1:开启
           1); // 是否开启菜单初始化标志位        //0:关闭          1:开启

  // 此处编写用户代码 例如外设初始化代码等

  while (true)
  {
     //small_driver_set_duty(500, -500); // 计算占空比输出

    // SendDataStreamToVOFA(1,(float)1);
      
      //ips200_show_char(120,120,Menu_command);
      //ips200_show_float(80,80,ReadBuf_Pid,1,1);
    //jump_flag = ReadBuf_Pid;
    //SendDataStreamToVOFA(5, (float)euler_angle.pitch, (float)euler_angle.roll, (float)car_speed,(float)motor_value.receive_left_speed_data, (float)motor_value.receive_right_speed_data);
    // printf("\r\npitch=%f, roll=%f", euler_angle.pitch,  euler_angle.roll);
     //printf("\r\npitch=%f, roll=%f, speed=%d\r\n", euler_angle.pitch, euler_angle.roll,((-motor_value.receive_left_speed_data + motor_value.receive_right_speed_data)/2));
    //printf("\r\npwm_ph4=%d, pwm_ph1=%d\r\n", pwm_4, pwm_1);
     SendDataStreamToVOFA(6, (float)euler_angle.pitch, (float)euler_angle.roll, (float)car_speed,(float)-motor_value.receive_left_speed_data, (float)motor_value.receive_right_speed_data,(float)Motor_Switch);
     

     //printf("left speed:%d, right speed:%d，car_speed:%f\r\n", motor_value.receive_left_speed_data, motor_value.receive_right_speed_data,car_speed);

    //printf("left speed:%d, right speed:%d\r\n", motor_value.receive_left_speed_data, motor_value.receive_right_speed_data);

    

    // 此处编写需要循环执行的代码
  }
}

// **************************** 代码区域 ****************************
