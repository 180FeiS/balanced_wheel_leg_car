


#include "zf_common_headfile.h"

// 外部全局PWM参数变量
extern int16 pwm_ph1;
extern int16 pwm_ph2;
extern int16 pwm_ph3;
extern int16 pwm_ph4;

// gyro_z 零偏观测（ekf.h 中声明）
extern float gyro_z_bias_mean;

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
           1,  // 是否开启PID标志位              //0:关闭          1:开启
           1,  // 是否开启惯导标志位              //0:关闭          1:开启
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
    SendDataStreamToVOFA(7, (float)euler_angle.pitch, (float)euler_angle.roll,(float)euler_angle.yaw, (float)N.Mileage_All,(float)N.Save_index,(float)R_Mileage,(float)L_Mileage);
#endif
   

     //printf("left speed:%d, right speed:%d，car_speed:%f\r\n", motor_value.receive_left_speed_data, motor_value.receive_right_speed_data,car_speed);

    //printf("left speed:%d, right speed:%d\r\n", motor_value.receive_left_speed_data, motor_value.receive_right_speed_data);

    

    // 此处编写需要循环执行的代码
  }
}

// **************************** 代码区域 ****************************
