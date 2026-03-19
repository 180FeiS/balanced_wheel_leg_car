 #include "zf_common_headfile.h"
#ifndef CODE_INIT_H_
#define CODE_INIT_H_


/*********************************************************************参数*********************************************************************/
#define BUZZER_PIN              (P19_4)                 //定义主板上蜂鸣器对应引脚

#define SERVO_1                 (TCPWM_CH58_P17_3)      // 定义主板上舵机1对应引脚 左前 TCPWM_CH17_P00_1
#define SERVO_2                 (TCPWM_CH57_P17_4)      // 定义主板上舵机2对应引脚 左后 TCPWM_CH18_P00_0
#define SERVO_3                 (TCPWM_CH31_P10_3)      // 定义主板上舵机3对应引脚 右前 TCPWM_CH13_P00_3
#define SERVO_4                 (TCPWM_CH30_P10_2)      // 定义主板上舵机4对应引脚 右后 TCPWM_CH14_P00_2  

// #define SERVO_1                 (TCPWM_CH30_P10_2)      // 定义主板上舵机1对应引脚 左前 TCPWM_CH17_P00_1
// #define SERVO_2                 (TCPWM_CH31_P10_3)      // 定义主板上舵机2对应引脚 左后 TCPWM_CH18_P00_0
// #define SERVO_3                 (TCPWM_CH58_P17_3)      // 定义主板上舵机3对应引脚 右前 TCPWM_CH13_P00_3
// #define SERVO_4                 (TCPWM_CH57_P17_4)      // 定义主板上舵机4对应引脚 右后 TCPWM_CH14_P00_2       
                                                        // 引脚定义错误，现象也会错误
#define SERVO_FREQ              (300)                   //定义主板上舵机频率
#define SERVO_LEFT              (55)                    //左极限值 待测
#define SERVO_RIGHT             (170)                   //右极限值 待测
#define SERVO1_MID              (4501)//(4457)          //舵机1中值     左上 待测
#define SERVO2_MID              (5312)//(4512)          //舵机2中值     左下 待测 大 上
#define SERVO3_MID              (4104)//(4963)          //舵机3中值     右上 待测 小 下
#define SERVO4_MID              (4163)//(4533)          //舵机4中值     右 下 待测大 下



#define SWITCH1                 (P21_5)
#define SWITCH2                 (P21_6)

#define LEG_DEBUG_MODE          (0)   // 0:正常模式(五连杆解算)  1:腿部调试模式(VOFA串口控制pwm)
/*********************************************************************参数*********************************************************************/


/*********************************************************************函数*********************************************************************/
void all_init(uint8 camera_flag, uint8 seekfree_flag, uint8 vofa_flag, uint8 imu_flag, uint8 servo_flag, uint8 foc_flag, uint8 pid_flag,uint8 guandao_flag, uint8 ekf_flag, uint8 key_flag, uint8 pit_flag, uint8 time_flag);   //总初始化

void buzzer_init(void);                     //蜂鸣器初始化

void buzzer_check(uint32 buzzer_time);      //蜂鸣器检测

void my_camera_init(void);                  //摄像头初始化

void camera_init_ips200(void);              //屏幕和摄像头初始化

void ips200_show_init(void);                //屏幕初始化

void servo_init(void);                      //舵机初始化


/*********************************************************************函数*********************************************************************/


#endif /* CODE_INIT_H_ */
