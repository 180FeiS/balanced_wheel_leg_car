#include "zf_common_headfile.h"
#ifndef CODE_VMC_H_
#define CODE_VMC_H_



/*********************************************************************参数*********************************************************************/
#define L1  6.0f    //左小腿长
#define L2  9.0f    //左大腿长
#define L3  9.0f    //右大腿长
#define L4  6.0f    //右小腿长
#define L5  3.7f    //舵机间距

/* PWM 查表角度范围（vmc.c A_max/A_min），与 pwm_table 标定一致，全局生效，勿改 */
#define VMC_A_TABLE_MAX   25.0f
/* 腿倾角查表外推上限，须与 vmc.c 中 A_EXT_MAX 一致；非颠簸场景 control 层限幅用 */
#define VMC_A_EXT_MAX   60.0f


/*********************************************************************参数*********************************************************************/


/*********************************************************************函数*********************************************************************/
void servo_control_table(float p, float angle, int16 *pwm1, int16 *pwm2);       //根据舵机PWM表设置舵机输出

float fast_sqrt(float num);                                                     //快速开根号

float fast_invsqrt(float num);                                                  //快速平方根倒数

void servo_control(int16 x, int16 y, float *ph1, float *ph4);                   //根据坐标的舵机VMC结算

void servo_control_th(float P, float th, float *ph1, float *ph4);               //根据腿长和角度的舵机VMC结算
/*********************************************************************函数*********************************************************************/


#endif /* CODE_VMC_H_ */
