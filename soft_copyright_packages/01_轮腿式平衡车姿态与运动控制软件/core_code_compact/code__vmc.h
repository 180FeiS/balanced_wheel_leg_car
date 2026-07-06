#include "zf_common_headfile.h"
#ifndef CODE_VMC_H_
#define CODE_VMC_H_
#define L1  6.0f
#define L2  9.0f
#define L3  9.0f
#define L4  6.0f
#define L5  3.7f
#define VMC_A_EXT_MAX   60.0f
void servo_control_table(float p, float angle, int16 *pwm1, int16 *pwm2);
float fast_sqrt(float num);
float fast_invsqrt(float num);
void servo_control(int16 x, int16 y, float *ph1, float *ph4);
void servo_control_th(float P, float th, float *ph1, float *ph4);
#endif
