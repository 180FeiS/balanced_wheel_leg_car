/*
 * Kalman.h
 *
 *  Created on: 2023年10月1日
 *      Author: Monst
 */
#ifndef __KALMAN_H_
#define __KALMAN_H_

typedef struct {
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float acc_x;
    float acc_y;
    float acc_z;
} icm_param_t;


typedef struct {
    float q0;
    float q1;
    float q2;
    float q3;
} quater_param_t;


typedef struct {
    float pitch;    //俯仰角
    float roll;     //偏航角
    float yaw;      //翻滚角
    float last_yaw;
    int8 Dirchange;
} euler_param_t;


typedef struct {
    float Xdata;
    float Ydata;
    float Zdata;
} gyro_param_t;



extern euler_param_t eulerAngle;
extern float angle_Z,angle_R,angle_P;

extern bool GyroOffset_init;
void gyroOffset_init(void);

float fast_sqrt(float x);

void ICM_AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az);

void ICM_getValues();

void ICM_getEulerianAngles(void);
#endif /* CODE_KALMAN_H_ */
