#include "zf_common_headfile.h"
#ifndef CODE_EKF_H_
#define CODE_EKF_H_
extern float v_hat, x_hat;
extern float acc_x_lpf;
extern float dt;
extern float dt_ekf;
extern float acc_b[3];
typedef struct
{
        float gyro_x;
        float gyro_y;
        float gyro_z;
        float acc_x;
        float acc_y;
        float acc_z;
} imu_t;
extern imu_t imu_data;
extern volatile float car_speed;
extern float gyro_z_bias_mean;
extern float yaw_raw_deg;
extern float yaw_zero_offset_deg;
void EKF_Init(void);
float Yaw_GetDeg(void);
float Yaw_GetRawDeg(void);
float Yaw_GetZeroOffsetDeg(void);
void Yaw_ResetZero(void);
void imu_get_values(void);
void EKF_UpData(void);
void SOSFilter(float *input, float *output, int length);
void EKF_V_UPData(void);
#endif
