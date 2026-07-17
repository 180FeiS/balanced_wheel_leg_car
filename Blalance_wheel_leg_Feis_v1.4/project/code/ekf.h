#include "zf_common_headfile.h"
#ifndef CODE_EKF_H_
#define CODE_EKF_H_

/*********************************************************************参数*********************************************************************/
extern float v_hat, x_hat; // 真实速度、位移
extern float acc_x_lpf;    // 加速度低通滤波

extern float dt; // ekf更新周期
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

/* gyro_z 零偏观测：静止 10s 后输出的 gyro_z 均值，单位 rad/s */
extern float gyro_z_bias_mean;

/* gyro_z 运行补偿量（rad/s）：imu_get_values 内执行 gyro_z -= gyro_z_bias_comp */
extern float gyro_z_bias_comp;

/* 标定状态：0=Idle 1=Running 2=Done */
#define GYRO_BIAS_CALIB_IDLE     0u
#define GYRO_BIAS_CALIB_RUNNING  1u
#define GYRO_BIAS_CALIB_DONE     2u

extern float yaw_drift_10s_deg; /* 10s 后 yaw 与 KEY3 起点的差值（度，±180） */
extern float gyro_bias_calib_yaw_start_deg; /* KEY3 按下瞬间的 yaw（度） */

/* yaw 零点偏移：euler_angle.yaw 为偏移后航向；yaw_raw_deg 为 EKF 原始解算 */
extern float yaw_raw_deg;
extern float yaw_zero_offset_deg;

/*********************************************************************参数*********************************************************************/

/*********************************************************************函数*********************************************************************/
void EKF_Init(void); // EKF初始化

float Yaw_GetDeg(void);           // 偏移后航向（同 euler_angle.yaw）
float Yaw_GetRawDeg(void);        // EKF 原始 yaw，未减零点偏移
float Yaw_GetUnwrappedDeg(void);  // 连续航向（度），不受 yaw_zero_offset_deg 影响，可跨 ±180° 累计
float Yaw_GetZeroOffsetDeg(void); // 当前 yaw 零点偏移
void Yaw_ResetZero(void);         // 将当前 raw 记为零点，显示 yaw 立即为 0（如 SWITCH2）
uint8 Yaw_AlignDisplayDeg(float target_display_yaw_deg); // 将 display yaw 对齐到目标角（仅改 offset，不改 unwrapped）

void imu_get_values(void); // 得到imu原始值

void EKF_UpData(void); // 更新EKF数据

void SOSFilter(float *input, float *output, int length); // Direct Form II 二阶节滤波

void EKF_V_UPData(void); // 更新EKF得到真实的位移、速度

void GyroBias_CalibStart(void);
uint8 GyroBias_GetCalibState(void);
float GyroBias_GetComp(void);
float GyroBias_GetYawDrift10sDeg(void);
float GyroBias_GetYawStartDeg(void);
uint8 GyroBias_GetRemainSec(void);
void GyroBias_SetComp(float bias_rad);
/*********************************************************************函数*********************************************************************/

#endif /* CODE_EKF_H_ */
