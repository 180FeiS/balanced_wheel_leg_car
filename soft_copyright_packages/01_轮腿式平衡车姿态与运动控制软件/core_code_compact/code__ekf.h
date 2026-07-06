#include "zf_common_headfile.h"
#ifndef CODE_EKF_H_
#define CODE_EKF_H_
/*********************************************************************����*********************************************************************/
extern float v_hat, x_hat; // ��ʵ�ٶȡ�λ��
extern float acc_x_lpf;    // ���ٶȵ�ͨ�˲�
extern float dt; // ekf��������
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
/* gyro_z ��ƫ�۲⣺��ֹ 10s ������� gyro_z ��ֵ����λ rad/s */
extern float gyro_z_bias_mean;
/* yaw ���ƫ�ƣ�euler_angle.yaw Ϊƫ�ƺ���yaw_raw_deg Ϊ EKF ԭʼ���� */
extern float yaw_raw_deg;
extern float yaw_zero_offset_deg;
/*********************************************************************����*********************************************************************/
/*********************************************************************����*********************************************************************/
void EKF_Init(void); // EKF��ʼ��
float Yaw_GetDeg(void);           // ƫ�ƺ���ͬ euler_angle.yaw��
float Yaw_GetRawDeg(void);        // EKF ԭʼ yaw��δ�����ƫ��
float Yaw_GetZeroOffsetDeg(void); // ��ǰ yaw ���ƫ��
void Yaw_ResetZero(void);         // ����ǰԭʼ yaw ��Ϊ����㣬ƫ�ƺ� yaw ����Ϊ 0�������п���ʱ����
void imu_get_values(void); // �õ�imuԭʼֵ
void EKF_UpData(void); // ����EKF����
void SOSFilter(float *input, float *output, int length); // Direct Form II ���׽��˲�
void EKF_V_UPData(void); // ����EKF�õ���ʵ��λ�ơ��ٶ�
/*********************************************************************����*********************************************************************/
#endif /* CODE_EKF_H_ */
