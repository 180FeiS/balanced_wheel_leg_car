#include "zf_common_headfile.h"

#define delta_T     0.0025f
//0.0025f
float I_ex, I_ey, I_ez;

quater_param_t Q_info = {1, 0, 0};
euler_param_t eulerAngle;

icm_param_t icm_data;
gyro_param_t GyroOffset;

bool GyroOffset_init = 0;

float param_Kp = 0.00;
float param_Ki = 0.0000;  //0.001

float angle_Z = 0,angle_R = 0,angle_P = 0;

static float fast_sqrt(float x) {
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long *) &y;
    i = 0x5f3759df - (i >> 1);
    y = *(float *) &i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}


void gyroOffset_init(void)
{
    imu660ra_init();  
    GyroOffset.Xdata = 0;
    GyroOffset.Ydata = 0;
    GyroOffset.Zdata = 0;
    uint8 cnt=200;
    for (uint16_t i = 0; i < cnt; ++i) {
    imu660ra_get_acc();
    imu660ra_get_gyro();
    GyroOffset.Xdata += imu660ra_gyro_x;
    GyroOffset.Ydata += imu660ra_gyro_y;
    GyroOffset.Zdata += imu660ra_gyro_z;
    system_delay_ms(10);
    }
    GyroOffset.Xdata /= (float)cnt;
    GyroOffset.Ydata /= (float)cnt;
    GyroOffset.Zdata /= (float)cnt;
    eulerAngle.Dirchange=0;
    GyroOffset_init = 1;
}


#define alpha           0.4f


void ICM_getValues() {

    icm_data.acc_x = (((float) imu660ra_acc_x) * alpha) * 8 / 4096 + icm_data.acc_x * (1 - alpha);
    icm_data.acc_y = (((float) imu660ra_acc_y) * alpha) * 8 / 4096 + icm_data.acc_y * (1 - alpha);
    icm_data.acc_z = (((float) imu660ra_acc_z) * alpha) * 8 / 4096 + icm_data.acc_z * (1 - alpha);



    icm_data.gyro_x = ((float) imu660ra_gyro_x - GyroOffset.Xdata) * GNSS_PI / 180 / 16.4f;
    icm_data.gyro_y = ((float) imu660ra_gyro_y - GyroOffset.Ydata) * GNSS_PI / 180 / 16.4f;
    icm_data.gyro_z = ((float) imu660ra_gyro_z - GyroOffset.Zdata) * GNSS_PI / 180 / 16.4f;
}



void ICM_AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az) {
    float halfT = 0.5 * delta_T;
    float vx, vy, vz;
    float ex, ey, ez;
    float q0 = Q_info.q0;
    float q1 = Q_info.q1;
    float q2 = Q_info.q2;
    float q3 = Q_info.q3;
    float q0q0 = q0 * q0;
    float q0q1 = q0 * q1;
    float q0q2 = q0 * q2;
    //float q0q3 = q0 * q3;
    float q1q1 = q1 * q1;
    //float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q3q3 = q3 * q3;
    float delta_2 = 0.17;

    float norm = fast_sqrt(ax * ax + ay * ay + az * az);
    ax = ax * norm;
    ay = ay * norm;
    az = az * norm;

    vx = 2 * (q1q3 - q0q2);
    vy = 2 * (q0q1 + q2q3);
    vz = q0q0 - q1q1 - q2q2 + q3q3;
    //vz = (q0*q0-0.5f+q3 * q3) * 2;

    ex = ay * vz - az * vy;
    ey = az * vx - ax * vz;
    ez = ax * vy - ay * vx;


    I_ex += halfT * ex;
    I_ey += halfT * ey;
    I_ez += halfT * ez;

    gx = gx + param_Kp * ex + param_Ki * I_ex;
    gy = gy + param_Kp * ey + param_Ki * I_ey;
    gz = gz + param_Kp * ez + param_Ki * I_ez;




    q0 = q0 + (-q1 * gx - q2 * gy - q3 * gz) * halfT;
    q1 = q1 + (q0 * gx + q2 * gz - q3 * gy) * halfT;
    q2 = q2 + (q0 * gy - q1 * gz + q3 * gx) * halfT;
    q3 = q3 + (q0 * gz + q1 * gy - q2 * gx) * halfT;
    delta_2=(2*halfT*gx)*(2*halfT*gx)+(2*halfT*gy)*(2*halfT*gy)+(2*halfT*gz)*(2*halfT*gz);

    q0 = (1-delta_2/8)*q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
    q1 = (1-delta_2/8)*q1 + (q0*gx + q2*gz - q3*gy)*halfT;
    q2 = (1-delta_2/8)*q2 + (q0*gy - q1*gz + q3*gx)*halfT;
    q3 = (1-delta_2/8)*q3 + (q0*gz + q1*gy - q2*gx)*halfT;



    norm = fast_sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    Q_info.q0 = q0 * norm;
    Q_info.q1 = q1 * norm;
    Q_info.q2 = q2 * norm;
    Q_info.q3 = q3 * norm;
}


void ICM_getEulerianAngles(void) {
    static uint8 change_f=0;
    ICM_getValues();
    ICM_AHRSupdate(icm_data.gyro_x, icm_data.gyro_y, icm_data.gyro_z, icm_data.acc_x, icm_data.acc_y, icm_data.acc_z);
    float q0 = Q_info.q0;
    float q1 = Q_info.q1;
    float q2 = Q_info.q2;
    float q3 = Q_info.q3;


    eulerAngle.pitch = asin(-2 * q1 * q3 + 2 * q0 * q2) * 180 / GNSS_PI; // pitch
    eulerAngle.roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1) * 180 / GNSS_PI; // roll
    eulerAngle.yaw =  atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2 * q2 - 2 * q3 * q3 + 1) * 180/ GNSS_PI; // yaw
//
    if (eulerAngle.yaw >= 180) { eulerAngle.yaw -= 360; } else if (eulerAngle.yaw <= -180) { eulerAngle.yaw += 360; }

    //偏航角和翻滚角
    if((eulerAngle.yaw-eulerAngle.last_yaw) < -350) eulerAngle.Dirchange++;
    else if ((eulerAngle.yaw-eulerAngle.last_yaw) > 350) eulerAngle.Dirchange--;

    angle_Z=360*eulerAngle.Dirchange+eulerAngle.yaw;//180*eulerAngle.Dirchange+eulerAngle.yaw
    eulerAngle.last_yaw=eulerAngle.yaw;

}
