#include "zf_common_headfile.h"
#define ikun (0.00215885305953045270200687624638f)
matrix_t exf_x;
matrix_t error;
EulerAngles euler_angle;
imu_t imu_data = {0, 0, 0, 0, 0, 0};
matrix_type r_yz = 0.01f;
float acc_b[3] = {0, 0, 9.8};
const matrix_type q[4][4] = {
    {0.007, 0, 0, 0}, {0, 0.007, 0, 0}, {0, 0, 0.007, 0}, {0, 0, 0, 0.007}};
const matrix_type r[3][3] = {{100000, 0, 0}, {0, 100000, 0}, {0, 0, 100000}};
const matrix_type p[4][4] = {{1000000, 0, 0, 0},
                             {0, 1000000, 0, 0},
                             {0, 0, 1000000, 0},
                             {0, 0, 0, 1000000}};
const matrix_type ekf[4] = {1, 0, 0, 0};
static matrix_t Q;
static matrix_t R;
static matrix_t P;
static int16 imu660rc_acc_x_l = 0;
static int16 imu660rc_acc_y_l = 0;
static int16 imu660rc_acc_z_l = 0;
static float gyro_z_sum = 0.0f;
static uint32_t gyro_z_count = 0u;
float yaw_raw_deg = 0.0f;
float yaw_zero_offset_deg = 0.0f;
static float PK[4] = {1000, 100, 100, 1000};
static float Kk[2] = {0, 0};
float Q_ekf[4] = {0.3, 0.001, 0.001, 0.2};
float R_ekf = 0.1;
float v_hat = 0;
float x_hat = 0;
float acc_x_lpf = 0;
const float K_V = 0.05;
float gyro_z_bias_mean = 0.0f;
#define GYRO_Z_BIAS_SAMPLES 10000u
#define GYRO_Z_BIAS_COMPENSATION 0.0001f
float numerator[3][3] = {
    {1.0, -1.4180, 1.0},
    {1.0, -0.8076, 1.0},
    {1.0, 1.0, 0}
};
float denominator[3][3] = {
    {1.0000, -1.8559, 0.9505}, {1.0000, -1.8218, 0.8639}, {1.0000, -0.9083, 0}};
float scaleValues[4] = {0.0493, 0.0353, 0.1511, 1.0000};
float dt = 0.001f;
float dt_ekf = 0.01f;
volatile float car_speed = 0;
static void quaternion_to_euler(void);
static float yaw_wrap180_deg(float yaw_deg)
{
  while (yaw_deg > 180.0f)
  {
    yaw_deg -= 360.0f;
  }
  while (yaw_deg < -180.0f)
  {
    yaw_deg += 360.0f;
  }
  return yaw_deg;
}
static float yaw_apply_zero_offset(float raw_yaw_deg)
{
  return yaw_wrap180_deg(raw_yaw_deg - yaw_zero_offset_deg);
}
float Yaw_GetDeg(void)
{
  return (float)euler_angle.yaw;
}
float Yaw_GetRawDeg(void)
{
  return yaw_raw_deg;
}
float Yaw_GetZeroOffsetDeg(void)
{
  return yaw_zero_offset_deg;
}
void Yaw_ResetZero(void)
{
  yaw_zero_offset_deg = yaw_raw_deg;
  euler_angle.yaw = yaw_apply_zero_offset(yaw_raw_deg);
}
void EKF_Init(void)
{
  Matrix_From_Array(&exf_x, (const matrix_type *)ekf, 4, 1);
  Matrix_From_Array(&Q, (const matrix_type *)q, 4, 4);
  Matrix_From_Array(&R, (const matrix_type *)r, 3, 3);
  Matrix_From_Array(&P, (const matrix_type *)p, 4, 4);
  imu660rc_get_acc();
  float ax = imu660rc_acc_x;
  float ay = imu660rc_acc_y;
  float az = imu660rc_acc_z;
  float norm = fast_invsqrt((float)ax * ax + ay * ay + az * az);
  ax *= norm;
  ay *= norm;
  az *= norm;
  float pitch = asinf(ax);
  float roll = atan2f(ay, az);
  float cy = cosf(roll * 0.5f);
  float sy = sinf(roll * 0.5f);
  float cp = cosf(pitch * 0.5f);
  float sp = sinf(pitch * 0.5f);
  exf_x.data[0][0] = cy * cp;
  exf_x.data[1][0] = cy * sp;
  exf_x.data[2][0] = sy * cp;
  exf_x.data[3][0] = sy * sp;
  normalize_vector(&exf_x);
}
static void quaternion_to_euler(void)
{
  const float G = 9.8;
  float q0 = (exf_x.data[0][0]);
  float q1 = (exf_x.data[1][0]);
  float q2 = (exf_x.data[2][0]);
  float q3 = (exf_x.data[3][0]);
  float ax = imu660rc_acc_transition(imu_data.acc_x) * 9.8f;
  float ay = imu660rc_acc_transition(imu_data.acc_y) * 9.8f;
  float az = imu660rc_acc_transition(imu_data.acc_z) * 9.8f;
  acc_b[0] = ax - (2 * G * q1 * q3 - 2 * G * q0 * q2);
  acc_b[1] = ay - (2 * G * q0 * q1 + 2 * G * q2 * q3);
  acc_b[2] = az - (G * q0 * q0 - G * q1 * q1 - G * q2 * q2 + G * q3 * q3);
  euler_angle.pitch = asinf(-2 * q1 * q3 + 2 * q0 * q2) * DEG_TO_RAD;
  euler_angle.roll =
      atan2f(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1) *
      DEG_TO_RAD;
  yaw_raw_deg =
      atan2f(2 * q1 * q2 + 2 * q0 * q3, -2 * q1 * q1 - 2 * q3 * q3 + 1) *
      DEG_TO_RAD;
  euler_angle.yaw = yaw_apply_zero_offset(yaw_raw_deg);
}
void imu_get_values(void)
{
  imu660rc_get_gyro();
  imu660rc_get_acc();
  imu_data.acc_x = K * (imu660rc_acc_x) + (1 - K) * imu660rc_acc_x_l;
  imu_data.acc_y = K * (imu660rc_acc_y) + (1 - K) * imu660rc_acc_y_l;
  imu_data.acc_z = K * (imu660rc_acc_z) + (1 - K) * imu660rc_acc_z_l;
  imu660rc_acc_x_l = imu_data.acc_x;
  imu660rc_acc_y_l = imu_data.acc_y;
  imu660rc_acc_z_l = imu_data.acc_z;
  imu_data.gyro_x = imu660rc_gyro_transition(imu660rc_gyro_x) * PI / 180.0f;
  imu_data.gyro_y = imu660rc_gyro_transition(imu660rc_gyro_y) * PI / 180.0f;
  imu_data.gyro_z = imu660rc_gyro_transition(imu660rc_gyro_z) * PI / 180.0f;
  imu_data.gyro_z -= GYRO_Z_BIAS_COMPENSATION;
}
void EKF_UpData(void)
{
  static uint16 time_now = 0;
  float gx, gy, gz;
  imu_get_values();
  gx = imu_data.gyro_x;
  gy = imu_data.gyro_y;
  gz = imu_data.gyro_z;
  gyro_z_sum += gz;
  gyro_z_count++;
  if (gyro_z_count >= GYRO_Z_BIAS_SAMPLES)
  {
    gyro_z_bias_mean = gyro_z_sum / (float)GYRO_Z_BIAS_SAMPLES;
    gyro_z_sum = 0.0f;
    gyro_z_count = 0u;
  }
  matrix_t Z;
  Matrix_Init(&Z, 3, 1);
  Z.data[0][0] = (matrix_type)imu_data.acc_x;
  Z.data[1][0] = (matrix_type)imu_data.acc_y;
  Z.data[2][0] = (matrix_type)imu_data.acc_z;
  normalize_vector(&Z);
  matrix_type f[4][4] = {{1, -0.5f * gx * dt, -0.5f * gy * dt, -0.5f * gz * dt},
                         {0.5f * gx * dt, 1, 0.5f * gz * dt, -0.5f * gy * dt},
                         {0.5f * gy * dt, -0.5f * gz * dt, 1, 0.5f * gx * dt},
                         {0.5f * gz * dt, 0.5f * gy * dt, -0.5f * gx * dt, 1}};
  matrix_t F, FT;
  Matrix_From_Array(&F, (const matrix_type *)f, 4, 4);
  FT = Matrix_Transpose(&F);
  exf_x = multiply_matrices(&F, &exf_x);
  normalize_vector(&exf_x);
  float q0 = (exf_x.data[0][0]);
  float q1 = (exf_x.data[1][0]);
  float q2 = (exf_x.data[2][0]);
  float q3 = (exf_x.data[3][0]);
  matrix_type h[3][4] = {{-2 * q2, 2 * q3, -2 * q0, 2 * q1},
                         {2 * q1, 2 * q0, 2 * q3, 2 * q2},
                         {2 * q0, -2 * q1, -2 * q2, 2 * q3}};
  matrix_t H, HT;
  Matrix_From_Array(&H, (const matrix_type *)h, 3, 4);
  HT = Matrix_Transpose(&H);
  matrix_t PK_;
  PK_ = multiply_matrices(&F, &P);
  PK_ = multiply_matrices(&PK_, &FT);
  P = add_matrices(&PK_, &Q);
  matrix_t DK, invDK;
  DK = multiply_matrices(&H, &P);
  DK = multiply_matrices(&DK, &HT);
  DK = add_matrices(&DK, &R);
  if (inverse_matrix(&DK, &invDK))
  {
    quaternion_to_euler();
    return;
  }
  matrix_t EK, EKT;
  EK = multiply_matrices(&H, &exf_x);
  EK = subtract_matrices(&Z, &EK);
  EKT = Matrix_Transpose(&EK);
  error = multiply_matrices(&EKT, &invDK);
  error = multiply_matrices(&error, &EK);
  if (error.data[0][0] > r_yz)
  {
    quaternion_to_euler();
    return;
  }
  matrix_t Kk;
  Kk = multiply_matrices(&P, &HT);
  Kk = multiply_matrices(&Kk, &invDK);
  matrix_t temp;
  temp = multiply_matrices(&Kk, &EK);
  exf_x = add_matrices(&exf_x, &temp);
  normalize_vector(&exf_x);
  matrix_t I;
  Matrix_Identity(&I, 4);
  temp = multiply_matrices(&Kk, &H);
  temp = subtract_matrices(&I, &temp);
  P = multiply_matrices(&temp, &P);
  quaternion_to_euler();
}
void SOSFilter(float *input, float *output, int length)
{
  float x[3][2] = {{0}};
  float y[3][2] = {{0}};
  for (int n = 0; n < length; n++)
  {
    float in = input[n];
    for (int i = 0; i < 3; i++)
    {
      float out = (numerator[i][0] * in + numerator[i][1] * x[i][0] +
                   numerator[i][2] * x[i][1]) /
                  (denominator[i][0] + denominator[i][1] * y[i][0] +
                   denominator[i][2] * y[i][1]);
      x[i][1] = x[i][0];
      x[i][0] = in;
      y[i][1] = y[i][0];
      y[i][0] = out;
      out *= scaleValues[i];
      in = out;
    }
    output[n] = in;
  }
}
void EKF_V_UPData(void)
{
  static uint16 ekf_time = 0;
  float a = -(acc_b[0] * cosf(euler_angle.pitch / DEG_TO_RAD) - 0.0195);
  SOSFilter(&a, &acc_x_lpf, 1);
  a = acc_x_lpf;
  x_hat = x_hat + v_hat * dt_ekf;
  v_hat = v_hat + a * dt_ekf;
  PK[0] = PK[0] + dt_ekf * PK[2] + dt_ekf * (PK[1] + dt_ekf * PK[3]) + Q_ekf[0];
  PK[1] = PK[1] + dt_ekf * PK[3] + Q_ekf[1];
  PK[2] = PK[2] + dt_ekf * PK[3] + Q_ekf[2];
  PK[3] = PK[3] + Q_ekf[3];
  Kk[0] = PK[1] / (PK[3] + R_ekf);
  Kk[1] = PK[3] / (PK[3] + R_ekf);
  float v = (motor_value.receive_left_speed_data +
             -motor_value.receive_right_speed_data) *
            ikun;
  x_hat = x_hat + Kk[0] * (v - v_hat);
  v_hat = v_hat + Kk[1] * (v - v_hat);
  PK[0] = PK[0] - Kk[0] * PK[2];
  PK[1] = PK[1] - Kk[0] * PK[3];
  PK[2] = -PK[2] * (Kk[1] - 1);
  PK[3] = -PK[3] * (Kk[1] - 1);
}
