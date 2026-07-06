#include "vofa.h"
#include "init.h"
#include <stdlib.h>
#include <string.h>
const uint8_t vofa_justfloat_frame_tail[4] = {0x00, 0x00, 0x80, 0x7f};
#define Wired_Mode 1
float ReadBuf_Pid = 0;
extern int16 pwm_ph1;
extern int16 pwm_ph2;
extern int16 pwm_ph3;
extern int16 pwm_ph4;
void SendDataStreamToVOFA(uint32_t Count, float Data, ...)
{
    volatile float Temp = Data;
    va_list Args;
    va_start(Args, Data);
#if Wired_Mode
    wireless_uart_send_buffer((uint8 *)&Data, sizeof(float));
#else
    uart_write_buffer(UART_1, (uint8 *)&Data, sizeof(float));
#endif
    for (uint32_t i = 1; i < Count; i++)
    {
        Temp = va_arg(Args, double);
#if Wired_Mode
        wireless_uart_send_buffer((uint8 *)&Temp, sizeof(float));
#else
        uart_write_buffer(UART_1, (uint8 *)&Temp, sizeof(float));
#endif
    }
#if Wired_Mode
    wireless_uart_send_buffer((uint8 *)vofa_justfloat_frame_tail, 4);
#else
    uart_write_buffer(UART_1, (uint8 *)vofa_justfloat_frame_tail, 4);
#endif
    va_end(Args);
}
uint32 WirelessUart_ReadBuff_Count = 0;
uint8 WirelessUart_ReadBuff_Data[64] = {0};
uint8 Menu_command = 0;
void ReadDataFromPc()
{
#if Wired_Mode
    WirelessUart_ReadBuff_Count = wireless_uart_read_buffer(WirelessUart_ReadBuff_Data, sizeof(WirelessUart_ReadBuff_Data));
    if (WirelessUart_ReadBuff_Count)
    {
        if (Menu_TryConsumePcMotorSpeedString(WirelessUart_ReadBuff_Data, WirelessUart_ReadBuff_Count))
        {
            memset(WirelessUart_ReadBuff_Data, 0, WirelessUart_ReadBuff_Count);
            WirelessUart_ReadBuff_Count = 0;
        }
        else
        {
        Menu_command = WirelessUart_ReadBuff_Data[0];
        if (WirelessUart_ReadBuff_Data[0] >= '0' && WirelessUart_ReadBuff_Data[0] <= '9')
        {
            ReadBuf_Pid = atof((const char *)&WirelessUart_ReadBuff_Data[0]);
        }
        if (WirelessUart_ReadBuff_Data[0] == 's')
        {
        }
        if (WirelessUart_ReadBuff_Count >= 3) {
            if (WirelessUart_ReadBuff_Data[0] == 'p') {
                switch (WirelessUart_ReadBuff_Data[1]) {
                    case '1':
                        if (WirelessUart_ReadBuff_Data[2] == '+') {
                            pwm_ph1 += 10;
                        } else if (WirelessUart_ReadBuff_Data[2] == '-') {
                            pwm_ph1 -= 10;
                        }
                        break;
                    case '2':
                        if (WirelessUart_ReadBuff_Data[2] == '+') {
                            pwm_ph2 += 10;
                        } else if (WirelessUart_ReadBuff_Data[2] == '-') {
                            pwm_ph2 -= 10;
                        }
                        break;
                    case '3':
                        if (WirelessUart_ReadBuff_Data[2] == '+') {
                            pwm_ph3 += 10;
                        } else if (WirelessUart_ReadBuff_Data[2] == '-') {
                            pwm_ph3 -= 10;
                        }
                        break;
                    case '4':
                        if (WirelessUart_ReadBuff_Data[2] == '+') {
                            pwm_ph4 += 10;
                        } else if (WirelessUart_ReadBuff_Data[2] == '-') {
                            pwm_ph4 -= 10;
                        }
                        break;
                }
            }
        }
        memset(WirelessUart_ReadBuff_Data, 0, WirelessUart_ReadBuff_Count);
        WirelessUart_ReadBuff_Count = 0;
        }
    }
#else
    WirelessUart_ReadBuff_Count = debug_read_ring_buffer(WirelessUart_ReadBuff_Data, sizeof(WirelessUart_ReadBuff_Data));
    if (WirelessUart_ReadBuff_Count)
    {
        debug_send_buffer(ReadPos, sizeof(ReadPos));
        if (Menu_TryConsumePcMotorSpeedString(WirelessUart_ReadBuff_Data, WirelessUart_ReadBuff_Count))
        {
            memset(WirelessUart_ReadBuff_Data, 0, WirelessUart_ReadBuff_Count);
            WirelessUart_ReadBuff_Count = 0;
        }
        else
        {
        Menu_command = WirelessUart_ReadBuff_Data[0];
        if (WirelessUart_ReadBuff_Data[0] >= '0' && WirelessUart_ReadBuff_Data[0] <= '9')
        {
            ReadBuf_Pid = atof((const char *)&WirelessUart_ReadBuff_Data[0]);
        }
        if (WirelessUart_ReadBuff_Count >= 3) {
            if (WirelessUart_ReadBuff_Data[0] == 'p') {
                switch (WirelessUart_ReadBuff_Data[1]) {
                    case '1':
                        if (WirelessUart_ReadBuff_Data[2] == '+') {
                            pwm_ph1 += 10;
                        } else if (WirelessUart_ReadBuff_Data[2] == '-') {
                            pwm_ph1 -= 10;
                        }
                        break;
                    case '2':
                        if (WirelessUart_ReadBuff_Data[2] == '+') {
                            pwm_ph2 += 10;
                        } else if (WirelessUart_ReadBuff_Data[2] == '-') {
                            pwm_ph2 -= 10;
                        }
                        break;
                    case '3':
                        if (WirelessUart_ReadBuff_Data[2] == '+') {
                            pwm_ph3 += 10;
                        } else if (WirelessUart_ReadBuff_Data[2] == '-') {
                            pwm_ph3 -= 10;
                        }
                        break;
                    case '4':
                        if (WirelessUart_ReadBuff_Data[2] == '+') {
                            pwm_ph4 += 10;
                        } else if (WirelessUart_ReadBuff_Data[2] == '-') {
                            pwm_ph4 -= 10;
                        }
                        break;
                }
            }
        }
        memset(WirelessUart_ReadBuff_Data, 0, WirelessUart_ReadBuff_Count);
        WirelessUart_ReadBuff_Count = 0;
        }
    }
#endif
}
void SendDataToVofa()
{
}
#if defined(CY_CORE_CM7_1)
#include "dualcore_shared.h"
#include "navigation.h"
void vofa_send_nav_from_dualcore_snapshot(void)
{
  dualcore_ctrl_to_ui_t c;
  dualcore_ctrl_to_ui_pull(&c);
  uint8 g = (uint8)(c.nag_vofa_group % NAG_VOFA_GROUP_COUNT);
  switch (g)
  {
  case 0:
    SendDataStreamToVOFA(4, c.euler_pitch, c.euler_roll, c.euler_yaw, c.gyro_z_bias_mean);
    break;
  case VOFA_GROUP_SPEED_DEBUG:
    SendDataStreamToVOFA(2, c.speed_target_effective, c.car_speed);
    break;
  case VOFA_GROUP_FUSION_DEBUG:
    SendDataStreamToVOFA(6,
                         c.fusion_x_m,
                         c.fusion_y_m,
                         c.fusion_v_mps,
                         c.fusion_gps_residual_m,
                         c.fusion_hold_dist_m,
                         c.fusion_heading_bias_deg);
    break;
  case VOFA_GROUP_ODO_SLIP_DEBUG:
    SendDataStreamToVOFA(6,
                         c.odo_wheel_left_cmps,
                         c.odo_wheel_right_cmps,
                         c.odo_gyro_z_dps,
                         c.odo_vc_from_l_cmps,
                         c.odo_vc_from_r_cmps,
                         c.odo_corr_speed_cmps);
    break;
  default:
    break;
  }
}
#endif
