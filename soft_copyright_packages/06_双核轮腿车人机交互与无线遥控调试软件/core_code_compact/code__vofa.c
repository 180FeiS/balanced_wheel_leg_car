#include "vofa.h"
#include "init.h"
#include <stdlib.h>
#include <string.h>
const uint8_t vofa_justfloat_frame_tail[4] = {0x00, 0x00, 0x80, 0x7f};
#define Wired_Mode 1 // 0-有线模式    1-无线模式
float ReadBuf_Pid = 0;
// 外部全局PWM参数变量
extern int16 pwm_ph1;
extern int16 pwm_ph2;
extern int16 pwm_ph3;
extern int16 pwm_ph4;
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     以一个数据justfloat格式发送到VOFA
// 参数说明     uart_index_enum * UARTx     串口号
// 参数说明     uint32_t Count              发送数据个数
// 参数说明     float Data                  发送数据
// 返回参数     void
// 使用示例     SendDataStreamToVOFA(1,Car_AimSpeed);
//             SendDataStreamToVOFA(3,data1,data2,data3);
// 备注信息     V1.0.0  发送时数据需要强制转换float类型
//             V1.0.1  修复写入有线串口的bug             2024年7月26日
//-------------------------------------------------------------------------------------------------------------------
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
uint32 WirelessUart_ReadBuff_Count = 0;     // 读取无线串口接收数据长度
uint8 WirelessUart_ReadBuff_Data[64] = {0}; // 读取无线串口接收数据
uint8 Menu_command = 0;                     // 菜单控制指令
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     从电脑接收数据
// 参数说明     void
// 返回参数     void
// 使用示例     ReadDataFromPc(WirelessUart_ReadBuff_Data,WirelessUart_ReadBuff_Count);
// 备注信息     V1.0.0  发送时数据需要强制转换float类型
//             V1.0.1  修复写入有线串口的bug             2024年7月26日
//             V1.1.2  pid参数调节功能，需要加前缀a进行调节 2024年7月31日
//             V1.1.3  利用串口控制pwm参数 2026年3月7日
//                 p1+ ：左上增加10
//                 p1- ：左上减少10
//                 p2+ ：左下增加10
//                 p2- ：左下减少10
//                 p3+ ：右上增加10
//                 p3- ：右上减少10
//                 p4+ ：右下增加10
//                 p4- ：右下减少10
//                 使用前需要注释control.c中right_leg_control和left_leg_control的servo_control_table函数.
//             V1.1.4  速度多字节帧 V<数值> 由 Menu.c:Menu_TryConsumePcMotorSpeedString 解析 2026年4月
//-------------------------------------------------------------------------------------------------------------------
void ReadDataFromPc()
{
// 有线模式下读取数据
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
            //Flash.Flash_state = FLASH_READ;
            //Flash.Flash_Error = FLASH_RUNNING;
        }
        if (WirelessUart_ReadBuff_Data[0] == 's')
        {
            //Car_AimSpeed = (int16)atof(&WirelessUart_ReadBuff_Data[1]);
        }
        // PWM参数控制指令解析
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
                // 指令处理完成，PWM参数已更新
            }
        }
        memset(WirelessUart_ReadBuff_Data, 0, WirelessUart_ReadBuff_Count); // 清除接收缓存
        WirelessUart_ReadBuff_Count = 0;
        }
    }
#else // 在Debug模式下读取数据
    WirelessUart_ReadBuff_Count = debug_read_ring_buffer(WirelessUart_ReadBuff_Data, sizeof(WirelessUart_ReadBuff_Data));
    if (WirelessUart_ReadBuff_Count)
    {
        // debug_send_buffer(WirelessUart_ReadBuff_Data,WirelessUart_ReadBuff_Count);
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
            //Flash_Read_State = FLASH_RUNNING;
        }
        // PWM参数控制指令解析
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
                // 指令处理完成，PWM参数已更新
            }
        }
        memset(WirelessUart_ReadBuff_Data, 0, WirelessUart_ReadBuff_Count); // 清除接收缓存
        WirelessUart_ReadBuff_Count = 0;
        }
    }
#endif
}
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     发送数据
// 参数说明     void
// 返回参数     void
// 使用示例     SendDataToVofa();
// 备注信息     V1.0.0
//-------------------------------------------------------------------------------------------------------------------
void SendDataToVofa()
{
    // SendDataStreamToVOFA(4,(float)Gyro.pitch,(float)0,(float)Left_Motor_Pwm,(float)Gyro.gyro_y);
}
#if defined(CY_CORE_CM7_1)
#include "dualcore_shared.h"
#include "navigation.h"
/*-------------------------------------------------------------------------------------------------------------------
 * 函数简介     双核模式下在 CM7_1 将姿态/速度/融合调试帧发到 VOFA（无线 JustFloat）
 * 前置条件     CM7_0 主循环已周期性调用 dualcore_ctrl_to_ui_publish()；本核 all_init_cm7_1_ui 已 wireless_uart_init
 * 分组说明     nag_vofa_group % NAG_VOFA_GROUP_COUNT；菜单 n / DUALCORE_UI_CMD_NAG_VOFA_GROUP_NEXT 循环切组。
 *              0=IMU 姿态；1=速度目标/实测；2=GPS+惯导融合；3=里程纠偏/打滑（详见 vofa.h）。
 * 调用关系     SendDataStreamToVOFA 内部仍写 vofa_justfloat_frame_tail 作为帧尾
 *-------------------------------------------------------------------------------------------------------------------*/
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
#endif /* CY_CORE_CM7_1 */
