#ifndef _VOFA_H_
#define _VOFA_H_

#include "zf_common_headfile.h"

void SendDataStreamToVOFA(uint32_t Count, float Data, ...);
void ReadDataFromPc(void); /* 除单字节 Menu_command 外，支持 V<数值> 扩展帧，详见 vofa.c 内注释 */
void SendDataToVofa(void);

extern uint32 WirelessUart_ReadBuff_Count;   // 读取无线串口数据长度
extern uint8 WirelessUart_ReadBuff_Data[64]; // 读取无线串口缓冲数组
extern uint8 Menu_command;                   // 菜单进行指令
extern float ReadBuf_Pid;

#if defined(CY_CORE_CM7_1)
/* 仅双核 CM7_1：读 shared ctrl 快照，经 wireless_uart 发 JustFloat（须本核已 wireless_uart_init） */
void vofa_send_nav_from_dualcore_snapshot(void);

/*
 * VOFA 组 1 — 速度调试（菜单 n 切组至 group==1）：
 *   ch1 speed_target_effective  速度环实际目标
 *   ch2 car_speed               当前实测车速
 */
#define VOFA_GROUP_SPEED_DEBUG (1u)

/*
 * VOFA 组 2 — GPS+惯导融合调试（菜单 n 切组至 2）：
 *   ch1 fusion_x_m           融合东向位移(m)
 *   ch2 fusion_y_m           融合北向位移(m)
 *   ch3 fusion_v_mps         标定后前向速度(m/s)
 *   ch4 fusion_gps_residual_m  GPS 与融合位置残差(m)，>3 常见跳点
 *   ch5 fusion_gps_weight    本帧 GPS 修正增益
 *   ch6 fusion_gps_used      1=本帧 GPS 参与修正
 */
#define VOFA_GROUP_FUSION_DEBUG (2u)
#endif

#endif
