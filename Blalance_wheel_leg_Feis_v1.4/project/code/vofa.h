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
#endif

#endif
