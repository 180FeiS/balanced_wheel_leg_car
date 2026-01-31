#ifndef _VOFA_H_
#define _VOFA_H_

#include "zf_common_headfile.h"

void SendDataStreamToVOFA(uint32_t Count, float Data, ...);
void ReadDataFromPc(void);
void SendDataToVofa(void);

extern uint32 WirelessUart_ReadBuff_Count;   // 读取无线串口数据长度
extern uint8 WirelessUart_ReadBuff_Data[64]; // 读取无线串口缓冲数组
extern uint8 Menu_command;                   // 菜单进行指令
extern float ReadBuf_Pid;

#endif
