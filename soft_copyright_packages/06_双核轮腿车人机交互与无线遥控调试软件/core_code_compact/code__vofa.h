#ifndef _VOFA_H_
#define _VOFA_H_
#include "zf_common_headfile.h"
void SendDataStreamToVOFA(uint32_t Count, float Data, ...);
void ReadDataFromPc(void);
void SendDataToVofa(void);
extern uint32 WirelessUart_ReadBuff_Count;
extern uint8 WirelessUart_ReadBuff_Data[64];
extern uint8 Menu_command;
extern float ReadBuf_Pid;
#if defined(CY_CORE_CM7_1)
void vofa_send_nav_from_dualcore_snapshot(void);
#define VOFA_GROUP_SPEED_DEBUG (1u)
#define VOFA_GROUP_FUSION_DEBUG (2u)
#define VOFA_GROUP_ODO_SLIP_DEBUG (3u)
#endif
#endif
