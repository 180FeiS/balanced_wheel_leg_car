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
 * VOFA 组 9 — 元素调速实车调试（菜单 n / 上位机切组循环至 group==9）：
 *   ch1 speed_target_effective  速度环实际目标；含导航门控、弯道限速、元素区段调速与提前加减速
 *   ch2 car_speed               当前实测车速；与 ch1 对比可观察加减速响应与跟踪误差
 *   ch3 Run_index               回放推进到的导航点索引；观察预减速/锥桶区间切换是否与点位对齐
 *   ch4 Event_Active_Type      当前元素类型枚举：0=SPIN 1=ENTER_TURN 2=EXIT_TURN
 *                               3=ENTER_CONES 4=EXIT_CONES 5=SINGLE_BRIDGE 6=BUMP 7=ENTER_STAIR 8=EXIT_STAIR
 * 建议与 navigation.h 中 Nag_*_Target_Speed / Launch 页 StairDec（nag_enter_stair_pre_decel_dist_cm）等参数联调。
 */
#define VOFA_GROUP_EVENT_SPEED_DEBUG (9u)

/*
 * VOFA 组 10 — GPS+惯导融合调试（菜单 n 切组至 10）：
 *   ch1 fusion_x_m           融合东向位移(m)
 *   ch2 fusion_y_m           融合北向位移(m)
 *   ch3 fusion_v_mps         标定后前向速度(m/s)
 *   ch4 fusion_gps_residual_m  GPS 与融合位置残差(m)，>3 常见跳点
 *   ch5 fusion_gps_weight    本帧 GPS 修正增益
 *   ch6 fusion_gps_used      1=本帧 GPS 参与修正
 */
#define VOFA_GROUP_FUSION_DEBUG (10u)
#endif

#endif
