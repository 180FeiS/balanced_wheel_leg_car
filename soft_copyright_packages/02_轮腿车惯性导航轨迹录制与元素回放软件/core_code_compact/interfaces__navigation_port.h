#ifndef CODE_INTERFACES_NAVIGATION_PORT_H_
#define CODE_INTERFACES_NAVIGATION_PORT_H_
/*
 * 惯导导航软件对外 API（供运动控制/视觉/人机交互调用）。
 * 软著 02 主归属 navigation.c；其他模块仅保留本头文件。
 */
#include "zf_common_typedef.h"
void Init_Nag(void);
void Nag_System(void);
void Nag_Begin_Record(void);
void Nag_Begin_Replay(void);
void Nag_Request_Stop_Record(void);
void NagFlashRead(void);
void Nag_CompleteReplayAfterOrigin(void);
uint8 Nag_HeadingHold_ShouldRequest(void);
float Nag_HeadingHold_GetTargetYaw(void);
void Nag_BridgeDetectUpdate(void);
void Nag_NotifyStairJumpDone(void);
float Nag_GetControlSpeedTarget(void);
extern uint8 Nag_Vofa_Group;
#define Nag_Sample_Dt 0.001f
typedef struct Nag Nag;
extern Nag N;
#endif
