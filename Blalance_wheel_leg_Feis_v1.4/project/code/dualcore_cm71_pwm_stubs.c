/*********************************************************************************************************************
 * CM7_1 仅链接 UI/菜单/vofa 时的弱符号占位：
 * - pwm_ph*：vofa.c 舵机调试串口指令
 * - uart_control_callback：cm7_1_isr 中 UART2/4 与 CM7_0 模板一致；本核无电调串口时可空实现
 *********************************************************************************************************************/
#include "zf_common_typedef.h"

#if defined(CY_CORE_CM7_1)
int16 pwm_ph1 = 0;
int16 pwm_ph2 = 0;
int16 pwm_ph3 = 0;
int16 pwm_ph4 = 0;

void uart_control_callback(void)
{
}
#endif
