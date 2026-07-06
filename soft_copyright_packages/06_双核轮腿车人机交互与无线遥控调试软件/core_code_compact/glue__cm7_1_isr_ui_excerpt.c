/*
 * 人机交互软件 — 按键扫描 ISR 摘录（软著 06）
 */
#include "zf_common_headfile.h"
void pit0_ch2_isr(void)
{
    pit_isr_flag_clear(PIT_CH2);
    key_scanner();
    menu_key_capture_event();
}
