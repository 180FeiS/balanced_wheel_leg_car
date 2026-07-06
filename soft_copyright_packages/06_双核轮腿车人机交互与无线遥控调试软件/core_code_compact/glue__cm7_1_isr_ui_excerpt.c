#include "zf_common_headfile.h"
void pit0_ch2_isr(void)
{
    pit_isr_flag_clear(PIT_CH2);
    key_scanner();
    menu_key_capture_event();
}
