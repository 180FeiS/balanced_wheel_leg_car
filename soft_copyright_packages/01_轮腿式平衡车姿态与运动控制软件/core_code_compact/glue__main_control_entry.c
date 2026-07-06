#include "zf_common_headfile.h"
int main(void)
{
    clock_init(SYSTEM_CLOCK_250M);
    debug_init();
    all_init_cm7_0_control();
    while (true)
    {
        dualcore_ui_cmd_consume_all();
        remote_lora_apply_validate_motor();
    }
}
