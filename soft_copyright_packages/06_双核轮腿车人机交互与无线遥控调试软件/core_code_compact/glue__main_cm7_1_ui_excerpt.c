#include "zf_common_headfile.h"
#include "Menu.h"
#include "dualcore_shared.h"
#include "remote_lora.h"
static void cm71_vofa_main_loop_tx_dispatch(void)
{
    vofa_send_nav_from_dualcore_snapshot();
}
int main(void)
{
    clock_init(SYSTEM_CLOCK_250M);
    debug_info_init();
    all_init_cm7_1_ui();
    remote_lora_init();
    while (true)
    {
        selectMenu_Key();
        Menu_UpdateImageAeArm();
        ui_pull_ctrl_snapshot();
        selectMenu();
        remote_lora_update_from_driver_and_publish();
        dualcore_vision_publish_after_step(0u);
        cm71_vofa_main_loop_tx_dispatch();
    }
}
