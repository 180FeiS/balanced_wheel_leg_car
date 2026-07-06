#include "dualcore_shared.h"
#include "remote_lora.h"
#include "Menu.h"
#include "zf_device_lora3a22.h"
#include <string.h>
#if defined(CY_CORE_CM7_1)
static dualcore_remote_to_ctrl_t s_last_published;
static uint8 s_inited;
static int16 remote_lora_clamp_axis(int16 v)
{
    if (v > REMOTE_LORA_JOYSTICK_ABS_MAX)
    {
        return (int16)REMOTE_LORA_JOYSTICK_ABS_MAX;
    }
    if (v < -(int16)REMOTE_LORA_JOYSTICK_ABS_MAX)
    {
        return (int16)-(int16)REMOTE_LORA_JOYSTICK_ABS_MAX;
    }
    return v;
}
static void remote_lora_apply_defaults(dualcore_remote_to_ctrl_t *s)
{
    memset(s, 0, sizeof(*s));
    s->left_x = 0;
    s->left_y = 0;
    s->right_x = 0;
    s->right_y = 0;
    s->key[0] = 0u;
    s->key[1] = 0u;
    s->key[2] = 0u;
    s->key[3] = 0u;
    s->switch_key[0] = 1u;
    s->switch_key[1] = 1u;
    s->switch_key[2] = 1u;
    s->switch_key[3] = 1u;
}
void remote_lora_init(void)
{
    remote_lora_apply_defaults(&s_last_published);
    lora3a22_init();
    s_inited = 1u;
}
uint8 remote_lora_is_remote_menu_enabled(void)
{
#if defined(CY_CORE_CM7_1)
    dualcore_ctrl_to_ui_t dc;
    dualcore_ctrl_to_ui_pull(&dc);
    return dc.menu_input_remote_first;
#else
    return 0u;
#endif
}
void remote_lora_get_last_published(struct dualcore_remote_to_ctrl *out)
{
    if (out != NULL)
    {
        *out = s_last_published;
    }
}
void remote_lora_update_from_driver_and_publish(void)
{
    dualcore_remote_to_ctrl_t snap;
    uint8 had_new_frame = 0u;
    if (!s_inited)
    {
        remote_lora_init();
    }
    remote_lora_apply_defaults(&snap);
    if (lora3a22_finsh_flag != 0u)
    {
        had_new_frame = 1u;
        lora3a22_finsh_flag = 0u;
    }
    snap.enabled = remote_lora_is_remote_menu_enabled();
    if (snap.enabled == 0u)
    {
        remote_lora_apply_defaults(&snap);
        snap.enabled = 0u;
        snap.online = 0u;
        snap.fresh = 0u;
        dualcore_remote_publish(&snap);
        s_last_published = snap;
        return;
    }
    snap.online = lora3a22_state_flag ? 1u : 0u;
    snap.fresh = had_new_frame;
    if (snap.online != 0u)
    {
        snap.left_x = remote_lora_clamp_axis(lora3a22_uart_transfer.joystick[0]);
        snap.left_y = remote_lora_clamp_axis(lora3a22_uart_transfer.joystick[1]);
        snap.right_x = remote_lora_clamp_axis(lora3a22_uart_transfer.joystick[2]);
        snap.right_y = remote_lora_clamp_axis(lora3a22_uart_transfer.joystick[3]);
        snap.key[0] = lora3a22_uart_transfer.key[0] ? 1u : 0u;
        snap.key[1] = lora3a22_uart_transfer.key[1] ? 1u : 0u;
        snap.key[2] = lora3a22_uart_transfer.key[2] ? 1u : 0u;
        snap.key[3] = lora3a22_uart_transfer.key[3] ? 1u : 0u;
        snap.switch_key[0] = lora3a22_uart_transfer.switch_key[0] ? 1u : 0u;
        snap.switch_key[1] = lora3a22_uart_transfer.switch_key[1] ? 1u : 0u;
        snap.switch_key[2] = lora3a22_uart_transfer.switch_key[2] ? 1u : 0u;
        snap.switch_key[3] = lora3a22_uart_transfer.switch_key[3] ? 1u : 0u;
    }
    else
    {
        remote_lora_apply_defaults(&snap);
        snap.enabled = 1u;
        snap.online = 0u;
        snap.fresh = 0u;
    }
    dualcore_remote_publish(&snap);
    s_last_published = snap;
}
#else
void remote_lora_init(void)
{
}
void remote_lora_update_from_driver_and_publish(void)
{
}
uint8 remote_lora_is_remote_menu_enabled(void)
{
    return 0u;
}
void remote_lora_get_last_published(dualcore_remote_to_ctrl_t *out)
{
    (void)out;
}
#endif
