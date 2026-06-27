/*********************************************************************************************************************
 * LORA 遥控（CM7_1）：依赖 zf_device_lora3a22 帧解析，汇总为 dualcore_remote_to_ctrl_t 经 dualcore_remote_publish 给 CM7_0。
 * 仅当 MENU_INPUT_REMOTE_MENU_FIRST==1 时 enabled=1 且携带实时轴/键/拨码；否则发布安全默认（键 0、拨码 1、摇杆 0）。
 * 摇杆含义与 CM7_0 侧应用：joystick[0..1] 左杆 x/y，[2..3] 右杆 x/y；偏航角速度由右杆 right_x 映射（见 remote_lora_apply_validate_motor）。
 *********************************************************************************************************************/
#include "dualcore_shared.h"
#include "remote_lora.h"
#include "Menu.h"
#include "zf_device_lora3a22.h"
#include <string.h>

#if defined(CY_CORE_CM7_1)

static dualcore_remote_to_ctrl_t s_last_published;
static uint8 s_inited;

/** 将单轴摇杆采样限幅到 ±REMOTE_LORA_JOYSTICK_ABS_MAX，与遥控端量程一致。 */
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

/** 遥控未在线或禁用时使用的安全默认：摇杆置 0、拨码置 1、按键置 0。 */
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

/** 初始化 LORA 驱动与上次发布快照；应在周期任务前调用一次。 */
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

/**
 * 从 lora3a22 驱动取最新帧，填充 dualcore_remote_to_ctrl_t 并发布到双核共享区。
 * online=0 时仍发布 enabled 状态但摇杆为默认 0，CM7_0 侧应结合 online 做失控保护。
 */
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
        /* 与 lora3a22_uart_transfer 索引一致：0/1 左杆 x/y，2/3 右杆 x/y */
        snap.left_x = remote_lora_clamp_axis(lora3a22_uart_transfer.joystick[0]);
        snap.left_y = remote_lora_clamp_axis(lora3a22_uart_transfer.joystick[1]);
        snap.right_x = remote_lora_clamp_axis(lora3a22_uart_transfer.joystick[2]);
        snap.right_y = remote_lora_clamp_axis(lora3a22_uart_transfer.joystick[3]);

        /* key0/1 左/右摇杆键，key2/3 左/右侧向键；CM7_0 应用层：key2 横滚开关、key3 跳跃（可改 remote_lora.h 下标宏） */
        snap.key[0] = lora3a22_uart_transfer.key[0] ? 1u : 0u;
        snap.key[1] = lora3a22_uart_transfer.key[1] ? 1u : 0u;
        snap.key[2] = lora3a22_uart_transfer.key[2] ? 1u : 0u;
        snap.key[3] = lora3a22_uart_transfer.key[3] ? 1u : 0u;

        /* 左边 0/1 为左拨码 1/2；CM7_0 用 switch_key[0] 回放、switch_key[0]=1 时 [1] 录/停，见 remote_lora.h REMOTE_LORA_LEFT_SWITCH* */
        snap.switch_key[0] = lora3a22_uart_transfer.switch_key[0] ? 1u : 0u;
        snap.switch_key[1] = lora3a22_uart_transfer.switch_key[1] ? 1u : 0u;
        snap.switch_key[2] = lora3a22_uart_transfer.switch_key[2] ? 1u : 0u;
        snap.switch_key[3] = lora3a22_uart_transfer.switch_key[3] ? 1u : 0u;
    }
    else
    {
        /* 离线：保持默认键 0、拨码 1、摇杆 0 */
        remote_lora_apply_defaults(&snap);
        snap.enabled = 1u;
        snap.online = 0u;
        snap.fresh = 0u;
    }

    dualcore_remote_publish(&snap);
    s_last_published = snap;
}

#else /* !CY_CORE_CM7_1 */

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

#endif /* CY_CORE_CM7_1 */
