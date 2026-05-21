/*
 * @Author: bom
 * @Version: V1.0
 * @Date: 2024-12-12 22:02:51
 * @LastEditTime: 2024-12-24 22:46:12
 * @FilePath: \Balance_Car V3.0.1\project\code\Menu.c
 * @Description:
 */
/*********************************************************************************************************************
 移植使用说明：
 1. 需要包含的头文件:
    - Menu.h: 菜单相关定义和声明
    - string.h: 字符串处理函数
    - UI.h: 界面显示相关函数

 2. 关键结构和变量:
    - HASH_TABLE_t hashMenu: 全局哈希表变量，存储所有菜单项
    - MENU_MEMBER_t menuMember: 当前选中的菜单项信息

 3. 使用步骤:
    a) 调用 MenuInit() 初始化菜单系统
    b) 在 MenuInit() 函数中定义菜单结构:
       - 设置 menuMember.gui (界面显示函数)
       - 设置 menuMember.act (菜单动作函数)
       - 设置 menuMember.pos (菜单位置编号，如"1"、"1.1"等)
       - 调用 hashMenu.vPtr->insert() 插入菜单项

 4. 菜单控制接口（与参考工程结构语义一致；板载键在 selectMenu_Key 中映射到此语义）:
    - hashMenu.vPtr->searchUp(): 返回上级菜单
    - hashMenu.vPtr->searchDown(): 进入下级菜单
    - hashMenu.vPtr->searchLeft(): 切换到上一个同级菜单
    - hashMenu.vPtr->searchRight(): 切换到下一个同级菜单

    【易错点：虚表语义 vs 板载键名】（避免以后再改回去踩坑）
    - vtable 四个指针名的历史含义必须与「菜单树」一致（与 Balance_Car_initial Menu.c 相同）：
      searchDown=进子(HashDepthDown)、searchUp=返父(HashDepthUp)、Left/Right=同级(HashPeerLeft/PeerRight)。
    - 曾有人把 HashTableCtor 写成「按键方向语义」（例如把 Up/Down 绑成同级、Left/Right 绑成返父/进子），
      这样与参考工程及 HashDepth/HashPeer 函数名完全对不上，Run/Debug 等分支行为会整体错位。
    - 本工程板载习惯是 KEY1/2 切同级、KEY3 进入、KEY4 返回：只能在 selectMenu_Key()（及串口 a/b/c/d）
      里把物理事件映射到上述四个 search*，不要再改 HashTableCtor 去「迎合键名」。

 5. 注意事项:
    - 菜单位置编号格式为: "1"、"2"(一级菜单)，"1.1"、"1.2"(二级菜单)
    - 最大支持菜单深度由 HASH_KEY_LEN 定义
    - 最大菜单项数量由 HASH_SIZE 定义

    最终调用
    menuMember.gui();
    menuMember.act();
********************************************************************************************************************/

/*********************************************************************************************************************
 * 修改记录：
 * 版本号      日期          作者      说明
 * V1.0.0    2024-07-24    Bron     搭建新工程
 * V1.0.2    2024-07-27    Bron     搭建二级菜单框架
 * V1.1.0    2024-07-30    Bron     五向按键改为vofa模拟按键
 * V1.1.4    2024-08-01    Bron     加入液晶串口切换模式
 * V2.0.1    2024-12-19    Bron     整合Menu和Manu_Basic
 ********************************************************************************************************************/

#include "zf_common_headfile.h"
#include <stdlib.h>
#include <string.h>
#include "dualcore_shared.h"
#include "my_gps.h"
#if defined(CY_CORE_CM7_0)
#include "control.h"
#include "navigation.h"
#include "flash.h"
#endif

/* 全局变量定义 */
HASH_TABLE_t hashMenu;    // 储存菜单项的哈希表
MENU_MEMBER_t menuMember; // 当前选中的菜单项信息
char ReadPos[HASH_KEY_LEN] = {'1', 0, 0, 0, 0};

#define MENU_KEY_EVENT_QUEUE_LEN 8

typedef enum
{
    MENU_KEY_NAV_NONE = 0,
    MENU_KEY_NAV_LEFT,
    MENU_KEY_NAV_RIGHT,
    MENU_KEY_NAV_DOWN,
    MENU_KEY_NAV_UP,
} menu_key_nav_enum;

static volatile uint8 menu_key_nav_queue[MENU_KEY_EVENT_QUEUE_LEN];
static volatile uint8 menu_key_nav_head = 0;
static volatile uint8 menu_key_nav_tail = 0;

/* 内部函数声明 */
static void HashTableCtor(HASH_TABLE_t *const This);                                                // 哈希表初始化
static uint16_t CreatHashKey(const char *skey);                                                     // 生成哈希值
static void *MemSet(void *src, int value, int n);                                                   // 内存初始化
static void *MemCpy(void *dest, void *src, unsigned int size);                                      // 内存复制
static uint8_t HashInsert(HASH_TABLE_t *const This, MENU_MEMBER_t *const tempMember);               // 插入菜单项
static uint8_t HashDepthUp(HASH_TABLE_t *const This, MENU_MEMBER_t *const tempMember);              // 返回上级菜单
static uint8_t HashDepthDown(HASH_TABLE_t *const This, MENU_MEMBER_t *const tempMember);            // 进入下级菜单
static uint8_t HashPeerLeft(HASH_TABLE_t *const This, MENU_MEMBER_t *const tempMember);             // 切换到左侧同级菜单
static uint8_t HashPeerRight(HASH_TABLE_t *const This, MENU_MEMBER_t *const tempMember);            // 切换到右侧同级菜单
static uint8_t FindHashValue(HASH_TABLE_t *const This, MENU_MEMBER_t *const tempMember, char *str); // 查找菜单项
static void MenuKeyEventPush(menu_key_nav_enum nav);
static uint8 MenuKeyEventPop(menu_key_nav_enum *nav);
static uint8 MenuIsNavDebugPage(void);
static uint8 MenuIsGpsDebugPage(void);
static uint8 MenuIsRunLaunchSpeedPage(void);
static uint8 MenuIsRunFlashPage(void);
static void MenuApplyRunLaunchSpeed(float speed);
static void MenuAdjustRunLaunchSpeed(float delta);
static uint8 MenuTryHandleRunLaunchSpeedKeyEvent(void);
static uint8 MenuTryHandleRunFlashKeyEvent(void);
static uint8 MenuTryHandleGpsDebugKeyEvent(void);

/* 发车速度页三档设定值。KEY1 在 0/500/1000 三档间循环，KEY2/KEY3 只调整当前下标对应的档位。 */
static float s_run_launch_speed_presets[3] = {0.0f, 500.0f, 1000.0f};
static uint8 s_run_launch_speed_preset_index = 0u;

/*-------------------------------------------------------------------------
 * 菜单接口函数-用户只需更改此部分
 *-------------------------------------------------------------------------*/

/*
 * @Function: selectMenu_Key
 * @Description: 使用按键菜单选择处理函数
 * @Param: Void
 * @Return: Void
 * @Example: selectMenu();
 */

/* 返回 1：整帧已由扩展协议消费，ReadDataFromPc 不应再写入 Menu_command。
 * selectMenu() 仅处理单字节；多字节 V<数值> 在本函数解析。
 * 格式：V<数值>  例 V1200 / V0 / V-800（写入 run_launch_speed；仅惯导回放进入执行态时装载）
 */
uint8 Menu_TryConsumePcMotorSpeedString(const uint8 *data, uint32 count)
{
    char tmp[68];
    uint32 n;

    if ((data == NULL) || (count == 0u))
    {
        return 0u;
    }
    n = count;
    if (n >= sizeof(tmp))
    {
        n = (uint32)(sizeof(tmp) - 1u);
    }
    memcpy(tmp, data, n);
    tmp[n] = '\0';

    if ((tmp[0] == 'V') && (n >= 2u))
    {
#if defined(CY_CORE_CM7_1)
        (void)dualcore_ui_cmd_push(DUALCORE_UI_CMD_MOTOR_SPEED_FROM_PC, 0, (float)atof(&tmp[1]));
#else
        motor_user_speed_cmd_set_from_pc((float)atof(&tmp[1]));
#endif
        return 1u;
    }
    return 0u;
}

/*-------------------------------------------------------------------------
 * 拨码与速度基准（须周期性调用，如 selectMenu_Key / selectMenu 内）：
 * 1. SWITCH2：边沿触发 motor_poll_switch2_speed_baseline()，在安全态下执行 yaw 零点重置（当前朝向变为 0），成功翻转 LED1。
 * 2. SWITCH1：Motor_Switch 唯一来源（失控锁存除外）。
 * 3. 导航未进入回放执行态前，速度环仍由 Nag_GetControlSpeedTarget() 门控为 0。
 * 4. Motor_Runaway_Latch：最高优先级关电机；遥控优先关闭时须 SWITCH1 到 OFF 后才清除锁存。
 *    MENU_INPUT_REMOTE_MENU_FIRST==1 时不读拨码，锁存仅能遥控清或复位；关断与轮速失控仍见 control.c。
 *-------------------------------------------------------------------------*/
void dip_switch_motor_sync_from_hw(void)
{
#if defined(CY_CORE_CM7_1)
    (void)0;
#else
#if MENU_INPUT_REMOTE_MENU_FIRST
    /* 遥控优先且非板载调试：不读 SWITCH；板载调试时读 GPIO，等同宏=0。 */
    if (g_remote_local_keys_debug == 0u)
    {
        return;
    }
#endif
    uint8 dip_motor = (gpio_get_level(SWITCH1) == GPIO_LOW) ? MOTOR_ON : MOTOR_OFF;

    motor_poll_switch2_speed_baseline();

    if (Motor_Runaway_Latch)
    {
        Motor_Switch = MOTOR_OFF;
        if (dip_motor == MOTOR_OFF)
        {
            Motor_Runaway_Latch = 0;
        }
        return;
    }

    {
        uint8 prev = Motor_Switch;

        Motor_Switch = dip_motor;

        if (prev != Motor_Switch && Motor_Switch == MOTOR_OFF)
        {
#if !DUALCORE_UI_ON_CM7_1
            ips200_clear();
            menuMember.gui();
            menuMember.act();
#endif
        }
    }
#endif /* !CY_CORE_CM7_1 */
}

/*
 * 按键扫描入队：MENU_KEY_NAV_* 只表示「队列里的方向事件类型」，语义在 selectMenu_Key() 才落到 hash 虚表。
 * 例如 KEY3 推到 MENU_KEY_NAV_RIGHT，但实际菜单动作是「进入下级」，对应 hash 侧应调 searchDown()，
 * 不要看到 RIGHT 字面就去绑 searchRight()（那会变成同级切换）。
 */
void menu_key_capture_event(void)
{
#if defined(CY_CORE_CM7_1)
   dualcore_ctrl_to_ui_t dc;
   dualcore_ctrl_to_ui_pull(&dc);
#if MENU_INPUT_REMOTE_MENU_FIRST
   if (dc.remote_local_keys_debug == 0u)
   {
       return;
   }
#endif
   uint8 nav_recording_active = dc.nav_recording_active;
   uint8 event_active = dc.event_active;
   if(MenuIsRunLaunchSpeedPage() && MenuTryHandleRunLaunchSpeedKeyEvent())
   {
        return;
   }
   if(MenuIsRunFlashPage() && MenuTryHandleRunFlashKeyEvent())
   {
        return;
   }
   if(MenuIsGpsDebugPage() && MenuTryHandleGpsDebugKeyEvent())
   {
        return;
   }
   if(MenuIsNavDebugPage())
   {
        if(key_get_state(KEY_1) == KEY_SHORT_PRESS)
        {
            (void)dualcore_ui_cmd_push(DUALCORE_UI_CMD_KEY_NAV_RECORD, 0, 0.0f);
            gpio_toggle_level(LED1);
            key_clear_state(KEY_1);
        }
        if(key_get_state(KEY_2) == KEY_SHORT_PRESS)
        {
            (void)dualcore_ui_cmd_push(DUALCORE_UI_CMD_KEY_NAV_STOP_REC, 0, 0.0f);
            gpio_toggle_level(LED1);
            key_clear_state(KEY_2);
        }
        if(key_get_state(KEY_3) == KEY_SHORT_PRESS)
        {
            (void)dualcore_ui_cmd_push(DUALCORE_UI_CMD_KEY_NAV_KEY3, 0, 0.0f);
            gpio_toggle_level(LED1);
            key_clear_state(KEY_3);
        }
        if(key_get_state(KEY_4) == KEY_SHORT_PRESS)
        {
            if (nav_recording_active)
            {
                (void)dualcore_ui_cmd_push(DUALCORE_UI_CMD_NAG_EVENT_MARK, 0, 0.0f);
            }
            else if (event_active)
            {
                (void)dualcore_ui_cmd_push(DUALCORE_UI_CMD_NAG_EVENT_DONE, 0, 0.0f);
            }
            else
            {
                MenuKeyEventPush(MENU_KEY_NAV_LEFT);
            }
            gpio_toggle_level(LED1);
            key_clear_state(KEY_4);
        }
   }
   else
   {
        if(key_get_state(KEY_1) == KEY_SHORT_PRESS)
        {
            MenuKeyEventPush(MENU_KEY_NAV_UP);
            gpio_toggle_level(LED1);
            key_clear_state(KEY_1);
        }
        if(key_get_state(KEY_2) == KEY_SHORT_PRESS)
        {
            MenuKeyEventPush(MENU_KEY_NAV_DOWN);
            gpio_toggle_level(LED1);
            key_clear_state(KEY_2);
        }
        if(key_get_state(KEY_3) == KEY_SHORT_PRESS)
        {
            MenuKeyEventPush(MENU_KEY_NAV_RIGHT);
            gpio_toggle_level(LED1);
            key_clear_state(KEY_3);
        }
        if(key_get_state(KEY_4) == KEY_SHORT_PRESS)
        {
            MenuKeyEventPush(MENU_KEY_NAV_LEFT);
            gpio_toggle_level(LED1);
            key_clear_state(KEY_4);
        }
   }
#else
#if MENU_INPUT_REMOTE_MENU_FIRST
    if (g_remote_local_keys_debug == 0u)
    {
        return;
    }
#endif
   if(MenuIsRunLaunchSpeedPage() && MenuTryHandleRunLaunchSpeedKeyEvent())
   {
        return;
   }
   if(MenuIsRunFlashPage() && MenuTryHandleRunFlashKeyEvent())
   {
        return;
   }
   if(MenuIsGpsDebugPage() && MenuTryHandleGpsDebugKeyEvent())
   {
        return;
   }
   if(MenuIsNavDebugPage())
   {
        if(key_get_state(KEY_1) == KEY_SHORT_PRESS)
        {
            Nag_Begin_Record();
            gpio_toggle_level(LED1);
            key_clear_state(KEY_1);
        }
        if(key_get_state(KEY_2) == KEY_SHORT_PRESS)
        {
            Nag_Request_Stop_Record();
            gpio_toggle_level(LED1);
            key_clear_state(KEY_2);
        }
        /* 录制态统一门控：
         * 1. Nag_SystemRun_Index == 1 表示仍处于录制流程；
         * 2. End_f == 0 表示尚未进入停止录制后的收尾写 flash 阶段。
         * 这样 KEY3/KEY4 不会在录制收尾窗口里继续改元素类型或打点。
         */
        uint8 nav_recording_active = (N.Nag_SystemRun_Index == 1 && N.End_f == 0);
        if(key_get_state(KEY_3) == KEY_SHORT_PRESS)
        {
            if (nav_recording_active)
            {
                Nag_Cycle_Record_Event_Type();
            }
            else
            {
                Nag_Begin_Replay();
            }
            gpio_toggle_level(LED1);
            key_clear_state(KEY_3);
        }
        if(key_get_state(KEY_4) == KEY_SHORT_PRESS)
        {
            /* KEY4 优先级：
             * 1. 录制中：单击保存当前元素点（Nag_Request_Event_Mark）；
             * 2. 非录制且元素接管中：手动通知元素完成，恢复导航；
             * 3. 其它情况：返回上一级菜单。
             */
            if (nav_recording_active)
            {
                Nag_Request_Event_Mark();
            }
            else if (N.Event_Active)
            {
                Nag_Notify_Event_Done();
            }
            else
            {
                MenuKeyEventPush(MENU_KEY_NAV_LEFT);
            }
            gpio_toggle_level(LED1);
            key_clear_state(KEY_4);
        }
   }
   else
   {
        if(key_get_state(KEY_1) == KEY_SHORT_PRESS)
        {
            MenuKeyEventPush(MENU_KEY_NAV_UP);
            gpio_toggle_level(LED1);
            key_clear_state(KEY_1);
        }
        if(key_get_state(KEY_2) == KEY_SHORT_PRESS)
        {
            MenuKeyEventPush(MENU_KEY_NAV_DOWN);
            gpio_toggle_level(LED1);
            key_clear_state(KEY_2);
        }
        if(key_get_state(KEY_3) == KEY_SHORT_PRESS)
        {
            MenuKeyEventPush(MENU_KEY_NAV_RIGHT);
            gpio_toggle_level(LED1);
            key_clear_state(KEY_3);
        }
        if(key_get_state(KEY_4) == KEY_SHORT_PRESS)
        {
            MenuKeyEventPush(MENU_KEY_NAV_LEFT);
            gpio_toggle_level(LED1);
            key_clear_state(KEY_4);
        }
    }
#endif /* !CY_CORE_CM7_1 */
}

void selectMenu_Key(void)
{
   uint8 menu_nav = 0;
   menu_key_nav_enum nav = MENU_KEY_NAV_NONE;

#if !defined(CY_CORE_CM7_1)
   dip_switch_motor_sync_from_hw();
#endif
#if defined(CY_CORE_CM7_1)
   dualcore_ctrl_to_ui_t dc_k;
   dualcore_ctrl_to_ui_pull(&dc_k);
   uint8 motor_sw_key = dc_k.motor_switch;
#else
   uint8 motor_sw_key = Motor_Switch;
#endif

   /*
    * 板载映射（保持手感）：KEY1 同级上一项、KEY2 同级下一项、KEY3 进入下级、KEY4 返回上级。
    * 对应 vPtr：searchLeft / searchRight / searchDown / searchUp（均为菜单树语义，勿与 MENU_KEY_NAV_* 字面混读）。
    */
   while(MenuKeyEventPop(&nav))
   {
        switch(nav)
        {
        case MENU_KEY_NAV_LEFT:
             /* KEY4：返回上一级（结构语义 searchUp） */
             hashMenu.vPtr->searchUp(&hashMenu, &menuMember);
             menu_nav = 1;
             break;
        case MENU_KEY_NAV_RIGHT:
             /* KEY3：进入下一级（结构语义 searchDown） */
             hashMenu.vPtr->searchDown(&hashMenu, &menuMember);
             menu_nav = 1;
             break;
        case MENU_KEY_NAV_DOWN:
             /* KEY2：同级下一项（结构语义 searchRight） */
             hashMenu.vPtr->searchRight(&hashMenu, &menuMember);
             menu_nav = 1;
             break;
        case MENU_KEY_NAV_UP:
             /* KEY1：同级上一项（结构语义 searchLeft） */
             hashMenu.vPtr->searchLeft(&hashMenu, &menuMember);
             menu_nav = 1;
             break;
        default:
             break;
        }
   }

   /* 与 selectMenu() 一致：导航后立刻重绘，且须在主循环内完成，避免与 ips200 SPI 冲突。 */
   if(menu_nav && (motor_sw_key == MOTOR_OFF))
   {
        ips200_clear();
        menuMember.gui();
        menuMember.act();
   }
}

static void MenuKeyEventPush(menu_key_nav_enum nav)
{
    uint8 next_head = (uint8)((menu_key_nav_head + 1U) % MENU_KEY_EVENT_QUEUE_LEN);

    if(next_head != menu_key_nav_tail)
    {
        menu_key_nav_queue[menu_key_nav_head] = (uint8)nav;
        menu_key_nav_head = next_head;
    }
}

static uint8 MenuKeyEventPop(menu_key_nav_enum *nav)
{
    uint8 has_event = 0;
    uint32 interrupt_status = interrupt_global_disable();

    if(menu_key_nav_head != menu_key_nav_tail)
    {
        *nav = (menu_key_nav_enum)menu_key_nav_queue[menu_key_nav_tail];
        menu_key_nav_tail = (uint8)((menu_key_nav_tail + 1U) % MENU_KEY_EVENT_QUEUE_LEN);
        has_event = 1;
    }

    interrupt_global_enable(interrupt_status);
    return has_event;
}

static uint8 MenuIsNavDebugPage(void)
{
    return (uint8)(strcmp(menuMember.pos, "2.2.1") == 0);
}

static uint8 MenuIsGpsDebugPage(void)
{
    return (uint8)(strcmp(menuMember.pos, "2.3.1") == 0);
}

/* 仅发车速度三级页拦截 KEY1/2/3；不要扩大到 "3.1" 的 Launch 二级列表页。 */
static uint8 MenuIsRunLaunchSpeedPage(void)
{
    return (uint8)(strcmp(menuMember.pos, "3.1.1") == 0);
}

/* Run/Flash 二级项没有真实下级；在该页按 KEY3（进入）时解释为保存 run_launch_speed 到 flash。 */
static uint8 MenuIsRunFlashPage(void)
{
    return (uint8)(strcmp(menuMember.pos, "3.2") == 0);
}

/* 应用发车速度设定值：只写 run_launch_speed，不直接写 motor_user_speed_cmd。
 * 真正运行速度仍由惯导回放进入执行态前统一装载。
 */
static void MenuApplyRunLaunchSpeed(float speed)
{
#if defined(CY_CORE_CM7_1)
    (void)dualcore_ui_cmd_push(DUALCORE_UI_CMD_RUN_LAUNCH_SPEED_SET_ABS, 0, speed);
#else
    run_launch_speed = speed;
#endif
}

/* 调整当前三档里的选中档位，并立即同步到 run_launch_speed 设定值。 */
static void MenuAdjustRunLaunchSpeed(float delta)
{
    s_run_launch_speed_presets[s_run_launch_speed_preset_index] += delta;
    MenuApplyRunLaunchSpeed(s_run_launch_speed_presets[s_run_launch_speed_preset_index]);
}

/* 返回 1 表示 KEY1/2/3 已被发车速度页消费，必须避免再进入 MenuKeyEventPush 导航队列。 */
static uint8 MenuTryHandleRunLaunchSpeedKeyEvent(void)
{
    if (key_get_state(KEY_1) == KEY_SHORT_PRESS)
    {
        s_run_launch_speed_preset_index = (uint8)((s_run_launch_speed_preset_index + 1u) % 3u);
        MenuApplyRunLaunchSpeed(s_run_launch_speed_presets[s_run_launch_speed_preset_index]);
        gpio_toggle_level(LED1);
        key_clear_state(KEY_1);
        return 1u;
    }
    if (key_get_state(KEY_2) == KEY_SHORT_PRESS)
    {
        MenuAdjustRunLaunchSpeed(-100.0f);
        gpio_toggle_level(LED1);
        key_clear_state(KEY_2);
        return 1u;
    }
    if (key_get_state(KEY_3) == KEY_SHORT_PRESS)
    {
        MenuAdjustRunLaunchSpeed(100.0f);
        gpio_toggle_level(LED1);
        key_clear_state(KEY_3);
        return 1u;
    }
    return 0u;
}

/* 返回 1 表示 Run/Flash 页已消费 KEY3 保存动作；KEY1/KEY2/KEY4 仍走普通菜单导航。 */
static uint8 MenuTryHandleRunFlashKeyEvent(void)
{
    if (key_get_state(KEY_3) == KEY_SHORT_PRESS)
    {
#if defined(CY_CORE_CM7_1)
        (void)dualcore_ui_cmd_push(DUALCORE_UI_CMD_RUN_LAUNCH_SPEED_SAVE_FLASH, 0, 0.0f);
#else
        flash_RunLaunchSpeed_Write();
#endif
        gpio_toggle_level(LED1);
        key_clear_state(KEY_3);
        return 1u;
    }
    return 0u;
}

/* GPS 调试页是 GPS 功能唯一按键入口：
 * Idle：KEY1 开始打点，KEY2 结束并写 flash，KEY3 发车进入 GPS 导航（GPS_ApplyLaunchSpeed 记录起点经纬度；
 *       驶过约 GPS_NAV_GPS_FIRST_DISTANCE_M 后用 RMC gnss.direction 作 GPS_first 标定偏置再追点），KEY4 返回上级；
 * Recording：KEY1 占位，KEY2 结束并写 flash，KEY3 保存当前点，KEY4 切换待保存元素。
 */
static uint8 MenuTryHandleGpsDebugKeyEvent(void)
{
#if defined(CY_CORE_CM7_1)
    dualcore_ctrl_to_ui_t dc;
    dualcore_ctrl_to_ui_pull(&dc);
    uint8 recording_active = dc.gps_recording_active;
#else
    uint8 recording_active = gps_recording_active;
#endif

    if (key_get_state(KEY_1) == KEY_SHORT_PRESS)
    {
        if (!recording_active)
        {
#if defined(CY_CORE_CM7_1)
            (void)dualcore_ui_cmd_push(DUALCORE_UI_CMD_GPS_BEGIN_RECORD, 0, 0.0f);
#else
            GPS_BeginRecord();
#endif
        }
        gpio_toggle_level(LED1);
        key_clear_state(KEY_1);
        return 1u;
    }
    if (key_get_state(KEY_2) == KEY_SHORT_PRESS)
    {
#if defined(CY_CORE_CM7_1)
        (void)dualcore_ui_cmd_push(DUALCORE_UI_CMD_GPS_END_SAVE_FLASH, 0, 0.0f);
#else
        GPS_EndRecord();
        flash_GpsPoints_Write();
#endif
        gpio_toggle_level(LED1);
        key_clear_state(KEY_2);
        return 1u;
    }
    if (key_get_state(KEY_3) == KEY_SHORT_PRESS)
    {
        if (recording_active)
        {
#if defined(CY_CORE_CM7_1)
            (void)dualcore_ui_cmd_push(DUALCORE_UI_CMD_GPS_SAVE_POINT, 0, 0.0f);
#else
            (void)GPS_SaveCurrentPointFromCoord(gnss.latitude, gnss.longitude);
#endif
        }
        else
        {
#if defined(CY_CORE_CM7_1)
            (void)dualcore_ui_cmd_push(DUALCORE_UI_CMD_GPS_LAUNCH, 0, 0.0f);
#else
            GPS_ApplyLaunchSpeed();
#endif
        }
        gpio_toggle_level(LED1);
        key_clear_state(KEY_3);
        return 1u;
    }
    if (recording_active && key_get_state(KEY_4) == KEY_SHORT_PRESS)
    {
#if defined(CY_CORE_CM7_1)
        (void)dualcore_ui_cmd_push(DUALCORE_UI_CMD_GPS_CYCLE_ELEMENT, 0, 0.0f);
#else
        (void)GPS_CycleCurrentElement();
#endif
        gpio_toggle_level(LED1);
        key_clear_state(KEY_4);
        return 1u;
    }
    return 0u;
}

/*
 * @Function: selectMenu
 * @Description: 菜单选择处理函数
 * @Param: Void
 * @Return: Void
 * @Example: selectMenu();
 */


void selectMenu(void)
{
    ReadDataFromPc();
#if !defined(CY_CORE_CM7_1)
    dip_switch_motor_sync_from_hw();
#endif
#if defined(CY_CORE_CM7_1)
    dualcore_ctrl_to_ui_t dc_s;
    dualcore_ctrl_to_ui_pull(&dc_s);
    uint8 motor_sw_sel = dc_s.motor_switch;
#else
    uint8 motor_sw_sel = Motor_Switch;
#endif
#if MENU_INPUT_REMOTE_MENU_FIRST
#if defined(CY_CORE_CM7_1)
    if (dc_s.remote_local_keys_debug == 0u)
#else
    if (g_remote_local_keys_debug == 0u)
#endif
    {
    /* 全遥控时 Menu_command 的单字节导航：语义须与 selectMenu_Key 一致。
     * 注意：老的 Balance_Car_initial（System_startup.c）里曾为 a/b=c/d / c=Down / d=Up，
     * 与此处不同；改串口脚本或上位机前务必对照本节与 selectMenu_Key。 */
    switch (Menu_command)
    {
    case 'a':
        /* 与板载键一致：同级上一项 */
        hashMenu.vPtr->searchLeft(&hashMenu, &menuMember);
        ips200_clear();
        break;

    case 'b':
        /* 与板载键一致：同级下一项 */
        hashMenu.vPtr->searchRight(&hashMenu, &menuMember);
        ips200_clear();
        break;

    case 'c':
        /* 与板载键一致：进入下级 */
        hashMenu.vPtr->searchDown(&hashMenu, &menuMember);
        ips200_clear();
        break;

    case 'd':
        /* 与板载键一致：返回上级 */
        hashMenu.vPtr->searchUp(&hashMenu, &menuMember);
        ips200_clear();
        break;
        /*
    case 'e':
        Flash.Flash_Error = FLASH_RUNNING;
        Flash.Flash_state = FLASH_WRITE;
        ips200_clear();
        break;
    case 'f':
        Flash.Flash_Error = FLASH_RUNNING;
        Flash.Flash_state = FLASH_CLEAR;
        ips200_clear();
        break;
    case 'g':
        Flash.Flash_Error = FLASH_RUNNING;
        Flash.Flash_state = FLASH_MENU;
        break;
        */
    case 'i':
#if defined(CY_CORE_CM7_1)
        (void)dualcore_ui_cmd_push(DUALCORE_UI_CMD_JUMP, 0, 0.0f);
#else
        jump_flag = 1;
#endif
        break;
    case 'j':
#if defined(CY_CORE_CM7_1)
        (void)dualcore_ui_cmd_push(DUALCORE_UI_CMD_ROLL_BALANCE_TOGGLE, 0, 0.0f);
#else
        roll_balance_en = !roll_balance_en;
#endif
        break;
    case 'k':
#if defined(CY_CORE_CM7_1)
        (void)dualcore_ui_cmd_push(DUALCORE_UI_CMD_NAG_BEGIN_RECORD, 0, 0.0f);
#else
        Nag_Begin_Record();
#endif
        break;
    case 'l':
#if defined(CY_CORE_CM7_1)
        (void)dualcore_ui_cmd_push(DUALCORE_UI_CMD_NAG_BEGIN_REPLAY, 0, 0.0f);
#else
        Nag_Begin_Replay();
#endif
        break;
    case 'm':
#if defined(CY_CORE_CM7_1)
        (void)dualcore_ui_cmd_push(DUALCORE_UI_CMD_NAG_STOP_RECORD, 0, 0.0f);
#else
        Nag_Request_Stop_Record();
#endif
        break;
    case 'n':
#if defined(CY_CORE_CM7_1)
        (void)dualcore_ui_cmd_push(DUALCORE_UI_CMD_NAG_VOFA_GROUP_NEXT, 0, 0.0f);
#else
        Nag_Vofa_Group = (uint8)((Nag_Vofa_Group + 1) % NAG_VOFA_GROUP_COUNT);
#endif
        break;
    case 'o':
#if defined(CY_CORE_CM7_1)
        (void)dualcore_ui_cmd_push(DUALCORE_UI_CMD_SPIN_START, (uint32)(int8)1, 2.0f);
#else
        spin_task_start(2.0f, 1);
#endif
        break;
    case 'p':
#if defined(CY_CORE_CM7_1)
        (void)dualcore_ui_cmd_push(DUALCORE_UI_CMD_STEER_REL_DEG, 0, 30.0f);
#else
        steer_request_relative_yaw(30.0f);
#endif
        break;
    case 'q':
#if defined(CY_CORE_CM7_1)
        (void)dualcore_ui_cmd_push(DUALCORE_UI_CMD_SPEED_DELTA, 0, 500.0f);
#else
        run_launch_speed += 500.0f;
#endif
        break;
    case 'r':
#if defined(CY_CORE_CM7_1)
        (void)dualcore_ui_cmd_push(DUALCORE_UI_CMD_SPEED_DELTA, 0, -500.0f);
#else
        run_launch_speed -= 500.0f;
#endif
        break;
    case 's':
#if defined(CY_CORE_CM7_1)
        (void)dualcore_ui_cmd_push(DUALCORE_UI_CMD_SPEED_SET_ABS, 0, 0.0f);
#else
        run_launch_speed = 0.0f;
#endif
        break;
    case 't': /* 惯导录制：单击保存当前元素点 */
#if defined(CY_CORE_CM7_1)
        (void)dualcore_ui_cmd_push(DUALCORE_UI_CMD_NAG_EVENT_MARK, 0, 0.0f);
#else
        Nag_Request_Event_Mark();
#endif
        break;
    case 'u':
#if defined(CY_CORE_CM7_1)
        (void)dualcore_ui_cmd_push(DUALCORE_UI_CMD_NAG_CYCLE_EVENT_TYPE, 0, 0.0f);
#else
        Nag_Cycle_Record_Event_Type();
#endif
        break;
    case 'v':
#if defined(CY_CORE_CM7_1)
        (void)dualcore_ui_cmd_push(DUALCORE_UI_CMD_NAG_EVENT_DONE, 0, 0.0f);
#else
        Nag_Notify_Event_Done();
#endif
        break;
    }
    }
#else
    /* 按键+拨码模式：不消费串口单字节菜单命令，避免与按键双触发。 */
#endif

    Menu_command = 0;
    if(motor_sw_sel == MOTOR_OFF
  //  || MOTOR_ON
    )
    {
        menuMember.gui();
        menuMember.act();
    }
}

/**
 * @Function: MenuInit
 * @Description: 菜单初始化
 * @Param: Void
 * @Return: Void
 * @Example: MenuInit();
 * 注意：如果启用MENU_SELECT更改上电默认菜单选择，注意此初始化必须放到Flash_Init()之后，先读取flash数据再对菜单进行初始化
 */
void MenuInit()
{
    /*创建哈希表*/
    HashTableCtor(&hashMenu);

    /*输入内容，插入哈希表*/
    menuMember.gui = GUI_1;
    menuMember.act = ACT_1;
    strcpy(menuMember.pos, "1");
    hashMenu.vPtr->insert(&hashMenu, &menuMember);

    menuMember.gui = GUI_2;
    menuMember.act = ACT_2;
    strcpy(menuMember.pos, "2");
    hashMenu.vPtr->insert(&hashMenu, &menuMember);

    menuMember.gui = GUI_3;
    menuMember.act = ACT_3;
    strcpy(menuMember.pos, "3");
    hashMenu.vPtr->insert(&hashMenu, &menuMember);

    menuMember.gui = GUI_1_1;
    menuMember.act = ACT_1_1;
    strcpy(menuMember.pos, "1.1");
    hashMenu.vPtr->insert(&hashMenu, &menuMember);

    menuMember.gui = GUI_1_2;
    menuMember.act = ACT_1_2;
    strcpy(menuMember.pos, "1.2");
    hashMenu.vPtr->insert(&hashMenu, &menuMember);

    menuMember.gui = GUI_1_3;
    menuMember.act = ACT_1_3;
    strcpy(menuMember.pos, "1.3");
    hashMenu.vPtr->insert(&hashMenu, &menuMember);

    menuMember.gui = GUI_1_4;
    menuMember.act = ACT_1_4;
    strcpy(menuMember.pos, "1.4");
    hashMenu.vPtr->insert(&hashMenu, &menuMember);

    menuMember.gui = GUI_1_5;
    menuMember.act = ACT_1_5;
    strcpy(menuMember.pos, "1.5");
    hashMenu.vPtr->insert(&hashMenu, &menuMember);

    menuMember.gui = GUI_2_1;
    menuMember.act = ACT_2_1;
    strcpy(menuMember.pos, "2.1");
    hashMenu.vPtr->insert(&hashMenu, &menuMember);

    menuMember.gui = GUI_2_2;
    menuMember.act = ACT_2_2;
    strcpy(menuMember.pos, "2.2");
    hashMenu.vPtr->insert(&hashMenu, &menuMember);

    menuMember.gui = GUI_2_3;
    menuMember.act = ACT_2_3;
    strcpy(menuMember.pos, "2.3");
    hashMenu.vPtr->insert(&hashMenu, &menuMember);

    menuMember.gui = GUI_2_4;
    menuMember.act = ACT_2_4;
    strcpy(menuMember.pos, "2.4");
    hashMenu.vPtr->insert(&hashMenu, &menuMember);

    menuMember.gui = GUI_2_5;
    menuMember.act = ACT_2_5;
    strcpy(menuMember.pos, "2.5");
    hashMenu.vPtr->insert(&hashMenu, &menuMember);

    menuMember.gui = GUI_2_6;
    menuMember.act = ACT_2_6;
    strcpy(menuMember.pos, "2.6");
    hashMenu.vPtr->insert(&hashMenu, &menuMember);

    menuMember.gui = GUI_3_1;
    menuMember.act = ACT_3_1;
    strcpy(menuMember.pos, "3.1");
    hashMenu.vPtr->insert(&hashMenu, &menuMember);

    menuMember.gui = GUI_3_2;
    menuMember.act = ACT_3_2;
    strcpy(menuMember.pos, "3.2");
    hashMenu.vPtr->insert(&hashMenu, &menuMember);

    menuMember.gui = GUI_3_3;
    menuMember.act = ACT_3_3;
    strcpy(menuMember.pos, "3.3");
    hashMenu.vPtr->insert(&hashMenu, &menuMember);

    menuMember.gui = GUI_3_1_1;
    menuMember.act = ACT_3_1_1;
    strcpy(menuMember.pos, "3.1.1");
    hashMenu.vPtr->insert(&hashMenu, &menuMember);
    
    menuMember.gui = GUI_1_1_1;
    menuMember.act = ACT_1_1_1;
    strcpy(menuMember.pos, "1.1.1");
    hashMenu.vPtr->insert(&hashMenu, &menuMember);

        menuMember.gui = GUI_1_2_1;
        menuMember.act = ACT_1_2_1;
        strcpy(menuMember.pos, "1.2.1");
        hashMenu.vPtr->insert(&hashMenu, &menuMember);

        menuMember.gui = GUI_1_5_1;
        menuMember.act = ACT_1_5_1;
        strcpy(menuMember.pos, "1.5.1");
        hashMenu.vPtr->insert(&hashMenu, &menuMember);
/*
        menuMember.gui = GUI_1_3_1;
        menuMember.act = ACT_1_3_1;
        strcpy(menuMember.pos, "1.3.1");
        hashMenu.vPtr->insert(&hashMenu, &menuMember);

        menuMember.gui = GUI_1_4_1;
        menuMember.act = ACT_1_4_1;
        strcpy(menuMember.pos, "1.4.1");
        hashMenu.vPtr->insert(&hashMenu, &menuMember);

        //    menuMember.gui=GUT_1_5_1;
        //    menuMember.act=ACT_1_5_1;
        //    strcpy(menuMember.pos."1.5.1");
        //    hashMenu.vPte->insert(&hasMenu.&menuMember);

        //
        //     menuMember.gui=GUI_1_5_1;
        //     menuMember.act=ACT_1_5_1;
        //     strcpy(menuMember.pos,"1.5.1");
        //     hashMenu.vPtr->insert(&hashMenu,&menuMember);
        //
*/
        menuMember.gui = GUI_2_1_1;
        menuMember.act = ACT_2_1_1;
        strcpy(menuMember.pos, "2.1.1");
        hashMenu.vPtr->insert(&hashMenu, &menuMember);

        menuMember.gui = GUI_2_1_2;
        menuMember.act = ACT_2_1_2;
        strcpy(menuMember.pos, "2.1.2");
        hashMenu.vPtr->insert(&hashMenu, &menuMember);

        menuMember.gui = GUI_2_1_3;
        menuMember.act = ACT_2_1_3;
        strcpy(menuMember.pos, "2.1.3");
        hashMenu.vPtr->insert(&hashMenu, &menuMember);

        menuMember.gui = GUI_2_2_1;
        menuMember.act = ACT_2_2_1;
        strcpy(menuMember.pos, "2.2.1");
        hashMenu.vPtr->insert(&hashMenu, &menuMember);

        menuMember.gui = GUI_2_3_1;
        menuMember.act = ACT_2_3_1;
        strcpy(menuMember.pos, "2.3.1");
        hashMenu.vPtr->insert(&hashMenu, &menuMember);
/*

        menuMember.gui = GUI_2_2_1;
        menuMember.act = ACT_2_2_1;
        strcpy(menuMember.pos, "2.2.1");
        hashMenu.vPtr->insert(&hashMenu, &menuMember);

        menuMember.gui = GUI_2_2_2;
        menuMember.act = ACT_2_2_2;
        strcpy(menuMember.pos, "2.2.2");
        hashMenu.vPtr->insert(&hashMenu, &menuMember);

            menuMember.gui=GUI_2_2_3;
            menuMember.act=ACT_2_2_3;
            strcpy(menuMember.pos,"2.2.3");
            hashMenu.vPtr->insert(&hashMenu,&menuMember);

            menuMember.gui=GUI_2_2_4;
            menuMember.act=ACT_2_2_4;
            strcpy(menuMember.pos,"2.2.4");
            hashMenu.vPtr->insert(&hashMenu,&menuMember);

            menuMember.gui=GUI_2_2_5;
            menuMember.act=ACT_2_2_5;
            strcpy(menuMember.pos,"2.2.5");
            hashMenu.vPtr->insert(&hashMenu,&menuMember);

            menuMember.gui=GUI_2_2_6;
            menuMember.act=ACT_2_2_6;
            strcpy(menuMember.pos,"2.2.6");
            hashMenu.vPtr->insert(&hashMenu,&menuMember);

            menuMember.gui=GUI_2_3_1;
            menuMember.act=ACT_2_3_1;
            strcpy(menuMember.pos,"2.3.1");
            hashMenu.vPtr->insert(&hashMenu,&menuMember);

            menuMember.gui=GUI_2_3_2;
            menuMember.act=ACT_2_3_2;
            strcpy(menuMember.pos,"2.3.2");
            hashMenu.vPtr->insert(&hashMenu,&menuMember);

            menuMember.gui=GUI_2_3_3;
            menuMember.act=ACT_2_3_3;
            strcpy(menuMember.pos,"2.3.3");
            hashMenu.vPtr->insert(&hashMenu,&menuMember);

            menuMember.gui=GUI_2_3_4;
            menuMember.act=ACT_2_3_4;
            strcpy(menuMember.pos,"2.3.4");
            hashMenu.vPtr->insert(&hashMenu,&menuMember);
*/
#if MENU_SELECT
    hashMenu.vPtr->search(&hashMenu, &menuMember, &ReadPos[0]);
#else
    hashMenu.vPtr->search(&hashMenu, &menuMember, "2.2.1");
#endif
}

/*----------------------------------------------------------------
                            驱动层
----------------------------------------------------------------*/
/*----------------------------------------------------------------
 * @Function: HashTableCtor
 * @Description: Hash表构造函数
 * @Param: {HASH_TABLE_t*} const This    哈希链表结构体
 * @Return: Void
 * @Example: HashTableCtor(&hashMenu);
 */
static void HashTableCtor(HASH_TABLE_t *const This)
{
    static HASH_VTBL_t vTable;
    MemSet(This, 0, sizeof(HASH_TABLE_t));

    vTable.insert = HashInsert;
    vTable.search = FindHashValue;
    /* 固定为「菜单树语义」绑定，参见文件头「易错点」。
     * 若把 Down/Right 等对调成「按键字面」，会与 HashPeer/ HashDepth 实现及参考工程语义全部冲突 */
    vTable.searchDown = HashDepthDown;
    vTable.searchUp = HashDepthUp;
    vTable.searchLeft = HashPeerLeft;
    vTable.searchRight = HashPeerRight;

    This->vPtr = &vTable;
}

/*----------------------------------------------------------------
 * @Function: CreatHashKey
 * @Description: 生成HASH表关键字
 * @param {char*} skey
 * @Return: Void
 * @Example: hashKey = CreatHashKey(tempMember->pos);
 */
static uint16_t CreatHashKey(const char *skey)
{
    // 这里传进去的是字符串
    char *p = (char *)skey; // 类型强转
    uint16_t hashKey = 0;
    if (*p) // 字符串非空
    {
        for (; *p != '\0'; p++)
        {
            hashKey = (hashKey << 5) - hashKey + *p; // 生成关键字（Key）
        }
    }
    return hashKey % (HASH_SIZE);
}

/*----------------------------------------------------------------
 * @Function: HashInsert
 * @Description: 将元素插入表中
 * @Param: {HASH_TABLE_t*} const This
 * @Param: {MENU_MEMBER_t*} const tempMember
 * @Return: Void
 * @Example: HashInsert(&hashMenu, &menuMember);
 */
static uint8_t HashInsert(HASH_TABLE_t *const This, MENU_MEMBER_t *const tempMember)
{
    if (This->hashTableSize >= HASH_SIZE - 1) // 如果表满了
    {
        return HASH_ERROR;
    }
    uint16_t hashKey = CreatHashKey(tempMember->pos);
    while (This->hashTable[hashKey].pos[0] != 0)
    {
        hashKey = (hashKey + 1) % HASH_SIZE;
    }

    MemCpy(&(This->hashTable[hashKey]), tempMember, sizeof(MENU_MEMBER_t));
    This->hashTableSize++;
    return HASH_OK;
}

/*----------------------------------------------------------------
 * @Function: HashDepthUp
 * @Description: 上一级
 * @Param: {HASH_TABLE_t*} const This
 * @Param: {MENU_MEMBER_t*} const tempMember
 * @Return: Void
 * @Example: HashDepthUp(&hashMenu, &menuMember);
 */
static uint8_t HashDepthUp(HASH_TABLE_t *const This, MENU_MEMBER_t *const tempMember)
{
    char tempPos[HASH_KEY_LEN];
    uint8_t flag;
    char *ptrPos;
    if (strlen(tempMember->pos) <= 1)
    {
        return HASH_ERROR; /*避免内存越界*/
    }
    strcpy((char *)tempPos, tempMember->pos);
    ptrPos = tempPos;
    while (*(++ptrPos) != '\0') // 使指针指向字符串最后"\0"
    {
    }
    *(ptrPos - 1) = '\0';
    *(ptrPos - 2) = '\0';
    flag = FindHashValue(This, tempMember, tempPos); // 将找到的新菜单写入tempMember中
    if (flag == HASH_ERROR)
    {
        return HASH_ERROR;
    }
    else
    {
        return HASH_OK;
    }
}

/*----------------------------------------------------------------
 * @Function: HashDepthDown
 * @Description: 下一级
 * @Param: {HASH_TABLE_t*} const This
 * @Param: {MENU_MEMBER_t*} const tempMember
 * @Return: Void
 * @Example: HashDepthDown(&hashMenu, &menuMember);
 */
static uint8_t HashDepthDown(HASH_TABLE_t *const This, MENU_MEMBER_t *const tempMember)
{
    char tempPos[HASH_KEY_LEN];
    uint8_t flag;
    char *ptrPos;
    if (strlen(tempMember->pos) >= HASH_KEY_LEN - 3)
    {
        return HASH_ERROR; /*避免内存越界*/
    }
    strcpy(tempPos, tempMember->pos);
    ptrPos = tempPos;
    while (*(++ptrPos) != '\0') // 使指针指向字符串最后"\0"
    {
    }
    *ptrPos = '.';
    *(ptrPos + 1) = '1';
    *(ptrPos + 2) = '\0';
    flag = FindHashValue(This, tempMember, tempPos); // 将找到的新菜单写入tempMember中
    if (flag == HASH_ERROR)
    {
        return HASH_ERROR;
    }
    else
    {
        return HASH_OK;
    }
}

/*----------------------------------------------------------------
 * @Function: HashPeerLeft
 * @Description: 左平级
 * @Param: {HASH_TABLE_t*} const This
 * @Param: {MENU_MEMBER_t*} const tempMember
 * @Return: Void
 * @Example: HashPeerLeft(&hashMenu, &menuMember);
 */
static uint8_t HashPeerLeft(HASH_TABLE_t *const This, MENU_MEMBER_t *const tempMember)
{
    char tempPos[HASH_KEY_LEN];
    uint8_t temp;
    uint8_t flag;
    char *ptrPos;

    strcpy(tempPos, tempMember->pos);
    ptrPos = tempPos;
    while (*(++ptrPos) != '\0') // 使指针指向字符串最后"\0"
    {
    }
    temp = *(--ptrPos) - 1;
    *(ptrPos) = temp;                                // 将最后位置的上一个字符值-1；比如说从"1.1.3"变为"1.1.2"
    flag = FindHashValue(This, tempMember, tempPos); // 将找到的新菜单写入tempMember中
    if (flag == HASH_ERROR)
    {
        return HASH_ERROR;
    }
    else
    {
        return HASH_OK;
    }
}

/*----------------------------------------------------------------
 * @Function: HashPeerRight
 * @Description: 右平级
 * @Param: {HASH_TABLE_t*} const This
 * @Param: {MENU_MEMBER_t*} const tempMember
 * @Return: Void
 * @Example: HashPeerRight(&hashMenu, &menuMember);
 */
static uint8_t HashPeerRight(HASH_TABLE_t *const This, MENU_MEMBER_t *const tempMember)
{
    char tempPos[HASH_KEY_LEN];
    uint8_t temp;
    uint8_t flag;
    char *ptrPos;

    strcpy(tempPos, tempMember->pos);
    ptrPos = tempPos;
    while (*(++ptrPos) != '\0') // 使指针指向字符串最后"\0"
    {
    }
    temp = *(--ptrPos) + 1;
    *(ptrPos) = temp;                                // 将最后位置的上一个字符值-1；比如说从"1.1.3"变为"1.1.2"
    flag = FindHashValue(This, tempMember, tempPos); // 将找到的新菜单写入tempMember中
    if (flag == HASH_ERROR)
    {
        return HASH_ERROR;
    }
    else
    {
        return HASH_OK;
    }
}

/*----------------------------------------------------------------
 * @Function: FindHashValue
 * @Description: 由关键字返回对应值
 * @Param: {HASH_TABLE_t*} const This
 * @Param: {MENU_MEMBER_t*} const tempMember
 * @param {char*} str
 * @Return: Void
 * @Example: FindHashValue(&hashMenu, &menuMember,tempPos);
 */
static uint8_t FindHashValue(HASH_TABLE_t *const This, MENU_MEMBER_t *const tempMember, char *str)
{
    int count = HASH_SIZE, hashKey;
    hashKey = CreatHashKey(str);
    while (count--)
    {
        if (This->hashTable[hashKey].pos[0] == '\0')
        {
            hashKey = (hashKey + 1) % HASH_SIZE;
            continue;
        }
        if (strcmp(This->hashTable[hashKey].pos, str) == 0)
        {
            MemCpy(tempMember, &(This->hashTable[hashKey]), sizeof(MENU_MEMBER_t));
            return HASH_OK;
        }
        hashKey = (hashKey + 1) % HASH_SIZE;
    }
    return HASH_ERROR;
};

/*----------------------------------------------------------------
 * @Function: MemSet
 * @Description: 申请内存空间并初始化
 * @param {void} *src
 * @param {int} value
 * @param {int} n
 * @Return: Void
 * @Example: MemSet(This, 0, sizeof(HASH_TABLE_t));
 */
static void *MemSet(void *src, int value, int n) /*初始化*/
{
    char *c_src = (char *)src;
    while (n--)
    {
        *c_src++ = value;
    }
    return src;
}

/*----------------------------------------------------------------
 * @Function: MemCpy
 * @Description: 复制内存空间
 * @param {void} *dest
 * @param {void} *src
 * @param {unsigned int} size
 * @Return: Void
 * @Example:     MemCpy(tempMember, &(This->hashTable[hashKey]),sizeof(MENU_MEMBER_t)); //将找到的菜单写入tempMember
 */
static void *MemCpy(void *dest, void *src, unsigned int size)
{
    char *b_dest = (char *)dest, *b_src = (char *)src;
    unsigned int len;
    for (len = size; len > 0; len--)
    {
        *b_dest++ = *b_src++;
    }
    return dest;
}
