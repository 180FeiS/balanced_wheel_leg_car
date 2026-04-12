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

 4. 菜单控制接口:
    - hashMenu.vPtr->searchUp(): 切换到上一个同级菜单
    - hashMenu.vPtr->searchDown(): 切换到下一个同级菜单
    - hashMenu.vPtr->searchLeft(): 返回上级菜单
    - hashMenu.vPtr->searchRight(): 进入下级菜单

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
 * 格式：V<数值>  例 V1200 / V0 / V-800（写入 motor_user_speed_cmd，无速度锁；SWITCH2 仍可改写）
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
        motor_user_speed_cmd_set_from_pc((float)atof(&tmp[1]));
        return 1u;
    }
    return 0u;
}

/*-------------------------------------------------------------------------
 * 拨码与速度基准（须周期性调用，如 selectMenu_Key / selectMenu 内）：
 * 1. SWITCH2：motor_poll_switch2_speed_baseline() 按档位/边沿刷新 motor_user_speed_cmd（1000/1500）。
 * 2. SWITCH1：Motor_Switch 唯一来源（失控锁存除外）。
 * 3. 导航未进入回放执行态前，速度环仍由 Nag_GetControlSpeedTarget() 门控为 0。
 * 4. Motor_Runaway_Latch：最高优先级关电机；须 SWITCH1 到 OFF 后才清除锁存。
 *-------------------------------------------------------------------------*/
void dip_switch_motor_sync_from_hw(void)
{
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
            ips200_clear();
            menuMember.gui();
            menuMember.act();
        }
    }
}

void menu_key_capture_event(void)
{
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
             * 1. 录制中：标记元素 enter/exit；
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
}

void selectMenu_Key(void)
{
   uint8 menu_nav = 0;
   menu_key_nav_enum nav = MENU_KEY_NAV_NONE;

   dip_switch_motor_sync_from_hw();

   while(MenuKeyEventPop(&nav))
   {
        switch(nav)
        {
        case MENU_KEY_NAV_LEFT:
             /* 左：返回上一级 */
             hashMenu.vPtr->searchLeft(&hashMenu, &menuMember);
             menu_nav = 1;
             break;
        case MENU_KEY_NAV_RIGHT:
             /* 右：进入下一级 */
             hashMenu.vPtr->searchRight(&hashMenu, &menuMember);
             menu_nav = 1;
             break;
        case MENU_KEY_NAV_DOWN:
             /* 下：切换到下一个同级项 */
             hashMenu.vPtr->searchDown(&hashMenu, &menuMember);
             menu_nav = 1;
             break;
        case MENU_KEY_NAV_UP:
             /* 上：切换到上一个同级项 */
             hashMenu.vPtr->searchUp(&hashMenu, &menuMember);
             menu_nav = 1;
             break;
        default:
             break;
        }
   }

   /* 与 selectMenu() 一致：导航后立刻重绘，且须在主循环内完成，避免与 ips200 SPI 冲突。 */
   if(menu_nav && (Motor_Switch == MOTOR_OFF))
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
    dip_switch_motor_sync_from_hw();
    switch (Menu_command)
    {
    case 'a':
        hashMenu.vPtr->searchUp(&hashMenu, &menuMember);
        ips200_clear();
        break;

    case 'b':
        hashMenu.vPtr->searchDown(&hashMenu, &menuMember);
        ips200_clear();
        break;

    case 'c':
        hashMenu.vPtr->searchRight(&hashMenu, &menuMember);
        ips200_clear();
        break;

    case 'd':
        hashMenu.vPtr->searchLeft(&hashMenu, &menuMember);
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
        jump_flag = 1;
        break;
    case 'j':
        roll_balance_en = !roll_balance_en;
        break;
    case 'k':
        Nag_Begin_Record();
        break;
    case 'l':
        Nag_Begin_Replay();
        break;
    case 'm':
        Nag_Request_Stop_Record();
        break;
    case 'n':
        Nag_Vofa_Group = (uint8)((Nag_Vofa_Group + 1) % 6);
        break;
    case 'o':
        /* 调试入口：发送字符 o 后，直接启动 1 圈正向自旋。
         * 自旋保持独立任务，不走 steer_request_target_yaw() 请求链，
         * 否则会变成持续重置普通转向目标，破坏当前的自旋收尾与互斥逻辑。
         */
        spin_task_start(2.0f, 1);
        
        break;
    case 'p':
        /* 调试入口：发送字符 p 后，登记一次相对转角请求。
         * 当前航向 + 相对角度 的换算由 control.c 统一处理，
         * 真正的 steer_set_target_yaw() 会在下一拍 1ms ISR 里安全执行。
         */
        steer_request_relative_yaw(30.0f);
        break;
    case 'q':
        /* 调试：步进增加用户速度基准（拨动 SWITCH2 仍会按档位刷新为 1000/1500） */
        motor_user_speed_cmd += 500.0f;
        break;
    case 'r':
        motor_user_speed_cmd -= 500.0f;
        break;
    case 's':
        /* 紧急清零速度命令；拨动 SWITCH2 仍会回到 1000/1500 */
        motor_user_speed_cmd = 0.0f;
        break;
    case 't':
        /* 录制阶段手动标记元素 enter/exit：
         * 第一次按下记录 enter_index，第二次按下记录 exit_index；
         * 当前版本先只写 RAM，不写 flash，方便先把“切出/接回”链路跑通。
         */
        Nag_Request_Event_Mark();
        break;
    case 'u':
        /* 切换下一条待录元素类型。
         * 建议录制时先固定一种元素把流程跑通，再逐步区分 STEP/JUMP 等类型。
         */
        Nag_Cycle_Record_Event_Type();
        break;
    case 'v':
        /* 回放或人工调试时，手动通知“当前元素已完成”，导航将从 exit_index 继续。 */
        Nag_Notify_Event_Done();
        break;
    }
    
    Menu_command = 0;
    if(Motor_Switch == MOTOR_OFF
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

    // menuMember.gui=GUI_1_5;
    // menuMember.act=ACT_1_5;
    // strcpy(menuMember.pos,"1.5");
    // hashMenu.vPtr->insert(&hashMenu,&menuMember);

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
    
    menuMember.gui = GUI_1_1_1;
    menuMember.act = ACT_1_1_1;
    strcpy(menuMember.pos, "1.1.1");
    hashMenu.vPtr->insert(&hashMenu, &menuMember);

        menuMember.gui = GUI_1_2_1;
        menuMember.act = ACT_1_2_1;
        strcpy(menuMember.pos, "1.2.1");
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
/*

        menuMember.gui = GUI_2_2_1;
        menuMember.act = ACT_2_2_1;
        strcpy(menuMember.pos, "2.2.1");
        hashMenu.vPtr->insert(&hashMenu, &menuMember);

        menuMember.gui = GUI_2_2_2;
        menuMember.act = ACT_2_2_2;
        strcpy(menuMember.pos, "2.2.2");
        hashMenu.vPtr->insert(&hashMenu, &menuMember);
/*

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
    hashMenu.vPtr->search(&hashMenu, &menuMember, "2.1.1");
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
    /* 对外接口按按键方向语义绑定：
     * Up/Down   -> 同级切换
     * Left      -> 返回上级
     * Right     -> 进入下级
     */
    vTable.searchDown = HashPeerRight;
    vTable.searchUp = HashPeerLeft;
    vTable.searchLeft = HashDepthUp;
    vTable.searchRight = HashDepthDown;

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
