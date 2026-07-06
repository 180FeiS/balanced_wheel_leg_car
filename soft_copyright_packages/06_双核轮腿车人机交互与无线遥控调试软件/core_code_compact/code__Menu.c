/*
 * @Author: bom
 * @Version: V1.0
 * @Date: 2024-12-12 22:02:51
 * @LastEditTime: 2024-12-24 22:46:12
 * @FilePath: \Balance_Car V3.0.1\project\code\Menu.c
 * @Description:
 */
/*********************************************************************************************************************
 ��ֲʹ��˵����
 1. ��Ҫ������ͷ�ļ�:
    - Menu.h: �˵���ض��������
    - string.h: �ַ�����������
    - UI.h: ������ʾ��غ���
 2. �ؼ��ṹ�ͱ���:
    - HASH_TABLE_t hashMenu: ȫ�ֹ�ϣ���������洢���в˵���
    - MENU_MEMBER_t menuMember: ��ǰѡ�еĲ˵�����Ϣ
 3. ʹ�ò���:
    a) ���� MenuInit() ��ʼ���˵�ϵͳ
    b) �� MenuInit() �����ж���˵��ṹ:
       - ���� menuMember.gui (������ʾ����)
       - ���� menuMember.act (�˵���������)
       - ���� menuMember.pos (�˵�λ�ñ�ţ���"1"��"1.1"��)
       - ���� hashMenu.vPtr->insert() ����˵���
 4. �˵����ƽӿڣ���ο����̽ṹ����һ�£����ؼ��� selectMenu_Key ��ӳ�䵽�����壩:
    - hashMenu.vPtr->searchUp(): �����ϼ��˵�
    - hashMenu.vPtr->searchDown(): �����¼��˵�
    - hashMenu.vPtr->searchLeft(): �л�����һ��ͬ���˵�
    - hashMenu.vPtr->searchRight(): �л�����һ��ͬ���˵�
    ���״��㣺������� vs ���ؼ������������Ժ��ٸĻ�ȥ�ȿӣ�
    - vtable �ĸ�ָ��������ʷ��������롸�˵�����һ�£��� Balance_Car_initial Menu.c ��ͬ����
      searchDown=����(HashDepthDown)��searchUp=����(HashDepthUp)��Left/Right=ͬ��(HashPeerLeft/PeerRight)��
    - �����˰� HashTableCtor д�ɡ������������塹������� Up/Down ���ͬ����Left/Right ��ɷ���/���ӣ���
      ������ο����̼� HashDepth/HashPeer ��������ȫ�Բ��ϣ�Run/Debug �ȷ�֧��Ϊ�������λ��
    - �����̰���ϰ���� KEY1/2 ��ͬ����KEY3 ���롢KEY4 ���أ�ֻ���� selectMenu_Key()�������� a/b/c/d��
      ��������¼�ӳ�䵽�����ĸ� search*����Ҫ�ٸ� HashTableCtor ȥ��ӭ�ϼ�������
 5. ע������:
    - �˵�λ�ñ�Ÿ�ʽΪ: "1"��"2"(һ���˵�)��"1.1"��"1.2"(�����˵�)
    - ���֧�ֲ˵������ HASH_KEY_LEN ����
    - ���˵��������� HASH_SIZE ����
    ���յ���
    menuMember.gui();
    menuMember.act();
********************************************************************************************************************/
/*********************************************************************************************************************
 * �޸ļ�¼��
 * �汾��      ����          ����      ˵��
 * V1.0.0    2024-07-24    Bron     ��¹���
 * V1.0.2    2024-07-27    Bron     ������˵����
 * V1.1.0    2024-07-30    Bron     ���򰴼���Ϊvofaģ�ⰴ��
 * V1.1.4    2024-08-01    Bron     ����Һ�������л�ģʽ
 * V2.0.1    2024-12-19    Bron     ����Menu��Manu_Basic
 ********************************************************************************************************************/
#include "zf_common_headfile.h"
#include <stdlib.h>
#include <string.h>
#include "dualcore_shared.h"
#include "my_gps.h"
#include "image.h"
#include "control.h"
#if defined(CY_CORE_CM7_0)
#include "navigation.h"
#include "flash.h"
#include "nav_fusion.h"
#endif
/* ȫ�ֱ������� */
HASH_TABLE_t hashMenu;    // ����˵���Ĺ�ϣ��
MENU_MEMBER_t menuMember; // ��ǰѡ�еĲ˵�����Ϣ
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
/* �ڲ��������� */
static void HashTableCtor(HASH_TABLE_t *const This);                                                // ��ϣ����ʼ��
static uint16_t CreatHashKey(const char *skey);                                                     // ���ɹ�ϣֵ
static void *MemSet(void *src, int value, int n);                                                   // �ڴ��ʼ��
static void *MemCpy(void *dest, void *src, unsigned int size);                                      // �ڴ渴��
static uint8_t HashInsert(HASH_TABLE_t *const This, MENU_MEMBER_t *const tempMember);               // ����˵���
static uint8_t HashDepthUp(HASH_TABLE_t *const This, MENU_MEMBER_t *const tempMember);              // �����ϼ��˵�
static uint8_t HashDepthDown(HASH_TABLE_t *const This, MENU_MEMBER_t *const tempMember);            // �����¼��˵�
static uint8_t HashPeerLeft(HASH_TABLE_t *const This, MENU_MEMBER_t *const tempMember);             // �л������ͬ���˵�
static uint8_t HashPeerRight(HASH_TABLE_t *const This, MENU_MEMBER_t *const tempMember);            // �л����Ҳ�ͬ���˵�
static uint8_t FindHashValue(HASH_TABLE_t *const This, MENU_MEMBER_t *const tempMember, char *str); // ���Ҳ˵���
static void MenuKeyEventPush(menu_key_nav_enum nav);
static uint8 MenuKeyEventPop(menu_key_nav_enum *nav);
static uint8 MenuIsNavDebugPage(void);
static uint8 MenuIsGpsDebugPage(void);
static uint8 MenuIsRunLaunchSpeedPage(void);
static uint8 MenuIsRunJumpPage(void);
static uint8 MenuIsRunFlashPage(void);
static uint8 MenuIsRunConfigPage(void);
static void MenuAdjustRunLaunchParam(float delta);
static uint8 MenuTryHandleRunLaunchSpeedKeyEvent(void);
static uint8 MenuTryHandleRunJumpKeyEvent(void);
static uint8 MenuTryHandleRunFlashKeyEvent(void);
static uint8 MenuTryHandleRunConfigKeyEvent(void);
static uint8 MenuTryHandleGpsDebugKeyEvent(void);
static uint8 MenuIsRemoteMenuFirst(void);
static uint8 s_was_in_image_section = 0u;
uint8 MenuIsImageSectionPage(void)
{
    return (uint8)(strncmp(menuMember.pos, "2.1", 3) == 0);
}
void Menu_UpdateImageAeArm(void)
{
    uint8 in_image = MenuIsImageSectionPage();
    if (in_image && (s_was_in_image_section == 0u))
    {
        image_ae_session_arm();
    }
    if (!in_image)
    {
        s_was_in_image_section = 0u;
    }
    else
    {
        s_was_in_image_section = 1u;
    }
}
/* Launch ҳ��KEY1 ѭ��ѡ���ֶΣ�KEY2/KEY3 ���ٶȡ�100�������10 ���ڡ� */
static uint8 s_run_launch_field_index = 0u;
/* Config ҳ��KEY1 ѭ��ѡ��Ԥ�����ֶΣ�KEY2 �л���ǰ�ֶ�ȡֵ�� */
static uint8 s_run_config_field_index = 0u;
/* Jump ҳ��KEY1 ѭ��ѡ���ֶΣ�KEY2/KEY3 �� ��0.5 ���ڡ� */
static uint8 s_run_jump_field_index = 0u;
uint8 Menu_GetRunLaunchFieldIndex(void)
{
    return s_run_launch_field_index;
}
uint8 Menu_GetRunConfigFieldIndex(void)
{
    return s_run_config_field_index;
}
uint8 Menu_GetRunJumpFieldIndex(void)
{
    return s_run_jump_field_index;
}
void Menu_RunConfigToggleField(uint8 field_index)
{
#if defined(CY_CORE_CM7_0)
    if (field_index == Run_Config_Field_InputMode)
    {
        g_menu_input_remote_first = (uint8)(g_menu_input_remote_first ? 0u : 1u);
    }
    else if (field_index == Run_Config_Field_VofaEnable)
    {
        g_menu_vofa_enable = (uint8)(g_menu_vofa_enable ? 0u : 1u);
    }
    else if (field_index == Run_Config_Field_VofaGroup)
    {
        Nag_Vofa_Group = (uint8)((Nag_Vofa_Group + 1u) % NAG_VOFA_GROUP_COUNT);
    }
    else if (field_index == Run_Config_Field_FusionEnable)
    {
        g_menu_nav_fusion_enable = (uint8)(g_menu_nav_fusion_enable ? 0u : 1u);
        if (g_menu_nav_fusion_enable == 0u)
        {
            NavFusion_Reset();
        }
    }
#else
    (void)field_index;
#endif
}
static uint8 MenuIsRemoteMenuFirst(void)
{
#if defined(CY_CORE_CM7_1)
    dualcore_ctrl_to_ui_t dc_rm;
    dualcore_ctrl_to_ui_pull(&dc_rm);
    return dc_rm.menu_input_remote_first;
#else
    return g_menu_input_remote_first;
#endif
}
/*-------------------------------------------------------------------------
 * �˵��ӿں���-�û�ֻ����Ĵ˲���
 *-------------------------------------------------------------------------*/
/*
 * @Function: selectMenu_Key
 * @Description: ʹ�ð����˵�ѡ��������
 * @Param: Void
 * @Return: Void
 * @Example: selectMenu();
 */
/* ���� 1����֡������չЭ�����ѣ�ReadDataFromPc ��Ӧ��д�� Menu_command��
 * selectMenu() ���������ֽڣ����ֽ� V<��ֵ> �ڱ�����������
 * ��ʽ��V<��ֵ>  �� V1200 / V0 / V-800��д�� run_launch_speed�����ߵ��طŽ���ִ��̬ʱװ�أ�
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
 * �������ٶȻ�׼���������Ե��ã��� selectMenu_Key / selectMenu �ڣ���
 * 1. SWITCH2�����ش��� motor_poll_switch2_speed_baseline()������ʱ��ִ�� yaw ������ã���ǰ�����Ϊ 0�����ɹ���ת LED1��
 * 2. SWITCH1��Motor_Switch Ψһ��Դ��ʧ��������⣩��
 * 3. ����δ����ط�ִ��̬ǰ���ٶȻ����� Nag_GetControlSpeedTarget() �ſ�Ϊ 0��
 * 4. Motor_Runaway_Latch��������ȼ��ص����ң�����ȹر�ʱ�� SWITCH1 �� OFF ���������档
 *    g_menu_input_remote_first==1 ʱ�������룬�������ң�����λ���ض�������ʧ���Լ� control.c��
 *-------------------------------------------------------------------------*/
void dip_switch_motor_sync_from_hw(void)
{
#if defined(CY_CORE_CM7_1)
    (void)0;
#else
    /* ң�������ҷǰ��ص��ԣ����� SWITCH�����ص���ʱ�� GPIO����ͬ����ģʽ�� */
    if (g_menu_input_remote_first != 0u && g_remote_local_keys_debug == 0u)
    {
        return;
    }
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
 * ����ɨ����ӣ�MENU_KEY_NAV_* ֻ��ʾ��������ķ����¼����͡��������� selectMenu_Key() ���䵽 hash �����
 * ���� KEY3 �Ƶ� MENU_KEY_NAV_RIGHT����ʵ�ʲ˵������ǡ������¼�������Ӧ hash ��Ӧ�� searchDown()��
 * ��Ҫ���� RIGHT �����ȥ�� searchRight()���ǻ���ͬ���л�����
 */
void menu_key_capture_event(void)
{
#if defined(CY_CORE_CM7_1)
   dualcore_ctrl_to_ui_t dc;
   dualcore_ctrl_to_ui_pull(&dc);
   if (MenuIsRemoteMenuFirst() != 0u && dc.remote_local_keys_debug == 0u)
   {
       return;
   }
   uint8 nav_recording_active = dc.nav_recording_active;
   uint8 event_active = dc.event_active;
   if(MenuIsRunLaunchSpeedPage() && MenuTryHandleRunLaunchSpeedKeyEvent())
   {
        return;
   }
   if(MenuIsRunJumpPage() && MenuTryHandleRunJumpKeyEvent())
   {
        return;
   }
   if(MenuIsRunFlashPage() && MenuTryHandleRunFlashKeyEvent())
   {
        return;
   }
   if(MenuIsRunConfigPage() && MenuTryHandleRunConfigKeyEvent())
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
   if (MenuIsRemoteMenuFirst() != 0u && g_remote_local_keys_debug == 0u)
   {
       return;
   }
   if(MenuIsRunLaunchSpeedPage() && MenuTryHandleRunLaunchSpeedKeyEvent())
   {
        return;
   }
   if(MenuIsRunJumpPage() && MenuTryHandleRunJumpKeyEvent())
   {
        return;
   }
   if(MenuIsRunFlashPage() && MenuTryHandleRunFlashKeyEvent())
   {
        return;
   }
   if(MenuIsRunConfigPage() && MenuTryHandleRunConfigKeyEvent())
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
        /* ¼��̬ͳһ�ſأ�
         * 1. Nag_SystemRun_Index == 1 ��ʾ�Դ���¼�����̣�
         * 2. End_f == 0 ��ʾ��δ����ֹͣ¼�ƺ����βд flash �׶Ρ�
         * ���� KEY3/KEY4 ������¼����β�����������Ԫ�����ͻ��㡣
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
            /* KEY4 ���ȼ���
             * 1. ¼���У��������浱ǰԪ�ص㣨Nag_Request_Event_Mark����
             * 2. ��¼����Ԫ�ؽӹ��У��ֶ�֪ͨԪ����ɣ��ָ�������
             * 3. ���������������һ���˵���
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
    * ����ӳ�䣨�����ָУ���KEY1 ͬ����һ�KEY2 ͬ����һ�KEY3 �����¼���KEY4 �����ϼ���
    * ��Ӧ vPtr��searchLeft / searchRight / searchDown / searchUp����Ϊ�˵������壬���� MENU_KEY_NAV_* ����������
    */
   while(MenuKeyEventPop(&nav))
   {
        switch(nav)
        {
        case MENU_KEY_NAV_LEFT:
             /* KEY4��������һ�����ṹ���� searchUp�� */
             hashMenu.vPtr->searchUp(&hashMenu, &menuMember);
             menu_nav = 1;
             break;
        case MENU_KEY_NAV_RIGHT:
             /* KEY3��������һ�����ṹ���� searchDown�� */
             hashMenu.vPtr->searchDown(&hashMenu, &menuMember);
             menu_nav = 1;
             break;
        case MENU_KEY_NAV_DOWN:
             /* KEY2��ͬ����һ��ṹ���� searchRight�� */
             hashMenu.vPtr->searchRight(&hashMenu, &menuMember);
             menu_nav = 1;
             break;
        case MENU_KEY_NAV_UP:
             /* KEY1��ͬ����һ��ṹ���� searchLeft�� */
             hashMenu.vPtr->searchLeft(&hashMenu, &menuMember);
             menu_nav = 1;
             break;
        default:
             break;
        }
   }
   /* �� selectMenu() һ�£������������ػ棬��������ѭ������ɣ������� ips200 SPI ��ͻ�� */
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
/* �������ٶ�����ҳ���� KEY1/2/3����Ҫ���� "3.1" �� Launch �����б�ҳ�� */
static uint8 MenuIsRunLaunchSpeedPage(void)
{
    return (uint8)(strcmp(menuMember.pos, "3.1.1") == 0);
}
/* Run/Flash ������û����ʵ�¼����ڸ�ҳ�� KEY3�����룩ʱ����Ϊ���� Run ������ flash�� */
static uint8 MenuIsRunFlashPage(void)
{
    return (uint8)(strcmp(menuMember.pos, "3.2") == 0);
}
/* Run/Config ����ҳ��KEY1 ѡ�ֶΣ�KEY2 �л�ȡֵ�� */
static uint8 MenuIsRunConfigPage(void)
{
    return (uint8)(strcmp(menuMember.pos, "3.3.1") == 0);
}
static uint8 MenuIsRunJumpPage(void)
{
    return (uint8)(strcmp(menuMember.pos, "3.4.1") == 0);
}
static void MenuAdjustRunLaunchParam(float delta)
{
#if defined(CY_CORE_CM7_1)
    (void)dualcore_ui_cmd_push(DUALCORE_UI_CMD_RUN_LAUNCH_PARAM_DELTA,
                               (uint32)s_run_launch_field_index, delta);
#else
    Nag_LaunchParamAdjust(s_run_launch_field_index, delta);
#endif
}
/* ���� 1 ��ʾ KEY1/2/3 �ѱ� Launch ҳ���ѣ�KEY4 ����ͨ�÷��ء� */
static uint8 MenuTryHandleRunLaunchSpeedKeyEvent(void)
{
    float step = 0.0f;
    if (key_get_state(KEY_1) == KEY_SHORT_PRESS)
    {
        s_run_launch_field_index =
            (uint8)((s_run_launch_field_index + 1u) % Nag_Run_Launch_Param_Count);
        gpio_toggle_level(LED1);
        key_clear_state(KEY_1);
        return 1u;
    }
    if (key_get_state(KEY_2) == KEY_SHORT_PRESS)
    {
        step = Nag_LaunchParamGetStep(s_run_launch_field_index);
        MenuAdjustRunLaunchParam(step);
        gpio_toggle_level(LED1);
        key_clear_state(KEY_2);
        return 1u;
    }
    if (key_get_state(KEY_3) == KEY_SHORT_PRESS)
    {
        step = -Nag_LaunchParamGetStep(s_run_launch_field_index);
        MenuAdjustRunLaunchParam(step);
        gpio_toggle_level(LED1);
        key_clear_state(KEY_3);
        return 1u;
    }
    return 0u;
}
static void MenuAdjustRunJumpParam(float delta)
{
#if defined(CY_CORE_CM7_1)
    (void)dualcore_ui_cmd_push(DUALCORE_UI_CMD_RUN_JUMP_PARAM_DELTA,
                               (uint32)s_run_jump_field_index, delta);
#else
    JumpParamAdjust(s_run_jump_field_index, delta);
#endif
}
/* ���� 1 ��ʾ KEY1/2/3 �ѱ� Jump ҳ���ѣ�KEY4 ����ͨ�÷��ء� */
static uint8 MenuTryHandleRunJumpKeyEvent(void)
{
    float step = JumpParamGetStep(s_run_jump_field_index);
    if (key_get_state(KEY_1) == KEY_SHORT_PRESS)
    {
        s_run_jump_field_index =
            (uint8)((s_run_jump_field_index + 1u) % Run_Jump_Param_Count);
        gpio_toggle_level(LED1);
        key_clear_state(KEY_1);
        return 1u;
    }
    if (key_get_state(KEY_2) == KEY_SHORT_PRESS)
    {
        MenuAdjustRunJumpParam(step);
        gpio_toggle_level(LED1);
        key_clear_state(KEY_2);
        return 1u;
    }
    if (key_get_state(KEY_3) == KEY_SHORT_PRESS)
    {
        MenuAdjustRunJumpParam(-step);
        gpio_toggle_level(LED1);
        key_clear_state(KEY_3);
        return 1u;
    }
    return 0u;
}
/* ���� 1 ��ʾ Run/Flash ҳ������ KEY3 ���涯����KEY1/KEY2/KEY4 ������ͨ�˵������� */
static uint8 MenuTryHandleRunFlashKeyEvent(void)
{
    if (key_get_state(KEY_3) == KEY_SHORT_PRESS)
    {
#if defined(CY_CORE_CM7_1)
        (void)dualcore_ui_cmd_push(DUALCORE_UI_CMD_RUN_LAUNCH_SPEED_SAVE_FLASH, 0, 0.0f);
#else
        flash_RunLaunchSpeed_Write();
        flash_JumpParams_Write();
#endif
        gpio_toggle_level(LED1);
        key_clear_state(KEY_3);
        return 1u;
    }
    return 0u;
}
/* ���� 1 ��ʾ Config ҳ������ KEY1/KEY2��KEY3 �޶�����KEY4 ����ͨ�÷��ء� */
static uint8 MenuTryHandleRunConfigKeyEvent(void)
{
    if (key_get_state(KEY_1) == KEY_SHORT_PRESS)
    {
        s_run_config_field_index =
            (uint8)((s_run_config_field_index + 1u) % Run_Config_Field_Count);
        gpio_toggle_level(LED1);
        key_clear_state(KEY_1);
        return 1u;
    }
    if (key_get_state(KEY_2) == KEY_SHORT_PRESS)
    {
#if defined(CY_CORE_CM7_1)
        (void)dualcore_ui_cmd_push(DUALCORE_UI_CMD_RUN_CONFIG_TOGGLE,
                                   (uint32)s_run_config_field_index, 0.0f);
#else
        Menu_RunConfigToggleField(s_run_config_field_index);
#endif
        gpio_toggle_level(LED1);
        key_clear_state(KEY_2);
        return 1u;
    }
    return 0u;
}
/* GPS ����ҳ�� GPS ����Ψһ������ڣ�
 * Idle��KEY1 ��ʼ��㣬KEY2 ������д flash��KEY3 �������� GPS ������GPS_ApplyLaunchSpeed ��¼��㾭γ�ȣ�
 *       ʻ��Լ GPS_NAV_GPS_FIRST_DISTANCE_M ���� RMC gnss.direction �� GPS_first �궨ƫ����׷�㣩��KEY4 �����ϼ���
 * Recording��KEY1 ռλ��KEY2 ������д flash��KEY3 ���浱ǰ�㣬KEY4 �л�������Ԫ�ء�
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
 * @Description: �˵�ѡ��������
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
    if (MenuIsRemoteMenuFirst() != 0u)
    {
#if defined(CY_CORE_CM7_1)
    if (dc_s.remote_local_keys_debug == 0u)
#else
    if (g_remote_local_keys_debug == 0u)
#endif
    {
    /* ȫң��ʱ Menu_command �ĵ��ֽڵ������������� selectMenu_Key һ�¡�
     * ע�⣺�ϵ� Balance_Car_initial��System_startup.c������Ϊ a/b=c/d / c=Down / d=Up��
     * ��˴���ͬ���Ĵ��ڽű�����λ��ǰ��ض��ձ����� selectMenu_Key�� */
    switch (Menu_command)
    {
    case 'a':
        /* ����ؼ�һ�£�ͬ����һ�� */
        hashMenu.vPtr->searchLeft(&hashMenu, &menuMember);
        ips200_clear();
        break;
    case 'b':
        /* ����ؼ�һ�£�ͬ����һ�� */
        hashMenu.vPtr->searchRight(&hashMenu, &menuMember);
        ips200_clear();
        break;
    case 'c':
        /* ����ؼ�һ�£������¼� */
        hashMenu.vPtr->searchDown(&hashMenu, &menuMember);
        ips200_clear();
        break;
    case 'd':
        /* ����ؼ�һ�£������ϼ� */
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
    case 't': /* �ߵ�¼�ƣ��������浱ǰԪ�ص� */
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
    }
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
 * @Description: �˵���ʼ��
 * @Param: Void
 * @Return: Void
 * @Example: MenuInit();
 * ע�⣺�������MENU_SELECT�����ϵ�Ĭ�ϲ˵�ѡ��ע��˳�ʼ������ŵ�Flash_Init()֮���ȶ�ȡflash�����ٶԲ˵����г�ʼ��
 */
void MenuInit()
{
    /*������ϣ��*/
    HashTableCtor(&hashMenu);
    /*�������ݣ������ϣ��*/
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
    menuMember.gui = GUI_3_4;
    menuMember.act = ACT_3_4;
    strcpy(menuMember.pos, "3.4");
    hashMenu.vPtr->insert(&hashMenu, &menuMember);
    menuMember.gui = GUI_3_1_1;
    menuMember.act = ACT_3_1_1;
    strcpy(menuMember.pos, "3.1.1");
    hashMenu.vPtr->insert(&hashMenu, &menuMember);
    menuMember.gui = GUI_3_3_1;
    menuMember.act = ACT_3_3_1;
    strcpy(menuMember.pos, "3.3.1");
    hashMenu.vPtr->insert(&hashMenu, &menuMember);
    menuMember.gui = GUI_3_4_1;
    menuMember.act = ACT_3_4_1;
    strcpy(menuMember.pos, "3.4.1");
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
        menuMember.gui = GUI_2_1_1_1;
        menuMember.act = ACT_2_1_1_1;
        strcpy(menuMember.pos, "2.1.1.1");
        hashMenu.vPtr->insert(&hashMenu, &menuMember);
        menuMember.gui = GUI_2_1_2_1;
        menuMember.act = ACT_2_1_2_1;
        strcpy(menuMember.pos, "2.1.2.1");
        hashMenu.vPtr->insert(&hashMenu, &menuMember);
        menuMember.gui = GUI_2_1_3_1;
        menuMember.act = ACT_2_1_3_1;
        strcpy(menuMember.pos, "2.1.3.1");
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
                            ������
----------------------------------------------------------------*/
/*----------------------------------------------------------------
 * @Function: HashTableCtor
 * @Description: Hash�����캯��
 * @Param: {HASH_TABLE_t*} const This    ��ϣ�����ṹ��
 * @Return: Void
 * @Example: HashTableCtor(&hashMenu);
 */
static void HashTableCtor(HASH_TABLE_t *const This)
{
    static HASH_VTBL_t vTable;
    MemSet(This, 0, sizeof(HASH_TABLE_t));
    vTable.insert = HashInsert;
    vTable.search = FindHashValue;
    /* �̶�Ϊ���˵������塹�󶨣��μ��ļ�ͷ���״��㡹��
     * ���� Down/Right �ȶԵ��ɡ��������桹������ HashPeer/ HashDepth ʵ�ּ��ο���������ȫ����ͻ */
    vTable.searchDown = HashDepthDown;
    vTable.searchUp = HashDepthUp;
    vTable.searchLeft = HashPeerLeft;
    vTable.searchRight = HashPeerRight;
    This->vPtr = &vTable;
}
/*----------------------------------------------------------------
 * @Function: CreatHashKey
 * @Description: ����HASH���ؼ���
 * @param {char*} skey
 * @Return: Void
 * @Example: hashKey = CreatHashKey(tempMember->pos);
 */
static uint16_t CreatHashKey(const char *skey)
{
    // ���ﴫ��ȥ�����ַ���
    char *p = (char *)skey; // ����ǿת
    uint16_t hashKey = 0;
    if (*p) // �ַ����ǿ�
    {
        for (; *p != '\0'; p++)
        {
            hashKey = (hashKey << 5) - hashKey + *p; // ���ɹؼ��֣�Key��
        }
    }
    return hashKey % (HASH_SIZE);
}
/*----------------------------------------------------------------
 * @Function: HashInsert
 * @Description: ��Ԫ�ز������
 * @Param: {HASH_TABLE_t*} const This
 * @Param: {MENU_MEMBER_t*} const tempMember
 * @Return: Void
 * @Example: HashInsert(&hashMenu, &menuMember);
 */
static uint8_t HashInsert(HASH_TABLE_t *const This, MENU_MEMBER_t *const tempMember)
{
    if (This->hashTableSize >= HASH_SIZE - 1) // ���������
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
 * @Description: ��һ��
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
        return HASH_ERROR; /*�����ڴ�Խ��*/
    }
    strcpy((char *)tempPos, tempMember->pos);
    ptrPos = tempPos;
    while (*(++ptrPos) != '\0') // ʹָ��ָ���ַ������"\0"
    {
    }
    *(ptrPos - 1) = '\0';
    *(ptrPos - 2) = '\0';
    flag = FindHashValue(This, tempMember, tempPos); // ���ҵ����²˵�д��tempMember��
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
 * @Description: ��һ��
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
        return HASH_ERROR; /*�����ڴ�Խ��*/
    }
    strcpy(tempPos, tempMember->pos);
    ptrPos = tempPos;
    while (*(++ptrPos) != '\0') // ʹָ��ָ���ַ������"\0"
    {
    }
    *ptrPos = '.';
    *(ptrPos + 1) = '1';
    *(ptrPos + 2) = '\0';
    flag = FindHashValue(This, tempMember, tempPos); // ���ҵ����²˵�д��tempMember��
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
 * @Description: ��ƽ��
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
    while (*(++ptrPos) != '\0') // ʹָ��ָ���ַ������"\0"
    {
    }
    temp = *(--ptrPos) - 1;
    *(ptrPos) = temp;                                // �����λ�õ���һ���ַ�ֵ-1������˵��"1.1.3"��Ϊ"1.1.2"
    flag = FindHashValue(This, tempMember, tempPos); // ���ҵ����²˵�д��tempMember��
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
 * @Description: ��ƽ��
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
    while (*(++ptrPos) != '\0') // ʹָ��ָ���ַ������"\0"
    {
    }
    temp = *(--ptrPos) + 1;
    *(ptrPos) = temp;                                // �����λ�õ���һ���ַ�ֵ-1������˵��"1.1.3"��Ϊ"1.1.2"
    flag = FindHashValue(This, tempMember, tempPos); // ���ҵ����²˵�д��tempMember��
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
 * @Description: �ɹؼ��ַ��ض�Ӧֵ
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
 * @Description: �����ڴ�ռ䲢��ʼ��
 * @param {void} *src
 * @param {int} value
 * @param {int} n
 * @Return: Void
 * @Example: MemSet(This, 0, sizeof(HASH_TABLE_t));
 */
static void *MemSet(void *src, int value, int n) /*��ʼ��*/
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
 * @Description: �����ڴ�ռ�
 * @param {void} *dest
 * @param {void} *src
 * @param {unsigned int} size
 * @Return: Void
 * @Example:     MemCpy(tempMember, &(This->hashTable[hashKey]),sizeof(MENU_MEMBER_t)); //���ҵ��Ĳ˵�д��tempMember
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
