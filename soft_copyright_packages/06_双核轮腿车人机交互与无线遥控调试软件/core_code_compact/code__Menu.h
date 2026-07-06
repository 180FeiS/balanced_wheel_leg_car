/*
 * @Author: bom
 * @Version: V1.0
 * @Date: 2024-12-12 22:02:51
 * @LastEditTime: 2024-12-24 21:38:53
 * @FilePath: \Balance_Car V3.0.1\project\code\Menu.h
 * @Description:
 */
/*********************************************************************************************************************
 * �޸ļ�¼
 * ����              ����             �汾           ˵��
 * 2024-07-24        Bron            V1.0.0         ��¹���
 * 2024-12-19        Bron            V2.0.1         ����Menu��Manu_Basic
 ********************************************************************************************************************/
#ifndef CODE_LOGIC_MENU_H_
#define CODE_LOGIC_MENU_H_
#include "zf_common_typedef.h"
#include "zf_common_headfile.h"
// #define MAX_EXPLAIN    10        /*���˵������*/
#define HASH_KEY_LEN 20            // ��ϣ���д洢��λ���ַ�������󳤶�
#define HASH_SIZE 50               // ��ϣ����С(���ɴ洢�Ĳ˵�������)
#define HASH_OK 1                  // ��ϣ�����ɹ�����ֵ
#define HASH_ERROR (HASH_SIZE + 2) // ��ϣ����ʧ�ܷ���ֵ
/*
 * �˵�����Դ��Flash ���䣬Run��Config �༭��Run��Save �־û�������Ч Flash ʱ�ñ���Ĭ�� 0����
 * 0 = ����+���룺���ؼ��� SWITCH1/2������ң�ؿ�����ģʽ�л���selectMenu ��ִ�� Menu_command �� switch�������밴��˫������
 * 1 = ң�����ȣ�ȫң�أ����ؼ��뱾�ز���Ĭ�ϲ����룩��CM7_0 �� dip_switch_motor_sync_from_hw Ĭ�ϲ��� SWITCH1/2��
 *    ����ң�ص� 4 ·�����ƽ��remote_lora.h��REMOTE_LORA_DEBUG_MODE_SWITCH_INDEX / REMOTE_LORA_LOCAL_KEYS_ACTIVE_LEVEL��
 *    �е������ص��ԡ����� 0 ���Ƶİ����뱾�ز���·����ȫң��ʱ Motor_Runaway_Latch ���ܿ���������������Թض����� control.c Ϊ׼��
 * ����ʱ���� g_menu_input_remote_first��CM7_0 ���У�CM7_1 �� dualcore ���ն�ȡ����
 */
#ifndef MENU_INPUT_REMOTE_MENU_FIRST
#define MENU_INPUT_REMOTE_MENU_FIRST 1
#endif
#define Run_Config_Field_InputMode 0u
#define Run_Config_Field_VofaEnable 1u
#define Run_Config_Field_VofaGroup 2u  /* VOFA ������ 0=IMU 1=�ٶ� 2=�ں� 3=��̾�ƫ��Flash V9 ���� */
#define Run_Config_Field_FusionEnable 3u /* �����ںϿ��� 0=�� 1=����Flash V10 ���� */
#define Run_Config_Field_Count 4u
extern uint8 g_menu_input_remote_first;
/*
 * VOFA ���ߵ��Կ��أ�Flash ���䣬Run��Config �༭��Run��Save �־û� V6����
 * 0 = �ر� JustFloat ���߷�����Ĭ�ϣ������� LORA ң�ع��� UART_1 ʱ������ţ���
 * 1 = ������CM7_1 ��ѭ�����˱�־���� cm71_vofa_main_loop_tx_dispatch()��
 * ����ʱ���� g_menu_vofa_enable��CM7_0 ���У�CM7_1 �� dualcore ���ն�ȡ����
 */
extern uint8 g_menu_vofa_enable;
/*
 * �����ںϿ��أ�Flash ���䣬Run��Config �༭��Run��Save �־û� V10����
 * 0 = �ر��ںϣ�GPS/�ߵ��˻�ԭ GNSS �� car_speed ���֣�1 = ���� nav_fusion��
 * ����ʱ���� g_menu_nav_fusion_enable��CM7_0 ���У�CM7_1 �� dualcore ���ն�ȡ����
 */
extern uint8 g_menu_nav_fusion_enable;
/*
 * ������ API Լ����hashMenu.vPtr->searchUp/Down/Left/Right ��ʾ�˵������������ǰ������������ҡ���
 * ������/�����ֽ���ζ�Ӧ�����ǣ�һ��ֻ�� Menu.c �� selectMenu_Key �� Menu_command ��֧�д�����
 * GPS ������ҳ KEY3 Idle �����μ� MenuTryHandleGpsDebugKeyEvent ע�ͣ�����¼ + GPS_NAV_GPS_FIRST_DISTANCE_M �� COG �궨����
 */
/*
 * �˵�λ�ñ���˵����
 * ʹ�õ�ָ�ʽ��ʾ�˵���Ĳ㼶��ϵ�����磺
 * "1"    - ��һ���˵��ĵ�1��ѡ��
 * "2.3"  - ��һ���˵��ĵ�2��ѡ���µĵ�3����ѡ��
 * "1.1.2"- ��һ���˵��ĵ�1��ѡ���µĵ�1����ѡ��ĵ�2����ѡ��
 */
// ǰ��������ϣ���麯�����ṹ
struct HASH_VTBL;
// �˵���ṹ��
typedef struct MENU_MEMBER
{
    char pos[HASH_KEY_LEN]; // �˵����λ�ñ���
    void (*gui)();          // ���ڸ��¸ò˵����GUI��ʾ
    void (*act)();          // ѡ�иò˵���ʱִ�еĲ���
} MENU_MEMBER_t;
// ��ϣ���ṹ��
typedef struct HASH_TABLE
{
    MENU_MEMBER_t hashTable[HASH_SIZE]; // �洢�˵���Ĺ�ϣ������
    struct HASH_VTBL *vPtr;             // ��ϣ�������������麯����
    int hashTableSize;                  // ��ǰ��ϣ���еĲ˵�������
} HASH_TABLE_t;
// ��ϣ�����������麯����
typedef struct HASH_VTBL
{
    // ��������
    uint8_t (*insert)(HASH_TABLE_t *const This, MENU_MEMBER_t *const tempMember);            // ���ϣ���в����²˵���
    uint8_t (*search)(HASH_TABLE_t *const This, MENU_MEMBER_t *const tempMember, char *str); // ����λ�ñ�����Ҳ˵���
    // �˵������������ṹ���壺��ο����� Balance_Car_initial Menu.c HashTableCtor һ�£�
    uint8_t (*searchLeft)(HASH_TABLE_t *const This, MENU_MEMBER_t *const tempMember);  // �л�����һ��ͬ��ѡ��
    uint8_t (*searchRight)(HASH_TABLE_t *const This, MENU_MEMBER_t *const tempMember); // �л�����һ��ͬ��ѡ��
    uint8_t (*searchUp)(HASH_TABLE_t *const This, MENU_MEMBER_t *const tempMember);    // �����ϼ��˵�
    uint8_t (*searchDown)(HASH_TABLE_t *const This, MENU_MEMBER_t *const tempMember); // ���뵱ǰѡ����Ӳ˵�
} HASH_VTBL_t;
// ȫ�ֱ�������ǰѡ�еĲ˵���
extern MENU_MEMBER_t menuMember;
extern HASH_TABLE_t hashMenu;
extern char ReadPos[HASH_KEY_LEN];
// �˵�ϵͳ��ʼ������
extern void MenuInit(void);
extern void selectMenu(void);
extern void selectMenu_Key(void);
extern void menu_key_capture_event(void);
extern void dip_switch_motor_sync_from_hw(void);
/* ���ڶ��ֽ���չ֡ V<��ֵ>���� ReadDataFromPc ��д�� Menu_command ǰ���ã������� 1 ��ʾ����֡���ѡ� */
extern uint8 Menu_TryConsumePcMotorSpeedString(const uint8 *data, uint32 count);
/* Run -> Launch ����ҳ��ǰѡ�еĲ����ֶ�������0..Nag_Run_Launch_Param_Count-1�� */
extern uint8 Menu_GetRunLaunchFieldIndex(void);
/* Run -> Config ����ҳ��ǰѡ�е�Ԥ�����ֶ�������0..Run_Config_Field_Count-1�� */
extern uint8 Menu_GetRunConfigFieldIndex(void);
extern void Menu_RunConfigToggleField(uint8 field_index);
/* Run -> Jump ����ҳ��ǰѡ�еĲ����ֶ�������0..Run_Jump_Param_Count-1�� */
extern uint8 Menu_GetRunJumpFieldIndex(void);
/* Debug -> Image��pos 2.1*�������ؽ���ʱ arm �Զ��ع�Ự */
extern uint8 MenuIsImageSectionPage(void);
extern void Menu_UpdateImageAeArm(void);
#endif /* CODE_LOGIC_MENU_H_ */
