#ifndef CODE_LOGIC_MENU_H_
#define CODE_LOGIC_MENU_H_
#include "zf_common_typedef.h"
#include "zf_common_headfile.h"
#define HASH_KEY_LEN 20
#define HASH_SIZE 50
#define HASH_OK 1
#define HASH_ERROR (HASH_SIZE + 2)
#ifndef MENU_INPUT_REMOTE_MENU_FIRST
#define MENU_INPUT_REMOTE_MENU_FIRST 1
#endif
#define Run_Config_Field_InputMode 0u
#define Run_Config_Field_VofaEnable 1u
#define Run_Config_Field_VofaGroup 2u
#define Run_Config_Field_FusionEnable 3u
#define Run_Config_Field_Count 4u
extern uint8 g_menu_input_remote_first;
extern uint8 g_menu_vofa_enable;
extern uint8 g_menu_nav_fusion_enable;
struct HASH_VTBL;
typedef struct MENU_MEMBER
{
    char pos[HASH_KEY_LEN];
    void (*gui)();
    void (*act)();
} MENU_MEMBER_t;
typedef struct HASH_TABLE
{
    MENU_MEMBER_t hashTable[HASH_SIZE];
    struct HASH_VTBL *vPtr;
    int hashTableSize;
} HASH_TABLE_t;
typedef struct HASH_VTBL
{
    uint8_t (*insert)(HASH_TABLE_t *const This, MENU_MEMBER_t *const tempMember);
    uint8_t (*search)(HASH_TABLE_t *const This, MENU_MEMBER_t *const tempMember, char *str);
    uint8_t (*searchLeft)(HASH_TABLE_t *const This, MENU_MEMBER_t *const tempMember);
    uint8_t (*searchRight)(HASH_TABLE_t *const This, MENU_MEMBER_t *const tempMember);
    uint8_t (*searchUp)(HASH_TABLE_t *const This, MENU_MEMBER_t *const tempMember);
    uint8_t (*searchDown)(HASH_TABLE_t *const This, MENU_MEMBER_t *const tempMember);
} HASH_VTBL_t;
extern MENU_MEMBER_t menuMember;
extern HASH_TABLE_t hashMenu;
extern char ReadPos[HASH_KEY_LEN];
extern void MenuInit(void);
extern void selectMenu(void);
extern void selectMenu_Key(void);
extern void menu_key_capture_event(void);
extern void dip_switch_motor_sync_from_hw(void);
extern uint8 Menu_TryConsumePcMotorSpeedString(const uint8 *data, uint32 count);
extern uint8 Menu_GetRunLaunchFieldIndex(void);
extern uint8 Menu_GetRunConfigFieldIndex(void);
extern void Menu_RunConfigToggleField(uint8 field_index);
extern uint8 Menu_GetRunJumpFieldIndex(void);
extern uint8 MenuIsImageSectionPage(void);
extern void Menu_UpdateImageAeArm(void);
#endif
