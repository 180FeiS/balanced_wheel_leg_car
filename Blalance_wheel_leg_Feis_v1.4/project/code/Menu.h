/*
 * @Author: bom
 * @Version: V1.0
 * @Date: 2024-12-12 22:02:51
 * @LastEditTime: 2024-12-24 21:38:53
 * @FilePath: \Balance_Car V3.0.1\project\code\Menu.h
 * @Description:
 */
/*********************************************************************************************************************
 * 修改记录
 * 日期              作者             版本           说明
 * 2024-07-24        Bron            V1.0.0         搭建新工程
 * 2024-12-19        Bron            V2.0.1         整合Menu和Manu_Basic
 ********************************************************************************************************************/

#ifndef CODE_LOGIC_MENU_H_
#define CODE_LOGIC_MENU_H_
#include "zf_common_typedef.h"
#include "zf_common_headfile.h"

// #define MAX_EXPLAIN    10        /*最大说明缓冲*/

#define HASH_KEY_LEN 20            // 哈希表中存储的位置字符串的最大长度
#define HASH_SIZE 50               // 哈希表大小(最多可存储的菜单项数量)
#define HASH_OK 1                  // 哈希操作成功返回值
#define HASH_ERROR (HASH_SIZE + 2) // 哈希操作失败返回值

/*
 * 菜单输入源（Flash 可配，Run→Config 编辑、Run→Save 持久化；无有效 Flash 时用编译默认 0）。
 * 0 = 按键+拨码：板载键与 SWITCH1/2；不读遥控快照做模式切换；selectMenu 不执行 Menu_command 的 switch，避免与按键双触发。
 * 1 = 遥控优先：全遥控（板载键与本地拨码默认不介入）；CM7_0 侧 dip_switch_motor_sync_from_hw 默认不读 SWITCH1/2；
 *    可用遥控第 4 路拨码电平（remote_lora.h：REMOTE_LORA_DEBUG_MODE_SWITCH_INDEX / REMOTE_LORA_LOCAL_KEYS_ACTIVE_LEVEL）
 *    切到「板载调试」：与 0 类似的按键与本地拨码路径；全遥控时 Motor_Runaway_Latch 不能靠拨码清除；保护性关断仍以 control.c 为准。
 * 运行时变量 g_menu_input_remote_first（CM7_0 持有，CM7_1 经 dualcore 快照读取）。
 */
#ifndef MENU_INPUT_REMOTE_MENU_FIRST
#define MENU_INPUT_REMOTE_MENU_FIRST 1
#endif

#define Run_Config_Field_InputMode 0u
#define Run_Config_Field_VofaEnable 1u
#define Run_Config_Field_VofaGroup 2u  /* VOFA 调试组 0=IMU 1=速度 2=融合 3=里程纠偏；Flash V9 可配 */
#define Run_Config_Field_FusionEnable 3u /* 导航融合开关 0=关 1=开；Flash V10 可配 */
#define Run_Config_Field_Count 4u

extern uint8 g_menu_input_remote_first;
/*
 * VOFA 无线调试开关（Flash 可配，Run→Config 编辑、Run→Save 持久化 V6）。
 * 0 = 关闭 JustFloat 无线发包（默认，避免与 LORA 遥控共用 UART_1 时互相干扰）。
 * 1 = 开启；CM7_1 主循环按此标志调用 cm71_vofa_main_loop_tx_dispatch()。
 * 运行时变量 g_menu_vofa_enable（CM7_0 持有，CM7_1 经 dualcore 快照读取）。
 */
extern uint8 g_menu_vofa_enable;
/*
 * 导航融合开关（Flash 可配，Run→Config 编辑、Run→Save 持久化 V10）。
 * 0 = 关闭融合，GPS/惯导退回原 GNSS 与 car_speed 积分；1 = 启用 nav_fusion。
 * 运行时变量 g_menu_nav_fusion_enable（CM7_0 持有，CM7_1 经 dualcore 快照读取）。
 */
extern uint8 g_menu_nav_fusion_enable;

/*
 * 【导航 API 约定】hashMenu.vPtr->searchUp/Down/Left/Right 表示菜单树操作，不是按键“上下左右”。
 * 物理键/串口字节如何对应到它们，一律只在 Menu.c 的 selectMenu_Key 与 Menu_command 分支中处理。
 * GPS 调试子页 KEY3 Idle 发车参见 MenuTryHandleGpsDebugKeyEvent 注释（起点记录 + GPS_NAV_GPS_FIRST_DISTANCE_M 后 COG 标定）。
 */

/*
 * 菜单位置编码说明：
 * 使用点分格式表示菜单项的层级关系，例如：
 * "1"    - 第一级菜单的第1个选项
 * "2.3"  - 第一级菜单的第2个选项下的第3个子选项
 * "1.1.2"- 第一级菜单的第1个选项下的第1个子选项的第2个子选项
 */

// 前向声明哈希表虚函数表结构
struct HASH_VTBL;

// 菜单项结构体
typedef struct MENU_MEMBER
{
    char pos[HASH_KEY_LEN]; // 菜单项的位置编码
    void (*gui)();          // 用于更新该菜单项的GUI显示
    void (*act)();          // 选中该菜单项时执行的操作
} MENU_MEMBER_t;

// 哈希表结构体
typedef struct HASH_TABLE
{
    MENU_MEMBER_t hashTable[HASH_SIZE]; // 存储菜单项的哈希表数组
    struct HASH_VTBL *vPtr;             // 哈希表操作函数的虚函数表
    int hashTableSize;                  // 当前哈希表中的菜单项数量
} HASH_TABLE_t;

// 哈希表操作函数虚函数表
typedef struct HASH_VTBL
{
    // 基础操作
    uint8_t (*insert)(HASH_TABLE_t *const This, MENU_MEMBER_t *const tempMember);            // 向哈希表中插入新菜单项
    uint8_t (*search)(HASH_TABLE_t *const This, MENU_MEMBER_t *const tempMember, char *str); // 根据位置编码查找菜单项

    // 菜单导航操作（结构语义：与参考工程 Balance_Car_initial Menu.c HashTableCtor 一致）
    uint8_t (*searchLeft)(HASH_TABLE_t *const This, MENU_MEMBER_t *const tempMember);  // 切换到上一个同级选项
    uint8_t (*searchRight)(HASH_TABLE_t *const This, MENU_MEMBER_t *const tempMember); // 切换到下一个同级选项
    uint8_t (*searchUp)(HASH_TABLE_t *const This, MENU_MEMBER_t *const tempMember);    // 返回上级菜单
    uint8_t (*searchDown)(HASH_TABLE_t *const This, MENU_MEMBER_t *const tempMember); // 进入当前选项的子菜单
} HASH_VTBL_t;

// 全局变量：当前选中的菜单项
extern MENU_MEMBER_t menuMember;
extern HASH_TABLE_t hashMenu;
extern char ReadPos[HASH_KEY_LEN];
// 菜单系统初始化函数
extern void MenuInit(void);
extern void selectMenu(void);
extern void selectMenu_Key(void);
extern void menu_key_capture_event(void);
extern void dip_switch_motor_sync_from_hw(void);

/* 串口多字节扩展帧 V<数值>（由 ReadDataFromPc 在写入 Menu_command 前调用）。返回 1 表示已整帧消费。 */
extern uint8 Menu_TryConsumePcMotorSpeedString(const uint8 *data, uint32 count);

/* Run -> Launch 三级页当前选中的参数字段索引（0..Nag_Run_Launch_Param_Count-1） */
extern uint8 Menu_GetRunLaunchFieldIndex(void);

/* Run -> Config 三级页当前选中的预配置字段索引（0..Run_Config_Field_Count-1） */
extern uint8 Menu_GetRunConfigFieldIndex(void);
extern void Menu_RunConfigToggleField(uint8 field_index);

/* Run -> Jump 三级页当前选中的参数字段索引（0..Run_Jump_Param_Count-1） */
extern uint8 Menu_GetRunJumpFieldIndex(void);

/* Debug -> Image（pos 2.1*）：边沿进入时 arm 自动曝光会话 */
extern uint8 MenuIsImageSectionPage(void);
extern void Menu_UpdateImageAeArm(void);

#endif /* CODE_LOGIC_MENU_H_ */
