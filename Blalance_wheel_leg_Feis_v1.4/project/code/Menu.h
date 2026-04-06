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

    // 菜单导航操作
    uint8_t (*searchLeft)(HASH_TABLE_t *const This, MENU_MEMBER_t *const tempMember);  // 查找同级菜单中的上一个选项
    uint8_t (*searchRight)(HASH_TABLE_t *const This, MENU_MEMBER_t *const tempMember); // 进入当前选项的子菜单
    uint8_t (*searchUp)(HASH_TABLE_t *const This, MENU_MEMBER_t *const tempMember);    // 查找当前菜单中的上一个选项
    uint8_t (*searchDown)(HASH_TABLE_t *const This, MENU_MEMBER_t *const tempMember);  // 查找当前菜单中的下一个选项
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


#endif /* CODE_LOGIC_MENU_H_ */
