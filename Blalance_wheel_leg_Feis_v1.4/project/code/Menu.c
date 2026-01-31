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
    - hashMenu.vPtr->searchUp(): 返回上级菜单
    - hashMenu.vPtr->searchDown(): 进入下级菜单
    - hashMenu.vPtr->searchLeft(): 切换到左侧同级菜单
    - hashMenu.vPtr->searchRight(): 切换到右侧同级菜单

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

/* 全局变量定义 */
HASH_TABLE_t hashMenu;    // 储存菜单项的哈希表
MENU_MEMBER_t menuMember; // 当前选中的菜单项信息
char ReadPos[HASH_KEY_LEN] = {'1', 0, 0, 0, 0};

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

void selectMenu_Key(void)
{
   key_scanner();
   if(key_get_state(KEY_1) == KEY_SHORT_PRESS)
   {
        hashMenu.vPtr->searchLeft(&hashMenu, &menuMember);
        ips200_clear();
        key_clear_state(KEY_1);
   }
   if(key_get_state(KEY_2) == KEY_SHORT_PRESS)
   {
        hashMenu.vPtr->searchRight(&hashMenu, &menuMember);
        ips200_clear();
        key_clear_state(KEY_2);
   }
   if(key_get_state(KEY_3) == KEY_SHORT_PRESS)
   {
        hashMenu.vPtr->searchDown(&hashMenu, &menuMember);
        ips200_clear();
        key_clear_state(KEY_3);
   }
   if(key_get_state(KEY_4) == KEY_SHORT_PRESS)
   {
        hashMenu.vPtr->searchUp(&hashMenu, &menuMember);
        ips200_clear();
        key_clear_state(KEY_4);
   }
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
    switch (Menu_command)
    {
    case 'a':
        hashMenu.vPtr->searchLeft(&hashMenu, &menuMember);
        ips200_clear();
        break;

    case 'b':
        hashMenu.vPtr->searchRight(&hashMenu, &menuMember);
        ips200_clear();
        break;

    case 'c':
        hashMenu.vPtr->searchDown(&hashMenu, &menuMember);
        ips200_clear();
        break;

    case 'd':
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
    case 'h':
       if(Motor_Switch == MOTOR_OFF)
        {
            Motor_Switch = MOTOR_ON;
        }
        else
        {
            Motor_Switch = MOTOR_OFF;
        }
        //Motor_OpenFlag = 1;
        break;
    case 'i':
        jump_flag = 1;
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
    hashMenu.vPtr->search(&hashMenu, &menuMember, "1");
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
    int count = HASH_SIZE, sLen, hashKey;
    hashKey = CreatHashKey(str);
    sLen = strlen(str);
    sLen = (sLen > HASH_KEY_LEN) ? HASH_KEY_LEN : sLen;
    while ((strncmp(This->hashTable[hashKey].pos, str, sLen) != 0) && count--)
    {
        hashKey = (hashKey + 1) % HASH_SIZE; // 开放地址法则线性探测
    }
    if (count < 0)
    {
        return HASH_ERROR; /*遍历完这个哈希表都没有找到*/
    }
    MemCpy(tempMember, &(This->hashTable[hashKey]), sizeof(MENU_MEMBER_t)); // 将找到的菜单写入tempMember
    return HASH_OK;
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
