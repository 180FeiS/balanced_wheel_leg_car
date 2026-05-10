/*
 * @Author: bom
 * @Version: V1.0
 * @Date: 2024-12-18 17:23:09
 * @LastEditTime: 2025-03-14 16:07:09
 * @FilePath: \Balance_Car V3.0.1\project\code\UI.c
 * @Description: 
 */
/*********************************************************************************************************************
* @ile            UI.c
 @author          bom
 @version         V1.0
 @date            2024-12-18
 @brief           平衡�?UI控制程序

 @note
 1. �?文件包含平衡车所有UI显示和控制功�?
 2. �?持LCD显示、串口调试和参数配置
 3. 包含完整的�?�级菜单系统
********************************************************************************************************************/

/*********************************************************************************************************************
* �?改�?�录
* 日期              作��?             版本           说明
* 2024-07-24        Bron            V1.0.0         �?建新工程
* 2024-07-27        Bron            V1.0.2         �?建了二级菜单的�?�架
* 2024-07-30        Bron            V1.1.1         更换了x和y的表示方式，完善了级菜单
* 2024-07-31        Bron            V1.1.2         更新了pid调参菜单方式，加�?flash
* 2024-08-01        Bron            V1.1.4         �?调了pid参数整数小数位数显示
* 2024-08-02        Bron            V1.2.0         将重复显示的内�?��?�整到函数中
* 2024-12-19        Bron            V2.0.1         移�?�到新工程，重新整理菜单
********************************************************************************************************************/
#include "zf_common_headfile.h"
#include "image.h"
#if defined(CY_CORE_CM7_1)
#include "dualcore_shared.h"
static dualcore_ctrl_to_ui_t s_ui_dc;
#else
#include "control.h"
#endif

/* Run：须�? pos 3.1~3.3 �? menuMember �?头�?�一致，否则 HashPeer 切项时画�?与真�? pos 不同步��勿�? pos�?3」上用�?�表�? */
static void GUI_Run_ShowSubmenuList(uint8 selected_row_index);

void ui_pull_ctrl_snapshot(void)
{
#if defined(CY_CORE_CM7_1)
    dualcore_ctrl_to_ui_pull(&s_ui_dc);
#else
#endif
}

/*********************************************************************************************************************
* 多级菜单框架
*********************************************************************************************************************
* 1. 测试外�?�模�? (GUI_1)
*    - 1_1. 测试电机
*      - 1_1_1. 电机详细信息
*    - 1_2. 测试编码�?
*      - 1_2_1. 编码器�?�细信息
*    - 1_3. 测试摄像�?
*      - 1_3_1. 摄像头�?�细信息
*    - 1_4. 测试院�螺仪
*      - 1_4_1. 院�螺仪详细信息
*
* 2. 调试模式 (GUI_2)
*    - 2_1. 图像设置
*      - 2_1_1. 台阶棢��?
*      - 2_1_2. 单边桥�?���?
*      - 2_1_3. 颠簸�?段�?���?
*    - 2_2. 速度�?设置
*      - 2_2_1. 角��度环P
*      - 2_2_2. 角��度环I
*      - 2_2_3. 角度环P
*      - 2_2_4. 角��度环D
*      - 2_2_5. 速度环P
*      - 2_2_6. 速度环D
*    - 2_3. �?向环设置
*      - 2_3_1. �?向内环P
*      - 2_3_2. �?向内环D
*      - 2_3_3. �?向�?�环P
*      - 2_3_4. �?向�?�环D
*    - 2_4. 速度设置
*    - 2_5. 更新Flash参数
*    - 2_6. 清空FLASH缓存�?
*
* 3. 运�?�模�? —��? GUI 必须�? pos 层级丢�致（易错：一级��?3」与二级�?3.1」勿画同丢�块子菜单）：
*    - GUI_3 �? pos �?3」：顶层 Test/Debug/Run 三�?�中�? GUI_1/2 同源的一�? Run�?
*    - GUI_3_1～GUI_3_3 �? pos �?3.1�?3.3」：二级列表 Launch/Flash/More，三页应用同丢�文本、不同�??头�?�，配合 HashPeer 切换�?
*    - GUI_3_1_1 �? pos �?3.1.1」：Launch 下三级发车��度壳；发车 UI/逻辑不应塞在 GUI_3_1，否则会破坏「��中再进入��的树��?
*********************************************************************************************************************/

/*********************************************************************************************************************
* 显示模式定义
*********************************************************************************************************************/
typedef enum {
    DISPLAY_MODE_IPS200,   // 液晶模式
    DISPLAY_MODE_SERIAL  // 串口模式
} DisplayMode;

DisplayMode currentDisplayMode = DISPLAY_MODE_IPS200; // 默�?�显示模式为液晶模式
float FPS = 0;
/*内部函数声明*/
static void GUI_Display_FPS(void)__attribute__((unused));
static void GUI_Display_Image_Sidebar()__attribute__((unused));
static void GUI_Display_Image_Below(void)__attribute__((unused));
static void GUI_SetAngleLoop()__attribute__((unused));
static void GUI_SetDirLoop()__attribute__((unused));
static void GUI_Display_Level1_Common1(void)__attribute__((unused));
static void GUI_Display_Level2_Common1(void)__attribute__((unused));
static void GUI_Display_Level2_Common2(void)__attribute__((unused));
static void GUI_Display_Level2_Common3(void)__attribute__((unused));
static void GUI_Display_Level3_Common1(void)__attribute__((unused));
static void GUI_Display_Level3_Common2(void)__attribute__((unused));
static void GUI_Display_Level3_ImageDetect(uint8 current_idx)__attribute__((unused));
static void GUI_Display_Level3_Common3(void)__attribute__((unused));
static void GUI_Display_Level3_Common4(void)__attribute__((unused));
static void GUI_Display_Level3_Common5(void)__attribute__((unused));
/*********************************************************************************************************************
* 丢�级菜单函�?
*********************************************************************************************************************/
static void GUI_Display_FPS(void)
{
    ips200_show_string(184,ROW_1,"FPS:");
    //FPS = 1000000 / Scheduler.Tasks[1].Stats.LastExecuteTime;
    ips200_show_int(208,ROW_1,(uint16_t)FPS,3);
}

static void GUI_Display_Level1_Common1(void)
{
    ips200_draw_line(0,20,239,20,IPS200_DEFAULT_PENCOLOR);
    ips200_show_string(0,ROW_1,"Meun ");

    GUI_Display_FPS();
}

void GUI_1(void) // 测试外�?�模�?
{
    GUI_Display_Level1_Common1();

    ips200_show_string(80,ROW_7," Test");//80
    ips200_show_string(80,ROW_10," Debug");
    ips200_show_string(80,ROW_13," Run ");

    
    ips200_show_string(48,ROW_7,"-->");//8*6
    ips200_show_string(136,ROW_7,"<--");//8*6
}
void ACT_1()
{
    ReadPos[0] = '1';
    ReadPos[1] = 0x00;
    ReadPos[2] = 0x00;
    ReadPos[3] = 0x00;
    ReadPos[4] = 0x00;
}

void GUI_2(void) // 调试模式
{
    GUI_Display_Level1_Common1();

    ips200_show_string(80,ROW_7," Test");//80
    ips200_show_string(80,ROW_10," Debug");
    ips200_show_string(80,ROW_13," Run ");

    ips200_show_string(48,ROW_10,"-->");//8*6
    ips200_show_string(136,ROW_10,"<--");//8*6

}
void ACT_2()
{
    ReadPos[0] = '2';
    ReadPos[1] = 0x00;
    ReadPos[2] = 0x00;
    ReadPos[3] = 0x00;
    ReadPos[4] = 0x00;
}

void GUI_3(void) // pos�?3」：丢��? Run（与 GUI_1/2 同源）；子列表仅属于 pos 3.1�?3.3
{
    GUI_Display_Level1_Common1();

    ips200_show_string(80,ROW_7," Test");//80
    ips200_show_string(80,ROW_10," Debug");
    ips200_show_string(80,ROW_13," Run ");

    ips200_show_string(48,ROW_13,"-->");//8*6
    ips200_show_string(136,ROW_13,"<--");//8*6

}
void ACT_3()
{
    ReadPos[0] = '3';
    ReadPos[1] = 0x00;
    ReadPos[2] = 0x00;
    ReadPos[3] = 0x00;
    ReadPos[4] = 0x00;
}

/*********************************************************************************************************************
* 二级菜单函数
*********************************************************************************************************************/
static void GUI_Display_Level2_Common1(void)
{
    ips200_draw_line(0,20,239,20,IPS200_DEFAULT_PENCOLOR);
    ips200_show_string(0,ROW_1,"Test ");

    GUI_Display_FPS();
}

void GUI_1_1(void) // 测试电机
{
    GUI_Display_Level2_Common1();

    ips200_show_string(80,ROW_6,"Motor");//80
    ips200_show_string(80,ROW_8,"Encode");
    ips200_show_string(80,ROW_10,"Image");
    ips200_show_string(80,ROW_12,"Gyro");
    

    ips200_show_string(48,ROW_6,"-->");//8*6
    ips200_show_string(136,ROW_6,"<--");//8*6
    // 实现测试电机的��辑
}
void ACT_1_1()
{
    ReadPos[0] = '1';
    ReadPos[1] = '.';
    ReadPos[2] = '1';
    ReadPos[3] = 0x00;
    ReadPos[4] = 0x00;
}

void GUI_1_2(void) // 测试编码�?
{
    // 实现测试编码器的逻辑
    GUI_Display_Level2_Common1();

    ips200_show_string(80,ROW_6,"Motor");//80
    ips200_show_string(80,ROW_8,"Encode");
    ips200_show_string(80,ROW_10,"Image");
    ips200_show_string(80,ROW_12,"Gyro");
    

    ips200_show_string(48,ROW_8,"-->");//8*6
    ips200_show_string(136,ROW_8,"<--");//8*6
}
void ACT_1_2()
{
    ReadPos[0] = '1';
    ReadPos[1] = '.';
    ReadPos[2] = '2';
    ReadPos[3] = 0x00;
    ReadPos[4] = 0x00;      
}

void GUI_1_3(void) // 测试摄像�?
{
    // 实现测试摄像头的逻辑
    GUI_Display_Level2_Common1();

    ips200_show_string(80,ROW_6,"Motor");//80
    ips200_show_string(80,ROW_8,"Encode");
    ips200_show_string(80,ROW_10,"Image");
    ips200_show_string(80,ROW_12,"Gyro");
    

    ips200_show_string(48,ROW_10,"-->");//8*6
    ips200_show_string(136,ROW_10,"<--");//8*6
}
void ACT_1_3()
{
    ReadPos[0] = '1';
    ReadPos[1] = '.';
    ReadPos[2] = '3';
    ReadPos[3] = 0x00;
    ReadPos[4] = 0x00;
}

void GUI_1_4(void) // 测试院�螺仪
{
    // 实现测试院�螺仪的��辑
    GUI_Display_Level2_Common1();

    ips200_show_string(80,ROW_6,"Motor");//80
    ips200_show_string(80,ROW_8,"Encode");
    ips200_show_string(80,ROW_10,"Image");
    ips200_show_string(80,ROW_12,"Gyro");
    

    ips200_show_string(48,ROW_12,"-->");//8*6
    ips200_show_string(136,ROW_12,"<--");//8*6
}
void ACT_1_4()
{
    ReadPos[0] = '1';
    ReadPos[1] = '.';
    ReadPos[2] = '4';   
    ReadPos[3] = 0x00;
    ReadPos[4] = 0x00;
}

// /*********************************************************************************************************************
// * 调试模式选择界面
// *********************************************************************************************************************/
static void GUI_Display_Level2_Common2(void)
{
    ips200_draw_line(0,20,239,20,IPS200_DEFAULT_PENCOLOR);
    ips200_show_string(0,ROW_1,"Debug");

    GUI_Display_FPS();
}

void GUI_2_1(void) // 图像设置
{
    GUI_Display_Level2_Common2();
    
    ips200_show_string(80,ROW_6," Image  ");
    ips200_show_string(80,ROW_8," NavDbg ");
    ips200_show_string(80,ROW_10," GPS    ");
    ips200_show_string(80,ROW_12," Speed  ");
    ips200_show_string(80,ROW_14,"W_Flash");
    ips200_show_string(80,ROW_16,"C_Flash ");
    

    ips200_show_string(48,ROW_6,"-->");
    ips200_show_string(152,ROW_6,"<--");
}
void ACT_2_1() 
{
    ReadPos[0] = '2';
    ReadPos[1] = '.';
    ReadPos[2] = '1';
    ReadPos[3] = 0x00;
    ReadPos[4] = 0x00;
}


void GUI_2_2(void) // 调试列表：NavDbg 行��中（与其它项一致为列表态；按��右」进�? GUI_2_2_1�?
{
    GUI_Display_Level2_Common2();

    ips200_show_string(80,ROW_6," Image  ");
    ips200_show_string(80,ROW_8," NavDbg ");
    ips200_show_string(80,ROW_10," GPS    ");
    ips200_show_string(80,ROW_12," Speed  ");
    ips200_show_string(80,ROW_14,"W_Flash");
    ips200_show_string(80,ROW_16,"C_Flash ");

    ips200_show_string(48,ROW_8,"-->");
    ips200_show_string(152,ROW_8,"<--");
}
void ACT_2_2()
{
    ReadPos[0] = '2';
    ReadPos[1] = '.';
    ReadPos[2] = '2';
    ReadPos[3] = 0x00;
    ReadPos[4] = 0x00;
}

void GUI_2_2_1(void) // �?导调试界�?（须�? 2.2 按��右」进入）
{
    GUI_Display_Level2_Common2();

    ips200_show_string(64,ROW_3,"Nav Debug");
    ips200_draw_line(24,ROW_5-1,215,ROW_5-1,IPS200_DEFAULT_PENCOLOR);

    ips200_show_string(24,ROW_5,"KEY1: Record");
    ips200_show_string(24,ROW_6,"KEY2: Replay");
    ips200_show_string(24,ROW_7,"KEY3: Stop");
    ips200_show_string(24,ROW_8,"KEY4: Return");

    ips200_draw_line(24,ROW_9 ,215,ROW_9 ,IPS200_DEFAULT_PENCOLOR);

    ips200_show_string(24,ROW_10,"Mileage:");
#if defined(CY_CORE_CM7_1)
    ips200_show_float(104,ROW_10,s_ui_dc.mileage_debug_total,4,2);

    ips200_show_string(24,ROW_11,"SaveIdx:");
    ips200_show_uint(104,ROW_11,(uint32)s_ui_dc.save_index,5);

    ips200_show_string(24,ROW_12,"FlashPg:");
    ips200_show_uint(104,ROW_12,(uint32)s_ui_dc.flash_page_index,3);
#else
    ips200_show_float(104,ROW_10,N.Mileage_Debug_Total,4,2);

    ips200_show_string(24,ROW_11,"SaveIdx:");
    ips200_show_uint(104,ROW_11,N.Save_index,5);

    ips200_show_string(24,ROW_12,"FlashPg:");
    ips200_show_uint(104,ROW_12,N.Flash_page_index,3);
#endif

    ips200_show_string(24,ROW_14,"Rec  / Replay / Stop");
    ips200_show_string(24,ROW_15,"LED1 toggles on press");
}
void ACT_2_2_1()
{
    ReadPos[0] = '2';
    ReadPos[1] = '.';
    ReadPos[2] = '2';
    ReadPos[3] = '.';
    ReadPos[4] = '1';
    ReadPos[5] = 0x00;
}


void GUI_2_3(void) // �?向环设置
{
    GUI_Display_Level2_Common2();
    
    ips200_show_string(80,ROW_6," Image  ");
    ips200_show_string(80,ROW_8," NavDbg ");
    ips200_show_string(80,ROW_10," GPS    ");
    ips200_show_string(80,ROW_12," Speed  ");
    ips200_show_string(80,ROW_14,"W_Flash");
    ips200_show_string(80,ROW_16,"C_Flash ");
    

    ips200_show_string(48,ROW_10,"-->");
    ips200_show_string(152,ROW_10,"<--");
}
void ACT_2_3()
{
    ReadPos[0] = '2';
    ReadPos[1] = '.';
    ReadPos[2] = '3';
    ReadPos[3] = 0x00;
    ReadPos[4] = 0x00;
}

void GUI_2_3_1(void)
{
    GUI_Display_Level2_Common2();

    ips200_show_string(64,ROW_3,"GPS Data");
    ips200_draw_line(24,ROW_5-1,215,ROW_5-1,IPS200_DEFAULT_PENCOLOR);

#if defined(CY_CORE_CM7_1)
    ips200_show_string(0,ROW_5,"year:");
    ips200_show_uint(40,ROW_5,(uint32)s_ui_dc.gps_year,4);
    ips200_show_string(80,ROW_5,"mon:");
    ips200_show_uint(112,ROW_5,(uint32)s_ui_dc.gps_month,2);
    ips200_show_string(144,ROW_5,"day:");
    ips200_show_uint(176,ROW_5,(uint32)s_ui_dc.gps_day,2);

    ips200_show_string(0,ROW_6,"time:");
    ips200_show_uint(40,ROW_6,(uint32)s_ui_dc.gps_hour,2);
    ips200_show_string(56,ROW_6,":");
    ips200_show_uint(64,ROW_6,(uint32)s_ui_dc.gps_minute,2);
    ips200_show_string(80,ROW_6,":");
    ips200_show_uint(88,ROW_6,(uint32)s_ui_dc.gps_second,2);

    ips200_show_string(0,ROW_7,"state:");
    ips200_show_uint(48,ROW_7,(uint32)s_ui_dc.gps_state,3);
    ips200_show_string(104,ROW_7,"sat:");
    ips200_show_uint(136,ROW_7,(uint32)s_ui_dc.gps_satellite_used,3);

    ips200_show_string(0,ROW_8,"lat:");
    ips200_show_float(40,ROW_8,s_ui_dc.gps_latitude,4,6);
    ips200_show_string(0,ROW_9,"lon:");
    ips200_show_float(40,ROW_9,s_ui_dc.gps_longitude,4,6);
    ips200_show_string(0,ROW_10,"spd:");
    ips200_show_float(40,ROW_10,s_ui_dc.gps_speed,4,2);
    ips200_show_string(112,ROW_10,"dir:");
    ips200_show_float(152,ROW_10,s_ui_dc.gps_direction,4,2);
    ips200_show_string(0,ROW_11,"h:");
    ips200_show_float(40,ROW_11,s_ui_dc.gps_height,4,2);
    ips200_show_string(112,ROW_11,"valid:");
    ips200_show_uint(160,ROW_11,(uint32)s_ui_dc.gps_valid,1);
#else
    ips200_show_string(0,ROW_5,"year:");
    ips200_show_uint(40,ROW_5,(uint32)gnss.time.year,4);
    ips200_show_string(80,ROW_5,"mon:");
    ips200_show_uint(112,ROW_5,(uint32)gnss.time.month,2);
    ips200_show_string(144,ROW_5,"day:");
    ips200_show_uint(176,ROW_5,(uint32)gnss.time.day,2);

    ips200_show_string(0,ROW_6,"time:");
    ips200_show_uint(40,ROW_6,(uint32)gnss.time.hour,2);
    ips200_show_string(56,ROW_6,":");
    ips200_show_uint(64,ROW_6,(uint32)gnss.time.minute,2);
    ips200_show_string(80,ROW_6,":");
    ips200_show_uint(88,ROW_6,(uint32)gnss.time.second,2);

    ips200_show_string(0,ROW_7,"state:");
    ips200_show_uint(48,ROW_7,(uint32)gnss.state,3);
    ips200_show_string(104,ROW_7,"sat:");
    ips200_show_uint(136,ROW_7,(uint32)gnss.satellite_used,3);

    ips200_show_string(0,ROW_8,"lat:");
    ips200_show_float(40,ROW_8,gnss.latitude,4,6);
    ips200_show_string(0,ROW_9,"lon:");
    ips200_show_float(40,ROW_9,gnss.longitude,4,6);
    ips200_show_string(0,ROW_10,"spd:");
    ips200_show_float(40,ROW_10,gnss.speed,4,2);
    ips200_show_string(112,ROW_10,"dir:");
    ips200_show_float(152,ROW_10,gnss.direction,4,2);
    ips200_show_string(0,ROW_11,"h:");
    ips200_show_float(40,ROW_11,gnss.height,4,2);
#endif
}
void ACT_2_3_1()
{
    ReadPos[0] = '2';
    ReadPos[1] = '.';
    ReadPos[2] = '3';
    ReadPos[3] = '.';
    ReadPos[4] = '1';
    ReadPos[5] = 0x00;
}

void GUI_2_4(void) // 速度设置
{
    GUI_Display_Level2_Common2();
    
    ips200_show_string(80,ROW_6," Image  ");
    ips200_show_string(80,ROW_8," NavDbg ");
    ips200_show_string(80,ROW_10," GPS    ");
    ips200_show_string(80,ROW_12," Speed  ");
    ips200_show_string(80,ROW_14,"W_Flash");
    ips200_show_string(80,ROW_16,"C_Flash ");
    

    ips200_show_string(48,ROW_12,"-->");
    ips200_show_string(152,ROW_12,"<--");
}
void ACT_2_4()
{
    ReadPos[0] = '2';
    ReadPos[1] = '.';
    ReadPos[2] = '4';
    ReadPos[3] = 0x00;
    ReadPos[4] = 0x00;
}

void GUI_2_5(void) // 更新Flash参数
{
    GUI_Display_Level2_Common2();
    
    ips200_show_string(80,ROW_6," Image  ");
    ips200_show_string(80,ROW_8," NavDbg ");
    ips200_show_string(80,ROW_10," GPS    ");
    ips200_show_string(80,ROW_12," Speed  ");
    ips200_show_string(80,ROW_14,"W_Flash");
    ips200_show_string(80,ROW_16,"C_Flash ");
    

    ips200_show_string(48,ROW_14,"-->");
    ips200_show_string(152,ROW_14,"<--");
}
void ACT_2_5()
{
    ReadPos[0] = '2';
    ReadPos[1] = '.';
    ReadPos[2] = '5';
    ReadPos[3] = 0x00;
    ReadPos[4] = 0x00;
#if FLASH_MODE
    if(Flash.Flash_Error == FLASH_RUNNING && Flash.Flash_state == FLASH_WRITE)
    {
        while(Flash_Write());
        Flash.Flash_Error = FLASH_STOP;
        Flash.Flash_state = FLASH_WAIT;
        ips200_show_string(88,ROW_1,"Flash_OK!");
    }
#endif
}

void GUI_2_6(void) // 清空FLASH缓存�?
{
    GUI_Display_Level2_Common2();
    
    ips200_show_string(80,ROW_6," Image  ");
    ips200_show_string(80,ROW_8," NavDbg ");
    ips200_show_string(80,ROW_10," GPS    ");
    ips200_show_string(80,ROW_12," Speed  ");
    ips200_show_string(80,ROW_14,"W_Flash");
    ips200_show_string(80,ROW_16,"C_Flash ");
    

    ips200_show_string(48,ROW_16,"-->");
    ips200_show_string(152,ROW_16,"<--");
}
void ACT_2_6()
{
    ReadPos[0] = '2';
    ReadPos[1] = '.';
    ReadPos[2] = '6';
    ReadPos[3] = 0x00;
    ReadPos[4] = 0x00;
#if FLASH_MODE
    if(Flash.Flash_Error == FLASH_RUNNING && Flash.Flash_state == FLASH_CLEAR)
    {
        while(Flash_Clear());
        Flash.Flash_Error = FLASH_STOP;
        Flash.Flash_state = FLASH_WAIT;
        ips200_show_string(88,ROW_1,"CFlash_OK!");
    }
#endif
}
// *********************************************************************************************************************
// * Run 二级/三级：Common3 顶栏 + 列表或发车页。一�? GUI_3 不得复用下列列表，否则与 menu 树错位��?
// *********************************************************************************************************************
static void GUI_Display_Level2_Common3(void)
{
    ips200_draw_line(0,20,239,20,IPS200_DEFAULT_PENCOLOR);
    ips200_show_string(0,ROW_1,"Run");

    GUI_Display_FPS();
}

/** 二级 Run 列表：与 GUI_2_1 版式对齐；selected_row_index 必须与当�? pos �?�? 1/2/3 丢��? */
static void GUI_Run_ShowSubmenuList(uint8 selected_row_index)
{
    int16 ay;

    GUI_Display_Level2_Common3();

    ips200_show_string(80, ROW_8, " Launch ");
    ips200_show_string(80, ROW_10, " SaveSpd ");
    ips200_show_string(80, ROW_12, " More   ");

    switch (selected_row_index)
    {
    default:
        ay = ROW_8;
        break;
    case 1u:
        ay = ROW_10;
        break;
    case 2u:
        ay = ROW_12;
        break;
    }
    ips200_show_string(48, ay, "-->");
    ips200_show_string(152, ay, "<--");
    if (selected_row_index == 1u)
    {
        ips200_show_string(24, ROW_15, "KEY3: save speed");
    }
}

void GUI_3_1(void) // Launch 二级项：列表�?头�??丢��?
{
    GUI_Run_ShowSubmenuList(0u);
}
void ACT_3_1()
{
    ReadPos[0] = '3';
    ReadPos[1] = '.';
    ReadPos[2] = '1';
    ReadPos[3] = 0x00;
    ReadPos[4] = 0x00;
}

void GUI_3_2(void)
{
    GUI_Run_ShowSubmenuList(1u);
}
void ACT_3_2()
{
    ReadPos[0] = '3';
    ReadPos[1] = '.';
    ReadPos[2] = '2';
    ReadPos[3] = 0x00;
    ReadPos[4] = 0x00;
}

void GUI_3_3(void)
{
    GUI_Run_ShowSubmenuList(2u);
}
void ACT_3_3()
{
    ReadPos[0] = '3';
    ReadPos[1] = '.';
    ReadPos[2] = '3';
    ReadPos[3] = 0x00;
    ReadPos[4] = 0x00;
}

void GUI_3_1_1(void) // 发车速度三级页：KEY1 �? 0/500/1000，KEY2/3 �?�? run_launch_speed
{
    GUI_Display_Level2_Common3();

    ips200_show_string(56, ROW_3, "Launch Spd");
    ips200_draw_line(24, ROW_5 - 1, 215, ROW_5 - 1, IPS200_DEFAULT_PENCOLOR);

    ips200_show_string(24, ROW_5, "KEY1: preset next");
    ips200_show_string(24, ROW_6, "KEY2/3: -/+100");
    ips200_show_string(24, ROW_7, "KEY4: back");

    ips200_draw_line(24, ROW_9, 215, ROW_9, IPS200_DEFAULT_PENCOLOR);

    ips200_show_string(24, ROW_10, "Launch:");
#if defined(CY_CORE_CM7_1)
    ips200_show_float(96, ROW_10, (double)s_ui_dc.run_launch_speed, 5, 1);
#else
    ips200_show_float(96, ROW_10, (double)run_launch_speed, 5, 1);
#endif

    ips200_show_string(24, ROW_14, "Preset: 0/500/1000");
    ips200_show_string(24, ROW_15, "LED1 blink on key Short");
}

void ACT_3_1_1()
{
    ReadPos[0] = '3';
    ReadPos[1] = '.';
    ReadPos[2] = '1';
    ReadPos[3] = '.';
    ReadPos[4] = '1';
}
/*********************************************************************************************************************
* 三级菜单函数
*********************************************************************************************************************/

/*********************************************************************************************************************
* 图像界面
*********************************************************************************************************************/
static void GUI_Display_Level3_Common1(void)
{
    ips200_draw_line(0,20,239,20,IPS200_DEFAULT_PENCOLOR);

    GUI_Display_FPS();
}

void GUI_1_1_1(void) // 测试电机
{
    GUI_Display_Level3_Common1();
    ips200_show_string(0,ROW_1,"Motor ");

#if defined(CY_CORE_CM7_1)
    if(s_ui_dc.motor_switch == MOTOR_ON)ips200_show_string(24,ROW_3,"Motor:On");
    else ips200_show_string(24,ROW_3,"Motor:Off");

    ips200_show_string(24,ROW_4,"L_PWM:");  ips200_show_int(88,ROW_4,(int)s_ui_dc.left_motor_pwm,5);
    ips200_show_string(24,ROW_5,"R_PWM:");  ips200_show_int(88,ROW_5,(int)s_ui_dc.right_motor_pwm,5);
#else
    if(Motor_Switch == MOTOR_ON)ips200_show_string(24,ROW_3,"Motor:On");
    else ips200_show_string(24,ROW_3,"Motor:Off");

    ips200_show_string(24,ROW_4,"L_PWM:");  ips200_show_int(88,ROW_4,Left_Motor_Pwm,5);
    ips200_show_string(24,ROW_5,"R_PWM:");  ips200_show_int(88,ROW_5,Right_Motor_Pwm,5);
#endif
  




}
void ACT_1_1_1()
{
    ReadPos[0] = '1';
    ReadPos[1] = '.';
    ReadPos[2] = '1';
    ReadPos[3] = '.';
    ReadPos[4] = '1';   
}

void GUI_1_2_1(void)
{
    GUI_Display_Level3_Common1();
    ips200_show_string(0,ROW_1,"Encode");
#if defined(CY_CORE_CM7_1)
    ips200_show_string(24,ROW_4,"L_Speed:");    ips200_show_int(88,ROW_4,(int)s_ui_dc.left_motor_speed,5);
    ips200_show_string(24,ROW_5,"R_Speed:");    ips200_show_int(88,ROW_5,(int)s_ui_dc.right_motor_speed,5);
    ips200_show_string(24,ROW_6,"Car_Speed:");  ips200_show_int(88,ROW_6,(int)s_ui_dc.car_speed,5);
#else
    ips200_show_string(24,ROW_4,"L_Speed:");    ips200_show_int(88,ROW_4,Left_Motor_Speed,5);
    ips200_show_string(24,ROW_5,"R_Speed:");    ips200_show_int(88,ROW_5,Right_Motor_Speed,5);
    ips200_show_string(24,ROW_6,"Car_Speed:");  ips200_show_int(88,ROW_6,car_speed,5);
#endif
    //ips200_show_string(24,ROW_7,"Integ:");      ips200_show_int(88,ROW_7,Integ_Encode,5);
}
void ACT_1_2_1()
{
    ReadPos[0] = '1';
    ReadPos[1] = '.';
    ReadPos[2] = '2';
    ReadPos[3] = '.';
    ReadPos[4] = '1';
}

static void GUI_Display_Level3_ImageDetect(uint8 current_idx)
{
    GUI_Display_Level3_Common1();
    ips200_show_string(0,ROW_1,"Image");

    ips200_show_string(24,ROW_3,"Step Detection");
    ips200_show_string(24,ROW_4,"Single Bridge");
    ips200_show_string(24,ROW_5,"Bumpy Road");

    switch(current_idx)
    {
        case 1:
            ips200_show_string(0,ROW_3,"->");
            break;
        case 2:
            ips200_show_string(0,ROW_4,"->");
            break;
        case 3:
            ips200_show_string(0,ROW_5,"->");
            break;
        default:
            break;
    }
}

void GUI_2_1_1(void) // 台阶棢��?
{
    GUI_Display_Level3_ImageDetect(1);

    ips200_show_string(0,ROW_7,"Detected:");
    ips200_show_string(88,ROW_7,step_data.detected ? "Yes" : "No ");

    ips200_show_string(0,ROW_6,"压缩灰度");
    ips200_show_string(0,ROW_8,"Distance:");
    ips200_show_float(88,ROW_8,step_data.distance_cm,3,1);
    ips200_show_string(136,ROW_8,"cm");

    ips200_show_string(0,ROW_9,"Height:");
    ips200_show_uint(88,ROW_9,step_data.step_height_pix,3);
    ips200_show_string(136,ROW_9,"pix");

    /*
     * 主图�?1/2 压缩灰度 `image_two_value`，与视�?�主域��AE 统�?�坐标系丢�致��?
     * `step_detection` 若仍基于全场 `mt9v03x_image`（raw），与屏上压缩�?�感�?能不丢�致，属有意分工��?
     */
    image_photo_compress(mt9v03x_image[0]);
    ips200_show_gray_image(0,ROW_10,image_two_value[0],
                           IMAGE_COMPRESS_W,IMAGE_COMPRESS_H,
                           MT9V03X_W,MT9V03X_H,0);

}
void ACT_2_1_1()
{
    ReadPos[0] = '2';
    ReadPos[1] = '.';
    ReadPos[2] = '1';
    ReadPos[3] = '.';
    ReadPos[4] = '1';
}

void GUI_2_1_2(void) // 单边桥�?���?
{
    GUI_Display_Level3_ImageDetect(2);

    ips200_show_string(0,ROW_7,"Bridge");
    /*
     * 单列「压缩·AE 后��：�?页不叠双图，避免竖向两窗占位不足、dis_* 强压畸变；与台阶页�?�照请切菜单�?
     * `image_camera_auto_exposure()` 内有界迭代，�?能短时阻塞；AE 后在当前曝光下再压一帧用于显示��?
     */
    ips200_show_string(0,ROW_8,"压缩 AE�?");
    image_camera_auto_exposure();
    image_photo_compress(mt9v03x_image[0]);
    ips200_show_gray_image(0,ROW_10,image_two_value[0],
                           IMAGE_COMPRESS_W,IMAGE_COMPRESS_H,
                           MT9V03X_W,MT9V03X_H,0);
}
void ACT_2_1_2()
{
    ReadPos[0] = '2';
    ReadPos[1] = '.';
    ReadPos[2] = '1';
    ReadPos[3] = '.';
    ReadPos[4] = '2';
}

void GUI_2_1_3(void) // 颠簸�?段�?���?
{
    GUI_Display_Level3_ImageDetect(3);

    ips200_show_string(0,ROW_8,"Bumpy Road");
    ips200_show_string(0,ROW_9,"Reserved Page");
    ips200_show_string(0,ROW_11,"Use this page");
    ips200_show_string(0,ROW_12,"for future image");
    ips200_show_string(0,ROW_13,"detection logic.");
}
void ACT_2_1_3()
{
    ReadPos[0] = '2';
    ReadPos[1] = '.';
    ReadPos[2] = '1';
    ReadPos[3] = '.';
    ReadPos[4] = '3';
}

// void GUI_1_3_1(void) // 测试摄像�?
// {
//     GUI_Display_Level3_Common1();
//     ips200_show_string(0,ROW_1,"Image");
//     ips200_show_gray_image(0,21,mt9v03x_image[0],MT9V03X_W,MT9V03X_H,MT9V03X_W,MT9V03X_H,0);
// }
// void ACT_1_3_1()
// {
//     ReadPos[0] = '1';
//     ReadPos[1] = '.';
//     ReadPos[2] = '3';
//     ReadPos[3] = '.';
//     ReadPos[4] = '1';
// }

// void GUI_1_4_1(void) // 测试院�螺仪
// {
//     GUI_Display_Level3_Common1();
//     ips200_show_string(0,ROW_1,"Gyro");
    
//     ips200_show_string(24,ROW_4,"Gyro_x:");  ips200_show_float(88,ROW_4,imu660ra_gyro_x,5,2);
//     ips200_show_string(24,ROW_5,"Gyro_y:");  ips200_show_float(88,ROW_5,imu660ra_gyro_y,5,2);
//     ips200_show_string(24,ROW_6,"Gyro_z:");  ips200_show_float(88,ROW_6,imu660ra_gyro_z,5,2);

//     ips200_show_string(24,ROW_8,"Acc_x:");  ips200_show_float(88,ROW_8,imu660ra_acc_x,5,2);
//     ips200_show_string(24,ROW_9,"Acc_y:");  ips200_show_float(88,ROW_9,imu660ra_acc_y,5,2);
//     ips200_show_string(24,ROW_10,"Acc_z:");  ips200_show_float(88,ROW_10,imu660ra_acc_z,5,2);

//     ips200_show_string(24,ROW_12,"Zero_x:");  ips200_show_float(88,ROW_12,Gyro.ZeroDrift_gyro_x,5,2);
//     ips200_show_string(24,ROW_13,"Zero_y:");  ips200_show_float(88,ROW_13,Gyro.ZeroDrift_gyro_y,5,2);
//     ips200_show_string(24,ROW_14,"Zero_z:");  ips200_show_float(88,ROW_14,Gyro.ZeroDrift_gyro_z,5,2);
    
//     ips200_show_string(24,ROW_16,"Pitch:");  ips200_show_float(88,ROW_16,Gyro.pitch,5,2);
//     ips200_show_string(24,ROW_17,"Roll:");  ips200_show_float(88,ROW_17,Gyro.roll,5,2);
//     ips200_show_string(24,ROW_18,"Yaw:");  ips200_show_float(88,ROW_18,Gyro.yaw,5,2);

// }
// void ACT_1_4_1()
// {
//     ReadPos[0] = '1';
//     ReadPos[1] = '.';
//     ReadPos[2] = '4';
//     ReadPos[3] = '.';
//     ReadPos[4] = '1';
// }


// // /*********************************************************************************************************************
// // * 直立�?调试界面
// // *********************************************************************************************************************/
// static void GUI_Display_Level3_Common2(void)
// {
//     ips200_draw_line(0,20,239,20,IPS200_DEFAULT_PENCOLOR);//�?�?
//     ips200_show_string(0,ROW_1,"Image");
//     ips200_draw_line(188,20,188,319,IPS200_DEFAULT_PENCOLOR);//竖线
//     GUI_Display_FPS();
// }

// void GUI_2_1_1(void) // 原�?�图�? + 二��化
// {
//     GUI_Display_Level3_Common2();

//     ips200_show_gray_image(0,21,mt9v03x_image[0],MT9V03X_W,MT9V03X_H,MT9V03X_W,MT9V03X_H,0);
//     ips200_show_gray_image(0,21+MT9V03X_H+2,ImageBin[0],IMAGE_W,IMAGE_H,MT9V03X_W,MT9V03X_H,0);

//     GUI_Display_Image_Sidebar();
//     GUI_Display_Image_Below();
// }
// void ACT_2_1_1()
// {
//     ReadPos[0] = '2';
//     ReadPos[1] = '.';
//     ReadPos[2] = '1';
//     ReadPos[3] = '.';
//     ReadPos[4] = '1';
// }

// void GUI_2_1_2(void) // 二��化 + 连续边线
// {
//     GUI_Display_Level3_Common2();

//     ips200_show_gray_image(0,21,ImageBin[0],IMAGE_W,IMAGE_H,MT9V03X_W,MT9V03X_H,0);
//     ips200_show_gray_image(0,21+MT9V03X_H+2,ImageEdgeLineNew[0],IMAGE_W,IMAGE_H,MT9V03X_W,MT9V03X_H,0);


//     GUI_Display_Image_Sidebar();
//     GUI_Display_Image_Below();
// }
// void ACT_2_1_2()
// {
//     ReadPos[0] = '2';
//     ReadPos[1] = '.';
//     ReadPos[2] = '1';
//     ReadPos[3] = '.';
//     ReadPos[4] = '2';
// }

// void GUI_2_1_3(void) // 连续边线 + 离散边线
// {
//     GUI_Display_Level3_Common2();

//     ips200_show_gray_image(0,21,ImageEdgeLine[0],IMAGE_W,IMAGE_H,MT9V03X_W,MT9V03X_H,0);
//     ips200_show_gray_image(0,21+MT9V03X_H+2,ImageEdgeLineNew[0],IMAGE_W,IMAGE_H,MT9V03X_W,MT9V03X_H,0);

//     GUI_Display_Image_Sidebar();
//     GUI_Display_Image_Below();
// }
// void ACT_2_1_3()
// {
//     ReadPos[0] = '2';
//     ReadPos[1] = '.';
//     ReadPos[2] = '1';
//     ReadPos[3] = '.';
//     ReadPos[4] = '3';
// }

// static void GUI_Display_Level3_Common3(void)
// {
//     ips200_draw_line(0,20,239,20,IPS200_DEFAULT_PENCOLOR);
//     ips200_show_string(0,ROW_1,"Angle_Pid ");
//     GUI_Display_FPS();
// }

// void GUI_2_2_1(void) // 角��度环P
// {
//     GUI_Display_Level3_Common3();
//     GUI_SetAngleLoop();
//     //画�??
//     ips200_draw_line(50,48,112,48,IPS200_DEFAULT_PENCOLOR);
//     ips200_draw_line(50,64,112,64,IPS200_DEFAULT_PENCOLOR);
//     ips200_draw_line(50,48,50,64,IPS200_DEFAULT_PENCOLOR);
//     ips200_draw_line(112,48,112,64,IPS200_DEFAULT_PENCOLOR);
//     ips200_show_string(120,ROW_5,"L_pwm");       ips200_show_int(174,ROW_5,Left_Motor_Pwm,5);
//     ips200_show_string(120,ROW_6,"R_pwm");       ips200_show_int(174,ROW_6,Right_Motor_Pwm,5);

//     ips200_show_string(120,ROW_10,"Ang_In");       ips200_show_float(174,ROW_10,Gyro.gyro_y,4,1);
    
// }
// void ACT_2_2_1()
// {
//     ReadPos[0] = '2';
//     ReadPos[1] = '.';
//     ReadPos[2] = '2';
//     ReadPos[3] = '.';
//     ReadPos[4] = '1';
//     if(Flash.Flash_Error == FLASH_RUNNING)
//     {
//         AngleDotPID.Kp = ReadBuf_Pid;
//         Flash.Flash_Error = FLASH_STOP;
//     }
// }

// void GUI_2_2_2(void) // 角��度环I
// {
//     GUI_Display_Level3_Common3();
//     GUI_SetAngleLoop();

//     //画�??
//     ips200_draw_line(168,48,238,48,IPS200_DEFAULT_PENCOLOR);
//     ips200_draw_line(168,64,238,64,IPS200_DEFAULT_PENCOLOR);
//     ips200_draw_line(168,48,168,64,IPS200_DEFAULT_PENCOLOR);
//     ips200_draw_line(238,48,238,64,IPS200_DEFAULT_PENCOLOR);
// }
// void ACT_2_2_2()
// {
//     ReadPos[0] = '2';
//     ReadPos[1] = '.';
//     ReadPos[2] = '2';
//     ReadPos[3] = '.';
//     ReadPos[4] = '2';
//     if(Flash.Flash_Error == FLASH_RUNNING)
//     {
//         AngleDotPID.Ki = ReadBuf_Pid;
//         Flash.Flash_Error = FLASH_STOP;
//     }
// }   

// void GUI_2_2_3(void) // 角度环P
// {
//     GUI_Display_Level3_Common3();
//     GUI_SetAngleLoop();

//     //画�??
//     ips200_draw_line(50,128,112,128,IPS200_DEFAULT_PENCOLOR);
//     ips200_draw_line(50,144,112,144,IPS200_DEFAULT_PENCOLOR);
//     ips200_draw_line(50,128,50,144,IPS200_DEFAULT_PENCOLOR);
//     ips200_draw_line(112,128,112,144,IPS200_DEFAULT_PENCOLOR);
// }
// void ACT_2_2_3()
// {
//     ReadPos[0] = '2';
//     ReadPos[1] = '.';
//     ReadPos[2] = '2';
//     ReadPos[3] = '.';
//     ReadPos[4] = '3';
//     if(Flash.Flash_Error == FLASH_RUNNING)
//     {
//         AnglePID.Kp = ReadBuf_Pid;
//         Flash.Flash_Error = FLASH_STOP;
//     }
// }

// void GUI_2_2_4(void) // 角��度环D
// {
//     GUI_Display_Level3_Common3();
//     GUI_SetAngleLoop();

//     //画�??
//     ips200_draw_line(168,128,238,128,IPS200_DEFAULT_PENCOLOR);
//     ips200_draw_line(168,144,238,144,IPS200_DEFAULT_PENCOLOR);
//     ips200_draw_line(168,128,168,144,IPS200_DEFAULT_PENCOLOR);
//     ips200_draw_line(238,128,238,144,IPS200_DEFAULT_PENCOLOR);
// }
// void ACT_2_2_4()
// {
//     ReadPos[0] = '2';
//     ReadPos[1] = '.';
//     ReadPos[2] = '2';
//     ReadPos[3] = '.';
//     ReadPos[4] = '4';
//     if(Flash.Flash_Error == FLASH_RUNNING)
//     {
//         AnglePID.Kd = ReadBuf_Pid;
//         Flash.Flash_Error = FLASH_STOP;
//     }
// }

// void GUI_2_2_5(void) // 速度环P
// {
//     GUI_Display_Level3_Common3();
//     GUI_SetAngleLoop();

//     //画�??
//     ips200_draw_line(50,208,112,208,IPS200_DEFAULT_PENCOLOR);
//     ips200_draw_line(50,224,112,224,IPS200_DEFAULT_PENCOLOR);
//     ips200_draw_line(50,208,50,224,IPS200_DEFAULT_PENCOLOR);
//     ips200_draw_line(112,208,112,224,IPS200_DEFAULT_PENCOLOR);
// }
// void ACT_2_2_5()
// {
//     ReadPos[0] = '2';
//     ReadPos[1] = '.';
//     ReadPos[2] = '2';
//     ReadPos[3] = '.';
//     ReadPos[4] = '5';
//     if(Flash.Flash_Error == FLASH_RUNNING)
//     {
//         SpeedPID.Kp = -ReadBuf_Pid;
//         Flash.Flash_Error = FLASH_STOP;
//     }
// }

// void GUI_2_2_6(void) // 速度环D
// {
//     GUI_Display_Level3_Common3();
//     GUI_SetAngleLoop();

//     //画�??
//     ips200_draw_line(168,208,238,208,IPS200_DEFAULT_PENCOLOR);
//     ips200_draw_line(168,224,238,224,IPS200_DEFAULT_PENCOLOR);
//     ips200_draw_line(168,208,168,224,IPS200_DEFAULT_PENCOLOR);
//     ips200_draw_line(238,208,238,224,IPS200_DEFAULT_PENCOLOR);
// }
// void ACT_2_2_6()
// {
//     ReadPos[0] = '2';
//     ReadPos[1] = '.';
//     ReadPos[2] = '2';
//     ReadPos[3] = '.';
//     ReadPos[4] = '6';

//     if(Flash.Flash_Error == FLASH_RUNNING)
//     {
//         SpeedPID.Kd = ReadBuf_Pid;
//         Flash.Flash_Error = FLASH_STOP;
//     }
// }

// // /*********************************************************************************************************************
// // * �?向环调式界面
// // *********************************************************************************************************************/
// static void GUI_Display_Level3_Common4(void) // �?向环调试界面
// {
//     ips200_draw_line(0,20,239,20,IPS200_DEFAULT_PENCOLOR);
//     ips200_show_string(0,ROW_1,"Dir_Pid ");
//     GUI_Display_FPS();
// }

// void GUI_2_3_1(void) // �?向内环P
// {
//     GUI_Display_Level3_Common4();
//     GUI_SetDirLoop();

//     //画�??
//     ips200_draw_line(50,48,112,48,IPS200_DEFAULT_PENCOLOR);
//     ips200_draw_line(50,64,112,64,IPS200_DEFAULT_PENCOLOR);
//     ips200_draw_line(50,48,50,64,IPS200_DEFAULT_PENCOLOR);
//     ips200_draw_line(112,48,112,64,IPS200_DEFAULT_PENCOLOR);
// }
// void ACT_2_3_1()
// {
//     ReadPos[0] = '2';
//     ReadPos[1] = '.';
//     ReadPos[2] = '3';
//     ReadPos[3] = '.';
//     ReadPos[4] = '1';
//     if(Flash.Flash_Error == FLASH_RUNNING)
//     {
//         DirAngleDotPID.Kp = ReadBuf_Pid;
//         Flash.Flash_Error = FLASH_STOP;
//     }
// }

// void GUI_2_3_2(void) // �?向内环D
// {
//     GUI_Display_Level3_Common4();
//     GUI_SetDirLoop();

//     //画�??
//     ips200_draw_line(168,48,238,48,IPS200_DEFAULT_PENCOLOR);
//     ips200_draw_line(168,64,238,64,IPS200_DEFAULT_PENCOLOR);
//     ips200_draw_line(168,48,168,64,IPS200_DEFAULT_PENCOLOR);
//     ips200_draw_line(238,48,238,64,IPS200_DEFAULT_PENCOLOR);
// }
// void ACT_2_3_2()
// {
//     ReadPos[0] = '2';
//     ReadPos[1] = '.';
//     ReadPos[2] = '3';
//     ReadPos[3] = '.';
//     ReadPos[4] = '2';
//     if(Flash.Flash_Error == FLASH_RUNNING)
//     {
//         DirAngleDotPID.Kd = ReadBuf_Pid;
//         Flash.Flash_Error = FLASH_STOP;
//     }
// }

// void GUI_2_3_3(void) // �?向�?�环P
// {
//     GUI_Display_Level3_Common4();
//     GUI_SetDirLoop();

//     //画�??
//     ips200_draw_line(50,128,112,128,IPS200_DEFAULT_PENCOLOR);
//     ips200_draw_line(50,144,112,144,IPS200_DEFAULT_PENCOLOR);
//     ips200_draw_line(50,128,50,144,IPS200_DEFAULT_PENCOLOR);
//     ips200_draw_line(112,128,112,144,IPS200_DEFAULT_PENCOLOR);
// }
// void ACT_2_3_3()
// {
//     ReadPos[0] = '2';
//     ReadPos[1] = '.';
//     ReadPos[2] = '3';
//     ReadPos[3] = '.';
//     ReadPos[4] = '3';
//     if(Flash.Flash_Error == FLASH_RUNNING)
//     {
//         DirOutPID.Kp = ReadBuf_Pid;
//         Flash.Flash_Error = FLASH_STOP;
//     }
// }

// void GUI_2_3_4(void) // �?向�?�环D 
// {
//     GUI_Display_Level3_Common4();
//     GUI_SetDirLoop();

//     //画�??
//     ips200_draw_line(168,128,238,128,IPS200_DEFAULT_PENCOLOR);
//     ips200_draw_line(168,144,238,144,IPS200_DEFAULT_PENCOLOR);
//     ips200_draw_line(168,128,168,144,IPS200_DEFAULT_PENCOLOR);
//     ips200_draw_line(238,128,238,144,IPS200_DEFAULT_PENCOLOR);
// }
// void ACT_2_3_4()
// {
//     ReadPos[0] = '2';
//     ReadPos[1] = '.';
//     ReadPos[2] = '3';
//     ReadPos[3] = '.';
//     ReadPos[4] = '4';
//     if(Flash.Flash_Error == FLASH_RUNNING)
//     {
//         DirOutPID.Kd = ReadBuf_Pid;
//         Flash.Flash_Error = FLASH_STOP;
//     }
// }

// // /*********************************************************************************************************************
// // * 速度决策设置界面
// // *********************************************************************************************************************/
// static void GUI_Display_Level3_Common5(void)
// {
//     ips200_draw_line(0,20,239,20,IPS200_DEFAULT_PENCOLOR);
//     ips200_show_string(0,ROW_1,"Speed_Select");
//     GUI_Display_FPS();
// }

// void GUI_2_4_1(void)
// {
//     GUI_Display_Level3_Common5();
// }
// void ACT_2_4_1()
// {
//     ReadPos[0] = '2';
//     ReadPos[1] = '.';
//     ReadPos[2] = '4';
//     ReadPos[3] = '.';
//     ReadPos[4] = '1';
// }

/*********************************************************************************************************************
* 其他辅助函数
*********************************************************************************************************************/

// static void GUI_Display_Image_Sidebar(void) // 图像界面侧边栏显示内�?
// {
//     ips200_show_string(192,ROW_3,"Th");         ips200_show_int(192,ROW_4,Threshold,3);
//     ips200_show_string(192,ROW_5,"Pitch");      ips200_show_float(192,ROW_6,Gyro.pitch,3,1);
//     ips200_show_string(192,ROW_7,"Pst");        ips200_show_int(192,ROW_8,ImagePst.Pst[0],3);
//     ips200_show_string(192,ROW_9,"Pwm");        ips200_show_int(192,ROW_10,DirPwm,4);
//     ips200_show_string(192,ROW_11,"Meun");      ips200_show_int(192,ROW_12,Menu_command,4);
//     ips200_show_string(192,ROW_13,"Encode");    ips200_show_float(192,ROW_14,AnglePID.Kp,1,3);
//     ips200_show_string(192,ROW_15,"Encode");    ips200_show_int(192,ROW_16,Car_Speed,4);

// }

// static void GUI_Display_Image_Below(void) // 图像界面下方显示内�??
// {
//     switch(ElementStatusMachine.CurrentStatus)
//     {
//         case NO_ELEMENT_STATUS:
//             ips200_show_string(80,ROW_1,"No_Element");
//         break;

//         case RIGHT_ROUNDABOUT_STATUS_1:
//             ips200_show_string(80,ROW_1,"RightHuan1");
//         break;

//         case RIGHT_ROUNDABOUT_STATUS_2:
//             ips200_show_string(80,ROW_1,"RightHuan2");
//         break;

//         case RIGHT_ROUNDABOUT_STATUS_3:
//             ips200_show_string(80,ROW_1,"RightHuan3");
//         break;

//         case RIGHT_ROUNDABOUT_STATUS_4:
//             ips200_show_string(80,ROW_1,"RightHuan4");
//         break;

//         case RIGHT_ROUNDABOUT_STATUS_5:
//             ips200_show_string(80,ROW_1,"RightHuan5");
//         break;

//         case RIGHT_ROUNDABOUT_STATUS_6:
//             ips200_show_string(80,ROW_1,"RightHuan6");
//         break;

//         case RIGHT_ROUNDABOUT_STATUS_7:
//             ips200_show_string(80,ROW_1,"RightHuan7");
//         break;

//         case RIGHT_ROUNDABOUT_STATUS_8:
//             ips200_show_string(80,ROW_1,"RightHuan8");
//         break;

//         case LEFT_ROUNDABOUT_STATUS_1:
//             ips200_show_string(80,ROW_1,"Left_Huan1");
//         break;

//         case LEFT_ROUNDABOUT_STATUS_2:
//             ips200_show_string(80,ROW_1,"Left_Huan2");
//         break;

//         case LEFT_ROUNDABOUT_STATUS_3:
//             ips200_show_string(80,ROW_1,"Left_Huan3");
//         break;

//         case LEFT_ROUNDABOUT_STATUS_4:
//             ips200_show_string(80,ROW_1,"Left_Huan4");
//         break;

//         case LEFT_ROUNDABOUT_STATUS_5:
//             ips200_show_string(80,ROW_1,"Left_Huan5");
//         break;

//         case LEFT_ROUNDABOUT_STATUS_6:
//             ips200_show_string(80,ROW_1,"Left_Huan6");
//         break;

//         case LEFT_ROUNDABOUT_STATUS_7:
//             ips200_show_string(80,ROW_1,"Left_Huan7");
//         break;

//         case LEFT_ROUNDABOUT_STATUS_8:
//             ips200_show_string(80,ROW_1,"Left_Huan8");
//         break;

//         case CROSS_STATUS_1:
//             ips200_show_string(80,ROW_1,"Cross_Mid1");
//         break;

//         case CROSS_STATUS_2:
//             ips200_show_string(80,ROW_1,"Cross_Mid2");
//         break;
        
//         case PODAO_STATUS_1:
//             ips200_show_string(80,ROW_1,"  Podao  ");
//         break;

//     } 
    
//     ips200_draw_line(0,21+(IMAGE_H - ImagePst.AimIndex),239,21+(IMAGE_H - ImagePst.AimIndex),IPS200_DEFAULT_PENCOLOR);

//     //拐点画线
//     for(uint8 i = 0;i < 6;i++)
//     {
//         //右下
//         if(RightCorner[0].Find == 1)
//         {
//             ips200_draw_point(Clip16(RightCorner[0].Position.x*2 + i,0,187),Clip16(RightCorner[0].Position.y*2 - i + 21,21,209),RGB565_RED);
//             ips200_draw_point(Clip16(RightCorner[0].Position.x*2 + i,0,187),Clip16(RightCorner[0].Position.y*2 + i + 21,21,209),RGB565_RED);
//             ips200_draw_point(Clip16(RightCorner[0].Position.x*2 - i,0,187),Clip16(RightCorner[0].Position.y*2 - i + 21,21,209),RGB565_RED);
//             ips200_draw_point(Clip16(RightCorner[0].Position.x*2 - i,0,187),Clip16(RightCorner[0].Position.y*2 + i + 21,21,209),RGB565_RED);
//         }
//         //左下
//         if(LeftCorner[0].Find == 1)
//         {
//             ips200_draw_point(Clip16(LeftCorner[0].Position.x*2 + i,0,187),Clip16(LeftCorner[0].Position.y*2 - i + 21,21,209),RGB565_YELLOW);
//             ips200_draw_point(Clip16(LeftCorner[0].Position.x*2 + i,0,187),Clip16(LeftCorner[0].Position.y*2 + i + 21,21,209),RGB565_YELLOW);
//             ips200_draw_point(Clip16(LeftCorner[0].Position.x*2 - i,0,187),Clip16(LeftCorner[0].Position.y*2 - i + 21,21,209),RGB565_YELLOW);
//             ips200_draw_point(Clip16(LeftCorner[0].Position.x*2 - i,0,187),Clip16(LeftCorner[0].Position.y*2 + i + 21,21,209),RGB565_YELLOW);
//         }

//         //右中
//         if(RightCorner[1].Find == 1)
//         {
//             ips200_draw_point(Clip16(RightCorner[1].Position.x*2 + i,0,187),Clip16(RightCorner[1].Position.y*2 - i + 21,21,209),RGB565_RED);
//             ips200_draw_point(Clip16(RightCorner[1].Position.x*2 + i,0,187),Clip16(RightCorner[1].Position.y*2 + i + 21,21,209),RGB565_RED);
//             ips200_draw_point(Clip16(RightCorner[1].Position.x*2 - i,0,187),Clip16(RightCorner[1].Position.y*2 - i + 21,21,209),RGB565_RED);
//             ips200_draw_point(Clip16(RightCorner[1].Position.x*2 - i,0,187),Clip16(RightCorner[1].Position.y*2 + i + 21,21,209),RGB565_RED);
//         }
//         //左中
//         if(LeftCorner[1].Find == 1)
//         {
//             ips200_draw_point(Clip16(LeftCorner[1].Position.x*2 + i,0,187),Clip16(LeftCorner[1].Position.y*2 - i + 21,21,209),RGB565_BLUE);
//             ips200_draw_point(Clip16(LeftCorner[1].Position.x*2 + i,0,187),Clip16(LeftCorner[1].Position.y*2 + i + 21,21,209),RGB565_BLUE);
//             ips200_draw_point(Clip16(LeftCorner[1].Position.x*2 - i,0,187),Clip16(LeftCorner[1].Position.y*2 - i + 21,21,209),RGB565_BLUE);
//             ips200_draw_point(Clip16(LeftCorner[1].Position.x*2 - i,0,187),Clip16(LeftCorner[1].Position.y*2 + i + 21,21,209),RGB565_BLUE);
//         }

//         //右上
//         if(RightCorner[2].Find == 1)
//         {
//             ips200_draw_point(Clip16(RightCorner[2].Position.x*2 + i,0,187),Clip16(RightCorner[2].Position.y*2 - i + 21,21,209),RGB565_MAGENTA);
//             ips200_draw_point(Clip16(RightCorner[2].Position.x*2 + i,0,187),Clip16(RightCorner[2].Position.y*2 + i + 21,21,209),RGB565_MAGENTA);
//             ips200_draw_point(Clip16(RightCorner[2].Position.x*2 - i,0,187),Clip16(RightCorner[2].Position.y*2 - i + 21,21,209),RGB565_MAGENTA);
//             ips200_draw_point(Clip16(RightCorner[2].Position.x*2 - i,0,187),Clip16(RightCorner[2].Position.y*2 + i + 21,21,209),RGB565_MAGENTA);
//         }
//         //左上
//         if(LeftCorner[2].Find == 1)
//         {
//             ips200_draw_point(Clip16(LeftCorner[2].Position.x*2 + i,0,187),Clip16(LeftCorner[2].Position.y*2 - i + 21,21,209),RGB565_BROWN);
//             ips200_draw_point(Clip16(LeftCorner[2].Position.x*2 + i,0,187),Clip16(LeftCorner[2].Position.y*2 + i + 21,21,209),RGB565_BROWN);
//             ips200_draw_point(Clip16(LeftCorner[2].Position.x*2 - i,0,187),Clip16(LeftCorner[2].Position.y*2 - i + 21,21,209),RGB565_BROWN);
//             ips200_draw_point(Clip16(LeftCorner[2].Position.x*2 - i,0,187),Clip16(LeftCorner[2].Position.y*2 + i + 21,21,209),RGB565_BROWN);
//         }
//     }



//     if(RightCorner[0].Find == 1)
//     {
//         ips200_show_string(0,ROW_17,"RB_x:");   ips200_show_int(64,ROW_17,RightCorner[0].Position.x,3);
//         ips200_show_string(0,ROW_18,"RB_y:");   ips200_show_int(64,ROW_18,RightCorner[0].Position.y,3);
//         ips200_show_string(0,ROW_19,"RB_C:");   ips200_show_int(64,ROW_19,RightCorner[0].Index,4);
//     }
//     if(LeftCorner[0].Find == 1)
//     {
//         ips200_show_string(96,ROW_17,"LB_x:");  ips200_show_int(144,ROW_17,LeftCorner[0].Position.x,3);
//         ips200_show_string(96,ROW_18,"LB_y:");  ips200_show_int(144,ROW_18,LeftCorner[0].Position.y,3);
//         ips200_show_string(96,ROW_19,"LB_C:");  ips200_show_int(144,ROW_19,LeftCorner[0].Index,4);
//     }
//     if(RightCorner[1].Find == 1)
//     {
//         ips200_show_string(0,ROW_17,"RM_x:");   ips200_show_int(64,ROW_17,RightCorner[1].Position.x,3);
//         ips200_show_string(0,ROW_18,"RM_y:");   ips200_show_int(64,ROW_18,RightCorner[1].Position.y,3);
//         ips200_show_string(0,ROW_19,"RM_C:");   ips200_show_int(64,ROW_19,RightCorner[1].Index,4);
//     }
//     if(LeftCorner[1].Find == 1)
//     {
//         ips200_show_string(96,ROW_17,"LM_x:");  ips200_show_int(144,ROW_17,LeftCorner[1].Position.x,3);
//         ips200_show_string(96,ROW_18,"LM_y:");  ips200_show_int(144,ROW_18,LeftCorner[1].Position.y,3);
//         ips200_show_string(96,ROW_19,"LM_C:");  ips200_show_int(144,ROW_19,LeftCorner[1].Index,4);
//     }

//     ips200_show_string(0,ROW_17,"RU_x:");   ips200_show_int(64,ROW_17,RightCorner[2].Position.x,3);
//     ips200_show_string(0,ROW_18,"RU_y:");   ips200_show_int(64,ROW_18,RightCorner[2].Position.y,3);
//     ips200_show_string(0,ROW_19,"RU_C:");   ips200_show_int(64,ROW_19,RightCorner[2].Index,4);

//     ips200_show_string(96,ROW_17,"LU_x:");  ips200_show_int(144,ROW_17,LeftCorner[2].Position.x,3);
//     ips200_show_string(96,ROW_18,"LU_y:");  ips200_show_int(144,ROW_18,LeftCorner[2].Position.y,3);
//     ips200_show_string(96,ROW_19,"LU_C:");  ips200_show_int(144,ROW_19,LeftCorner[2].Index,4);


//     ips200_show_string(0,ROW_13,"L_Count");     ips200_show_int(64,ROW_13,LeftBorder.Count,3);
//     ips200_show_string(0,ROW_14,"R_Count");     ips200_show_int(64,ROW_14,RightBorder.Count,3);

//     ips200_show_string(96,ROW_13,"L_Use");      ips200_show_int(144,ROW_13,LeftBorder.UsefullCount,3);
//     ips200_show_string(96,ROW_14,"R_Use");      ips200_show_int(144,ROW_14,RightBorder.UsefullCount,3);

//     ips200_show_string(0,ROW_15,"L_Count");     ips200_show_int(64,ROW_15,NewLeftBorder.Count,3);
//     ips200_show_string(0,ROW_16,"R_Count");     ips200_show_int(64,ROW_16,NewRightBorder.Count,3);

//     ips200_show_string(96,ROW_15,"L_Use");      ips200_show_int(144,ROW_15,NewLeftBorder.UsefullCount,3);
//     ips200_show_string(96,ROW_16,"R_Use");      ips200_show_int(144,ROW_16,NewRightBorder.UsefullCount,3);


// }

// static void GUI_SetAngleLoop(void) // 直立�?设置界面
// {
//     ips200_show_string(0,ROW_3,"AngleDot");
//     ips200_show_string(0,ROW_4,"Dot_Kp");   ips200_show_float(64,ROW_4,AngleDotPID.Kp,4,1);
//     ips200_show_string(120,ROW_4,"Dot_Ki"); ips200_show_float(174,ROW_4,AngleDotPID.Ki,4,1);
//     ips200_show_string(0,ROW_5,"Dot_In");       ips200_show_float(64,ROW_5,-Gyro.gyro_y,2,3);
//     ips200_show_string(0,ROW_6,"Dot_Out");      ips200_show_float(64,ROW_6,AngleDotPID.Out,4,1);
 

//     ips200_show_string(0,ROW_8,"Angle");
//     ips200_show_string(0,ROW_9,"Ang_Kp");   ips200_show_float(64,ROW_9,AnglePID.Kp,1,4);
//     ips200_show_string(120,ROW_9,"Ang_Kd"); ips200_show_float(174,ROW_9,AnglePID.Kd,1,4);
//     ips200_show_string(0,ROW_10,"Ang_In");       ips200_show_float(64,ROW_10,SpeedPIDOutPwm+Gyro.pitch,4,1);
//     ips200_show_string(0,ROW_11,"Ang_Out");     ips200_show_float(64,ROW_11,AnglePID.Out,1,4);
  


//     ips200_show_string(0,ROW_13,"Speed");
//     ips200_show_string(0,ROW_14,"Spd_Kp");  ips200_show_float(64,ROW_14,SpeedPID.Kp,1,4);
//     ips200_show_string(120,ROW_14,"Spd_Kd");ips200_show_float(174,ROW_14,SpeedPID.Kd,1,4);
//     ips200_show_string(0,ROW_15,"Spd_In");      ips200_show_float(64,ROW_15,Car_AimSpeed-Car_Speed,4,1);
//     ips200_show_string(0,ROW_16,"Spd_Out");     ips200_show_float(64,ROW_16,SpeedPID.Out,1,4);
//     ips200_show_string(120,ROW_15,"Car_Spd");   ips200_show_int(192,ROW_15,Left_Speed,4);//Car_Speed
//     ips200_show_string(120,ROW_16,"SpdOut2");   ips200_show_int(192,ROW_16,Right_Speed,4);//SpeedPIDOutPwm

//     ips200_show_string(0,ROW_18,"Pwm");         ips200_show_float(64,ROW_18,DirPwm,4,1);
// }

// static void GUI_SetDirLoop(void) // �?向环设置界面
// {
//     ips200_show_string(0,ROW_3,"Dir_In");
//     ips200_show_string(0,ROW_4,"In_Kp");   ips200_show_float(64,ROW_4,DirAngleDotPID.Kp,4,1);
//     ips200_show_string(120,ROW_4,"In_Kd"); ips200_show_float(174,ROW_4,DirAngleDotPID.Kd,4,1);

//     ips200_show_string(0,ROW_8,"Dir_Out");
//     ips200_show_string(0,ROW_9,"Out_Kp");   ips200_show_float(64,ROW_9,DirOutPID.Kp,4,1);
//     ips200_show_string(120,ROW_9,"Out_Kd"); ips200_show_float(174,ROW_9,DirOutPID.Kd,4,1);

//     ips200_show_string(0,ROW_18,"Pwm");         ips200_show_float(64,ROW_18,DirPwm,4,1);
// }

/*********************************************************************************************************************
* END OF FILE
*********************************************************************************************************************/