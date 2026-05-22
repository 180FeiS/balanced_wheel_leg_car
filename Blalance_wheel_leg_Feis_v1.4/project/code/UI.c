/*
 * @Author: bom
 * @Version: V1.0
 * @Date: 2024-12-18 17:23:09
 * @LastEditTime: 2025-03-14 16:07:09
 * @FilePath: \Balance_Car V3.0.1\project\code\UI.c
 * @Description: 
 */
/*********************************************************************************************************************
* @file            UI.c
* @author          bom
* @version         V1.0
* @date            2024-12-18
* @brief           平衡车 UI 控制程序
*
* @note
* 1. 本文件包含平衡车所有 UI 显示和控制功能
* 2. 支持 LCD 显示、串口调试和参数配置
* 3. 包含完整的多级菜单系统
********************************************************************************************************************/

/*********************************************************************************************************************
* 修改记录
* 日期              作者             版本           说明
* 2024-07-24        Bron            V1.0.0         搭建新工程
* 2024-07-27        Bron            V1.0.2         搭建了二级菜单的框架
* 2024-07-30        Bron            V1.1.1         更换了 u 和 d 的表示方式，完善了级菜单
* 2024-07-31        Bron            V1.1.2         更新了 pid 调参菜单方式，加入 flash
* 2024-08-01        Bron            V1.1.4         微调了 pid 参数整数小数位数显示
* 2024-08-02        Bron            V1.2.0         将重复显示的内容整理到函数中
* 2024-12-19        Bron            V2.0.1         移植到新工程，重新整理菜单
********************************************************************************************************************/
#include "zf_common_headfile.h"
#include "image.h"
#include "my_gps.h"
#if defined(CY_CORE_CM7_1)
#include "dualcore_shared.h"
static dualcore_ctrl_to_ui_t s_ui_dc;
#else
#include "control.h"
#endif

/* Run：须让 pos 3.1~3.3 的 menuMember 头部一致，否则 HashPeer 切项时画面与真实 pos 不同步；勿在 pos「3」上用子菜单列表。 */
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
* 1. 测试外设模式 (GUI_1)
*    - 1_1. 测试电机
*      - 1_1_1. 电机详细信息
*    - 1_2. 测试编码器
*      - 1_2_1. 编码器详细信息
*    - 1_3. 测试摄像头
*      - 1_3_1. 摄像头详细信息
*    - 1_4. 测试陀螺仪
*      - 1_4_1. 陀螺仪详细信息
*    - 1_5. GPS
*      - 1_5_1. GPS 数据
*
* 2. 调试模式 (GUI_2)
*    - 2_1. 图像设置
*      - 2_1_1. 台阶检测
*      - 2_1_2. 单边桥检测
*      - 2_1_3. 颠簸路段检测
*    - 2_2. 导航调试
*      - 2_2_1. 导航调试界面
*    - 2_3. GPS
*      - 2_3_1. GPS 空页面
*    - 2_4. 速度设置
*    - 2_5. 更新 Flash 参数
*    - 2_6. 清空 Flash 缓存区
*
* 3. 运行模式：GUI 必须与 pos 层级一致（易错：一级「3」与二级「3.1」勿画同一块子菜单）
*    - GUI_3 仅用于 pos「3」：顶层 Test/Debug/Run 三行中一行 Run
*    - GUI_3_1～GUI_3_3 用于 pos「3.1～3.3」：二级列表 Launch/Flash/More
*    - GUI_3_1_1 用于 pos「3.1.1」：Launch 下三级发车速度页
*********************************************************************************************************************/

/*********************************************************************************************************************
* 显示模式定义
*********************************************************************************************************************/
typedef enum {
    DISPLAY_MODE_IPS200,   // 液晶模式
    DISPLAY_MODE_SERIAL  // 串口模式
} DisplayMode;

DisplayMode currentDisplayMode = DISPLAY_MODE_IPS200; // 默认显示模式为液晶模式
float FPS = 0;
/* 内部函数声明 */
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
* 一级菜单函数
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

void GUI_1(void) // 测试外设模式
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

void GUI_3(void) // pos「3」：一级 Run（与 GUI_1/2 同源）；子列表仅属于 pos 3.1～3.3
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
    ips200_show_string(80,ROW_14,"GPS");
    

    ips200_show_string(48,ROW_6,"-->");//8*6
    ips200_show_string(136,ROW_6,"<--");//8*6
    // 实现测试电机的逻辑
}
void ACT_1_1()
{
    ReadPos[0] = '1';
    ReadPos[1] = '.';
    ReadPos[2] = '1';
    ReadPos[3] = 0x00;
    ReadPos[4] = 0x00;
}

void GUI_1_2(void) // 测试编码器
{
    // 实现测试编码器的逻辑
    GUI_Display_Level2_Common1();

    ips200_show_string(80,ROW_6,"Motor");//80
    ips200_show_string(80,ROW_8,"Encode");
    ips200_show_string(80,ROW_10,"Image");
    ips200_show_string(80,ROW_12,"Gyro");
    ips200_show_string(80,ROW_14,"GPS");
    

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

void GUI_1_3(void) // 测试摄像头
{
    // 实现测试摄像头的逻辑
    GUI_Display_Level2_Common1();

    ips200_show_string(80,ROW_6,"Motor");//80
    ips200_show_string(80,ROW_8,"Encode");
    ips200_show_string(80,ROW_10,"Image");
    ips200_show_string(80,ROW_12,"Gyro");
    ips200_show_string(80,ROW_14,"GPS");
    

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

void GUI_1_4(void) // 测试陀螺仪
{
    // 实现测试陀螺仪的逻辑
    GUI_Display_Level2_Common1();

    ips200_show_string(80,ROW_6,"Motor");//80
    ips200_show_string(80,ROW_8,"Encode");
    ips200_show_string(80,ROW_10,"Image");
    ips200_show_string(80,ROW_12,"Gyro");
    ips200_show_string(80,ROW_14,"GPS");
    

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

void GUI_1_5(void)
{
    GUI_Display_Level2_Common1();

    ips200_show_string(80,ROW_6,"Motor");//80
    ips200_show_string(80,ROW_8,"Encode");
    ips200_show_string(80,ROW_10,"Image");
    ips200_show_string(80,ROW_12,"Gyro");
    ips200_show_string(80,ROW_14,"GPS");


    ips200_show_string(48,ROW_14,"-->");//8*6
    ips200_show_string(136,ROW_14,"<--");//8*6
}
void ACT_1_5()
{
    ReadPos[0] = '1';
    ReadPos[1] = '.';
    ReadPos[2] = '5';
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


void GUI_2_2(void) // 调试列表：NavDbg 行选中（与其它项一致为列表态；按「右」进入 GUI_2_2_1）
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

void GUI_2_2_1(void) // 导航调试界面（须从 2.2 按「右」进入）
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


void GUI_2_3(void) // GPS（调试二级列表项）
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
    uint8 point_count = 0;
    uint8 current_show_point = 0;
    uint8 recording_active = 0;
    uint32 current_element = GPS_ELEMENT_NORMAL;
    uint8 nav_state = GPS_NAV_STATE_IDLE;
    uint8 nav_protect_reason = GPS_NAV_PROTECT_NONE;
    uint8 nav_target_index = 0;
    float nav_distance_m = 0.0f;
    float nav_imu_yaw_deg = 0.0f;
    float nav_yaw_err_deg = 0.0f;
    uint8 nav_align_state = GPS_NAV_ALIGN_WAIT;
    float nav_heading_bias_deg = 0.0f;
    float nav_gps_first_deg = 0.0f;
    float nav_dist_from_launch_m = 0.0f;
    uint8 nav_drift_valid = 0u;
    float nav_drift_dlat = 0.0f;
    float nav_drift_dlon = 0.0f;
    const char *element_name = "None";
    const char *nav_state_name = "Idle";
    const char *nav_protect_name = "None";

    GUI_Display_Level2_Common2();

#if defined(CY_CORE_CM7_1)
    dualcore_ctrl_to_ui_pull(&s_ui_dc);
    point_count = s_ui_dc.gps_point_count;
    current_show_point = s_ui_dc.gps_show_point;
    recording_active = s_ui_dc.gps_recording_active;
    current_element = s_ui_dc.gps_current_yuansu;
    nav_state = s_ui_dc.gps_nav_state;
    nav_protect_reason = s_ui_dc.gps_nav_protect_reason;
    nav_target_index = s_ui_dc.gps_nav_target_index;
    nav_distance_m = s_ui_dc.gps_nav_distance_m;
    nav_imu_yaw_deg = s_ui_dc.euler_yaw;
    nav_yaw_err_deg = s_ui_dc.gps_nav_yaw_err_deg;
    nav_align_state = s_ui_dc.gps_nav_align_state;
    nav_heading_bias_deg = s_ui_dc.gps_nav_heading_bias_deg;
    nav_gps_first_deg = s_ui_dc.gps_nav_gps_first_deg;
    nav_dist_from_launch_m = s_ui_dc.gps_nav_dist_from_launch_m;
    nav_drift_valid = s_ui_dc.gps_drift_corr_valid;
    nav_drift_dlat = s_ui_dc.gps_drift_delta_lat;
    nav_drift_dlon = s_ui_dc.gps_drift_delta_lon;
#else
    point_count = gps_point_count;
    current_show_point = show_point;
    recording_active = gps_recording_active;
    current_element = gps_current_yuansu;
    nav_state = gps_nav_state;
    nav_protect_reason = gps_nav_protect_reason;
    nav_target_index = gps_nav_target_index;
    nav_distance_m = gps_nav_distance_m;
    nav_imu_yaw_deg = (float)euler_angle.yaw;
    nav_yaw_err_deg = gps_nav_yaw_err_deg;
    nav_align_state = gps_nav_align_state;
    nav_heading_bias_deg = gps_nav_heading_bias_deg;
    nav_gps_first_deg = gps_nav_gps_first_deg;
    nav_dist_from_launch_m = gps_nav_dist_from_launch_m;
    nav_drift_valid = gps_drift_corr_valid;
    nav_drift_dlat = (float)gps_drift_delta_lat;
    nav_drift_dlon = (float)gps_drift_delta_lon;
#endif

    element_name = GPS_GetElementName(current_element);
    nav_state_name = GPS_GetNavStateName(nav_state);
    nav_protect_name = GPS_GetNavProtectName(nav_protect_reason);

    ips200_show_string(24, ROW_3, "GPS Debug");
    ips200_draw_line(16, ROW_5 - 1, 119, ROW_5 - 1, IPS200_DEFAULT_PENCOLOR);
    ips200_draw_line(128, ROW_5 - 1, 224, ROW_5 - 1, IPS200_DEFAULT_PENCOLOR);

#if defined(CY_CORE_CM7_1)
    ips200_show_string(0, ROW_5, "Valid:");
    ips200_show_uint(48, ROW_5, (uint32)s_ui_dc.gps_valid, 1);
    ips200_show_string(64, ROW_5, "Sat:");
    ips200_show_uint(104, ROW_5, (uint32)s_ui_dc.gps_satellite_used, 3);
    ips200_show_string(0, ROW_6, "Lat:");
    ips200_show_float(40, ROW_6, s_ui_dc.gps_latitude, 3, 6);
    ips200_show_string(0, ROW_7, "Lon:");
    ips200_show_float(40, ROW_7, s_ui_dc.gps_longitude, 3, 6);
#else
    ips200_show_string(0, ROW_5, "Valid:");
    ips200_show_uint(48, ROW_5, (uint32)((gnss.time.year != 0u) || (gnss.state != 0u) || (gnss.satellite_used != 0u)), 1);
    ips200_show_string(64, ROW_5, "Sat:");
    ips200_show_uint(104, ROW_5, (uint32)gnss.satellite_used, 3);
    ips200_show_string(0, ROW_6, "Lat:");
    ips200_show_float(40, ROW_6, gnss.latitude, 3, 6);
    ips200_show_string(0, ROW_7, "Lon:");
    ips200_show_float(40, ROW_7, gnss.longitude, 3, 6);
#endif

    ips200_show_string(0, ROW_8, "Pt:");
    ips200_show_uint(32, ROW_8, (uint32)point_count, 2);
    ips200_show_string(64, ROW_8, "Last:");
    ips200_show_uint(112, ROW_8, (uint32)current_show_point, 2);
    ips200_show_string(0, ROW_9, recording_active ? "Rec:Recording" : "Rec:Idle");
    ips200_show_string(0, ROW_10, "Elem:");
    ips200_show_string(48, ROW_10, element_name);
    if (recording_active)
    {
        ips200_show_string(0, ROW_11, "K1 --");
        ips200_show_string(0, ROW_12, "K2 End+Save");
        ips200_show_string(0, ROW_13, "K3 Mark");
        ips200_show_string(0, ROW_14, "K4 Elem");
    }
    else
    {
        ips200_show_string(0, ROW_11, "K1 Begin");
        ips200_show_string(0, ROW_12, "K2 End+Save");
        ips200_show_string(0, ROW_13, "K3 Launch");
        ips200_show_string(0, ROW_14, "K4 Back");
    }
    ips200_show_string(0, ROW_15, "Nav:");
    ips200_show_string(40, ROW_15, nav_state_name);
    ips200_show_string(0, ROW_16, "T:");
    ips200_show_uint(16, ROW_16, (uint32)nav_target_index, 2);
    ips200_show_string(40, ROW_16, "D:");
    ips200_show_float(56, ROW_16, nav_distance_m, 3, 1);
    ips200_show_string(116, ROW_16, "Lm:");
    ips200_show_float(144, ROW_16, nav_dist_from_launch_m, 3, 1);
    ips200_show_string(0, ROW_17, "Yaw:");
    ips200_show_float(40, ROW_17, nav_imu_yaw_deg, 3, 1);
    ips200_show_string(120, ROW_17, "Dv");
    ips200_show_uint(144, ROW_17, (uint32)nav_drift_valid, 1);
    ips200_show_string(160, ROW_17, "dLa");
    /* 240 宽屏：2+3 位浮点最长 7 字×8px，起笔 192 时末字 x=240 触发 ips200_show_char 断言 */
    ips200_show_float(184, ROW_17, nav_drift_dlat, 2, 3);
    ips200_show_string(0, ROW_18, "Err:");
    ips200_show_float(40, ROW_18, nav_yaw_err_deg, 3, 1);
    ips200_show_string(120, ROW_18, "dLo:");
    ips200_show_float(152, ROW_18, nav_drift_dlon, 2, 3);
    ips200_show_string(0, ROW_19, "Al:");
    ips200_show_uint(24, ROW_19, (uint32)nav_align_state, 1);
    ips200_show_string(40, ROW_19, "Bias:");
    ips200_show_float(88, ROW_19, nav_heading_bias_deg, 3, 1);
    ips200_show_string(144, ROW_19, "GF:");
    ips200_show_float(168, ROW_19, nav_gps_first_deg, 3, 1);
    ips200_show_string(0, ROW_20, "P:");
    ips200_show_string(16, ROW_20, nav_protect_name);

    ips200_show_string(144, ROW_3, "Path");
#if defined(CY_CORE_CM7_1)
    GPS_Path_DrawWithCar(s_ui_dc.gps_latitude_point,
                         s_ui_dc.gps_longitude_point,
                         GPS_POINT_MAX,
                         s_ui_dc.gps_valid,
                         (uint8)car_gps_dir,
                         s_ui_dc.gps_latitude,
                         s_ui_dc.gps_longitude,
                         136u,
                         ROW_5,
                         88u,
                         112u);
#else
    GPS_Path_DrawWithCar(latitude_point,
                         longitude_point,
                         GPS_POINT_MAX,
                         (uint8)((gnss.latitude != 0.0) && (gnss.longitude != 0.0)),
                         (uint8)car_gps_dir,
                         gnss.latitude,
                         gnss.longitude,
                         136u,
                         ROW_5,
                         88u,
                         112u);
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

void GUI_2_5(void) // 更新 Flash 参数
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

void GUI_2_6(void) // 清空 FLASH 缓存区
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
// * Run 二级/三级：Common3 顶栏 + 列表或发车页。一级 GUI_3 不得复用下列列表，否则与 menu 树错位
// *********************************************************************************************************************
static void GUI_Display_Level2_Common3(void)
{
    ips200_draw_line(0,20,239,20,IPS200_DEFAULT_PENCOLOR);
    ips200_show_string(0,ROW_1,"Run");

    GUI_Display_FPS();
}

/** 二级 Run 列表：与 GUI_2_1 版式对齐；selected_row_index 须与当前 pos 末位 1/2/3 一致 */
static void GUI_Run_ShowSubmenuList(uint8 selected_row_index)
{
    int16 ay;

    GUI_Display_Level2_Common3();

    ips200_show_string(80, ROW_8, " Launch ");
    ips200_show_string(80, ROW_10, " Save   ");
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
        ips200_show_string(24, ROW_15, "KEY3: save launch");
    }
}

void GUI_3_1(void) // Launch 二级项：列表壳，顶栏一行
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

static float GUI_RunLaunchParamValue(uint8 field_index)
{
#if defined(CY_CORE_CM7_1)
    switch (field_index)
    {
    case Nag_Launch_Field_Base_Spd:
        return s_ui_dc.run_launch_speed;
    case Nag_Launch_Field_Spin_Spd:
        return s_ui_dc.nag_spin_target_speed;
    case Nag_Launch_Field_Spin_Dec:
        return s_ui_dc.nag_spin_pre_decel_dist_cm;
    case Nag_Launch_Field_Turn_Spd:
        return s_ui_dc.nag_turnaround_target_speed;
    case Nag_Launch_Field_Turn_Dec:
        return s_ui_dc.nag_turnaround_pre_decel_dist_cm;
    case Nag_Launch_Field_Cone_Spd:
        return s_ui_dc.nag_enter_cones_target_speed;
    case Nag_Launch_Field_Cone_Dec:
        return s_ui_dc.nag_enter_cones_pre_decel_dist_cm;
    default:
        return 0.0f;
    }
#else
    return Nag_LaunchParamGet(field_index);
#endif
}

void GUI_3_1_1(void) /* Launch 三级页：KEY1 选字段，KEY2/3 调值，KEY4 返回 */
{
    static const char *const labels[Nag_Run_Launch_Param_Count] =
    {
        "BaseSpd", "SpinSpd", "SpinDec", "TurnSpd", "TurnDec", "ConeSpd", "ConeDec"
    };
    static const int16 rows[Nag_Run_Launch_Param_Count] =
    {
        ROW_4, ROW_5, ROW_6, ROW_7, ROW_8, ROW_9, ROW_10
    };
    uint8 field_index = 0u;
    uint8 selected = Menu_GetRunLaunchFieldIndex();

    GUI_Display_Level2_Common3();
    ips200_show_string(56, ROW_3, "Launch");
    ips200_draw_line(16, ROW_11, 223, ROW_11, IPS200_DEFAULT_PENCOLOR);

    for (field_index = 0u; field_index < Nag_Run_Launch_Param_Count; field_index++)
    {
        float value = GUI_RunLaunchParamValue(field_index);

        /* 每帧全页重绘时须擦除旧箭头，否则 KEY1 切换后上一行 "->" 仍残留 */
        if (selected == field_index)
        {
            ips200_show_string(0, rows[field_index], "->");
        }
        else
        {
            ips200_show_string(0, rows[field_index], "  ");
        }
        ips200_show_string(16, rows[field_index], labels[field_index]);
        /* ips200_show_float 要求 pointnum 为 1~6，不可为 0 */
        if (Nag_LaunchParamIsSpeed(field_index))
        {
            ips200_show_float(96, rows[field_index], (double)value, 5, 1);
        }
        else
        {
            ips200_show_int(96, rows[field_index], (int32)value, 4);
        }
    }

    ips200_show_string(8, ROW_12, "K1:nxt K2:+ K3:- K4:bk");
    /* ROW_13~ROW_14 预留后续扩展变量 */
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

void GUI_1_1_1(void) // 测试电机（详情页）
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

void GUI_1_5_1(void)
{
    GUI_Display_Level2_Common1();

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
void ACT_1_5_1()
{
    ReadPos[0] = '1';
    ReadPos[1] = '.';
    ReadPos[2] = '5';
    ReadPos[3] = '.';
    ReadPos[4] = '1';
    ReadPos[5] = 0x00;
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

void GUI_2_1_1(void) // 台阶检测
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
     * 主图：1/2 压缩灰度 image_two_value，与视觉主域、AE 统计坐标系一致。
     * step_detection 若仍基于全场 mt9v03x_image（raw），与屏上压缩观感可能不一致，属有意分层。
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

void GUI_2_1_2(void) // 单边桥检测
{
    GUI_Display_Level3_ImageDetect(2);

    ips200_show_string(0,ROW_7,"Bridge");
    /*
     * 单列「压缩 AE 后」：本页不叠双图，避免竖向两窗占位不足。与台阶页切菜单一同对照。
     * image_camera_auto_exposure() 内有界迭代，勿在 bot==0 盲清 armed；AE 后在当前曝光下再压一帧用于显示。
     */
    ips200_show_string(0,ROW_8,"压缩 AE 后");
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

void GUI_2_1_3(void) // 颠簸路段检测
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

// void GUI_1_3_1(void) // 测试摄像头
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

// void GUI_1_4_1(void) // 测试陀螺仪
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
// // * 直立环调试界面
// // *********************************************************************************************************************/
// static void GUI_Display_Level3_Common2(void)
// {
//     ips200_draw_line(0,20,239,20,IPS200_DEFAULT_PENCOLOR);//濡??缁??
//     ips200_show_string(0,ROW_1,"Image");
//     ips200_draw_line(188,20,188,319,IPS200_DEFAULT_PENCOLOR);//绔栫嚎
//     GUI_Display_FPS();
// }

// void GUI_2_1_1(void) // 鍘燂拷?瀣?娴橀??? + 浜屾晶鎱靛寲
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

// void GUI_2_1_2(void) // 浜屾晶鎱靛寲 + 杩炵画杈圭嚎
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

// void GUI_2_1_3(void) // 杩炵画杈圭嚎 + 绂绘暎杈圭嚎
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

// void GUI_2_2_1(void) // 瑙掓０閵﹀害鐜疨
// {
//     GUI_Display_Level3_Common3();
//     GUI_SetAngleLoop();
//     //鐢伙拷??
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

// void GUI_2_2_2(void) // 瑙掓０閵﹀害鐜疘
// {
//     GUI_Display_Level3_Common3();
//     GUI_SetAngleLoop();

//     //鐢伙拷??
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

// void GUI_2_2_3(void) // 瑙掑害鐜疨
// {
//     GUI_Display_Level3_Common3();
//     GUI_SetAngleLoop();

//     //鐢伙拷??
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

// void GUI_2_2_4(void) // 瑙掓０閵﹀害鐜疍
// {
//     GUI_Display_Level3_Common3();
//     GUI_SetAngleLoop();

//     //鐢伙拷??
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

// void GUI_2_2_5(void) // 閫熷害鐜疨
// {
//     GUI_Display_Level3_Common3();
//     GUI_SetAngleLoop();

//     //鐢伙拷??
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

// void GUI_2_2_6(void) // 閫熷害鐜疍
// {
//     GUI_Display_Level3_Common3();
//     GUI_SetAngleLoop();

//     //鐢伙拷??
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
// // * 鏉??鍚戠幆璋冨紡鐣岄潰
// // *********************************************************************************************************************/
// static void GUI_Display_Level3_Common4(void) // 鏉??鍚戠幆璋冭瘯鐣岄潰
// {
//     ips200_draw_line(0,20,239,20,IPS200_DEFAULT_PENCOLOR);
//     ips200_show_string(0,ROW_1,"Dir_Pid ");
//     GUI_Display_FPS();
// }

// void GUI_2_3_1(void) // 鏉??鍚戝唴鐜疨
// {
//     GUI_Display_Level3_Common4();
//     GUI_SetDirLoop();

//     //鐢伙拷??
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

// void GUI_2_3_2(void) // 鏉??鍚戝唴鐜疍
// {
//     GUI_Display_Level3_Common4();
//     GUI_SetDirLoop();

//     //鐢伙拷??
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

// void GUI_2_3_3(void) // 鏉??鍚戯拷?鏍?骞哖
// {
//     GUI_Display_Level3_Common4();
//     GUI_SetDirLoop();

//     //鐢伙拷??
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

// void GUI_2_3_4(void) // 鏉??鍚戯拷?鏍?骞咲
// {
//     GUI_Display_Level3_Common4();
//     GUI_SetDirLoop();

//     //鐢伙拷??
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
// // * 閫熷害鍐崇瓥璁剧疆鐣岄潰
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
* 鍏朵粬杈呭姪鍑芥暟
*********************************************************************************************************************/

// static void GUI_Display_Image_Sidebar(void) // 鍥惧儚鐣岄潰渚ц竟鏍忔樉绀哄唴鐎??
// {
//     ips200_show_string(192,ROW_3,"Th");         ips200_show_int(192,ROW_4,Threshold,3);
//     ips200_show_string(192,ROW_5,"Pitch");      ips200_show_float(192,ROW_6,Gyro.pitch,3,1);
//     ips200_show_string(192,ROW_7,"Pst");        ips200_show_int(192,ROW_8,ImagePst.Pst[0],3);
//     ips200_show_string(192,ROW_9,"Pwm");        ips200_show_int(192,ROW_10,DirPwm,4);
//     ips200_show_string(192,ROW_11,"Meun");      ips200_show_int(192,ROW_12,Menu_command,4);
//     ips200_show_string(192,ROW_13,"Encode");    ips200_show_float(192,ROW_14,AnglePID.Kp,1,3);
//     ips200_show_string(192,ROW_15,"Encode");    ips200_show_int(192,ROW_16,Car_Speed,4);

// }

// static void GUI_Display_Image_Below(void) // 鍥惧儚鐣岄潰涓嬫柟鏄剧ず鍐咃拷??
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

//     //鎷愮偣鐢荤嚎
//     for(uint8 i = 0;i < 6;i++)
//     {
//         //鍙充笅
//         if(RightCorner[0].Find == 1)
//         {
//             ips200_draw_point(Clip16(RightCorner[0].Position.x*2 + i,0,187),Clip16(RightCorner[0].Position.y*2 - i + 21,21,209),RGB565_RED);
//             ips200_draw_point(Clip16(RightCorner[0].Position.x*2 + i,0,187),Clip16(RightCorner[0].Position.y*2 + i + 21,21,209),RGB565_RED);
//             ips200_draw_point(Clip16(RightCorner[0].Position.x*2 - i,0,187),Clip16(RightCorner[0].Position.y*2 - i + 21,21,209),RGB565_RED);
//             ips200_draw_point(Clip16(RightCorner[0].Position.x*2 - i,0,187),Clip16(RightCorner[0].Position.y*2 + i + 21,21,209),RGB565_RED);
//         }
//         //宸︿笅
//         if(LeftCorner[0].Find == 1)
//         {
//             ips200_draw_point(Clip16(LeftCorner[0].Position.x*2 + i,0,187),Clip16(LeftCorner[0].Position.y*2 - i + 21,21,209),RGB565_YELLOW);
//             ips200_draw_point(Clip16(LeftCorner[0].Position.x*2 + i,0,187),Clip16(LeftCorner[0].Position.y*2 + i + 21,21,209),RGB565_YELLOW);
//             ips200_draw_point(Clip16(LeftCorner[0].Position.x*2 - i,0,187),Clip16(LeftCorner[0].Position.y*2 - i + 21,21,209),RGB565_YELLOW);
//             ips200_draw_point(Clip16(LeftCorner[0].Position.x*2 - i,0,187),Clip16(LeftCorner[0].Position.y*2 + i + 21,21,209),RGB565_YELLOW);
//         }

//         //鍙充腑
//         if(RightCorner[1].Find == 1)
//         {
//             ips200_draw_point(Clip16(RightCorner[1].Position.x*2 + i,0,187),Clip16(RightCorner[1].Position.y*2 - i + 21,21,209),RGB565_RED);
//             ips200_draw_point(Clip16(RightCorner[1].Position.x*2 + i,0,187),Clip16(RightCorner[1].Position.y*2 + i + 21,21,209),RGB565_RED);
//             ips200_draw_point(Clip16(RightCorner[1].Position.x*2 - i,0,187),Clip16(RightCorner[1].Position.y*2 - i + 21,21,209),RGB565_RED);
//             ips200_draw_point(Clip16(RightCorner[1].Position.x*2 - i,0,187),Clip16(RightCorner[1].Position.y*2 + i + 21,21,209),RGB565_RED);
//         }
//         //宸︿腑
//         if(LeftCorner[1].Find == 1)
//         {
//             ips200_draw_point(Clip16(LeftCorner[1].Position.x*2 + i,0,187),Clip16(LeftCorner[1].Position.y*2 - i + 21,21,209),RGB565_BLUE);
//             ips200_draw_point(Clip16(LeftCorner[1].Position.x*2 + i,0,187),Clip16(LeftCorner[1].Position.y*2 + i + 21,21,209),RGB565_BLUE);
//             ips200_draw_point(Clip16(LeftCorner[1].Position.x*2 - i,0,187),Clip16(LeftCorner[1].Position.y*2 - i + 21,21,209),RGB565_BLUE);
//             ips200_draw_point(Clip16(LeftCorner[1].Position.x*2 - i,0,187),Clip16(LeftCorner[1].Position.y*2 + i + 21,21,209),RGB565_BLUE);
//         }

//         //鍙充笂
//         if(RightCorner[2].Find == 1)
//         {
//             ips200_draw_point(Clip16(RightCorner[2].Position.x*2 + i,0,187),Clip16(RightCorner[2].Position.y*2 - i + 21,21,209),RGB565_MAGENTA);
//             ips200_draw_point(Clip16(RightCorner[2].Position.x*2 + i,0,187),Clip16(RightCorner[2].Position.y*2 + i + 21,21,209),RGB565_MAGENTA);
//             ips200_draw_point(Clip16(RightCorner[2].Position.x*2 - i,0,187),Clip16(RightCorner[2].Position.y*2 - i + 21,21,209),RGB565_MAGENTA);
//             ips200_draw_point(Clip16(RightCorner[2].Position.x*2 - i,0,187),Clip16(RightCorner[2].Position.y*2 + i + 21,21,209),RGB565_MAGENTA);
//         }
//         //宸︿笂
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

// static void GUI_SetAngleLoop(void) // 鐩寸珛閻??璁剧疆鐣岄潰
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

// static void GUI_SetDirLoop(void) // 鏉??鍚戠幆璁剧疆鐣岄潰
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