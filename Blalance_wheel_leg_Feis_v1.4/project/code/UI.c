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
#if defined(CY_CORE_CM7_1)
#include "single_bridge.h"
#include "step_detection.h"
#endif
#include "my_gps.h"
#if defined(CY_CORE_CM7_1)
#include "dualcore_shared.h"
static dualcore_ctrl_to_ui_t s_ui_dc;
#else
#include "control.h"
#include "nav_fusion.h"
#endif
#include "Menu.h"
#include "navigation.h"
#include "control.h"
#include "ekf.h"

/* Run：须让 pos 3.1~3.4 的 menuMember 头部一致，否则 HashPeer 切项时画面与真实 pos 不同步；勿在 pos「3」上用子菜单列表。 */
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

void GUI_2_1(void) // Debug 二级列表：Image 行
{
    GUI_Display_Level2_Common2();

    ips200_show_string(80, ROW_6, " Image  ");
    ips200_show_string(80, ROW_8, " NavDbg ");
    ips200_show_string(80, ROW_10, " GPS    ");
    ips200_show_string(80, ROW_12, " Speed  ");
    ips200_show_string(80, ROW_14, "W_Flash");
    ips200_show_string(80, ROW_16, "PathFix");

    ips200_show_string(48, ROW_6, "-->");
    ips200_show_string(152, ROW_6, "<--");
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
    ips200_show_string(80,ROW_16,"PathFix");

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
    uint8 nav_recording_active = 0;
    uint8 event_active = 0;
    uint8 event_record_type = 0;
    uint8 event_active_type = 0;
    uint8 end_f = 0;
    uint8 nag_system_run_index = 0;
    uint8 event_count = 0;
    uint8 display_type = 0;
    const char *type_name = "Unknown";
    const char *rec_state_name = "Rec:Idle";

    GUI_Display_Level2_Common2();

#if defined(CY_CORE_CM7_1)
    dualcore_ctrl_to_ui_pull(&s_ui_dc);
    nav_recording_active = s_ui_dc.nav_recording_active;
    event_active = s_ui_dc.event_active;
    event_record_type = s_ui_dc.event_record_type;
    event_active_type = s_ui_dc.event_active_type;
    end_f = s_ui_dc.end_f;
    nag_system_run_index = s_ui_dc.nag_system_run_index;
#else
    nav_recording_active = (uint8)((N.Nag_SystemRun_Index == 1u) && (N.End_f == 0u));
    event_active = N.Event_Active;
    event_record_type = N.Event_Record_Type;
    event_active_type = N.Event_Active_Type;
    end_f = (uint8)(N.End_f ? 1u : 0u);
    nag_system_run_index = N.Nag_SystemRun_Index;
    event_count = N.Event_Count;
#endif

    if (nav_recording_active)
    {
        rec_state_name = "Rec:Recording";
    }
    else if (nag_system_run_index == 1u && end_f)
    {
        rec_state_name = "Rec:Saving";
    }
    else if (event_active)
    {
        rec_state_name = "Run:Event";
    }
    else if (nag_system_run_index >= 2u)
    {
        rec_state_name = "Run:Replay";
    }

    display_type = event_active ? event_active_type : event_record_type;
    type_name = Nag_GetEventTypeName(display_type);

    ips200_show_string(64, ROW_3, "Nav Debug");

    ips200_show_string(0, ROW_5, rec_state_name);

#if defined(CY_CORE_CM7_1)
    if (nav_recording_active && s_ui_dc.menu_nav_fusion_enable != 0u && s_ui_dc.fusion_origin_calibrating != 0u)
    {
        ips200_show_string(120, ROW_5, "Origin:");
        ips200_show_uint(168, ROW_5, (uint32)s_ui_dc.fusion_origin_accepted, 2);
        ips200_show_string(184, ROW_5, "/50");
    }
#else
    if (nav_recording_active && g_menu_nav_fusion_enable != 0u && NavFusion_IsOriginCalibrating() != 0u)
    {
        ips200_show_string(120, ROW_5, "Origin:");
        ips200_show_uint(168, ROW_5, (uint32)NavFusion_GetOriginAcceptedCount(), 2);
        ips200_show_string(184, ROW_5, "/50");
    }
#endif

    if (event_active)
    {
        ips200_show_string(0, ROW_6, "ActEv:");
    }
    else
    {
        ips200_show_string(0, ROW_6, "Elem:");
    }
    ips200_show_string(48, ROW_6, type_name);
    ips200_show_string(112, ROW_6, "(");
    ips200_show_uint(120, ROW_6, (uint32)display_type, 1);
    ips200_show_string(128, ROW_6, ")");

#if defined(CY_CORE_CM7_1)
    ips200_show_string(144, ROW_6, "R");
    ips200_show_uint(160, ROW_6, (uint32)s_ui_dc.nag_record_subject, 1);
    ips200_show_string(176, ROW_6, "P");
    ips200_show_uint(192, ROW_6, (uint32)s_ui_dc.nag_replay_subject, 1);
#else
    ips200_show_string(144, ROW_6, "R");
    ips200_show_uint(160, ROW_6, (uint32)g_nag_record_subject, 1);
    ips200_show_string(176, ROW_6, "P");
    ips200_show_uint(192, ROW_6, (uint32)g_nag_replay_subject, 1);
#endif

#if defined(CY_CORE_CM7_1)
    ips200_show_string(0, ROW_7, "RunIdx:");
    ips200_show_uint(64, ROW_7, (uint32)nag_system_run_index, 1);
#else
    ips200_show_string(0, ROW_7, "EvCnt:");
    ips200_show_uint(56, ROW_7, (uint32)event_count, 2);
#endif

    if (nav_recording_active)
    {
        ips200_show_string(0, ROW_8, "K1 ReBegin");
        ips200_show_string(0, ROW_9, "K2 StopRec");
        ips200_show_string(0, ROW_10, "K3 NextType");
        ips200_show_string(0, ROW_11, "K4 Mark");
    }
    else if (event_active)
    {
        ips200_show_string(0, ROW_8, "K1 Begin");
        ips200_show_string(0, ROW_9, "K2 StopRec");
        ips200_show_string(0, ROW_10, "K3 Replay");
        ips200_show_string(0, ROW_11, "K4 EvDone");
    }
    else
    {
        ips200_show_string(0, ROW_8, "K1 Begin");
        ips200_show_string(0, ROW_9, "K2 StopRec");
        ips200_show_string(0, ROW_10, "K3 Replay");
        ips200_show_string(0, ROW_11, "K4 Back");
    }

    ips200_draw_line(0, ROW_12, 239, ROW_12, IPS200_DEFAULT_PENCOLOR);

    ips200_show_string(0, ROW_13, "Mileage:");
#if defined(CY_CORE_CM7_1)
    ips200_show_float(104, ROW_13, s_ui_dc.mileage_debug_total, 4, 2);

    ips200_show_string(0, ROW_14, "SaveIdx:");
    ips200_show_uint(104, ROW_14, (uint32)s_ui_dc.save_index, 5);

    ips200_show_string(0, ROW_15, "FlashPg:");
    ips200_show_uint(104, ROW_15, (uint32)s_ui_dc.flash_page_index, 3);
#else
    ips200_show_float(104, ROW_13, N.Mileage_Debug_Total, 4, 2);

    ips200_show_string(0, ROW_14, "SaveIdx:");
    ips200_show_uint(104, ROW_14, N.Save_index, 5);

    ips200_show_string(0, ROW_15, "FlashPg:");
    ips200_show_uint(104, ROW_15, N.Flash_page_index, 3);
#endif
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
    ips200_show_string(80,ROW_16,"PathFix");
    

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
    uint32 current_element = NAV_ELEM_NORMAL;
    uint32 tracking_element = NAV_ELEM_NORMAL;
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
    if (nav_target_index < GPS_POINT_MAX)
    {
        tracking_element = s_ui_dc.gps_yuansu[nav_target_index];
    }
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
    if (nav_target_index < GPS_POINT_MAX)
    {
        tracking_element = u32yuansu[nav_target_index];
    }
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
    if (!recording_active && nav_state == GPS_NAV_STATE_RUNNING)
    {
        ips200_show_string(96, ROW_10, "Tr:");
        ips200_show_string(112, ROW_10, GPS_GetElementName(tracking_element));
    }
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
    ips200_show_string(80,ROW_16,"PathFix");
    

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
    ips200_show_string(80,ROW_16,"PathFix");
    

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

void GUI_2_6(void) // PathFix（调试二级列表项，按 KEY3 进入 GUI_2_6_1）
{
    GUI_Display_Level2_Common2();

    ips200_show_string(80, ROW_6, " Image  ");
    ips200_show_string(80, ROW_8, " NavDbg ");
    ips200_show_string(80, ROW_10, " GPS    ");
    ips200_show_string(80, ROW_12, " Speed  ");
    ips200_show_string(80, ROW_14, "W_Flash");
    ips200_show_string(80, ROW_16, "PathFix");

    ips200_show_string(48, ROW_16, "-->");
    ips200_show_string(152, ROW_16, "<--");
}
void ACT_2_6()
{
    ReadPos[0] = '2';
    ReadPos[1] = '.';
    ReadPos[2] = '6';
    ReadPos[3] = 0x00;
    ReadPos[4] = 0x00;
}

#if defined(CY_CORE_CM7_1)
static uint16 UI_PathFix_ClampU16(int32 value, uint16 min_value, uint16 max_value)
{
    if (value < (int32)min_value)
    {
        return min_value;
    }
    if (value > (int32)max_value)
    {
        return max_value;
    }
    return (uint16)value;
}

static void UI_PathFix_DrawCrossMarker(uint16 x,
                                       uint16 y,
                                       uint16 x_offset,
                                       uint16 y_offset,
                                       uint16 width,
                                       uint16 height,
                                       uint16 color,
                                       uint16 span)
{
    uint16 x_max = (uint16)(x_offset + width);
    uint16 y_max = (uint16)(y_offset + height);
    uint16 x0 = UI_PathFix_ClampU16((int32)x, x_offset, x_max);
    uint16 y0 = UI_PathFix_ClampU16((int32)y, y_offset, y_max);

    ips200_draw_line(UI_PathFix_ClampU16((int32)x0 - (int32)span, x_offset, x_max), y0,
                     UI_PathFix_ClampU16((int32)x0 + (int32)span, x_offset, x_max), y0,
                     color);
    ips200_draw_line(x0, UI_PathFix_ClampU16((int32)y0 - (int32)span, y_offset, y_max),
                     x0, UI_PathFix_ClampU16((int32)y0 + (int32)span, y_offset, y_max),
                     color);
}

/* 与 navigation.c 一致：CM7_1 只读快照绘制选中高亮 */
static void UI_PathFix_DrawSelectedMarker(uint16 x,
                                          uint16 y,
                                          uint16 x_offset,
                                          uint16 y_offset,
                                          uint16 width,
                                          uint16 height)
{
    uint16 x_max = (uint16)(x_offset + width);
    uint16 y_max = (uint16)(y_offset + height);
    uint16 x0 = UI_PathFix_ClampU16((int32)x, x_offset, x_max);
    uint16 y0 = UI_PathFix_ClampU16((int32)y, y_offset, y_max);
    uint16 span = 4u;

    UI_PathFix_DrawCrossMarker(x0, y0, x_offset, y_offset, width, height, RGB565_BLACK, span);
    ips200_draw_line(UI_PathFix_ClampU16((int32)x0 - 3, x_offset, x_max),
                     UI_PathFix_ClampU16((int32)y0 - 3, y_offset, y_max),
                     UI_PathFix_ClampU16((int32)x0 + 3, x_offset, x_max),
                     UI_PathFix_ClampU16((int32)y0 + 3, y_offset, y_max),
                     RGB565_BLACK);
    ips200_draw_line(UI_PathFix_ClampU16((int32)x0 - 3, x_offset, x_max),
                     UI_PathFix_ClampU16((int32)y0 + 3, y_offset, y_max),
                     UI_PathFix_ClampU16((int32)x0 + 3, x_offset, x_max),
                     UI_PathFix_ClampU16((int32)y0 - 3, y_offset, y_max),
                     RGB565_BLACK);
}
#endif

void GUI_2_6_1(void) // PathFix 惯导路径修正功能页（须从 2.6 按 KEY3 进入）
{
    uint8 pathfix_active = 0u;
    uint8 pathfix_loaded = 0u;
    uint8 pathfix_dirty = 0u;
    uint8 replay_subject = 1u;
    uint16 pathfix_select = 0u;
    uint32 pathfix_count = 0u;
    int32 pathfix_yaw_x100 = 0;

    GUI_Display_Level2_Common2();
    ips200_show_string(56, ROW_3, "PathFix");

#if defined(CY_CORE_CM7_1)
    {
    uint16 draw_count = 0u;
    uint16 draw_sel = 0u;
    uint16 i = 0u;

    dualcore_ctrl_to_ui_pull(&s_ui_dc);
    pathfix_active = s_ui_dc.pathfix_active;
    pathfix_loaded = s_ui_dc.pathfix_loaded;
    pathfix_dirty = s_ui_dc.pathfix_dirty;
    replay_subject = s_ui_dc.pathfix_replay_subject;
    if (replay_subject == 0u)
    {
        replay_subject = s_ui_dc.nag_replay_subject;
    }
    pathfix_select = s_ui_dc.pathfix_select_index;
    pathfix_count = s_ui_dc.pathfix_point_count;
    pathfix_yaw_x100 = s_ui_dc.pathfix_select_yaw_x100;
    draw_count = s_ui_dc.pathfix_draw_count;
    draw_sel = s_ui_dc.pathfix_draw_sel_idx;

    if (pathfix_loaded != 0u)
    {
        uint16 x_off = 136u;
        uint16 y_off = (uint16)ROW_5;
        uint16 w = 88u;
        uint16 h = 112u;
        uint16 y = 0u;
        uint16 x_end = (uint16)(x_off + w);
        uint16 y_end = (uint16)(y_off + h);

        for (y = y_off; y <= y_end; y++)
        {
            ips200_draw_line(x_off, y, x_end, y, RGB565_WHITE);
        }
        /* 折线 → 起终点 → 元素十字(蓝) → 选中高亮(黑) */
        for (i = 1u; i < draw_count; i++)
        {
            ips200_draw_line((uint16)s_ui_dc.pathfix_draw_x[i - 1u],
                             (uint16)s_ui_dc.pathfix_draw_y[i - 1u],
                             (uint16)s_ui_dc.pathfix_draw_x[i],
                             (uint16)s_ui_dc.pathfix_draw_y[i],
                             RGB565_RED);
        }
        if (draw_count > 0u)
        {
            ips200_draw_point((uint16)s_ui_dc.pathfix_draw_x[0],
                              (uint16)s_ui_dc.pathfix_draw_y[0], RGB565_GREEN);
            if (draw_count > 1u)
            {
                ips200_draw_point((uint16)s_ui_dc.pathfix_draw_x[draw_count - 1u],
                                  (uint16)s_ui_dc.pathfix_draw_y[draw_count - 1u],
                                  RGB565_PURPLE);
            }
        }
        for (i = 0u; (i < (uint16)s_ui_dc.pathfix_elem_count) && (i < DUALCORE_PATHFIX_ELEM_MAX); i++)
        {
            UI_PathFix_DrawCrossMarker((uint16)s_ui_dc.pathfix_elem_x[i],
                                       (uint16)s_ui_dc.pathfix_elem_y[i],
                                       x_off, y_off, w, h,
                                       RGB565_BLUE, 3u);
        }
        if ((draw_count > 0u) && (draw_sel < draw_count))
        {
            UI_PathFix_DrawSelectedMarker((uint16)s_ui_dc.pathfix_draw_x[draw_sel],
                                          (uint16)s_ui_dc.pathfix_draw_y[draw_sel],
                                          x_off, y_off, w, h);
        }
    }
    }
#else
    pathfix_active = g_nag_pathfix.active;
    pathfix_loaded = g_nag_pathfix.loaded;
    pathfix_dirty = g_nag_pathfix.dirty;
    pathfix_select = g_nag_pathfix.select_index;
    pathfix_count = (uint32)g_nag_pathfix.point_count;
    replay_subject = g_nag_pathfix.replay_subject;
    if (replay_subject == 0u)
    {
        replay_subject = g_nag_replay_subject;
    }
    if ((pathfix_loaded != 0u) && (pathfix_select < g_nag_pathfix.point_count))
    {
        pathfix_yaw_x100 = Nav_read[pathfix_select];
    }
    if (pathfix_loaded != 0u)
    {
        Nag_PathFix_DrawViewport(136u, (uint16)ROW_5, 88u, 112u);
    }
#endif

    ips200_show_string(0, ROW_4, "Play:");
    ips200_show_uint(40, ROW_4, (uint32)replay_subject, 1);

    if (pathfix_loaded == 0u)
    {
        ips200_show_string(0, ROW_5, "Subj Empty");
        ips200_show_string(0, ROW_6, "Record first");
    }
    else
    {
        ips200_show_string(0, ROW_5, "Pt:");
        ips200_show_uint(24, ROW_5, (uint32)pathfix_select, 5);
        ips200_show_string(72, ROW_5, "/");
        ips200_show_uint(80, ROW_5, pathfix_count, 5);
        ips200_show_string(0, ROW_6, "Yaw:");
        ips200_show_float(40, ROW_6, (float)pathfix_yaw_x100 / 100.0f, 4, 2);
        ips200_show_string(0, ROW_7, pathfix_dirty ? "Dirty:Y" : "Dirty:N");
    }

    ips200_show_string(0, ROW_9, "K1 Anchor");
    ips200_show_string(0, ROW_10, "K2 Yaw-5");
    ips200_show_string(0, ROW_11, "K3 Yaw+5");
    ips200_show_string(0, ROW_12, "K4 SaveExit");

    (void)pathfix_active;
}
void ACT_2_6_1()
{
    ReadPos[0] = '2';
    ReadPos[1] = '.';
    ReadPos[2] = '6';
    ReadPos[3] = '.';
    ReadPos[4] = '1';
    ReadPos[5] = 0x00;
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

/** 二级 Run 列表：0=Launch 1=Save 2=Config 3=Jump 4=GyroBias 5=RecSubj 6=PlaySubj */
static void GUI_Run_ShowSubmenuList(uint8 selected_row_index)
{
    int16 ay;

    GUI_Display_Level2_Common3();

    ips200_show_string(80, ROW_8,  " Launch ");
    ips200_show_string(80, ROW_9,  " Save   ");
    ips200_show_string(80, ROW_10, " Config ");
    ips200_show_string(80, ROW_11, " Jump   ");
    ips200_show_string(80, ROW_12, "GyroBias");
    ips200_show_string(80, ROW_13, "RecSubj ");
    ips200_show_string(80, ROW_14, "PlaySubj");

    switch (selected_row_index)
    {
    default:
        ay = ROW_8;
        break;
    case 1u:
        ay = ROW_9;
        break;
    case 2u:
        ay = ROW_10;
        break;
    case 3u:
        ay = ROW_11;
        break;
    case 4u:
        ay = ROW_12;
        break;
    case 5u:
        ay = ROW_13;
        break;
    case 6u:
        ay = ROW_14;
        break;
    }
    ips200_show_string(48, ay, "-->");
    ips200_show_string(152, ay, "<--");
    if (selected_row_index == 1u)
    {
        ips200_show_string(24, ROW_15, "K3:save all");
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

void GUI_3_4(void)
{
    GUI_Run_ShowSubmenuList(3u);
}
void ACT_3_4()
{
    ReadPos[0] = '3';
    ReadPos[1] = '.';
    ReadPos[2] = '4';
    ReadPos[3] = 0x00;
    ReadPos[4] = 0x00;
}

void GUI_3_5(void)
{
    GUI_Run_ShowSubmenuList(4u);
}
void ACT_3_5()
{
    ReadPos[0] = '3';
    ReadPos[1] = '.';
    ReadPos[2] = '5';
    ReadPos[3] = 0x00;
    ReadPos[4] = 0x00;
}

void GUI_3_6(void)
{
    GUI_Run_ShowSubmenuList(5u);
}
void ACT_3_6()
{
    ReadPos[0] = '3';
    ReadPos[1] = '.';
    ReadPos[2] = '6';
    ReadPos[3] = 0x00;
    ReadPos[4] = 0x00;
}

void GUI_3_7(void)
{
    GUI_Run_ShowSubmenuList(6u);
}
void ACT_3_7()
{
    ReadPos[0] = '3';
    ReadPos[1] = '.';
    ReadPos[2] = '7';
    ReadPos[3] = 0x00;
    ReadPos[4] = 0x00;
}

void GUI_3_3_1(void)
{
    uint8 field_index = Menu_GetRunConfigFieldIndex();
    uint8 input_remote = 0u;
    uint8 vofa_enable = 0u;
    uint8 vofa_group = 0u;
    uint8 fusion_enable = 0u;
    uint8 odo_slip_enable = 0u;
    uint8 init_leg_long_sel = 1u;

    GUI_Display_Level2_Common3();
    ips200_show_string(56, ROW_3, "Config");
    ips200_draw_line(16, ROW_15, 223, ROW_15, IPS200_DEFAULT_PENCOLOR);

#if defined(CY_CORE_CM7_1)
    dualcore_ctrl_to_ui_pull(&s_ui_dc);
    input_remote = s_ui_dc.menu_input_remote_first;
    vofa_enable = s_ui_dc.menu_vofa_enable;
    vofa_group = (uint8)(s_ui_dc.nag_vofa_group % NAG_VOFA_GROUP_COUNT);
    fusion_enable = s_ui_dc.menu_nav_fusion_enable;
    odo_slip_enable = s_ui_dc.menu_odo_slip_enable;
    init_leg_long_sel = s_ui_dc.menu_init_leg_long_sel;
#else
    input_remote = g_menu_input_remote_first;
    vofa_enable = g_menu_vofa_enable;
    vofa_group = (uint8)(Nag_Vofa_Group % NAG_VOFA_GROUP_COUNT);
    fusion_enable = g_menu_nav_fusion_enable;
    odo_slip_enable = g_menu_odo_slip_enable;
    init_leg_long_sel = g_menu_init_leg_long_sel;
#endif

    if (field_index == Run_Config_Field_InputMode)
    {
        ips200_show_string(0, ROW_5, "->");
    }
    else
    {
        ips200_show_string(0, ROW_5, "  ");
    }
    ips200_show_string(16, ROW_5, "InputMode:");
    if (input_remote != 0u)
    {
        ips200_show_string(112, ROW_5, "Remote");
    }
    else
    {
        ips200_show_string(112, ROW_5, "Key   ");
    }

    if (field_index == Run_Config_Field_VofaEnable)
    {
        ips200_show_string(0, ROW_7, "->");
    }
    else
    {
        ips200_show_string(0, ROW_7, "  ");
    }
    ips200_show_string(16, ROW_7, "VofaEnable:");
    if (vofa_enable != 0u)
    {
        ips200_show_string(112, ROW_7, "On ");
    }
    else
    {
        ips200_show_string(112, ROW_7, "Off");
    }

    if (field_index == Run_Config_Field_VofaGroup)
    {
        ips200_show_string(0, ROW_9, "->");
    }
    else
    {
        ips200_show_string(0, ROW_9, "  ");
    }
    ips200_show_string(16, ROW_9, "VofaGroup:");
    switch (vofa_group)
    {
    case 1u:
        ips200_show_string(112, ROW_9, "1:Spd");
        break;
    case 2u:
        ips200_show_string(112, ROW_9, "2:Fus");
        break;
    case 3u:
        ips200_show_string(112, ROW_9, "3:Odo");
        break;
    case 4u:
        ips200_show_string(112, ROW_9, "4:Spn");
        break;
    default:
        ips200_show_string(112, ROW_9, "0:IMU");
        break;
    }

    if (field_index == Run_Config_Field_FusionEnable)
    {
        ips200_show_string(0, ROW_11, "->");
    }
    else
    {
        ips200_show_string(0, ROW_11, "  ");
    }
    ips200_show_string(16, ROW_11, "FusionEn:");
    if (fusion_enable != 0u)
    {
        ips200_show_string(112, ROW_11, "On ");
    }
    else
    {
        ips200_show_string(112, ROW_11, "Off");
    }

#if Nag_OdoSlip_Enable
    if (field_index == Run_Config_Field_OdoSlipEnable)
    {
        ips200_show_string(0, ROW_13, "->");
    }
    else
    {
        ips200_show_string(0, ROW_13, "  ");
    }
    ips200_show_string(16, ROW_13, "OdoSlipEn:");
    if (odo_slip_enable != 0u)
    {
        ips200_show_string(112, ROW_13, "On ");
    }
    else
    {
        ips200_show_string(112, ROW_13, "Off");
    }
#endif

    if (field_index == Run_Config_Field_InitLegLong)
    {
        ips200_show_string(0, ROW_14, "->");
    }
    else
    {
        ips200_show_string(0, ROW_14, "  ");
    }
    ips200_show_string(16, ROW_14, "InitLeg:");
    if (init_leg_long_sel == 0u)
    {
        ips200_show_string(112, ROW_14, "3.5");
    }
    else
    {
        ips200_show_string(112, ROW_14, "5.5");
    }

    ips200_show_string(8, ROW_16, "K1:nxt K2:chg K4:bk");
}

void ACT_3_3_1()
{
    ReadPos[0] = '3';
    ReadPos[1] = '.';
    ReadPos[2] = '3';
    ReadPos[3] = '.';
    ReadPos[4] = '1';
    ReadPos[5] = 0x00;
}

static float GUI_RunJumpParamValue(uint8 field_index)
{
#if defined(CY_CORE_CM7_1)
    switch (field_index)
    {
    case Run_Jump_Field_Takeoff_P:
        return s_ui_dc.jump_takeoff_p;
    case Run_Jump_Field_Retract_P:
        return s_ui_dc.jump_retract_p;
    case Run_Jump_Field_Prepare_P:
        return s_ui_dc.jump_prepare_p;
    case Run_Jump_Field_Buffer_P:
        return s_ui_dc.jump_buffer_p;
    case Run_Jump_Field_Takeoff_T:
        return s_ui_dc.jump_stage_takeoff_cycles;
    case Run_Jump_Field_Retract_T:
        return s_ui_dc.jump_stage_retract_cycles;
    case Run_Jump_Field_Prepare_T:
        return s_ui_dc.jump_stage_prepare_cycles;
    case Run_Jump_Field_Buffer_T:
        return s_ui_dc.jump_stage_buffer_cycles;
    case Run_Jump_Field_Buffer_Step:
        return s_ui_dc.jump_buffer_step_p_max;
    default:
        return 0.0f;
    }
#else
    return JumpParamGet(field_index);
#endif
}

void GUI_3_4_1(void)
{
    static const char *const labels[Run_Jump_Param_Count] =
    {
        "TkfP", "RetP", "PrepP", "BufP",
        "TkfT", "RetT", "PrepT", "BufT", "BufSp"
    };
    static const int16 rows[Run_Jump_Param_Count] =
    {
        ROW_4, ROW_5, ROW_6, ROW_7, ROW_8, ROW_9, ROW_10, ROW_11, ROW_12
    };
    uint8 field_index = 0u;
    uint8 selected = Menu_GetRunJumpFieldIndex();

    GUI_Display_Level2_Common3();
    ips200_show_string(64, ROW_3, "Jump");
    ips200_draw_line(16, ROW_14, 223, ROW_14, IPS200_DEFAULT_PENCOLOR);

#if defined(CY_CORE_CM7_1)
    dualcore_ctrl_to_ui_pull(&s_ui_dc);
#endif

    for (field_index = 0u; field_index < Run_Jump_Param_Count; field_index++)
    {
        float value = GUI_RunJumpParamValue(field_index);

        if (selected == field_index)
        {
            ips200_show_string(0, rows[field_index], "->");
        }
        else
        {
            ips200_show_string(0, rows[field_index], "  ");
        }
        ips200_show_string(16, rows[field_index], labels[field_index]);
        if (field_index == Run_Jump_Field_Buffer_Step)
        {
            ips200_show_float(96, rows[field_index], (double)value, 5, 2);
        }
        else
        {
            ips200_show_float(96, rows[field_index], (double)value, 5, 1);
        }
    }

    ips200_show_string(8, ROW_15, "K1:nxt K2:+ K3:- K4:bk");
}

void ACT_3_4_1()
{
    ReadPos[0] = '3';
    ReadPos[1] = '.';
    ReadPos[2] = '4';
    ReadPos[3] = '.';
    ReadPos[4] = '1';
    ReadPos[5] = 0x00;
}

void GUI_3_5_1(void)
{
    float bias_dps = 0.0f;
    float yaw_drift = 0.0f;
    float yaw_start = 0.0f;
    uint8 calib_state = GYRO_BIAS_CALIB_IDLE;
    uint8 remain_s = 0u;
    const char *state_text = "Idle";

#if defined(CY_CORE_CM7_1)
    dualcore_ctrl_to_ui_pull(&s_ui_dc);
    bias_dps = s_ui_dc.gyro_z_bias_comp * 180.0f / (float)PI;
    yaw_drift = s_ui_dc.yaw_drift_10s_deg;
    yaw_start = s_ui_dc.gyro_bias_calib_yaw_start_deg;
    calib_state = s_ui_dc.gyro_bias_calib_state;
    remain_s = s_ui_dc.gyro_bias_calib_remain_s;
#else
    bias_dps = GyroBias_GetComp() * 180.0f / (float)PI;
    yaw_drift = GyroBias_GetYawDrift10sDeg();
    yaw_start = GyroBias_GetYawStartDeg();
    calib_state = GyroBias_GetCalibState();
    remain_s = GyroBias_GetRemainSec();
#endif

    if (calib_state == GYRO_BIAS_CALIB_RUNNING)
    {
        state_text = "Run";
    }
    else if (calib_state == GYRO_BIAS_CALIB_DONE)
    {
        state_text = "Done";
    }

    GUI_Display_Level2_Common3();
    ips200_show_string(40, ROW_3, "GyroBias");
    ips200_draw_line(16, ROW_14, 223, ROW_14, IPS200_DEFAULT_PENCOLOR);

    ips200_show_string(8, ROW_5, "Bias(dps):");
    ips200_show_float(96, ROW_5, (double)bias_dps, 4, 3);

    ips200_show_string(8, ROW_7, "State:");
    ips200_show_string(64, ROW_7, state_text);

    ips200_show_string(8, ROW_9, "Remain(s):");
    ips200_show_uint(96, ROW_9, (uint32)remain_s, 2);

    if (calib_state != GYRO_BIAS_CALIB_IDLE)
    {
        ips200_show_string(8, ROW_10, "YawStart:");
        ips200_show_float(96, ROW_10, (double)yaw_start, 4, 2);
    }

    ips200_show_string(8, ROW_11, "YawDrift:");
    ips200_show_float(96, ROW_11, (double)yaw_drift, 4, 2);

    ips200_show_string(8, ROW_13, "Drift=end-start");
    ips200_show_string(8, ROW_15, "K3:start K4:back");
    ips200_show_string(8, ROW_16, "Save:Run->Save K3");
}

void ACT_3_5_1()
{
    ReadPos[0] = '3';
    ReadPos[1] = '.';
    ReadPos[2] = '5';
    ReadPos[3] = '.';
    ReadPos[4] = '1';
    ReadPos[5] = 0x00;
}

static void GUI_RunSubjectPage(const char *title, uint8 confirmed_subject, uint8 preview_subject)
{
    GUI_Display_Level2_Common3();
    ips200_show_string(40, ROW_3, title);
    ips200_draw_line(16, ROW_14, 223, ROW_14, IPS200_DEFAULT_PENCOLOR);

    ips200_show_string(8, ROW_6, "Saved:");
    ips200_show_uint(72, ROW_6, (uint32)confirmed_subject, 1);

    ips200_show_string(8, ROW_8, "Select:");
    ips200_show_uint(72, ROW_8, (uint32)preview_subject, 1);

    ips200_show_string(8, ROW_10, "Range 1-3");

    ips200_show_string(8, ROW_15, "K1/K2:sel K3:ok");
    ips200_show_string(8, ROW_16, "K4:back");
}

void GUI_3_6_1(void)
{
    uint8 confirmed = 1u;
    uint8 preview = Menu_GetRunRecSubjPreview();

#if defined(CY_CORE_CM7_1)
    dualcore_ctrl_to_ui_pull(&s_ui_dc);
    confirmed = s_ui_dc.nag_record_subject;
#else
    confirmed = g_nag_record_subject;
#endif

    GUI_RunSubjectPage("RecSubj", confirmed, preview);
}

void ACT_3_6_1()
{
    ReadPos[0] = '3';
    ReadPos[1] = '.';
    ReadPos[2] = '6';
    ReadPos[3] = '.';
    ReadPos[4] = '1';
    ReadPos[5] = 0x00;
}

void GUI_3_7_1(void)
{
    uint8 confirmed = 1u;
    uint8 preview = Menu_GetRunPlaySubjPreview();

#if defined(CY_CORE_CM7_1)
    dualcore_ctrl_to_ui_pull(&s_ui_dc);
    confirmed = s_ui_dc.nag_replay_subject;
#else
    confirmed = g_nag_replay_subject;
#endif

    GUI_RunSubjectPage("PlaySubj", confirmed, preview);
}

void ACT_3_7_1()
{
    ReadPos[0] = '3';
    ReadPos[1] = '.';
    ReadPos[2] = '7';
    ReadPos[3] = '.';
    ReadPos[4] = '1';
    ReadPos[5] = 0x00;
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
    case Nag_Launch_Field_TurnIn_Spd:
        return s_ui_dc.nag_enter_turn_target_speed;
    case Nag_Launch_Field_TurnIn_Dec:
        return s_ui_dc.nag_enter_turn_pre_decel_dist_cm;
    case Nag_Launch_Field_TurnOut_Spd:
        return s_ui_dc.nag_exit_turn_recovery_speed;
    case Nag_Launch_Field_TurnOut_Acc:
        return s_ui_dc.nag_exit_turn_pre_accel_dist_cm;
    case Nag_Launch_Field_Cone_Spd:
        return s_ui_dc.nag_enter_cones_target_speed;
    case Nag_Launch_Field_Cone_Dec:
        return s_ui_dc.nag_enter_cones_pre_decel_dist_cm;
    case Nag_Launch_Field_Stair_Spd:
        return s_ui_dc.nag_enter_stair_target_speed;
    case Nag_Launch_Field_Stair_Dec:
        return s_ui_dc.nag_enter_stair_pre_decel_dist_cm;
    case Nag_Launch_Field_BridgeIn_Spd:
        return s_ui_dc.nag_enter_bridge_target_speed;
    case Nag_Launch_Field_BridgeIn_Dec:
        return s_ui_dc.nag_enter_bridge_pre_decel_dist_cm;
    case Nag_Launch_Field_Bump_Dur:
        return s_ui_dc.nag_bump_duration_sec;
    case Nag_Launch_Field_Bump_Spd:
        return s_ui_dc.nag_enter_bump_target_speed;
    case Nag_Launch_Field_Stair2_Spd:
        return s_ui_dc.nag_enter_stair2_target_speed;
    case Nag_Launch_Field_Spin_Rate:
        return s_ui_dc.spin_rate_max_dps;
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
        "BaseSpd", "SpinSpd", "SpinDec",
        "TrnInSp", "TrnInDc", "TrnOutSp", "TrnOutAc",
        "ConeSpd", "ConeDec", "SpinRt", "StairSp", "StairDc",
        "BrgInSp", "BrgInDc", "BumpSec", "BumpSpd", "St2Spd"
    };
    static const int16 rows[8] =
    {
        ROW_4, ROW_5, ROW_6, ROW_7, ROW_8, ROW_9, ROW_10, ROW_11
    };
    static const int16 left_arrow_x = 0;
    static const int16 left_label_x = 16;
    static const int16 left_val_x = 74;
    static const int16 right_arrow_x = 120;
    static const int16 right_label_x = 132;
    static const int16 right_val_x = 180;
    static const uint8 launch_fields_per_page = 16u;
    uint8 field_index = 0u;
    uint8 selected = Menu_GetRunLaunchFieldIndex();
    uint8 display_start = 0u;
    uint8 display_end = 0u;
    uint8 row = 0u;
    int16 arrow_x = 0;
    int16 label_x = 0;
    int16 val_x = 0;

    if (selected >= launch_fields_per_page)
    {
        display_start = (uint8)(Nag_Run_Launch_Param_Count - launch_fields_per_page);
    }
    display_end = (uint8)(display_start + launch_fields_per_page);
    if (display_end > Nag_Run_Launch_Param_Count)
    {
        display_end = Nag_Run_Launch_Param_Count;
    }

    GUI_Display_Level2_Common3();
    ips200_show_string(56, ROW_3, "Launch");
    ips200_draw_line(16, ROW_12, 223, ROW_12, IPS200_DEFAULT_PENCOLOR);

#if defined(CY_CORE_CM7_1)
    dualcore_ctrl_to_ui_pull(&s_ui_dc);
#endif

    for (field_index = display_start; field_index < display_end; field_index++)
    {
        float value = GUI_RunLaunchParamValue(field_index);
        uint8 slot = (uint8)(field_index - display_start);

        if (slot < 8u)
        {
            row = slot;
            arrow_x = left_arrow_x;
            label_x = left_label_x;
            val_x = left_val_x;
        }
        else
        {
            row = (uint8)(slot - 8u);
            arrow_x = right_arrow_x;
            label_x = right_label_x;
            val_x = right_val_x;
        }

        /* 每帧全页重绘时须擦除旧箭头，否则 KEY1 切换后上一行 "->" 仍残留 */
        if (selected == field_index)
        {
            ips200_show_string(arrow_x, rows[row], "->");
        }
        else
        {
            ips200_show_string(arrow_x, rows[row], "  ");
        }
        ips200_show_string(label_x, rows[row], labels[field_index]);
        /* 8x16 字体宽 8px；240 屏右列数值起点须 <=184，速度用 4 位整数避免越界 */
        if (Nag_LaunchParamIsSpeed(field_index))
        {
            ips200_show_float(val_x, rows[row], (double)value, 4, 1);
        }
        else if (Nag_LaunchParamIsSpinRate(field_index))
        {
            ips200_show_int(val_x, rows[row], (int32)value, 4);
        }
        else if (Nag_LaunchParamIsBumpDuration(field_index))
        {
            ips200_show_int(val_x, rows[row], (int32)value, 3);
        }
        else
        {
            ips200_show_int(val_x, rows[row], (int32)value, 4);
        }
    }

    ips200_show_string(8, ROW_13, "K1:nxt K2:+ K3:- K4:bk");
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

/** Image 三级列表：与 GUI_Run_ShowSubmenuList 同构；selected 0=Step 1=Bridge 2=Bumpy */
static void GUI_Image_ShowSubmenuList(uint8 selected_row_index)
{
    int16 ay;

    ips200_draw_line(0, 20, 239, 20, IPS200_DEFAULT_PENCOLOR);
    ips200_show_string(0, ROW_1, "Image");
    GUI_Display_FPS();

    ips200_show_string(80, ROW_6, " Step   ");
    ips200_show_string(80, ROW_8, " Bridge ");
    ips200_show_string(80, ROW_10, " Bumpy  ");

    switch (selected_row_index)
    {
    default:
        ay = ROW_6;
        break;
    case 1u:
        ay = ROW_8;
        break;
    case 2u:
        ay = ROW_10;
        break;
    }
    ips200_show_string(48, ay, "-->");
    ips200_show_string(152, ay, "<--");

#if defined(CY_CORE_CM7_1)
    switch (image_ae_session_get_state())
    {
    case IMAGE_AE_RUNNING:
        ips200_show_string(0, ROW_18, "AE:Run   ");
        break;
    case IMAGE_AE_DONE:
        ips200_show_string(0, ROW_18, "AE:Done  ");
        break;
    case IMAGE_AE_FAILED:
        ips200_show_string(0, ROW_18, "AE:Fail  ");
        break;
    default:
        ips200_show_string(0, ROW_18, "AE:Idle  ");
        break;
    }
    ips200_show_string(0, ROW_19, "Exp:");
    ips200_show_int(40, ROW_19, (int32)image_camera_exposure, 5);
    ips200_show_string(96, ROW_19, "Lt:");
    ips200_show_int(120, ROW_19, (int32)test_printf_light, 6);
#endif
}

void GUI_2_1_1(void) // Image 三级列表：Step 行
{
    GUI_Image_ShowSubmenuList(0u);
}
void ACT_2_1_1()
{
    ReadPos[0] = '2';
    ReadPos[1] = '.';
    ReadPos[2] = '1';
    ReadPos[3] = '.';
    ReadPos[4] = '1';
    ReadPos[5] = 0x00;
}

void GUI_2_1_2(void) // Image 三级列表：Bridge 行
{
    GUI_Image_ShowSubmenuList(1u);
}
void ACT_2_1_2()
{
    ReadPos[0] = '2';
    ReadPos[1] = '.';
    ReadPos[2] = '1';
    ReadPos[3] = '.';
    ReadPos[4] = '2';
    ReadPos[5] = 0x00;
}

void GUI_2_1_3(void) // Image 三级列表：Bumpy 行
{
    GUI_Image_ShowSubmenuList(2u);
}
void ACT_2_1_3()
{
    ReadPos[0] = '2';
    ReadPos[1] = '.';
    ReadPos[2] = '1';
    ReadPos[3] = '.';
    ReadPos[4] = '3';
    ReadPos[5] = 0x00;
}

void GUI_2_1_1_1(void) // 台阶检测
{
    GUI_Display_Level2_Common2();
    ips200_show_string(56, ROW_3, "Step Detect");

    ips200_show_string(0, ROW_7, "Detected:");
    ips200_show_string(88, ROW_7, step_data.detected ? "Yes" : "No ");

    ips200_show_string(0, ROW_6, "压缩灰度");
#if defined(CY_CORE_CM7_1)
    ips200_show_string(120, ROW_6, "Bot:red");
#endif
    ips200_show_string(0, ROW_8, "Distance:");
    ips200_show_float(88, ROW_8, step_data.distance_cm, 3, 1);
    ips200_show_string(136, ROW_8, "cm");

    ips200_show_string(0, ROW_9, "Height:");
    ips200_show_uint(88, ROW_9, step_data.step_height_pix, 3);
    ips200_show_string(136, ROW_9, "pix");

#if defined(CY_CORE_CM7_1)
    step_debug_show(0, ROW_10);
    ips200_show_string(0, ROW_5, "BotRow:");
    ips200_show_uint(56, ROW_5, step_data.bottom_row_raw, 3);
#endif
}
void ACT_2_1_1_1()
{
    ReadPos[0] = '2';
    ReadPos[1] = '.';
    ReadPos[2] = '1';
    ReadPos[3] = '.';
    ReadPos[4] = '1';
    ReadPos[5] = '.';
    ReadPos[6] = '1';
    ReadPos[7] = 0x00;
}

void GUI_2_1_2_1(void) // 单边桥检测
{
    GUI_Display_Level2_Common2();
    ips200_show_string(48, ROW_2, "Single Bridge");
    ips200_show_string(0, ROW_3, "L:red R:grn C:blu");

#if defined(CY_CORE_CM7_1)
    single_bridge_debug_show(0, ROW_10, hd_threshold);

    ips200_show_string(0, ROW_4, "Pin:");
    if (single_bridge_get_pin_left() != 0u)
    {
        ips200_show_string(32, ROW_4, "L");
    }
    if (single_bridge_get_pin_right() != 0u)
    {
        ips200_show_string(48, ROW_4, "R");
    }
    ips200_show_string(64, ROW_4, "Det:");
    if (single_bridge_get_enter_ready() != 0u)
    {
        ips200_show_string(96, ROW_4, "In");
    }
    if (single_bridge_get_exit_ready() != 0u)
    {
        ips200_show_string(112, ROW_4, "Out");
    }

    ips200_show_string(0, ROW_5, "Vld:");
    ips200_show_int(32, ROW_5, (int32)single_bridge_get_track_valid(), 1);
    ips200_show_string(48, ROW_5, "N:");
    ips200_show_int(64, ROW_5, (int32)single_bridge_get_valid_row_count(), 2);
    ips200_show_string(0, ROW_6, "Wavg:");
    ips200_show_int(40, ROW_6, (int32)single_bridge_get_road_w_avg(), 3);
    ips200_show_string(88, ROW_6, "Wm:");
    ips200_show_int(112, ROW_6, single_bridge_get_width_max(), 2);

    ips200_show_string(0, ROW_7, "Th:");
    ips200_show_int(24, ROW_7, hd_threshold, 3);
    ips200_show_string(64, ROW_7, "End:");
    ips200_show_int(96, ROW_7, end_line, 3);

    ips200_show_string(0, ROW_8, "Err:");
    ips200_show_float(32, ROW_8, Cammer_Err, 4, 1);
#else
    ips200_show_string(0, ROW_7, "Bridge");
    ips200_show_string(0, ROW_8, "UI on M7_1");
#endif
}
void ACT_2_1_2_1()
{
    ReadPos[0] = '2';
    ReadPos[1] = '.';
    ReadPos[2] = '1';
    ReadPos[3] = '.';
    ReadPos[4] = '2';
    ReadPos[5] = '.';
    ReadPos[6] = '1';
    ReadPos[7] = 0x00;
}

void GUI_2_1_3_1(void) // 颠簸路段检测
{
    GUI_Display_Level2_Common2();
    ips200_show_string(56, ROW_3, "Bumpy Road");

    ips200_show_string(0, ROW_8, "Bumpy Road");
    ips200_show_string(0, ROW_9, "Reserved Page");
    ips200_show_string(0, ROW_11, "Use this page");
    ips200_show_string(0, ROW_12, "for future image");
    ips200_show_string(0, ROW_13, "detection logic.");
}
void ACT_2_1_3_1()
{
    ReadPos[0] = '2';
    ReadPos[1] = '.';
    ReadPos[2] = '1';
    ReadPos[3] = '.';
    ReadPos[4] = '3';
    ReadPos[5] = '.';
    ReadPos[6] = '1';
    ReadPos[7] = 0x00;
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