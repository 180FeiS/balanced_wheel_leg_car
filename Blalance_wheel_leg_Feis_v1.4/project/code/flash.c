#include "zf_common_headfile.h"
#include "flash.h"
#include "my_gps.h"
#include "control.h"
#include "navigation.h"
#include "ekf.h"

static uint8 nag_flash_index_read = 0;

#define GPS_POINTS_PAGE 48u
#define GPS_POINTS_MAGIC 0x47505350u
#define GPS_POINTS_VERSION 2u
#define GPS_POINTS_HEADER_WORDS 5u
#define GPS_POINTS_LAT_WORDS (GPS_POINT_MAX * 2u)
#define GPS_POINTS_LON_WORDS (GPS_POINT_MAX * 2u)
#define GPS_POINTS_LAT_BASE GPS_POINTS_HEADER_WORDS
#define GPS_POINTS_LON_BASE (GPS_POINTS_LAT_BASE + GPS_POINTS_LAT_WORDS)
#define GPS_POINTS_ELEMENT_BASE (GPS_POINTS_LON_BASE + GPS_POINTS_LON_WORDS)

static uint32 flash_RunLaunchSpeedChecksum(uint32 speed_raw)
{
    return Nag_Run_Launch_Speed_Magic ^ Nag_Run_Launch_Speed_Version ^ speed_raw;
}

#define Nag_Run_Launch_Param_Count_V2 7u

static void flash_RunLaunchParamsPack(void)
{
    flash_union_buffer[3].float_type = run_launch_speed;
    flash_union_buffer[4].float_type = nag_spin_target_speed;
    flash_union_buffer[5].float_type = nag_spin_pre_decel_dist_cm;
    flash_union_buffer[6].float_type = nag_enter_turn_target_speed;
    flash_union_buffer[7].float_type = nag_enter_turn_pre_decel_dist_cm;
    flash_union_buffer[8].float_type = nag_exit_turn_recovery_speed;
    flash_union_buffer[9].float_type = nag_exit_turn_pre_accel_dist_cm;
    flash_union_buffer[10].float_type = nag_enter_cones_target_speed;
    flash_union_buffer[11].float_type = nag_enter_cones_pre_decel_dist_cm;
    flash_union_buffer[12].float_type = nag_enter_stair_target_speed;
    flash_union_buffer[13].float_type = nag_enter_stair_pre_decel_dist_cm;
    flash_union_buffer[14].float_type = nag_enter_bridge_target_speed;
    flash_union_buffer[15].float_type = nag_enter_bridge_pre_decel_dist_cm;
    flash_union_buffer[16].float_type = spin_rate_max_dps;
    flash_union_buffer[17].uint32_type = (g_menu_input_remote_first != 0u) ? 1u : 0u;
    flash_union_buffer[18].uint32_type = (g_menu_vofa_enable != 0u) ? 1u : 0u;
    flash_union_buffer[19].uint32_type = (uint32)(Nag_Vofa_Group % NAG_VOFA_GROUP_COUNT);
    flash_union_buffer[20].uint32_type = (g_menu_nav_fusion_enable != 0u) ? 1u : 0u;
    flash_union_buffer[21].float_type = nag_bump_duration_sec;
    flash_union_buffer[22].uint32_type = (g_menu_odo_slip_enable != 0u) ? 1u : 0u;
}

static uint32 flash_RunLaunchParamsChecksumEx(uint32 version, uint8 param_count)
{
    uint32 checksum = Nag_Run_Launch_Speed_Magic ^ version;
    uint8 index = 0u;

    for (index = 0u; index < param_count; index++)
    {
        checksum ^= flash_union_buffer[3u + index].uint32_type;
    }
    return checksum;
}

static uint32 flash_RunLaunchParamsChecksum(void)
{
    return flash_RunLaunchParamsChecksumEx(Nag_Run_Launch_Params_Version_V13,
                                           Nag_Run_Launch_Config_Word_Count);
}

static void flash_RunLaunchBridgeApplyDefaults(void)
{
    nag_enter_bridge_target_speed = Nag_EnterBridge_Target_Speed_Default;
    nag_enter_bridge_pre_decel_dist_cm = Nag_EnterBridge_PreDecel_Dist_cm_Default;
}

static void flash_RunLaunchStairSpeedApplyDefaults(void)
{
    nag_enter_stair_target_speed = Nag_EnterStair_Target_Speed_Default;
}

static void flash_RunLaunchParamsUnpackV8(void)
{
    run_launch_speed = flash_union_buffer[3].float_type;
    nag_spin_target_speed = flash_union_buffer[4].float_type;
    nag_spin_pre_decel_dist_cm = flash_union_buffer[5].float_type;
    nag_enter_turn_target_speed = flash_union_buffer[6].float_type;
    nag_enter_turn_pre_decel_dist_cm = flash_union_buffer[7].float_type;
    nag_exit_turn_recovery_speed = flash_union_buffer[8].float_type;
    nag_exit_turn_pre_accel_dist_cm = flash_union_buffer[9].float_type;
    nag_enter_cones_target_speed = flash_union_buffer[10].float_type;
    nag_enter_cones_pre_decel_dist_cm = flash_union_buffer[11].float_type;
    nag_enter_stair_pre_decel_dist_cm = flash_union_buffer[12].float_type;
    nag_enter_bridge_target_speed = flash_union_buffer[13].float_type;
    nag_enter_bridge_pre_decel_dist_cm = flash_union_buffer[14].float_type;
    spin_set_rate_max_dps(flash_union_buffer[15].float_type);
    g_menu_input_remote_first = (uint8)(flash_union_buffer[16].uint32_type & 1u);
    g_menu_vofa_enable = (uint8)(flash_union_buffer[17].uint32_type & 1u);
    flash_RunLaunchStairSpeedApplyDefaults();
    g_menu_odo_slip_enable = 0u;
}

static void flash_RunLaunchParamsUnpackV9(void)
{
    flash_RunLaunchParamsUnpackV8();
    Nag_Vofa_Group = (uint8)(flash_union_buffer[18].uint32_type % NAG_VOFA_GROUP_COUNT);
    g_menu_odo_slip_enable = 0u;
}

static void flash_RunLaunchParamsUnpackV10(void)
{
    flash_RunLaunchParamsUnpackV9();
    g_menu_nav_fusion_enable = (uint8)(flash_union_buffer[19].uint32_type & 1u);
    flash_RunLaunchStairSpeedApplyDefaults();
    g_menu_odo_slip_enable = 0u;
}

static void flash_RunLaunchParamsUnpackV11(void)
{
    run_launch_speed = flash_union_buffer[3].float_type;
    nag_spin_target_speed = flash_union_buffer[4].float_type;
    nag_spin_pre_decel_dist_cm = flash_union_buffer[5].float_type;
    nag_enter_turn_target_speed = flash_union_buffer[6].float_type;
    nag_enter_turn_pre_decel_dist_cm = flash_union_buffer[7].float_type;
    nag_exit_turn_recovery_speed = flash_union_buffer[8].float_type;
    nag_exit_turn_pre_accel_dist_cm = flash_union_buffer[9].float_type;
    nag_enter_cones_target_speed = flash_union_buffer[10].float_type;
    nag_enter_cones_pre_decel_dist_cm = flash_union_buffer[11].float_type;
    nag_enter_stair_target_speed = flash_union_buffer[12].float_type;
    nag_enter_stair_pre_decel_dist_cm = flash_union_buffer[13].float_type;
    nag_enter_bridge_target_speed = flash_union_buffer[14].float_type;
    nag_enter_bridge_pre_decel_dist_cm = flash_union_buffer[15].float_type;
    spin_set_rate_max_dps(flash_union_buffer[16].float_type);
    g_menu_input_remote_first = (uint8)(flash_union_buffer[17].uint32_type & 1u);
    g_menu_vofa_enable = (uint8)(flash_union_buffer[18].uint32_type & 1u);
    Nag_Vofa_Group = (uint8)(flash_union_buffer[19].uint32_type % NAG_VOFA_GROUP_COUNT);
    g_menu_nav_fusion_enable = (uint8)(flash_union_buffer[20].uint32_type & 1u);
    nag_bump_duration_sec = Nag_Bump_Duration_Sec_Default;
    g_menu_odo_slip_enable = 0u;
}

static void flash_RunLaunchParamsUnpackV12(void)
{
    flash_RunLaunchParamsUnpackV11();
    nag_bump_duration_sec = flash_union_buffer[21].float_type;
    if (nag_bump_duration_sec < Nag_Bump_Duration_Sec_Min)
    {
        nag_bump_duration_sec = Nag_Bump_Duration_Sec_Min;
    }
    else if (nag_bump_duration_sec > Nag_Bump_Duration_Sec_Max)
    {
        nag_bump_duration_sec = Nag_Bump_Duration_Sec_Max;
    }
    g_menu_odo_slip_enable = 0u;
}

static void flash_RunLaunchParamsUnpackV13(void)
{
    flash_RunLaunchParamsUnpackV12();
    g_menu_odo_slip_enable = (uint8)(flash_union_buffer[22].uint32_type & 1u);
}

static void flash_RunLaunchParamsUnpackV7(void)
{
    run_launch_speed = flash_union_buffer[3].float_type;
    nag_spin_target_speed = flash_union_buffer[4].float_type;
    nag_spin_pre_decel_dist_cm = flash_union_buffer[5].float_type;
    nag_enter_turn_target_speed = flash_union_buffer[6].float_type;
    nag_enter_turn_pre_decel_dist_cm = flash_union_buffer[7].float_type;
    nag_exit_turn_recovery_speed = flash_union_buffer[8].float_type;
    nag_exit_turn_pre_accel_dist_cm = flash_union_buffer[9].float_type;
    nag_enter_cones_target_speed = flash_union_buffer[10].float_type;
    nag_enter_cones_pre_decel_dist_cm = flash_union_buffer[11].float_type;
    nag_enter_stair_pre_decel_dist_cm = flash_union_buffer[12].float_type;
    spin_set_rate_max_dps(flash_union_buffer[13].float_type);
    g_menu_input_remote_first = (uint8)(flash_union_buffer[14].uint32_type & 1u);
    g_menu_vofa_enable = (uint8)(flash_union_buffer[15].uint32_type & 1u);
    flash_RunLaunchBridgeApplyDefaults();
    flash_RunLaunchStairSpeedApplyDefaults();
    g_menu_odo_slip_enable = 0u;
}

static void flash_RunLaunchParamsUnpackV6(void)
{
    run_launch_speed = flash_union_buffer[3].float_type;
    nag_spin_target_speed = flash_union_buffer[4].float_type;
    nag_spin_pre_decel_dist_cm = flash_union_buffer[5].float_type;
    nag_enter_turn_target_speed = flash_union_buffer[6].float_type;
    nag_enter_turn_pre_decel_dist_cm = flash_union_buffer[7].float_type;
    nag_exit_turn_recovery_speed = flash_union_buffer[8].float_type;
    nag_exit_turn_pre_accel_dist_cm = flash_union_buffer[9].float_type;
    nag_enter_cones_target_speed = flash_union_buffer[10].float_type;
    nag_enter_cones_pre_decel_dist_cm = flash_union_buffer[11].float_type;
    nag_enter_stair_pre_decel_dist_cm = Nag_EnterStair_PreDecel_Dist_cm_Default;
    spin_set_rate_max_dps(flash_union_buffer[12].float_type);
    g_menu_input_remote_first = (uint8)(flash_union_buffer[13].uint32_type & 1u);
    g_menu_vofa_enable = (uint8)(flash_union_buffer[14].uint32_type & 1u);
    flash_RunLaunchBridgeApplyDefaults();
    flash_RunLaunchStairSpeedApplyDefaults();
    g_menu_odo_slip_enable = 0u;
}

static void flash_RunLaunchParamsUnpackV5(void)
{
    run_launch_speed = flash_union_buffer[3].float_type;
    nag_spin_target_speed = flash_union_buffer[4].float_type;
    nag_spin_pre_decel_dist_cm = flash_union_buffer[5].float_type;
    nag_enter_turn_target_speed = flash_union_buffer[6].float_type;
    nag_enter_turn_pre_decel_dist_cm = flash_union_buffer[7].float_type;
    nag_exit_turn_recovery_speed = flash_union_buffer[8].float_type;
    nag_exit_turn_pre_accel_dist_cm = flash_union_buffer[9].float_type;
    nag_enter_cones_target_speed = flash_union_buffer[10].float_type;
    nag_enter_cones_pre_decel_dist_cm = flash_union_buffer[11].float_type;
    spin_set_rate_max_dps(flash_union_buffer[12].float_type);
    g_menu_input_remote_first = (uint8)(flash_union_buffer[13].uint32_type & 1u);
    flash_RunLaunchBridgeApplyDefaults();
    flash_RunLaunchStairSpeedApplyDefaults();
    g_menu_odo_slip_enable = 0u;
}

static void flash_RunLaunchParamsUnpackV4(void)
{
    run_launch_speed = flash_union_buffer[3].float_type;
    nag_spin_target_speed = flash_union_buffer[4].float_type;
    nag_spin_pre_decel_dist_cm = flash_union_buffer[5].float_type;
    nag_enter_turn_target_speed = flash_union_buffer[6].float_type;
    nag_enter_turn_pre_decel_dist_cm = flash_union_buffer[7].float_type;
    nag_exit_turn_recovery_speed = flash_union_buffer[8].float_type;
    nag_exit_turn_pre_accel_dist_cm = flash_union_buffer[9].float_type;
    nag_enter_cones_target_speed = flash_union_buffer[10].float_type;
    nag_enter_cones_pre_decel_dist_cm = flash_union_buffer[11].float_type;
    spin_set_rate_max_dps(flash_union_buffer[12].float_type);
    flash_RunLaunchBridgeApplyDefaults();
    flash_RunLaunchStairSpeedApplyDefaults();
}

static void flash_RunLaunchParamsUnpackV3(void)
{
    run_launch_speed = flash_union_buffer[3].float_type;
    nag_spin_target_speed = flash_union_buffer[4].float_type;
    nag_spin_pre_decel_dist_cm = flash_union_buffer[5].float_type;
    nag_enter_turn_target_speed = flash_union_buffer[6].float_type;
    nag_enter_turn_pre_decel_dist_cm = flash_union_buffer[7].float_type;
    nag_exit_turn_recovery_speed = flash_union_buffer[8].float_type;
    nag_exit_turn_pre_accel_dist_cm = flash_union_buffer[9].float_type;
    nag_enter_cones_target_speed = flash_union_buffer[10].float_type;
    nag_enter_cones_pre_decel_dist_cm = flash_union_buffer[11].float_type;
    spin_set_rate_max_dps(Nag_Spin_Rate_Max_Dps_Default);
    flash_RunLaunchBridgeApplyDefaults();
    flash_RunLaunchStairSpeedApplyDefaults();
}

static void flash_RunLaunchParamsUnpackV2(void)
{
    run_launch_speed = flash_union_buffer[3].float_type;
    nag_spin_target_speed = flash_union_buffer[4].float_type;
    nag_spin_pre_decel_dist_cm = flash_union_buffer[5].float_type;
    nag_enter_turn_target_speed = flash_union_buffer[6].float_type;
    nag_enter_turn_pre_decel_dist_cm = flash_union_buffer[7].float_type;
    nag_exit_turn_recovery_speed = Nag_ExitTurn_Recovery_Speed_Default;
    nag_exit_turn_pre_accel_dist_cm = Nag_ExitTurn_PreAccel_Dist_cm_Default;
    nag_enter_cones_target_speed = flash_union_buffer[8].float_type;
    nag_enter_cones_pre_decel_dist_cm = flash_union_buffer[9].float_type;
    spin_set_rate_max_dps(Nag_Spin_Rate_Max_Dps_Default);
    flash_RunLaunchBridgeApplyDefaults();
    flash_RunLaunchStairSpeedApplyDefaults();
}

static void flash_Gps_DoubleToWords(double value, uint32 *word0, uint32 *word1)
{
    uint32 raw[2] = {0};

    memcpy(raw, &value, sizeof(value));
    *word0 = raw[0];
    *word1 = raw[1];
}

static double flash_Gps_WordsToDouble(uint32 word0, uint32 word1)
{
    uint32 raw[2];
    double value = 0.0;

    raw[0] = word0;
    raw[1] = word1;
    memcpy(&value, raw, sizeof(value));
    return value;
}

static uint32 flash_GpsPointsChecksum(uint8 point_count, uint32 current_element)
{
    uint32 checksum = GPS_POINTS_VERSION ^ (uint32)point_count ^ current_element;
    uint8 index = 0;

    for (index = 0; index < point_count; index++)
    {
        uint32 raw0 = 0;
        uint32 raw1 = 0;

        flash_Gps_DoubleToWords(latitude_point[index], &raw0, &raw1);
        checksum ^= raw0;
        checksum ^= raw1;
        flash_Gps_DoubleToWords(longitude_point[index], &raw0, &raw1);
        checksum ^= raw0;
        checksum ^= raw1;
        checksum ^= u32yuansu[index];
    }
    return checksum;
}

static uint32 flash_Nag_EventChecksum(uint8 event_count)
{
    uint32 checksum = Nag_Event_Version ^ (uint32)event_count;
    uint8 event_index = 0;

    for (event_index = 0; event_index < event_count; event_index++)
    {
        uint32 packed_index = ((uint32)Nag_Event_Table[event_index].enter_index << 16) |
                              (uint32)Nag_Event_Table[event_index].exit_index;
        uint32 packed_meta = ((uint32)Nag_Event_Table[event_index].type << 8) |
                             (uint32)Nag_Event_Table[event_index].valid;
        checksum ^= packed_index;
        checksum ^= packed_meta;
    }
    return checksum;
}

static void flash_Nag_ClearEventTable(void)
{
    memset(Nag_Event_Table, 0, sizeof(Nag_Event_Table));
    N.Event_Count = 0;
    N.Event_Record_Pending = 0;
}

static void flash_Nag_WriteEventPage(void)
{
    uint8 event_index = 0;

    flash_buffer_clear();
    flash_union_buffer[0].uint32_type = Nag_Event_Magic;
    flash_union_buffer[1].uint32_type = Nag_Event_Version;
    flash_union_buffer[2].uint32_type = N.Event_Count;
    flash_union_buffer[3].uint32_type = flash_Nag_EventChecksum(N.Event_Count);

    for (event_index = 0; event_index < N.Event_Count; event_index++)
    {
        uint16 base = (uint16)(4 + event_index * 2);
        flash_union_buffer[base].uint32_type = ((uint32)Nag_Event_Table[event_index].enter_index << 16) |
                                               (uint32)Nag_Event_Table[event_index].exit_index;
        flash_union_buffer[base + 1].uint32_type = ((uint32)Nag_Event_Table[event_index].type << 8) |
                                                   (uint32)Nag_Event_Table[event_index].valid;
    }

    if (flash_check(0, Nag_Event_Page))
    {
        flash_erase_page(0, Nag_Event_Page);
    }
    flash_write_page_from_buffer(0, Nag_Event_Page, FLASH_PAGE_LENGTH);
    flash_buffer_clear();
}

static void flash_Nag_ReadEventPage(void)
{
    uint32 event_magic = 0;
    uint32 event_version = 0;
    uint32 event_count = 0;
    uint32 event_checksum = 0;
    uint32 expect_checksum = 0;
    uint8 event_index = 0;

    flash_Nag_ClearEventTable();

    if (!flash_check(0, Nag_Event_Page))
    {
        return;
    }

    flash_buffer_clear();
    flash_read_page_to_buffer(0, Nag_Event_Page, FLASH_PAGE_LENGTH);
    event_magic = flash_union_buffer[0].uint32_type;
    event_version = flash_union_buffer[1].uint32_type;
    event_count = flash_union_buffer[2].uint32_type;
    event_checksum = flash_union_buffer[3].uint32_type;

    if (event_magic != Nag_Event_Magic || event_version != Nag_Event_Version || event_count > Nag_Event_Max)
    {
        flash_buffer_clear();
        return;
    }

    for (event_index = 0; event_index < event_count; event_index++)
    {
        uint16 base = (uint16)(4 + event_index * 2);
        uint32 packed_index = flash_union_buffer[base].uint32_type;
        uint32 packed_meta = flash_union_buffer[base + 1].uint32_type;

        Nag_Event_Table[event_index].enter_index = (uint16)(packed_index >> 16);
        Nag_Event_Table[event_index].exit_index = (uint16)(packed_index & 0xFFFFu);
        Nag_Event_Table[event_index].type = (uint8)((packed_meta >> 8) & 0xFFu);
        Nag_Event_Table[event_index].valid = (uint8)(packed_meta & 0xFFu);

        /* exit_index==enter_index 为单点事件；exit_index>enter_index 为旧双点录制，均合法。 */
        if (!Nag_Event_Table[event_index].valid ||
            Nag_Event_Table[event_index].enter_index >= N.Save_index ||
            Nag_Event_Table[event_index].exit_index >= N.Save_index ||
            Nag_Event_Table[event_index].exit_index < Nag_Event_Table[event_index].enter_index)
        {
            flash_Nag_ClearEventTable();
            flash_buffer_clear();
            return;
        }
    }

    N.Event_Count = (uint8)event_count;
    expect_checksum = flash_Nag_EventChecksum(N.Event_Count);
    if (expect_checksum != event_checksum)
    {
        flash_Nag_ClearEventTable();
    }
    flash_buffer_clear();
}

/* 保存 Run Launch 参数 + 输入模式 + VOFA 开关 + VOFA 组 + 融合开关 + 颠簸时长 + 打滑纠偏（页 47 V13）。 */
void flash_RunLaunchSpeed_Write(void)
{
    flash_buffer_clear();
    flash_union_buffer[0].uint32_type = Nag_Run_Launch_Speed_Magic;
    flash_union_buffer[1].uint32_type = Nag_Run_Launch_Params_Version_V13;
    flash_RunLaunchParamsPack();
    flash_union_buffer[2].uint32_type = flash_RunLaunchParamsChecksum();

    if (flash_check(0, Nag_Run_Launch_Speed_Page))
    {
        flash_erase_page(0, Nag_Run_Launch_Speed_Page);
    }
    flash_write_page_from_buffer(0, Nag_Run_Launch_Speed_Page, FLASH_PAGE_LENGTH);
    flash_buffer_clear();
}

/* 上电读回 Run 发车速度。校验失败时保持 control.c 中的默认值，避免空页误写速度。 */
void flash_RunLaunchSpeed_Read(void)
{
    uint32 speed_magic = 0;
    uint32 speed_version = 0;
    uint32 speed_checksum = 0;
    uint32 speed_raw = 0;

    if (!flash_check(0, Nag_Run_Launch_Speed_Page))
    {
        return;
    }

    flash_buffer_clear();
    flash_read_page_to_buffer(0, Nag_Run_Launch_Speed_Page, FLASH_PAGE_LENGTH);
    speed_magic = flash_union_buffer[0].uint32_type;
    speed_version = flash_union_buffer[1].uint32_type;
    speed_checksum = flash_union_buffer[2].uint32_type;
    speed_raw = flash_union_buffer[3].uint32_type;

    if (speed_magic != Nag_Run_Launch_Speed_Magic)
    {
        flash_buffer_clear();
        return;
    }

    if ((speed_version == Nag_Run_Launch_Params_Version_V13) &&
        (speed_checksum == flash_RunLaunchParamsChecksumEx(Nag_Run_Launch_Params_Version_V13,
                                                           Nag_Run_Launch_Config_Word_Count)))
    {
        flash_RunLaunchParamsUnpackV13();
    }
    else if ((speed_version == Nag_Run_Launch_Params_Version_V12) &&
        (speed_checksum == flash_RunLaunchParamsChecksumEx(Nag_Run_Launch_Params_Version_V12,
                                                           Nag_Run_Launch_Config_Word_Count_V12)))
    {
        flash_RunLaunchParamsUnpackV12();
    }
    else if ((speed_version == Nag_Run_Launch_Params_Version_V11) &&
        (speed_checksum == flash_RunLaunchParamsChecksumEx(Nag_Run_Launch_Params_Version_V11,
                                                           Nag_Run_Launch_Config_Word_Count_V11)))
    {
        flash_RunLaunchParamsUnpackV11();
    }
    else if ((speed_version == Nag_Run_Launch_Params_Version_V10) &&
        (speed_checksum == flash_RunLaunchParamsChecksumEx(Nag_Run_Launch_Params_Version_V10,
                                                           Nag_Run_Launch_Config_Word_Count_V10)))
    {
        flash_RunLaunchParamsUnpackV10();
    }
    else if ((speed_version == Nag_Run_Launch_Params_Version_V9) &&
        (speed_checksum == flash_RunLaunchParamsChecksumEx(Nag_Run_Launch_Params_Version_V9,
                                                           Nag_Run_Launch_Config_Word_Count_V9)))
    {
        flash_RunLaunchParamsUnpackV9();
    }
    else if ((speed_version == Nag_Run_Launch_Params_Version_V8) &&
        (speed_checksum == flash_RunLaunchParamsChecksumEx(Nag_Run_Launch_Params_Version_V8,
                                                           Nag_Run_Launch_Config_Word_Count_V8)))
    {
        flash_RunLaunchParamsUnpackV8();
    }
    else if ((speed_version == Nag_Run_Launch_Params_Version_V7) &&
        (speed_checksum == flash_RunLaunchParamsChecksumEx(Nag_Run_Launch_Params_Version_V7,
                                                           Nag_Run_Launch_Config_Word_Count_V7)))
    {
        flash_RunLaunchParamsUnpackV7();
    }
    else if ((speed_version == Nag_Run_Launch_Params_Version_V6) &&
        (speed_checksum == flash_RunLaunchParamsChecksumEx(Nag_Run_Launch_Params_Version_V6,
                                                           Nag_Run_Launch_Config_Word_Count_V6)))
    {
        flash_RunLaunchParamsUnpackV6();
    }
    else if ((speed_version == Nag_Run_Launch_Params_Version_V5) &&
        (speed_checksum == flash_RunLaunchParamsChecksumEx(Nag_Run_Launch_Params_Version_V5,
                                                           Nag_Run_Launch_Config_Word_Count_V5)))
    {
        flash_RunLaunchParamsUnpackV5();
    }
    else if ((speed_version == Nag_Run_Launch_Params_Version_V4) &&
        (speed_checksum == flash_RunLaunchParamsChecksumEx(Nag_Run_Launch_Params_Version_V4,
                                                           Nag_Run_Launch_Param_Count)))
    {
        flash_RunLaunchParamsUnpackV4();
    }
    else if ((speed_version == Nag_Run_Launch_Params_Version_V3) &&
        (speed_checksum == flash_RunLaunchParamsChecksumEx(Nag_Run_Launch_Params_Version_V3,
                                                           9u)))
    {
        flash_RunLaunchParamsUnpackV3();
    }
    else if ((speed_version == Nag_Run_Launch_Params_Version) &&
             (speed_checksum == flash_RunLaunchParamsChecksumEx(Nag_Run_Launch_Params_Version,
                                                                Nag_Run_Launch_Param_Count_V2)))
    {
        flash_RunLaunchParamsUnpackV2();
    }
    else if ((speed_version == Nag_Run_Launch_Speed_Version) &&
             (speed_checksum == flash_RunLaunchSpeedChecksum(speed_raw)))
    {
        run_launch_speed = flash_union_buffer[3].float_type;
    }
    flash_buffer_clear();
}

#define Nag_Jump_Params_Page 50u
#define Nag_Jump_Params_Magic 0x4A504D50u   /* "JPMP" */
#define Nag_Jump_Params_Version 1u
#define Nag_Jump_Params_Version_V2 2u
#define Nag_Jump_Params_Word_Count_V1 8u
#define Nag_Jump_Params_Word_Count 9u

static uint32 flash_JumpParamsChecksumEx(uint32 version, uint8 word_count)
{
    uint32 checksum = Nag_Jump_Params_Magic ^ version;
    uint8 index = 0u;

    for (index = 0u; index < word_count; index++)
    {
        checksum ^= flash_union_buffer[3u + index].uint32_type;
    }
    return checksum;
}

static uint32 flash_JumpParamsChecksum(void)
{
    return flash_JumpParamsChecksumEx(Nag_Jump_Params_Version_V2,
                                      Nag_Jump_Params_Word_Count);
}

static void flash_JumpParamsPack(void)
{
    uint8 index = 0u;

    for (index = 0u; index < Nag_Jump_Params_Word_Count; index++)
    {
        flash_union_buffer[3u + index].float_type = JumpParamGet(index);
    }
}

static void flash_JumpParamsUnpackV1(void)
{
    uint8 index = 0u;

    for (index = 0u; index < Nag_Jump_Params_Word_Count_V1; index++)
    {
        JumpParamSet(index, flash_union_buffer[3u + index].float_type);
    }
    jump_buffer_step_p_max = JUMP_BUFFER_STEP_P_MAX_DEFAULT;
    JumpParamRecalcBufferTimeFromLeg();
}

static void flash_JumpParamsUnpack(void)
{
    uint8 index = 0u;

    for (index = 0u; index < Nag_Jump_Params_Word_Count; index++)
    {
        JumpParamSet(index, flash_union_buffer[3u + index].float_type);
    }
    JumpParamRecalcBufferTimeFromLeg();
}

void flash_JumpParams_Write(void)
{
    flash_buffer_clear();
    flash_union_buffer[0].uint32_type = Nag_Jump_Params_Magic;
    flash_union_buffer[1].uint32_type = Nag_Jump_Params_Version_V2;
    flash_JumpParamsPack();
    flash_union_buffer[2].uint32_type = flash_JumpParamsChecksum();

    if (flash_check(0, Nag_Jump_Params_Page))
    {
        flash_erase_page(0, Nag_Jump_Params_Page);
    }
    flash_write_page_from_buffer(0, Nag_Jump_Params_Page, FLASH_PAGE_LENGTH);
    flash_buffer_clear();
}

void flash_JumpParams_Read(void)
{
    uint32 jump_magic = 0;
    uint32 jump_version = 0;
    uint32 jump_checksum = 0;

    if (!flash_check(0, Nag_Jump_Params_Page))
    {
        return;
    }

    flash_buffer_clear();
    flash_read_page_to_buffer(0, Nag_Jump_Params_Page, FLASH_PAGE_LENGTH);
    jump_magic = flash_union_buffer[0].uint32_type;
    jump_version = flash_union_buffer[1].uint32_type;
    jump_checksum = flash_union_buffer[2].uint32_type;

    if ((jump_magic == Nag_Jump_Params_Magic) &&
        (jump_version == Nag_Jump_Params_Version_V2) &&
        (jump_checksum == flash_JumpParamsChecksumEx(Nag_Jump_Params_Version_V2,
                                                      Nag_Jump_Params_Word_Count)))
    {
        flash_JumpParamsUnpack();
    }
    else if ((jump_magic == Nag_Jump_Params_Magic) &&
             (jump_version == Nag_Jump_Params_Version) &&
             (jump_checksum == flash_JumpParamsChecksumEx(Nag_Jump_Params_Version,
                                                          Nag_Jump_Params_Word_Count_V1)))
    {
        flash_JumpParamsUnpackV1();
    }
    flash_buffer_clear();
}

#define Nag_Gyro_Bias_Page 51u
#define Nag_Gyro_Bias_Magic 0x475A4253u   /* "GZBS" */
#define Nag_Gyro_Bias_Version 1u

static uint32 flash_GyroBiasChecksum(void)
{
    return Nag_Gyro_Bias_Magic ^ Nag_Gyro_Bias_Version ^
           flash_union_buffer[3].uint32_type;
}

void flash_GyroBias_Write(void)
{
    flash_buffer_clear();
    flash_union_buffer[0].uint32_type = Nag_Gyro_Bias_Magic;
    flash_union_buffer[1].uint32_type = Nag_Gyro_Bias_Version;
    flash_union_buffer[3].float_type = gyro_z_bias_comp;
    flash_union_buffer[2].uint32_type = flash_GyroBiasChecksum();

    if (flash_check(0, Nag_Gyro_Bias_Page))
    {
        flash_erase_page(0, Nag_Gyro_Bias_Page);
    }
    flash_write_page_from_buffer(0, Nag_Gyro_Bias_Page, FLASH_PAGE_LENGTH);
    flash_buffer_clear();
}

void flash_GyroBias_Read(void)
{
    uint32 magic = 0u;
    uint32 version = 0u;
    uint32 checksum = 0u;

    if (!flash_check(0, Nag_Gyro_Bias_Page))
    {
        return;
    }

    flash_read_page_to_buffer(0, Nag_Gyro_Bias_Page, FLASH_PAGE_LENGTH);
    magic = flash_union_buffer[0].uint32_type;
    version = flash_union_buffer[1].uint32_type;
    checksum = flash_union_buffer[2].uint32_type;

    if ((magic == Nag_Gyro_Bias_Magic) &&
        (version == Nag_Gyro_Bias_Version) &&
        (checksum == flash_GyroBiasChecksum()))
    {
        GyroBias_SetComp(flash_union_buffer[3].float_type);
    }
    flash_buffer_clear();
}

void flash_GpsPoints_Write(void)
{
    uint8 point_count = GPS_GetValidPointCount();
    uint8 index = 0;

    GPS_NavForceEndPoint();
    point_count = GPS_GetValidPointCount();

    flash_buffer_clear();
    flash_union_buffer[0].uint32_type = GPS_POINTS_MAGIC;
    flash_union_buffer[1].uint32_type = GPS_POINTS_VERSION;
    flash_union_buffer[2].uint32_type = point_count;
    flash_union_buffer[3].uint32_type = gps_current_yuansu;
    flash_union_buffer[4].uint32_type = flash_GpsPointsChecksum(point_count, gps_current_yuansu);

    for (index = 0; index < point_count; index++)
    {
        uint32 raw0 = 0;
        uint32 raw1 = 0;
        uint16 lat_base = (uint16)(GPS_POINTS_LAT_BASE + index * 2u);
        uint16 lon_base = (uint16)(GPS_POINTS_LON_BASE + index * 2u);

        flash_Gps_DoubleToWords(latitude_point[index], &raw0, &raw1);
        flash_union_buffer[lat_base].uint32_type = raw0;
        flash_union_buffer[lat_base + 1u].uint32_type = raw1;
        flash_Gps_DoubleToWords(longitude_point[index], &raw0, &raw1);
        flash_union_buffer[lon_base].uint32_type = raw0;
        flash_union_buffer[lon_base + 1u].uint32_type = raw1;
        flash_union_buffer[GPS_POINTS_ELEMENT_BASE + index].uint32_type = u32yuansu[index];
    }

    if (flash_check(0, GPS_POINTS_PAGE))
    {
        flash_erase_page(0, GPS_POINTS_PAGE);
    }
    flash_write_page_from_buffer(0, GPS_POINTS_PAGE, FLASH_PAGE_LENGTH);
    flash_buffer_clear();
}

void flash_GpsPoints_Read(void)
{
    uint32 magic = 0;
    uint32 version = 0;
    uint32 point_count = 0;
    uint32 current_element = 0;
    uint32 checksum = 0;
    uint8 index = 0;

    if (!flash_check(0, GPS_POINTS_PAGE))
    {
        GPS_ClearPoints();
        return;
    }

    flash_buffer_clear();
    flash_read_page_to_buffer(0, GPS_POINTS_PAGE, FLASH_PAGE_LENGTH);

    magic = flash_union_buffer[0].uint32_type;
    version = flash_union_buffer[1].uint32_type;
    point_count = flash_union_buffer[2].uint32_type;
    current_element = flash_union_buffer[3].uint32_type;
    checksum = flash_union_buffer[4].uint32_type;

    if ((magic != GPS_POINTS_MAGIC) ||
        (version != GPS_POINTS_VERSION) ||
        (point_count > GPS_POINT_MAX) ||
        (current_element >= NAV_ELEM_COUNT))
    {
        GPS_ClearPoints();
        flash_buffer_clear();
        return;
    }

    GPS_ClearPoints();
    gps_current_yuansu = current_element;

    for (index = 0; index < point_count; index++)
    {
        uint16 lat_base = (uint16)(GPS_POINTS_LAT_BASE + index * 2u);
        uint16 lon_base = (uint16)(GPS_POINTS_LON_BASE + index * 2u);
        double lat = flash_Gps_WordsToDouble(flash_union_buffer[lat_base].uint32_type,
                                             flash_union_buffer[lat_base + 1u].uint32_type);
        double lon = flash_Gps_WordsToDouble(flash_union_buffer[lon_base].uint32_type,
                                             flash_union_buffer[lon_base + 1u].uint32_type);
        uint32 element = flash_union_buffer[GPS_POINTS_ELEMENT_BASE + index].uint32_type;

        if (element >= NAV_ELEM_COUNT)
        {
            GPS_ClearPoints();
            flash_buffer_clear();
            return;
        }
        GPS_SavePointFromCoord(index, lat, lon, element);
    }

    if (checksum != flash_GpsPointsChecksum((uint8)point_count, current_element))
    {
        GPS_ClearPoints();
    }
    flash_buffer_clear();
}

void flash_GpsPoints_Clear(void)
{
    GPS_ClearPoints();
    if (flash_check(0, GPS_POINTS_PAGE))
    {
        flash_erase_page(0, GPS_POINTS_PAGE);
    }
    flash_buffer_clear();
}

void flash_Nag_Write(){
   
    if(flash_check(0, N.Flash_page_index))flash_erase_page(0, N.Flash_page_index);                  
                       
    flash_write_page_from_buffer(0,N.Flash_page_index,FLASH_PAGE_LENGTH);
    if(N.End_f == 1)
    {    
     flash_buffer_clear();
     flash_union_buffer[MaxSize+2].uint32_type = N.Save_index;
     if(flash_check(0, Nag_End_Page))flash_erase_page(0, Nag_End_Page);
     flash_write_page_from_buffer(0,Nag_End_Page,FLASH_PAGE_LENGTH);
     flash_Nag_WriteEventPage();
    }
    
    
    flash_buffer_clear();
    

   
}

void flash_Nag_ResetReadState(void)
{
    nag_flash_index_read = 0;
}

void flash_Nag_Read(){
    flash_buffer_clear();

    if(0 == nag_flash_index_read)
    {
       flash_read_page_to_buffer(0,Nag_End_Page,FLASH_PAGE_LENGTH);
        N.Save_index = flash_union_buffer[MaxSize+2].uint32_type;       
        nag_flash_index_read = 1;
        flash_buffer_clear();
        flash_Nag_ReadEventPage();
    }
    if(flash_check(0, N.Flash_page_index))
    {        
        flash_read_page_to_buffer(0, N.Flash_page_index,FLASH_PAGE_LENGTH);
    }
}
