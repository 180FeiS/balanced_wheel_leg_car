#include "zf_common_headfile.h"
#include "my_gps.h"
#include "nav_fusion.h"
#include <math.h>
#include <string.h>
#define GPS_NAV_PI 3.14159265358979323846
#define GPS_NAV_EARTH_RADIUS_M 6371000.0
#define GPS_NAV_DEG_TO_RAD(x) ((x) * GPS_NAV_PI / 180.0)
#define GPS_NAV_RAD_TO_DEG(x) ((x) * 180.0 / GPS_NAV_PI)
#define GPS_PATH_MIN_RANGE_DEG 0.000001
#define GPS_PATH_MARGIN_RATIO 0.10
car_dir car_gps_dir = north;
uint8 now_point = 0;
uint8 tagert_point = 0;
uint8 save_point = 0;
uint8 show_point = 0;
uint8 gps_point_count = 0;
uint8 gps_recording_active = 0;
uint32 gps_current_yuansu = NAV_ELEM_NORMAL;
double latitude_point[GPS_POINT_MAX] = {0};
double longitude_point[GPS_POINT_MAX] = {0};
uint32 u32yuansu[GPS_POINT_MAX] = {0};
uint8 gps_nav_state = GPS_NAV_STATE_IDLE;
uint8 gps_nav_protect_reason = GPS_NAV_PROTECT_NONE;
uint8 gps_nav_target_index = 0;
double gps_nav_current_latitude = 0.0;
double gps_nav_current_longitude = 0.0;
double gps_nav_target_latitude = 0.0;
double gps_nav_target_longitude = 0.0;
float gps_nav_distance_m = 0.0f;
float gps_nav_geo_bearing_deg = 0.0f;
float gps_nav_body_target_yaw_deg = 0.0f;
float gps_nav_target_imu_yaw_deg = 0.0f;
float gps_nav_imu_yaw_deg = 0.0f;
float gps_nav_yaw_err_deg = 0.0f;
uint8 gps_nav_align_state = GPS_NAV_ALIGN_WAIT;
float gps_nav_heading_bias_deg = 0.0f;
float gps_nav_gps_first_deg = 0.0f;
float gps_nav_euler_ref_at_first_deg = 0.0f;
float gps_nav_dist_from_launch_m = 0.0f;
double gps_drift_delta_lat = 0.0;
double gps_drift_delta_lon = 0.0;
uint8 gps_drift_corr_valid = 0u;
static float gps_nav_launch_imu_yaw = 0.0f;
static float gps_nav_last_requested_yaw = 0.0f;
static uint8 gps_nav_request_valid = 0u;
static double gps_nav_launch_latitude = 0.0;
static double gps_nav_launch_longitude = 0.0;
static uint8 gps_nav_launch_fix_valid = 0u;
#if NAV_FUSION_ENABLE && GPS_NAV_USE_FUSION_POSITION && NAV_FUSION_ORIGIN_ENABLE
static uint8 gps_nav_origin_pending = 0u;
#endif
static uint16 GPS_ClampU16(int32 value, uint16 min_value, uint16 max_value)
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
static void GPS_FillPathViewport(uint16 x_offset, uint16 y_offset, uint16 width, uint16 height)
{
    uint16 y = 0;
    uint16 x_end = (uint16)(x_offset + width);
    uint16 y_end = (uint16)(y_offset + height);
    for (y = y_offset; y <= y_end; y++)
    {
        ips200_draw_line(x_offset, y, x_end, y, RGB565_WHITE);
    }
}
static void GPS_DrawViewportPoint(uint16 x,
                                  uint16 y,
                                  uint16 x_offset,
                                  uint16 y_offset,
                                  uint16 width,
                                  uint16 height,
                                  uint16 color)
{
    uint16 x_max = (uint16)(x_offset + width);
    uint16 y_max = (uint16)(y_offset + height);
    ips200_draw_point(x, y, color);
    ips200_draw_point(GPS_ClampU16((int32)x + 1, x_offset, x_max), y, color);
    ips200_draw_point(GPS_ClampU16((int32)x - 1, x_offset, x_max), y, color);
    ips200_draw_point(x, GPS_ClampU16((int32)y + 1, y_offset, y_max), color);
    ips200_draw_point(x, GPS_ClampU16((int32)y - 1, y_offset, y_max), color);
}
static void GPS_ExpandRange(double *min_value, double *max_value)
{
    double center = (*min_value + *max_value) * 0.5;
    double range = *max_value - *min_value;
    double margin = 0.0;
    if (fabs(range) < GPS_PATH_MIN_RANGE_DEG)
    {
        range = GPS_PATH_MIN_RANGE_DEG;
        *min_value = center - (range * 0.5);
        *max_value = center + (range * 0.5);
    }
    margin = range * GPS_PATH_MARGIN_RATIO;
    *min_value -= margin;
    *max_value += margin;
}
static float GPS_Wrap180(float angle_deg)
{
    while (angle_deg > 180.0f)
    {
        angle_deg -= 360.0f;
    }
    while (angle_deg < -180.0f)
    {
        angle_deg += 360.0f;
    }
    return angle_deg;
}
static float GPS_Wrap360(float angle_deg)
{
    while (angle_deg >= 360.0f)
    {
        angle_deg -= 360.0f;
    }
    while (angle_deg < 0.0f)
    {
        angle_deg += 360.0f;
    }
    return angle_deg;
}
static void GPS_NavClearDebug(void)
{
    gps_nav_current_latitude = 0.0;
    gps_nav_current_longitude = 0.0;
    gps_nav_target_latitude = 0.0;
    gps_nav_target_longitude = 0.0;
    gps_nav_distance_m = 0.0f;
    gps_nav_geo_bearing_deg = 0.0f;
    gps_nav_body_target_yaw_deg = 0.0f;
    gps_nav_target_imu_yaw_deg = 0.0f;
    gps_nav_imu_yaw_deg = 0.0f;
    gps_nav_yaw_err_deg = 0.0f;
    gps_nav_request_valid = 0u;
    gps_nav_last_requested_yaw = 0.0f;
    gps_nav_align_state = GPS_NAV_ALIGN_WAIT;
    gps_nav_heading_bias_deg = 0.0f;
    gps_nav_gps_first_deg = 0.0f;
    gps_nav_euler_ref_at_first_deg = 0.0f;
    gps_nav_dist_from_launch_m = 0.0f;
    gps_nav_launch_latitude = 0.0;
    gps_nav_launch_longitude = 0.0;
    gps_nav_launch_fix_valid = 0u;
    gps_drift_delta_lat = 0.0;
    gps_drift_delta_lon = 0.0;
    gps_drift_corr_valid = 0u;
}
#if !defined(CY_CORE_CM7_1)
static uint8 GPS_NavIsCurrentCoordValid(double latitude, double longitude)
{
    uint8 frame_seen = (uint8)((gnss.time.year != 0u) || (gnss.state != 0u) || (gnss.satellite_used != 0u));
#if NAV_FUSION_ENABLE && GPS_NAV_USE_FUSION_POSITION
    if (NavFusion_IsValid() != 0u && latitude != 0.0 && longitude != 0.0)
    {
        return 1u;
    }
#endif
    if (!frame_seen || latitude == 0.0 || longitude == 0.0)
    {
        return 0u;
    }
    return 1u;
}
static uint8 GPS_NavIsTargetValid(uint8 index)
{
    if (index >= gps_point_count || index >= GPS_POINT_MAX)
    {
        return 0u;
    }
    if (latitude_point[index] == 0.0 || longitude_point[index] == 0.0)
    {
        return 0u;
    }
    return 1u;
}
static void GPS_NavStop(uint8 state, uint8 reason)
{
    gps_nav_state = state;
    gps_nav_protect_reason = reason;
    gps_nav_request_valid = 0u;
    motor_user_speed_cmd = 0.0f;
    steer_yaw_request_pending = 0u;
    steer_yaw_delayed_by_spin = 0u;
    steer_task_stop();
#if NAV_FUSION_ENABLE && GPS_NAV_USE_FUSION_POSITION
    if (state == GPS_NAV_STATE_FINISHED || state == GPS_NAV_STATE_PROTECT)
    {
        NavFusion_Reset();
    }
#endif
}
static void GPS_NavTryUpdateDriftCorrection(void)
{
    if (gps_drift_corr_valid != 0u)
    {
        return;
    }
    if (gps_nav_launch_fix_valid == 0u)
    {
        return;
    }
    if (gps_point_count < 1u)
    {
        return;
    }
    if (latitude_point[0] == 0.0 || longitude_point[0] == 0.0)
    {
        return;
    }
    gps_drift_delta_lat = gps_nav_launch_latitude - latitude_point[0];
    gps_drift_delta_lon = gps_nav_launch_longitude - longitude_point[0];
    gps_drift_corr_valid = 1u;
}
static void GPS_NavSegmentMetrics(double lat_a,
                                  double lon_a,
                                  double lat_b,
                                  double lon_b,
                                  float *distance_m,
                                  float *bearing_deg_wrap360)
{
    double lat_ar = GPS_NAV_DEG_TO_RAD(lat_a);
    double lat_br = GPS_NAV_DEG_TO_RAD(lat_b);
    double d_lat_rad = GPS_NAV_DEG_TO_RAD(lat_b - lat_a);
    double d_lon_rad = GPS_NAV_DEG_TO_RAD(lon_b - lon_a);
    double avg_lat_rad = (lat_ar + lat_br) * 0.5;
    double north_m = d_lat_rad * GPS_NAV_EARTH_RADIUS_M;
    double east_m = d_lon_rad * GPS_NAV_EARTH_RADIUS_M * cos(avg_lat_rad);
    double bearing_deg = GPS_NAV_RAD_TO_DEG(atan2(east_m, north_m));
    if (distance_m != NULL)
    {
        *distance_m = (float)sqrt(east_m * east_m + north_m * north_m);
    }
    if (bearing_deg_wrap360 != NULL)
    {
        *bearing_deg_wrap360 = GPS_Wrap360((float)bearing_deg);
    }
}
static float GPS_GnssDirectionToSigned180(float direction_deg_0_360)
{
    float w = GPS_Wrap360(direction_deg_0_360);
    if (w > 180.0f)
    {
        return w - 360.0f;
    }
    return w;
}
static float GPS_NavAzimuthSubject2_deg(double lat1, double lon1, double lat2, double lon2)
{
    double az = get_two_points_azimuth(lat1, lon1, lat2, lon2);
    return GPS_GnssDirectionToSigned180((float)az);
}
static uint8 GPS_NavIsCogSampleValid(void)
{
    if (gnss.state == 0u)
    {
        return 0u;
    }
    return 1u;
}
static void GPS_NavApplyDriftToCoord(uint8 point_index, double *lat_out, double *lon_out)
{
    double lat = latitude_point[point_index];
    double lon = longitude_point[point_index];
    if (lat_out == NULL || lon_out == NULL)
    {
        return;
    }
    if (gps_drift_corr_valid != 0u)
    {
        lat += gps_drift_delta_lat;
        lon += gps_drift_delta_lon;
    }
    *lat_out = lat;
    *lon_out = lon;
}
float GPS_NavDistanceToPointM(uint8 point_index)
{
#if defined(CY_CORE_CM7_1)
    (void)point_index;
    return 0.0f;
#else
    double target_lat = 0.0;
    double target_lon = 0.0;
    float distance_m = 0.0f;
    if (point_index >= gps_point_count || point_index >= GPS_POINT_MAX)
    {
        return 0.0f;
    }
    if (!GPS_NavIsTargetValid(point_index))
    {
        return 0.0f;
    }
    GPS_NavApplyDriftToCoord(point_index, &target_lat, &target_lon);
    GPS_NavSegmentMetrics(gnss.latitude,
                          gnss.longitude,
                          target_lat,
                          target_lon,
                          &distance_m,
                          NULL);
    return distance_m;
#endif
}
void GPS_NavForceEndPoint(void)
{
    if (gps_point_count > 0u)
    {
        u32yuansu[gps_point_count - 1u] = NAV_ELEM_END;
    }
}
void GPS_NavTryEnterElement(uint8 point_index, uint8 unified_type)
{
#if defined(CY_CORE_CM7_1)
    (void)point_index;
    (void)unified_type;
#else
    uint8 ins_type = Nav_UnifiedToInsEvent(unified_type);
    if (ins_type == 0xFFu)
    {
        return;
    }
    N.Event_Active = 1u;
    N.Event_Active_Index = point_index;
    N.Active_Event_Enter = point_index;
    N.Active_Event_Exit = point_index;
    N.Event_Active_Type = ins_type;
    N.Event_Start_RunIndex = 0u;
    N.Event_State = NAG_EVENT_STATE_ENTERED;
    N.Event_Start_Latched = 0u;
    N.Event_Done_Latched = 0u;
    Nag_EventPrepareEnter(ins_type);
#endif
}
void GPS_NavOnElementDone(void)
{
#if defined(CY_CORE_CM7_1)
    (void)0;
#else
    if (tagert_point >= (uint8)(gps_point_count - 1u))
    {
        GPS_NavStop(GPS_NAV_STATE_FINISHED, GPS_NAV_PROTECT_FINISHED);
        return;
    }
    tagert_point++;
    gps_nav_target_index = tagert_point;
    gps_nav_request_valid = 0u;
#endif
}
static void GPS_NavCalcDistanceBearing(double current_latitude,
                                       double current_longitude,
                                       double target_latitude,
                                       double target_longitude)
{
    GPS_NavSegmentMetrics(current_latitude,
                          current_longitude,
                          target_latitude,
                          target_longitude,
                          &gps_nav_distance_m,
                          NULL);
    gps_nav_geo_bearing_deg =
        GPS_NavAzimuthSubject2_deg(current_latitude, current_longitude, target_latitude, target_longitude);
}
#endif
void GPS_ClearPoints(void)
{
    memset(latitude_point, 0, sizeof(latitude_point));
    memset(longitude_point, 0, sizeof(longitude_point));
    memset(u32yuansu, 0, sizeof(u32yuansu));
    now_point = 0;
    tagert_point = 0;
    save_point = 0;
    show_point = 0;
    gps_point_count = 0;
    gps_recording_active = 0;
    gps_current_yuansu = NAV_ELEM_NORMAL;
    gps_nav_state = GPS_NAV_STATE_IDLE;
    gps_nav_protect_reason = GPS_NAV_PROTECT_NONE;
    gps_nav_target_index = 0;
    GPS_NavClearDebug();
}
void GPS_BeginRecord(void)
{
    GPS_ClearPoints();
    gps_recording_active = 1u;
}
void GPS_EndRecord(void)
{
    gps_recording_active = 0u;
    GPS_NavForceEndPoint();
}
void GPS_ApplyLaunchSpeed(void)
{
#if defined(CY_CORE_CM7_1)
    (void)0;
#else
    tagert_point = 0u;
    now_point = 0u;
    gps_nav_target_index = 0u;
    gps_nav_state = GPS_NAV_STATE_RUNNING;
    gps_nav_protect_reason = GPS_NAV_PROTECT_NONE;
    GPS_NavClearDebug();
    if (N.Event_Active)
    {
        Nag_EventForceReset();
    }
    gps_nav_launch_imu_yaw = (float)euler_angle.yaw;
    gps_nav_launch_fix_valid = 0u;
    gps_drift_corr_valid = 0u;
    {
        uint8 defer_speed_for_origin = 0u;
#if NAV_FUSION_ENABLE && GPS_NAV_USE_FUSION_POSITION && NAV_FUSION_ORIGIN_ENABLE
        if (NavFusion_IsRuntimeEnabled() != 0u)
        {
            gps_nav_origin_pending = 1u;
            NavFusion_BeginOriginAverage(gps_nav_launch_imu_yaw);
            nav_heading_mode = NAV_HEADING_MODE_GPS;
            defer_speed_for_origin = 1u;
        }
#endif
        if (defer_speed_for_origin != 0u)
        {
            motor_user_speed_cmd = 0.0f;
        }
#if NAV_FUSION_ENABLE && GPS_NAV_USE_FUSION_POSITION
        else
        {
            uint8 gnss_live = (uint8)((gnss.time.year != 0u) || (gnss.state != 0u) || (gnss.satellite_used != 0u));
            if (gnss_live && (gnss.latitude != 0.0) && (gnss.longitude != 0.0))
            {
                gps_nav_launch_latitude = gnss.latitude;
                gps_nav_launch_longitude = gnss.longitude;
                gps_nav_launch_fix_valid = 1u;
            }
            GPS_NavTryUpdateDriftCorrection();
            if (gps_nav_launch_fix_valid != 0u && NavFusion_IsRuntimeEnabled() != 0u)
            {
                NavFusion_InitFromGps(gps_nav_launch_latitude,
                                      gps_nav_launch_longitude,
                                      gps_nav_launch_imu_yaw);
            }
            nav_heading_mode = NAV_HEADING_MODE_GPS;
            motor_user_speed_cmd = run_launch_speed;
        }
#else
        else
        {
            uint8 gnss_live = (uint8)((gnss.time.year != 0u) || (gnss.state != 0u) || (gnss.satellite_used != 0u));
            if (gnss_live && (gnss.latitude != 0.0) && (gnss.longitude != 0.0))
            {
                gps_nav_launch_latitude = gnss.latitude;
                gps_nav_launch_longitude = gnss.longitude;
                gps_nav_launch_fix_valid = 1u;
            }
            GPS_NavTryUpdateDriftCorrection();
            nav_heading_mode = NAV_HEADING_MODE_GPS;
            motor_user_speed_cmd = run_launch_speed;
        }
#endif
    }
#endif
}
void GPS_CompleteLaunchAfterOrigin(void)
{
#if defined(CY_CORE_CM7_1)
    (void)0;
#else
#if NAV_FUSION_ENABLE && GPS_NAV_USE_FUSION_POSITION && NAV_FUSION_ORIGIN_ENABLE
    if (gps_nav_origin_pending == 0u)
    {
        return;
    }
    if (NavFusion_IsValid() == 0u)
    {
        return;
    }
    if (NavFusion_GetOriginAvgResult(&gps_nav_launch_latitude, &gps_nav_launch_longitude) == 0u)
    {
        return;
    }
    gps_nav_launch_fix_valid = 1u;
    GPS_NavTryUpdateDriftCorrection();
    gps_nav_origin_pending = 0u;
    motor_user_speed_cmd = run_launch_speed;
#else
    (void)0;
#endif
#endif
}
void GPS_OnOriginCalibrationFailed(void)
{
#if defined(CY_CORE_CM7_1)
    (void)0;
#else
#if NAV_FUSION_ENABLE && GPS_NAV_USE_FUSION_POSITION && NAV_FUSION_ORIGIN_ENABLE
    if (gps_nav_origin_pending == 0u)
    {
        return;
    }
    gps_nav_origin_pending = 0u;
    gps_nav_state = GPS_NAV_STATE_IDLE;
    gps_nav_launch_fix_valid = 0u;
    motor_user_speed_cmd = 0.0f;
    NavFusion_Reset();
#else
    (void)0;
#endif
#endif
}
void GPS_PointNav_Run(void)
{
#if defined(CY_CORE_CM7_1)
    (void)0;
#else
    float target_delta = 0.0f;
    uint8 is_last_point = 0u;
    float dist_from_launch_m = 0.0f;
    double target_lat_used = 0.0;
    double target_lon_used = 0.0;
    if (nav_heading_mode != NAV_HEADING_MODE_GPS)
    {
        return;
    }
#if NAV_FUSION_ENABLE && GPS_NAV_USE_FUSION_POSITION && NAV_FUSION_ORIGIN_ENABLE
    if (NavFusion_IsRuntimeEnabled() != 0u && gps_nav_origin_pending != 0u)
    {
        motor_user_speed_cmd = 0.0f;
        return;
    }
#endif
    if (gps_nav_state == GPS_NAV_STATE_FINISHED || gps_nav_state == GPS_NAV_STATE_PROTECT)
    {
        motor_user_speed_cmd = 0.0f;
        return;
    }
    if (N.Event_Active)
    {
        gps_nav_state = GPS_NAV_STATE_RUNNING;
        return;
    }
    gps_nav_current_latitude = gnss.latitude;
    gps_nav_current_longitude = gnss.longitude;
#if NAV_FUSION_ENABLE && GPS_NAV_USE_FUSION_POSITION
    if (NavFusion_IsValid() != 0u)
    {
        NavFusion_GetPositionLatLon(&gps_nav_current_latitude, &gps_nav_current_longitude);
    }
#endif
    gps_nav_imu_yaw_deg = (float)euler_angle.yaw;
    if (!GPS_NavIsCurrentCoordValid(gps_nav_current_latitude, gps_nav_current_longitude))
    {
        GPS_NavStop(GPS_NAV_STATE_PROTECT, GPS_NAV_PROTECT_GPS_INVALID);
        return;
    }
    if (gps_nav_launch_fix_valid == 0u)
    {
        gps_nav_launch_latitude = gps_nav_current_latitude;
        gps_nav_launch_longitude = gps_nav_current_longitude;
        gps_nav_launch_fix_valid = 1u;
    }
    GPS_NavTryUpdateDriftCorrection();
    if (gps_point_count < GPS_NAV_MIN_POINT_COUNT || !GPS_NavIsTargetValid(tagert_point))
    {
        GPS_NavStop(GPS_NAV_STATE_PROTECT, GPS_NAV_PROTECT_TARGET_INVALID);
        return;
    }
    gps_nav_target_index = tagert_point;
    now_point = tagert_point;
    target_lat_used = latitude_point[tagert_point];
    target_lon_used = longitude_point[tagert_point];
    if (gps_drift_corr_valid != 0u)
    {
        target_lat_used += gps_drift_delta_lat;
        target_lon_used += gps_drift_delta_lon;
    }
    gps_nav_target_latitude = target_lat_used;
    gps_nav_target_longitude = target_lon_used;
    GPS_NavCalcDistanceBearing(gps_nav_current_latitude,
                               gps_nav_current_longitude,
                               target_lat_used,
                               target_lon_used);
    if (gps_nav_distance_m > GPS_NAV_MAX_DISTANCE_M)
    {
        GPS_NavStop(GPS_NAV_STATE_PROTECT, GPS_NAV_PROTECT_DISTANCE_TOO_FAR);
        return;
    }
    is_last_point = (uint8)(tagert_point >= (uint8)(gps_point_count - 1u));
    if (gps_nav_distance_m <= GPS_NAV_ARRIVE_RADIUS_M)
    {
        uint8 unified_elem = (uint8)u32yuansu[tagert_point];
        if (unified_elem == NAV_ELEM_END || is_last_point)
        {
            GPS_NavStop(GPS_NAV_STATE_FINISHED, GPS_NAV_PROTECT_FINISHED);
            return;
        }
        if (Nav_UnifiedIsPassThrough(unified_elem))
        {
            tagert_point++;
            gps_nav_target_index = tagert_point;
            gps_nav_request_valid = 0u;
            return;
        }
        if (Nav_UnifiedIsMarker(unified_elem) || Nav_UnifiedIsTakeover(unified_elem))
        {
            GPS_NavTryEnterElement(tagert_point, unified_elem);
            return;
        }
        tagert_point++;
        gps_nav_target_index = tagert_point;
        gps_nav_request_valid = 0u;
        return;
    }
    GPS_NavSegmentMetrics(gps_nav_launch_latitude,
                          gps_nav_launch_longitude,
                          gps_nav_current_latitude,
                          gps_nav_current_longitude,
                          &dist_from_launch_m,
                          NULL);
#if NAV_FUSION_ENABLE && GPS_NAV_USE_FUSION_POSITION
    if (NavFusion_IsValid() != 0u)
    {
        const NavFusionState *fusion_st = NavFusion_GetState();
        if (fusion_st != NULL)
        {
            dist_from_launch_m = sqrtf(fusion_st->x_m * fusion_st->x_m +
                                       fusion_st->y_m * fusion_st->y_m);
        }
    }
#endif
    gps_nav_dist_from_launch_m = dist_from_launch_m;
    if (gps_nav_align_state == GPS_NAV_ALIGN_WAIT)
    {
        if (dist_from_launch_m < GPS_NAV_GPS_FIRST_DISTANCE_M)
        {
            gps_nav_target_imu_yaw_deg = gps_nav_launch_imu_yaw;
            gps_nav_body_target_yaw_deg = 0.0f;
        }
        else
        {
            if (GPS_NavIsCogSampleValid() == 0u)
            {
                GPS_NavStop(GPS_NAV_STATE_PROTECT, GPS_NAV_PROTECT_COG_INVALID);
                return;
            }
            gps_nav_gps_first_deg = GPS_GnssDirectionToSigned180((float)gnss.direction);
            gps_nav_euler_ref_at_first_deg = gps_nav_imu_yaw_deg;
            gps_nav_heading_bias_deg =
                GPS_Wrap180(gps_nav_gps_first_deg - gps_nav_euler_ref_at_first_deg);
            gps_nav_align_state = GPS_NAV_ALIGN_TRACKING;
            gps_nav_request_valid = 0u;
            gps_nav_target_imu_yaw_deg =
                GPS_Wrap180(gps_nav_euler_ref_at_first_deg -
                            gps_nav_geo_bearing_deg +
                            gps_nav_gps_first_deg);
            gps_nav_body_target_yaw_deg =
                GPS_Wrap180(gps_nav_target_imu_yaw_deg - gps_nav_imu_yaw_deg);
        }
    }
    else
    {
        gps_nav_target_imu_yaw_deg =
            GPS_Wrap180(gps_nav_euler_ref_at_first_deg -
                        gps_nav_geo_bearing_deg +
                        gps_nav_gps_first_deg);
        gps_nav_body_target_yaw_deg =
            GPS_Wrap180(gps_nav_target_imu_yaw_deg - gps_nav_imu_yaw_deg);
    }
    gps_nav_yaw_err_deg = (float)ange_deviation1(gps_nav_target_imu_yaw_deg, gps_nav_imu_yaw_deg);
    gps_nav_state = GPS_NAV_STATE_RUNNING;
    gps_nav_protect_reason = GPS_NAV_PROTECT_NONE;
    target_delta = (float)fabsf((float)ange_deviation1(gps_nav_target_imu_yaw_deg, gps_nav_last_requested_yaw));
    if (!gps_nav_request_valid ||
        target_delta > GPS_NAV_REISSUE_YAW_DEG ||
        (!steer_enable && !steer_yaw_request_pending &&
         fabsf(gps_nav_yaw_err_deg) > GPS_NAV_REISSUE_YAW_DEG))
    {
        steer_request_target_yaw(gps_nav_target_imu_yaw_deg);
        gps_nav_last_requested_yaw = gps_nav_target_imu_yaw_deg;
        gps_nav_request_valid = 1u;
    }
#endif
}
uint8 GPS_GetValidPointCount(void)
{
    return gps_point_count;
}
const char *GPS_GetElementName(uint32 element)
{
    return Nav_GetUnifiedElementName((uint8)element);
}
const char *GPS_GetNavStateName(uint8 state)
{
    switch (state)
    {
    case GPS_NAV_STATE_IDLE:
        return "Idle";
    case GPS_NAV_STATE_RUNNING:
        return "Run";
    case GPS_NAV_STATE_FINISHED:
        return "Done";
    case GPS_NAV_STATE_PROTECT:
        return "Protect";
    default:
        return "Unknown";
    }
}
const char *GPS_GetNavProtectName(uint8 reason)
{
    switch (reason)
    {
    case GPS_NAV_PROTECT_NONE:
        return "None";
    case GPS_NAV_PROTECT_GPS_INVALID:
        return "GpsBad";
    case GPS_NAV_PROTECT_TARGET_INVALID:
        return "TargetBad";
    case GPS_NAV_PROTECT_DISTANCE_TOO_FAR:
        return "Far";
    case GPS_NAV_PROTECT_FINISHED:
        return "Finished";
    case GPS_NAV_PROTECT_COG_INVALID:
        return "CogBad";
    default:
        return "Unknown";
    }
}
uint32 GPS_CycleCurrentElement(void)
{
    gps_current_yuansu = Nav_CycleUnifiedElement((uint8)gps_current_yuansu);
    return gps_current_yuansu;
}
void GPS_SavePointFromCoord(uint8 point_num, double latitude, double longitude, uint32 yuansu_num)
{
    if (point_num >= GPS_POINT_MAX)
    {
        return;
    }
    latitude_point[point_num] = latitude;
    longitude_point[point_num] = longitude;
    u32yuansu[point_num] = yuansu_num;
    if (gps_point_count <= point_num)
    {
        gps_point_count = (uint8)(point_num + 1u);
    }
    show_point = point_num;
    save_point = (gps_point_count < GPS_POINT_MAX) ? gps_point_count : (GPS_POINT_MAX - 1u);
}
uint8 GPS_SaveCurrentPointFromCoord(double latitude, double longitude)
{
    if (save_point >= GPS_POINT_MAX)
    {
        return 0u;
    }
    GPS_SavePointFromCoord(save_point, latitude, longitude, gps_current_yuansu);
    return 1u;
}
void specialpoint(uint8 point_num, uint32 yuansu_num)
{
#if defined(CY_CORE_CM7_1)
    (void)point_num;
    (void)yuansu_num;
#else
    GPS_SavePointFromCoord(point_num, gnss.latitude, gnss.longitude, yuansu_num);
#endif
}
void gps_first_clearerr_from_coord(double cur_j, double cur_w, double first_j, double first_w, uint8 num)
{
    double d_j = cur_j - first_j;
    double d_w = cur_w - first_w;
    uint8 i = 0;
    uint8 count = (num > GPS_POINT_MAX) ? GPS_POINT_MAX : num;
    for (i = 0; i < count; i++)
    {
        latitude_point[i] = latitude_point[i] + d_w;
        longitude_point[i] = longitude_point[i] + d_j;
    }
}
void gps_first_clearerr(double first_j, double first_w, uint8 num)
{
#if defined(CY_CORE_CM7_1)
    (void)first_j;
    (void)first_w;
    (void)num;
#else
    gps_first_clearerr_from_coord(gnss.longitude, gnss.latitude, first_j, first_w, num);
#endif
}
void GPS_Path_Draw(const double *lat_buf, const double *lot_buf, uint16 len, uint8 carseat_enable, uint8 dir)
{
#if defined(CY_CORE_CM7_1)
    GPS_Path_DrawWithCar(lat_buf, lot_buf, len, 0u, dir, 0.0, 0.0, 31u, 9u, 64u, 90u);
    (void)carseat_enable;
#else
    GPS_Path_DrawWithCar(lat_buf, lot_buf, len, carseat_enable, dir, gnss.latitude, gnss.longitude, 31u, 9u, 64u, 90u);
#endif
}
void GPS_Path_DrawWithCar(const double *lat_buf,
                          const double *lot_buf,
                          uint16 len,
                          uint8 carseat_enable,
                          uint8 dir,
                          double car_latitude,
                          double car_longitude,
                          uint16 x_offset,
                          uint16 y_offset,
                          uint16 width,
                          uint16 height)
{
    double lat_max = 0.0;
    double lat_min = 90.0;
    double lon_max = 0.0;
    double lon_min = 360.0;
    double lat_range = 0.0;
    double lon_range = 0.0;
    uint16 point_len = 0;
    uint16 i = 0;
    uint16 lat_mapping[GPS_POINT_MAX] = {0};
    uint16 lon_mapping[GPS_POINT_MAX] = {0};
    uint16 car_y = 0;
    uint16 car_x = 0;
    uint8 car_valid = (uint8)(carseat_enable && (car_latitude != 0.0) && (car_longitude != 0.0));
    if ((lat_buf == NULL) || (lot_buf == NULL) || (width == 0u) || (height == 0u))
    {
        return;
    }
    GPS_FillPathViewport(x_offset, y_offset, width, height);
    if (len > GPS_POINT_MAX)
    {
        len = GPS_POINT_MAX;
    }
    for (i = 0; i < len; i++)
    {
        if ((lat_buf[i] == 0.0) || (lot_buf[i] == 0.0))
        {
            break;
        }
        if ((i + 1u < len) && (lat_buf[i + 1u] != 0.0) && (fabs(lat_buf[i + 1u] - lat_buf[i]) >= 3.0))
        {
            point_len = (uint16)(i + 1u);
            break;
        }
        point_len = (uint16)(i + 1u);
    }
    if ((point_len == 0u) && !car_valid)
    {
        return;
    }
    if (point_len == 0u)
    {
        lat_max = car_latitude;
        lat_min = car_latitude;
        lon_max = car_longitude;
        lon_min = car_longitude;
    }
    else
    {
        lat_max = lat_buf[0];
        lat_min = lat_buf[0];
        lon_max = lot_buf[0];
        lon_min = lot_buf[0];
        for (i = 1; i < point_len; i++)
        {
            if (lat_buf[i] > lat_max)
            {
                lat_max = lat_buf[i];
            }
            if (lat_buf[i] < lat_min)
            {
                lat_min = lat_buf[i];
            }
            if (lot_buf[i] > lon_max)
            {
                lon_max = lot_buf[i];
            }
            if (lot_buf[i] < lon_min)
            {
                lon_min = lot_buf[i];
            }
        }
    }
    GPS_ExpandRange(&lat_min, &lat_max);
    GPS_ExpandRange(&lon_min, &lon_max);
    lat_range = lat_max - lat_min;
    lon_range = lon_max - lon_min;
    if (fabs(lat_range) < GPS_PATH_MIN_RANGE_DEG)
    {
        lat_range = GPS_PATH_MIN_RANGE_DEG;
    }
    if (fabs(lon_range) < GPS_PATH_MIN_RANGE_DEG)
    {
        lon_range = GPS_PATH_MIN_RANGE_DEG;
    }
    for (i = 0; i < point_len; i++)
    {
        double y_ratio = (lat_max - lat_buf[i]) / lat_range;
        double x_ratio = (lon_max - lot_buf[i]) / lon_range;
        int32 mapped_y = (int32)y_offset + (int32)(y_ratio * (double)height);
        int32 mapped_x = (int32)x_offset + (int32)((1.0 - x_ratio) * (double)width);
        if (dir == south)
        {
            mapped_y = (int32)y_offset + (int32)height - (mapped_y - (int32)y_offset);
            mapped_x = (int32)x_offset + (int32)width - (mapped_x - (int32)x_offset);
        }
        lat_mapping[i] = GPS_ClampU16(mapped_y, y_offset, (uint16)(y_offset + height));
        lon_mapping[i] = GPS_ClampU16(mapped_x, x_offset, (uint16)(x_offset + width));
    }
    if (car_valid)
    {
        double car_y_ratio = (lat_max - car_latitude) / lat_range;
        double car_x_ratio = (lon_max - car_longitude) / lon_range;
        int32 mapped_y = (int32)y_offset + (int32)(car_y_ratio * (double)height);
        int32 mapped_x = (int32)x_offset + (int32)((1.0 - car_x_ratio) * (double)width);
        if (dir == south)
        {
            mapped_y = (int32)y_offset + (int32)height - (mapped_y - (int32)y_offset);
            mapped_x = (int32)x_offset + (int32)width - (mapped_x - (int32)x_offset);
        }
        car_y = GPS_ClampU16(mapped_y, y_offset, (uint16)(y_offset + height));
        car_x = GPS_ClampU16(mapped_x, x_offset, (uint16)(x_offset + width));
    }
    for (i = 1; i < point_len; i++)
    {
        ips200_draw_line(lon_mapping[i - 1u], lat_mapping[i - 1u], lon_mapping[i], lat_mapping[i], RGB565_RED);
    }
    for (i = 0; i < point_len; i++)
    {
        GPS_DrawViewportPoint(lon_mapping[i],
                              lat_mapping[i],
                              x_offset,
                              y_offset,
                              width,
                              height,
                              RGB565_BLUE);
    }
    if (car_valid)
    {
        GPS_DrawViewportPoint(car_x,
                              car_y,
                              x_offset,
                              y_offset,
                              width,
                              height,
                              RGB565_PURPLE);
    }
}
