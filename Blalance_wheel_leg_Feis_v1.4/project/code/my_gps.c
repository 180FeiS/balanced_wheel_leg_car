#include "zf_common_headfile.h"
#include "my_gps.h"

#include <math.h>
#include <string.h>

#define GPS_NAV_PI 3.14159265358979323846
#define GPS_NAV_EARTH_RADIUS_M 6371000.0
#define GPS_NAV_DEG_TO_RAD(x) ((x) * GPS_NAV_PI / 180.0)
#define GPS_NAV_RAD_TO_DEG(x) ((x) * 180.0 / GPS_NAV_PI)

car_dir car_gps_dir = north;

uint8 now_point = 0;
uint8 tagert_point = 0;
uint8 save_point = 0;
uint8 show_point = 0;
uint8 gps_point_count = 0;
uint8 gps_recording_active = 0;
uint32 gps_current_yuansu = GPS_ELEMENT_NORMAL;

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

static float gps_nav_launch_imu_yaw = 0.0f;
static float gps_nav_last_requested_yaw = 0.0f;
static uint8 gps_nav_request_valid = 0u;

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

static float GPS_GetLaunchGeoHeading(void)
{
    switch (car_gps_dir)
    {
    case north:
        return 0.0f;
    case dong:
        return 90.0f;
    case south:
        return 180.0f;
    case xi:
        return 270.0f;
    default:
        return 0.0f;
    }
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
}

#if !defined(CY_CORE_CM7_1)
static uint8 GPS_NavIsCurrentCoordValid(double latitude, double longitude)
{
    uint8 frame_seen = (uint8)((gnss.time.year != 0u) || (gnss.state != 0u) || (gnss.satellite_used != 0u));
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
}

static void GPS_NavCalcDistanceBearing(double current_latitude,
                                       double current_longitude,
                                       double target_latitude,
                                       double target_longitude)
{
    double current_lat_rad = GPS_NAV_DEG_TO_RAD(current_latitude);
    double target_lat_rad = GPS_NAV_DEG_TO_RAD(target_latitude);
    double d_lat_rad = GPS_NAV_DEG_TO_RAD(target_latitude - current_latitude);
    double d_lon_rad = GPS_NAV_DEG_TO_RAD(target_longitude - current_longitude);
    double avg_lat_rad = (current_lat_rad + target_lat_rad) * 0.5;
    double north_m = d_lat_rad * GPS_NAV_EARTH_RADIUS_M;
    double east_m = d_lon_rad * GPS_NAV_EARTH_RADIUS_M * cos(avg_lat_rad);
    double bearing_deg = GPS_NAV_RAD_TO_DEG(atan2(east_m, north_m));

    gps_nav_distance_m = (float)sqrt(east_m * east_m + north_m * north_m);
    gps_nav_geo_bearing_deg = GPS_Wrap360((float)bearing_deg);
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
    gps_current_yuansu = GPS_ELEMENT_NORMAL;
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
    gps_nav_launch_imu_yaw = (float)euler_angle.yaw;
    nav_heading_mode = NAV_HEADING_MODE_GPS;
    motor_user_speed_cmd = run_launch_speed;
#endif
}

void GPS_PointNav_Run(void)
{
#if defined(CY_CORE_CM7_1)
    (void)0;
#else
    float target_delta = 0.0f;
    float launch_geo_heading = 0.0f;
    uint8 is_last_point = 0u;

    if (nav_heading_mode != NAV_HEADING_MODE_GPS)
    {
        return;
    }

    if (gps_nav_state == GPS_NAV_STATE_FINISHED || gps_nav_state == GPS_NAV_STATE_PROTECT)
    {
        motor_user_speed_cmd = 0.0f;
        return;
    }

    gps_nav_current_latitude = gnss.latitude;
    gps_nav_current_longitude = gnss.longitude;
    gps_nav_imu_yaw_deg = (float)euler_angle.yaw;

    if (!GPS_NavIsCurrentCoordValid(gps_nav_current_latitude, gps_nav_current_longitude))
    {
        GPS_NavStop(GPS_NAV_STATE_PROTECT, GPS_NAV_PROTECT_GPS_INVALID);
        return;
    }

    if (gps_point_count < GPS_NAV_MIN_POINT_COUNT || !GPS_NavIsTargetValid(tagert_point))
    {
        GPS_NavStop(GPS_NAV_STATE_PROTECT, GPS_NAV_PROTECT_TARGET_INVALID);
        return;
    }

    gps_nav_target_index = tagert_point;
    now_point = tagert_point;
    gps_nav_target_latitude = latitude_point[tagert_point];
    gps_nav_target_longitude = longitude_point[tagert_point];

    GPS_NavCalcDistanceBearing(gps_nav_current_latitude,
                               gps_nav_current_longitude,
                               gps_nav_target_latitude,
                               gps_nav_target_longitude);

    if (gps_nav_distance_m > GPS_NAV_MAX_DISTANCE_M)
    {
        GPS_NavStop(GPS_NAV_STATE_PROTECT, GPS_NAV_PROTECT_DISTANCE_TOO_FAR);
        return;
    }

    is_last_point = (uint8)(tagert_point >= (uint8)(gps_point_count - 1u));
    if (gps_nav_distance_m <= GPS_NAV_ARRIVE_RADIUS_M)
    {
        if (is_last_point)
        {
            GPS_NavStop(GPS_NAV_STATE_FINISHED, GPS_NAV_PROTECT_FINISHED);
        }
        else
        {
            tagert_point++;
            gps_nav_target_index = tagert_point;
            gps_nav_request_valid = 0u;
        }
        return;
    }

    launch_geo_heading = GPS_GetLaunchGeoHeading();
    gps_nav_body_target_yaw_deg = GPS_Wrap180(gps_nav_geo_bearing_deg - launch_geo_heading);
    gps_nav_target_imu_yaw_deg = GPS_Wrap180(gps_nav_launch_imu_yaw + gps_nav_body_target_yaw_deg);
    gps_nav_yaw_err_deg = (float)ange_deviation1(gps_nav_target_imu_yaw_deg, gps_nav_imu_yaw_deg);
    gps_nav_state = GPS_NAV_STATE_RUNNING;
    gps_nav_protect_reason = GPS_NAV_PROTECT_NONE;

    /* GPS 5ms 周期只在目标航向发生有效变化时重新登记请求，避免持续重置转向 PID。 */
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
    switch (element)
    {
    case GPS_ELEMENT_NORMAL:
        return "Normal";
    case GPS_ELEMENT_TURNAROUND:
        return "Turn";
    case GPS_ELEMENT_END:
        return "End";
    case GPS_ELEMENT_STEP:
        return "Step";
    case GPS_ELEMENT_SINGLE_BRIDGE:
        return "Bridge";
    case GPS_ELEMENT_BUMP:
        return "Bump";
    case GPS_ELEMENT_GRASS:
        return "Grass";
    case GPS_ELEMENT_INS_IN:
        return "INS-In";
    case GPS_ELEMENT_INS_OUT:
        return "INS-Out";
    default:
        return "Unknown";
    }
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
    default:
        return "Unknown";
    }
}

uint32 GPS_CycleCurrentElement(void)
{
    gps_current_yuansu++;
    if (gps_current_yuansu >= GPS_ELEMENT_COUNT)
    {
        gps_current_yuansu = GPS_ELEMENT_NORMAL;
    }
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

    if (car_valid)
    {
        lat_max = car_latitude;
        lat_min = car_latitude;
        lon_max = car_longitude;
        lon_min = car_longitude;
    }

    for (i = 0; i < point_len; i++)
    {
        if (!car_valid && (i == 0u))
        {
            lat_max = lat_buf[i];
            lat_min = lat_buf[i];
            lon_max = lot_buf[i];
            lon_min = lot_buf[i];
        }
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

    lat_range = lat_max - lat_min;
    lon_range = lon_max - lon_min;
    if (fabs(lat_range) < 0.000001)
    {
        lat_range = 0.000001;
    }
    if (fabs(lon_range) < 0.000001)
    {
        lon_range = 0.000001;
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
        ips200_draw_point(lon_mapping[i], lat_mapping[i], RGB565_BLUE);
        ips200_draw_point((uint16)(lon_mapping[i] + 1u), lat_mapping[i], RGB565_BLUE);
        ips200_draw_point((uint16)(lon_mapping[i] - (lon_mapping[i] > 0u ? 1u : 0u)), lat_mapping[i], RGB565_BLUE);
        ips200_draw_point(lon_mapping[i], (uint16)(lat_mapping[i] + 1u), RGB565_BLUE);
        ips200_draw_point(lon_mapping[i], (uint16)(lat_mapping[i] - (lat_mapping[i] > 0u ? 1u : 0u)), RGB565_BLUE);
    }

    if (car_valid)
    {
        ips200_draw_point(car_x, car_y, RGB565_PURPLE);
        ips200_draw_point((uint16)(car_x + 1u), car_y, RGB565_PURPLE);
        ips200_draw_point((uint16)(car_x - (car_x > 0u ? 1u : 0u)), car_y, RGB565_PURPLE);
        ips200_draw_point(car_x, (uint16)(car_y + 1u), RGB565_PURPLE);
        ips200_draw_point(car_x, (uint16)(car_y - (car_y > 0u ? 1u : 0u)), RGB565_PURPLE);
    }
}
