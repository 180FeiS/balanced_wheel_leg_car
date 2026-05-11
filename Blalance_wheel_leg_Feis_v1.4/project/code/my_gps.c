#include "zf_common_headfile.h"
#include "my_gps.h"

#include <math.h>
#include <string.h>

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
    nav_heading_mode = NAV_HEADING_MODE_GPS;
    motor_user_speed_cmd = run_launch_speed;
#endif
}

void GPS_PointNav_Run(void)
{
#if defined(CY_CORE_CM7_1)
    (void)0;
#else
    if (nav_heading_mode != NAV_HEADING_MODE_GPS)
    {
        return;
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
