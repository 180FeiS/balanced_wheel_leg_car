/*
 * nav_fusion.c — GPS + IMU/编码器 二维松耦合融合
 *
 * 实车验证建议（按顺序）：
 * 1. 静止 30s：fusion_x/y 基本不变，gps_residual_m 反映 GPS 抖动。
 * 2. 直线 5~10m：标定 NAV_FUSION_SPEED_SCALE，融合里程与卷尺一致。
 * 3. GPS 单点：融合 GPS 导航到点误差应小于纯 GPS。
 * 4. 惯导长距离 + 元素停车：NAG_USE_FUSION_MILEAGE 开启后 Run_index 漂移减小。
 * 5. 发车原点：KEY3 后静止采集 50 个有效 GPS 点平均建 origin（见 NAV_FUSION_ORIGIN_*）。
 * 6. 融合惯导录制/回放：原点完成后锁定发车 yaw 直行 5m，COG 标定北向偏角，首段路径旋转后恢复 GPS 修正。
 *
 * VOFA：菜单 n 切到组 2（VOFA_GROUP_FUSION_DEBUG）观察 fusion_x/y、gps_residual_m 等。
 */
#include "zf_common_headfile.h"
#include "nav_fusion.h"
#include "control.h"
#include <math.h>
#include <string.h>
#if NAV_FUSION_ENABLE
#ifndef NAV_FUSION_SPEED_SCALE_USER
/* 默认与惯导里程标定一致：Nag_Speed_To_Mileage_Scale 为 cm/s，此处换算为 m/s */
#undef NAV_FUSION_SPEED_SCALE
#define NAV_FUSION_SPEED_SCALE (Nag_Speed_To_Mileage_Scale / 100.0f)
#endif
static NavFusionState g_fusion;
static double g_origin_lat = 0.0;
static double g_origin_lon = 0.0;
static float g_prev_x_m = 0.0f;
static float g_prev_y_m = 0.0f;
static uint8 g_prev_xy_valid = 0u;
static uint16 g_zero_speed_count = 0u;
static uint16 g_gps_timeout_ms = 0u;
static uint8 g_gps_good_streak = 0u;
#if NAV_FUSION_ORIGIN_ENABLE
static struct
{
    uint8 active;
    float yaw_deg;
    uint16 accepted;
    uint16 rejected;
    double lat_sum;
    double lon_sum;
    double mean_lat;
    double mean_lon;
    double result_lat;
    double result_lon;
    uint16 timeout_ms;
    uint8 failed_pulse;
    uint8 origin_done_pulse;
} g_origin_avg;
static uint8 NavFusion_FinishOriginAverage(void);
#endif
#if NAV_FUSION_HEADING_CALIB_ENABLE
static struct
{
    uint8 session_active;
    uint8 state;
    uint8 gps_update_allowed;
    uint8 done_pulse;
    uint8 failed_pulse;
    float launch_yaw_deg;
    float hold_dist_m;
    float heading_bias_deg;
    float cog_deg;
    float imu_ref_deg;
    uint16 timeout_ms;
} g_heading_calib;
static void NavFusion_HeadingCalibReset(void);
static void NavFusion_OnOriginReadyForHeadingCalib(void);
static void NavFusion_RotatePathToNorthFrame(float bias_deg);
static uint8 NavFusion_FinishHeadingAlign(float cog_deg_0_360, uint8 gps_state, float imu_yaw_deg);
static void NavFusion_TickHeadingCalibTimeout(void);
static float NavFusion_GnssDirectionToSigned180(float direction_deg_0_360);
void NavFusion_SyncMileageSnapshot(void);
#endif
static uint8 NavFusion_IsCoordValid(double lat, double lon)
{
    if (lat == 0.0 || lon == 0.0)
    {
        return 0u;
    }
    return 1u;
}
static float NavFusion_Wrap180(float angle_deg)
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
#if NAV_FUSION_HEADING_CALIB_ENABLE
static void NavFusion_HeadingCalibReset(void)
{
    memset(&g_heading_calib, 0, sizeof(g_heading_calib));
}
static float NavFusion_GnssDirectionToSigned180(float direction_deg_0_360)
{
    float w = direction_deg_0_360;
    while (w >= 360.0f)
    {
        w -= 360.0f;
    }
    while (w < 0.0f)
    {
        w += 360.0f;
    }
    if (w > 180.0f)
    {
        return w - 360.0f;
    }
    return w;
}
static void NavFusion_OnOriginReadyForHeadingCalib(void)
{
    if (g_heading_calib.session_active == 0u)
    {
        return;
    }
    g_heading_calib.state = NAV_FUSION_CALIB_STRAIGHT_HOLD;
    g_heading_calib.hold_dist_m = 0.0f;
    g_heading_calib.timeout_ms = 0u;
    g_heading_calib.gps_update_allowed = 0u;
}
static void NavFusion_RotatePathToNorthFrame(float bias_deg)
{
    float bias_rad;
    float sin_b;
    float cos_b;
    float x_old;
    float y_old;
    bias_rad = NAV_FUSION_DEG_TO_RAD(bias_deg);
    sin_b = sinf(bias_rad);
    cos_b = cosf(bias_rad);
    x_old = g_fusion.x_m;
    y_old = g_fusion.y_m;
    g_fusion.x_m = x_old * cos_b + y_old * sin_b;
    g_fusion.y_m = -x_old * sin_b + y_old * cos_b;
}
static uint8 NavFusion_FinishHeadingAlign(float cog_deg_0_360, uint8 gps_state, float imu_yaw_deg)
{
    float cog_signed;
    float imu_ref;
    float bias_deg;
    if (gps_state == 0u)
    {
        g_heading_calib.state = NAV_FUSION_CALIB_FAILED;
        g_heading_calib.failed_pulse = 1u;
        return NAV_FUSION_HEADING_FEED_FAILED;
    }
    cog_signed = NavFusion_GnssDirectionToSigned180(cog_deg_0_360);
    imu_ref = NavFusion_Wrap180(imu_yaw_deg);
    bias_deg = NavFusion_Wrap180(cog_signed - imu_ref);
    g_heading_calib.cog_deg = cog_signed;
    g_heading_calib.imu_ref_deg = imu_ref;
    g_heading_calib.heading_bias_deg = bias_deg;
    NavFusion_RotatePathToNorthFrame(bias_deg);
    NavFusion_SyncMileageSnapshot();
    g_heading_calib.gps_update_allowed = 1u;
    g_heading_calib.state = NAV_FUSION_CALIB_READY;
    g_heading_calib.done_pulse = 1u;
    return NAV_FUSION_HEADING_FEED_DONE;
}
static void NavFusion_TickHeadingCalibTimeout(void)
{
    if (g_heading_calib.session_active == 0u)
    {
        return;
    }
    if (g_heading_calib.state != NAV_FUSION_CALIB_STRAIGHT_HOLD &&
        g_heading_calib.state != NAV_FUSION_CALIB_HEADING_ALIGN)
    {
        return;
    }
    if (g_heading_calib.timeout_ms < 0xFFFFu)
    {
        g_heading_calib.timeout_ms++;
    }
    if (g_heading_calib.timeout_ms < NAV_FUSION_HEADING_CALIB_TIMEOUT_MS)
    {
        return;
    }
    g_heading_calib.state = NAV_FUSION_CALIB_FAILED;
    g_heading_calib.failed_pulse = 1u;
}
#endif /* NAV_FUSION_HEADING_CALIB_ENABLE */
#if NAV_FUSION_ORIGIN_ENABLE
static void NavFusion_OriginAvgReset(void)
{
    memset(&g_origin_avg, 0, sizeof(g_origin_avg));
}
/* 样本到临时参考点（运行均值）的平面距离，m；不依赖 g_origin_lat/lon */
static float NavFusion_DistanceToRefM(double lat, double lon, double ref_lat, double ref_lon)
{
    double ref_lat_rad;
    double d_lat_rad;
    double d_lon_rad;
    float x_m;
    float y_m;
    ref_lat_rad = (double)NAV_FUSION_DEG_TO_RAD((float)ref_lat);
    d_lat_rad = (double)NAV_FUSION_DEG_TO_RAD((float)(lat - ref_lat));
    d_lon_rad = (double)NAV_FUSION_DEG_TO_RAD((float)(lon - ref_lon));
    x_m = (float)(d_lon_rad * NAV_FUSION_EARTH_RADIUS_M * cos(ref_lat_rad));
    y_m = (float)(d_lat_rad * NAV_FUSION_EARTH_RADIUS_M);
    return sqrtf(x_m * x_m + y_m * y_m);
}
static uint8 NavFusion_FinishOriginAverage(void)
{
    double avg_lat;
    double avg_lon;
    if (g_origin_avg.accepted == 0u)
    {
        return NAV_FUSION_ORIGIN_FEED_FAILED;
    }
    avg_lat = g_origin_avg.lat_sum / (double)g_origin_avg.accepted;
    avg_lon = g_origin_avg.lon_sum / (double)g_origin_avg.accepted;
    g_origin_avg.result_lat = avg_lat;
    g_origin_avg.result_lon = avg_lon;
    if (NavFusion_InitFromGps(avg_lat, avg_lon, g_origin_avg.yaw_deg) == 0u)
    {
        return NAV_FUSION_ORIGIN_FEED_FAILED;
    }
#if NAV_FUSION_HEADING_CALIB_ENABLE
    if (g_heading_calib.session_active != 0u)
    {
        g_origin_avg.origin_done_pulse = 1u;
        NavFusion_OnOriginReadyForHeadingCalib();
    }
    else
    {
        NavFusion_SyncMileageSnapshot();
    }
#else
    NavFusion_SyncMileageSnapshot();
#endif
    g_origin_avg.active = 0u;
    return NAV_FUSION_ORIGIN_FEED_DONE;
}
static void NavFusion_TickOriginTimeout(void)
{
    uint8 finish_code;
    if (g_origin_avg.active == 0u)
    {
        return;
    }
    if (g_origin_avg.timeout_ms < 0xFFFFu)
    {
        g_origin_avg.timeout_ms++;
    }
    if (g_origin_avg.timeout_ms < NAV_FUSION_ORIGIN_TIMEOUT_MS)
    {
        return;
    }
    if (g_origin_avg.accepted >= NAV_FUSION_ORIGIN_MIN_SAMPLES)
    {
        finish_code = NavFusion_FinishOriginAverage();
        if (finish_code == NAV_FUSION_ORIGIN_FEED_DONE)
        {
            return;
        }
    }
    g_origin_avg.active = 0u;
    g_origin_avg.failed_pulse = 1u;
}
#endif /* NAV_FUSION_ORIGIN_ENABLE */
void NavFusion_LatLonToLocal(double lat, double lon, float *x_m, float *y_m)
{
    double origin_lat_rad;
    double d_lat_rad;
    double d_lon_rad;
    if (NavFusion_IsRuntimeEnabled() == 0u)
    {
        return;
    }
    origin_lat_rad = (double)NAV_FUSION_DEG_TO_RAD((float)g_origin_lat);
    d_lat_rad = (double)NAV_FUSION_DEG_TO_RAD((float)(lat - g_origin_lat));
    d_lon_rad = (double)NAV_FUSION_DEG_TO_RAD((float)(lon - g_origin_lon));
    if (x_m != NULL)
    {
        *x_m = (float)(d_lon_rad * NAV_FUSION_EARTH_RADIUS_M * cos(origin_lat_rad));
    }
    if (y_m != NULL)
    {
        *y_m = (float)(d_lat_rad * NAV_FUSION_EARTH_RADIUS_M);
    }
}
void NavFusion_Reset(void)
{
    memset(&g_fusion, 0, sizeof(g_fusion));
    g_origin_lat = 0.0;
    g_origin_lon = 0.0;
    g_prev_x_m = 0.0f;
    g_prev_y_m = 0.0f;
    g_prev_xy_valid = 0u;
    g_zero_speed_count = 0u;
    g_gps_timeout_ms = 0u;
    g_gps_good_streak = 0u;
#if NAV_FUSION_ORIGIN_ENABLE
    NavFusion_OriginAvgReset();
#endif
#if NAV_FUSION_HEADING_CALIB_ENABLE
    NavFusion_HeadingCalibReset();
#endif
}
uint8 NavFusion_InitFromGps(double lat, double lon, float yaw_deg)
{
    if (NavFusion_IsRuntimeEnabled() == 0u)
    {
        return 0u;
    }
    if (!NavFusion_IsCoordValid(lat, lon))
    {
        return 0u;
    }
    g_origin_lat = lat;
    g_origin_lon = lon;
    g_fusion.x_m = 0.0f;
    g_fusion.y_m = 0.0f;
    g_fusion.yaw_deg = NavFusion_Wrap180(yaw_deg);
    g_fusion.v_mps = 0.0f;
    g_fusion.gps_residual_m = 0.0f;
    g_fusion.gps_weight = 0.0f;
    g_fusion.valid = 1u;
    g_fusion.gps_used = 0u;
    g_prev_x_m = 0.0f;
    g_prev_y_m = 0.0f;
    g_prev_xy_valid = 1u;
    g_zero_speed_count = 0u;
    g_gps_timeout_ms = 0u;
    g_gps_good_streak = 0u;
    return 1u;
}
void NavFusion_Predict1ms(float yaw_deg, float speed_src)
{
    float yaw_rad;
    float v_mps;
    float sin_yaw;
    float cos_yaw;
    float pred_yaw_deg;
    if (NavFusion_IsRuntimeEnabled() == 0u)
    {
        return;
    }
#if NAV_FUSION_ORIGIN_ENABLE
    NavFusion_TickOriginTimeout();
#endif
#if NAV_FUSION_HEADING_CALIB_ENABLE
    NavFusion_TickHeadingCalibTimeout();
#endif
    if (g_fusion.valid == 0u)
    {
        return;
    }
    pred_yaw_deg = NavFusion_Wrap180(yaw_deg);
#if NAV_FUSION_HEADING_CALIB_ENABLE
    if (g_heading_calib.session_active != 0u)
    {
        if (g_heading_calib.state == NAV_FUSION_CALIB_STRAIGHT_HOLD ||
            g_heading_calib.state == NAV_FUSION_CALIB_HEADING_ALIGN)
        {
            pred_yaw_deg = g_heading_calib.launch_yaw_deg;
        }
        else if (g_heading_calib.state == NAV_FUSION_CALIB_READY)
        {
            pred_yaw_deg = NavFusion_Wrap180(yaw_deg + g_heading_calib.heading_bias_deg);
        }
    }
#endif
    g_fusion.yaw_deg = pred_yaw_deg;
    v_mps = fabsf(speed_src) * NAV_FUSION_SPEED_SCALE;
    g_fusion.v_mps = v_mps;
    if (g_gps_timeout_ms < 0xFFFFu)
    {
        g_gps_timeout_ms++;
    }
    if (fabsf(speed_src) < NAV_FUSION_ZERO_SPEED_THRESHOLD)
    {
        return;
    }
#if NAV_FUSION_HEADING_CALIB_ENABLE
    if (g_heading_calib.session_active != 0u &&
        g_heading_calib.state == NAV_FUSION_CALIB_STRAIGHT_HOLD)
    {
        g_heading_calib.hold_dist_m += v_mps * NAV_FUSION_DT_S;
        if (g_heading_calib.hold_dist_m >= NAV_FUSION_HEADING_CALIB_DISTANCE_M)
        {
            g_heading_calib.state = NAV_FUSION_CALIB_HEADING_ALIGN;
        }
    }
#endif
    yaw_rad = NAV_FUSION_DEG_TO_RAD(g_fusion.yaw_deg);
    sin_yaw = sinf(yaw_rad);
    cos_yaw = cosf(yaw_rad);
    /* 与 my_gps 局地平面一致：x 东、y 北；yaw 0° 为北时 dx≈sin(yaw), dy≈cos(yaw) */
    g_fusion.x_m += v_mps * sin_yaw * NAV_FUSION_DT_S;
    g_fusion.y_m += v_mps * cos_yaw * NAV_FUSION_DT_S;
}
void NavFusion_ZeroVelocityUpdate(float speed_src)
{
    if (NavFusion_IsRuntimeEnabled() == 0u)
    {
        return;
    }
    if (g_fusion.valid == 0u)
    {
        return;
    }
    if (fabsf(speed_src) < NAV_FUSION_ZERO_SPEED_THRESHOLD)
    {
        if (g_zero_speed_count < 0xFFFFu)
        {
            g_zero_speed_count++;
        }
    }
    else
    {
        g_zero_speed_count = 0u;
    }
    if (g_zero_speed_count >= NAV_FUSION_ZERO_SPEED_STABLE_COUNT)
    {
        g_fusion.v_mps = 0.0f;
    }
}
void NavFusion_UpdateGps(double lat, double lon, uint8 gps_state, uint8 satellite_used)
{
    float gps_x;
    float gps_y;
    float residual_x;
    float residual_y;
    float residual_m;
    float gain;
    if (NavFusion_IsRuntimeEnabled() == 0u)
    {
        return;
    }
#if NAV_FUSION_ORIGIN_ENABLE
    /* 原点采集中：禁止 auto-init 与 GPS 修正，避免 origin 未建立时 fusion 被拉动 */
    if (NavFusion_IsOriginCalibrating() != 0u)
    {
        return;
    }
#endif
#if NAV_FUSION_HEADING_CALIB_ENABLE
    if (NavFusion_IsGpsPositionUpdateAllowed() == 0u)
    {
        return;
    }
#endif
    (void)satellite_used;
    g_gps_timeout_ms = 0u;
    g_fusion.gps_used = 0u;
    g_fusion.gps_weight = 0.0f;
    if (gps_state == 0u || !NavFusion_IsCoordValid(lat, lon))
    {
        g_gps_good_streak = 0u;
        return;
    }
    if (g_fusion.valid == 0u)
    {
#if NAV_FUSION_ORIGIN_ENABLE
        /* 已启用多点原点平均时，禁止首帧自动建原点，须等 FeedOriginSample 完成 */
        return;
#else
        (void)NavFusion_InitFromGps(lat, lon, 0.0f);
#endif
    }
    g_gps_good_streak++;
    if (g_gps_good_streak < NAV_FUSION_GPS_GOOD_STREAK)
    {
        return;
    }
    NavFusion_LatLonToLocal(lat, lon, &gps_x, &gps_y);
    residual_x = gps_x - g_fusion.x_m;
    residual_y = gps_y - g_fusion.y_m;
    residual_m = sqrtf(residual_x * residual_x + residual_y * residual_y);
    g_fusion.gps_residual_m = residual_m;
    if (residual_m >= NAV_FUSION_GPS_GATE_M)
    {
        return;
    }
    gain = NAV_FUSION_GPS_GAIN;
    g_fusion.x_m += gain * residual_x;
    g_fusion.y_m += gain * residual_y;
    g_fusion.gps_weight = gain;
    g_fusion.gps_used = 1u;
}
const NavFusionState *NavFusion_GetState(void)
{
    if (NavFusion_IsRuntimeEnabled() == 0u)
    {
        return NULL;
    }
    return &g_fusion;
}
uint8 NavFusion_IsValid(void)
{
    if (NavFusion_IsRuntimeEnabled() == 0u)
    {
        return 0u;
    }
    if (g_fusion.valid == 0u)
    {
        return 0u;
    }
    if (g_gps_timeout_ms > NAV_FUSION_GPS_TIMEOUT_MS)
    {
        /* 超时后仍可用惯导预测，但标记本周期未用 GPS */
        return 1u;
    }
    return 1u;
}
void NavFusion_GetPositionLatLon(double *lat_out, double *lon_out)
{
    double origin_lat_rad;
    double d_lat_rad;
    double d_lon_rad;
    if (NavFusion_IsRuntimeEnabled() == 0u)
    {
        return;
    }
    if (lat_out == NULL || lon_out == NULL || g_fusion.valid == 0u)
    {
        return;
    }
    origin_lat_rad = (double)NAV_FUSION_DEG_TO_RAD((float)g_origin_lat);
    d_lat_rad = (double)g_fusion.y_m / NAV_FUSION_EARTH_RADIUS_M;
    d_lon_rad = (double)g_fusion.x_m / (NAV_FUSION_EARTH_RADIUS_M * cos(origin_lat_rad));
    *lat_out = g_origin_lat + (double)NAV_FUSION_RAD_TO_DEG((float)d_lat_rad);
    *lon_out = g_origin_lon + (double)NAV_FUSION_RAD_TO_DEG((float)d_lon_rad);
}
float NavFusion_GetMileageStepCm(void)
{
    float dx;
    float dy;
    float step_m;
    if (NavFusion_IsRuntimeEnabled() == 0u)
    {
        return -1.0f;
    }
    if (g_fusion.valid == 0u || g_prev_xy_valid == 0u)
    {
        return -1.0f;
    }
    dx = g_fusion.x_m - g_prev_x_m;
    dy = g_fusion.y_m - g_prev_y_m;
    g_prev_x_m = g_fusion.x_m;
    g_prev_y_m = g_fusion.y_m;
    step_m = sqrtf(dx * dx + dy * dy);
    return step_m * 100.0f;
}
void NavFusion_SyncMileageSnapshot(void)
{
    if (NavFusion_IsRuntimeEnabled() == 0u)
    {
        return;
    }
    if (g_fusion.valid == 0u)
    {
        return;
    }
    g_prev_x_m = g_fusion.x_m;
    g_prev_y_m = g_fusion.y_m;
    g_prev_xy_valid = 1u;
}
#if NAV_FUSION_ORIGIN_ENABLE
void NavFusion_BeginOriginAverage(float yaw_deg)
{
    if (NavFusion_IsRuntimeEnabled() == 0u)
    {
        return;
    }
    NavFusion_Reset();
    g_origin_avg.active = 1u;
    g_origin_avg.yaw_deg = NavFusion_Wrap180(yaw_deg);
    g_origin_avg.timeout_ms = 0u;
}
uint8 NavFusion_IsOriginCalibrating(void)
{
    if (NavFusion_IsRuntimeEnabled() == 0u)
    {
        return 0u;
    }
    return g_origin_avg.active;
}
uint16 NavFusion_GetOriginAcceptedCount(void)
{
    if (NavFusion_IsRuntimeEnabled() == 0u)
    {
        return 0u;
    }
    return g_origin_avg.accepted;
}
uint16 NavFusion_GetOriginRejectedCount(void)
{
    if (NavFusion_IsRuntimeEnabled() == 0u)
    {
        return 0u;
    }
    return g_origin_avg.rejected;
}
uint8 NavFusion_GetOriginAvgResult(double *lat_out, double *lon_out)
{
    if (NavFusion_IsRuntimeEnabled() == 0u)
    {
        return 0u;
    }
    if (g_fusion.valid == 0u || g_origin_avg.accepted == 0u)
    {
        return 0u;
    }
    if (lat_out != NULL)
    {
        *lat_out = g_origin_avg.result_lat;
    }
    if (lon_out != NULL)
    {
        *lon_out = g_origin_avg.result_lon;
    }
    return 1u;
}
uint8 NavFusion_FeedOriginSample(double lat, double lon, uint8 gps_state, uint8 satellite_used)
{
    float dist_m;
    if (NavFusion_IsRuntimeEnabled() == 0u)
    {
        return NAV_FUSION_ORIGIN_FEED_COLLECTING;
    }
    if (g_origin_avg.failed_pulse != 0u)
    {
        g_origin_avg.failed_pulse = 0u;
        return NAV_FUSION_ORIGIN_FEED_FAILED;
    }
    if (g_origin_avg.active == 0u)
    {
        return NAV_FUSION_ORIGIN_FEED_COLLECTING;
    }
    if (gps_state == 0u || !NavFusion_IsCoordValid(lat, lon))
    {
        g_origin_avg.rejected++;
        return NAV_FUSION_ORIGIN_FEED_COLLECTING;
    }
    if (satellite_used < NAV_FUSION_ORIGIN_MIN_SATELLITES)
    {
        g_origin_avg.rejected++;
        return NAV_FUSION_ORIGIN_FEED_COLLECTING;
    }
    if (g_origin_avg.accepted > 0u)
    {
        dist_m = NavFusion_DistanceToRefM(lat, lon, g_origin_avg.mean_lat, g_origin_avg.mean_lon);
        if (dist_m >= NAV_FUSION_ORIGIN_OUTLIER_M)
        {
            g_origin_avg.rejected++;
            return NAV_FUSION_ORIGIN_FEED_COLLECTING;
        }
    }
    g_origin_avg.lat_sum += lat;
    g_origin_avg.lon_sum += lon;
    g_origin_avg.accepted++;
    g_origin_avg.mean_lat = g_origin_avg.lat_sum / (double)g_origin_avg.accepted;
    g_origin_avg.mean_lon = g_origin_avg.lon_sum / (double)g_origin_avg.accepted;
    if (g_origin_avg.accepted >= NAV_FUSION_ORIGIN_SAMPLE_COUNT)
    {
        return NavFusion_FinishOriginAverage();
    }
    return NAV_FUSION_ORIGIN_FEED_COLLECTING;
}
uint8 NavFusion_ConsumeOriginFailure(void)
{
    if (NavFusion_IsRuntimeEnabled() == 0u)
    {
        return 0u;
    }
    if (g_origin_avg.failed_pulse == 0u)
    {
        return 0u;
    }
    g_origin_avg.failed_pulse = 0u;
    return 1u;
}
uint8 NavFusion_ConsumeOriginDonePulse(void)
{
    if (NavFusion_IsRuntimeEnabled() == 0u)
    {
        return 0u;
    }
    if (g_origin_avg.origin_done_pulse == 0u)
    {
        return 0u;
    }
    g_origin_avg.origin_done_pulse = 0u;
    return 1u;
}
#endif /* NAV_FUSION_ORIGIN_ENABLE */
#if NAV_FUSION_HEADING_CALIB_ENABLE
void NavFusion_BeginHeadingCalibSession(float launch_yaw_deg)
{
    if (NavFusion_IsRuntimeEnabled() == 0u)
    {
        return;
    }
    g_heading_calib.session_active = 1u;
    g_heading_calib.state = NAV_FUSION_CALIB_IDLE;
    g_heading_calib.launch_yaw_deg = NavFusion_Wrap180(launch_yaw_deg);
    g_heading_calib.hold_dist_m = 0.0f;
    g_heading_calib.heading_bias_deg = 0.0f;
    g_heading_calib.cog_deg = 0.0f;
    g_heading_calib.imu_ref_deg = 0.0f;
    g_heading_calib.timeout_ms = 0u;
    g_heading_calib.gps_update_allowed = 0u;
    g_heading_calib.done_pulse = 0u;
    g_heading_calib.failed_pulse = 0u;
    if (g_fusion.valid != 0u && NavFusion_IsOriginCalibrating() == 0u)
    {
        NavFusion_OnOriginReadyForHeadingCalib();
    }
}
uint8 NavFusion_IsHeadingCalibSessionActive(void)
{
    if (NavFusion_IsRuntimeEnabled() == 0u)
    {
        return 0u;
    }
    return g_heading_calib.session_active;
}
uint8 NavFusion_IsHeadingCalibrating(void)
{
    if (NavFusion_IsRuntimeEnabled() == 0u)
    {
        return 0u;
    }
    if (g_heading_calib.session_active == 0u)
    {
        return 0u;
    }
    if (g_heading_calib.state == NAV_FUSION_CALIB_STRAIGHT_HOLD ||
        g_heading_calib.state == NAV_FUSION_CALIB_HEADING_ALIGN)
    {
        return 1u;
    }
    return 0u;
}
uint8 NavFusion_IsHeadingAlignPending(void)
{
    if (NavFusion_IsRuntimeEnabled() == 0u)
    {
        return 0u;
    }
    return (uint8)(g_heading_calib.session_active != 0u &&
                   g_heading_calib.state == NAV_FUSION_CALIB_HEADING_ALIGN);
}
uint8 NavFusion_IsHeadingCalibReady(void)
{
    if (NavFusion_IsRuntimeEnabled() == 0u)
    {
        return 0u;
    }
    if (g_heading_calib.session_active == 0u)
    {
        return 1u;
    }
    return (uint8)(g_heading_calib.state == NAV_FUSION_CALIB_READY);
}
uint8 NavFusion_FeedHeadingAlignSample(float cog_deg_0_360, uint8 gps_state, float imu_yaw_deg)
{
    if (NavFusion_IsRuntimeEnabled() == 0u)
    {
        return NAV_FUSION_HEADING_FEED_COLLECTING;
    }
    if (g_heading_calib.failed_pulse != 0u)
    {
        g_heading_calib.failed_pulse = 0u;
        return NAV_FUSION_HEADING_FEED_FAILED;
    }
    if (g_heading_calib.session_active == 0u ||
        g_heading_calib.state != NAV_FUSION_CALIB_HEADING_ALIGN)
    {
        return NAV_FUSION_HEADING_FEED_COLLECTING;
    }
    return NavFusion_FinishHeadingAlign(cog_deg_0_360, gps_state, imu_yaw_deg);
}
uint8 NavFusion_IsGpsPositionUpdateAllowed(void)
{
    if (NavFusion_IsRuntimeEnabled() == 0u)
    {
        return 0u;
    }
    if (g_fusion.valid == 0u)
    {
        return 0u;
    }
    if (g_heading_calib.session_active == 0u)
    {
        return 1u;
    }
    return g_heading_calib.gps_update_allowed;
}
float NavFusion_GetLaunchYawHoldDeg(void)
{
    return g_heading_calib.launch_yaw_deg;
}
float NavFusion_GetHeadingBiasDeg(void)
{
    return g_heading_calib.heading_bias_deg;
}
float NavFusion_GetHoldDistM(void)
{
    return g_heading_calib.hold_dist_m;
}
float NavFusion_GetHeadingCogDeg(void)
{
    return g_heading_calib.cog_deg;
}
float NavFusion_GetHeadingImuRefDeg(void)
{
    return g_heading_calib.imu_ref_deg;
}
uint8 NavFusion_GetHeadingCalibState(void)
{
#if NAV_FUSION_ORIGIN_ENABLE
    if (NavFusion_IsOriginCalibrating() != 0u)
    {
        return NAV_FUSION_CALIB_ORIGIN;
    }
#endif
    if (g_heading_calib.session_active == 0u)
    {
        return NAV_FUSION_CALIB_IDLE;
    }
    return g_heading_calib.state;
}
uint8 NavFusion_ConsumeHeadingCalibDonePulse(void)
{
    if (NavFusion_IsRuntimeEnabled() == 0u)
    {
        return 0u;
    }
    if (g_heading_calib.done_pulse == 0u)
    {
        return 0u;
    }
    g_heading_calib.done_pulse = 0u;
    return 1u;
}
uint8 NavFusion_ConsumeHeadingCalibFailure(void)
{
    if (NavFusion_IsRuntimeEnabled() == 0u)
    {
        return 0u;
    }
    if (g_heading_calib.failed_pulse == 0u)
    {
        return 0u;
    }
    g_heading_calib.failed_pulse = 0u;
    return 1u;
}
#endif /* NAV_FUSION_HEADING_CALIB_ENABLE */
#if NAV_FUSION_ENABLE && !NAV_FUSION_ORIGIN_ENABLE
void NavFusion_BeginOriginAverage(float yaw_deg) { (void)yaw_deg; }
uint8 NavFusion_FeedOriginSample(double lat, double lon, uint8 gps_state, uint8 satellite_used)
{
    (void)lat;
    (void)lon;
    (void)gps_state;
    (void)satellite_used;
    return NAV_FUSION_ORIGIN_FEED_COLLECTING;
}
uint8 NavFusion_IsOriginCalibrating(void) { return 0u; }
uint16 NavFusion_GetOriginAcceptedCount(void) { return 0u; }
uint16 NavFusion_GetOriginRejectedCount(void) { return 0u; }
uint8 NavFusion_GetOriginAvgResult(double *lat_out, double *lon_out)
{
    (void)lat_out;
    (void)lon_out;
    return 0u;
}
uint8 NavFusion_ConsumeOriginFailure(void) { return 0u; }
uint8 NavFusion_ConsumeOriginDonePulse(void) { return 0u; }
#endif
#if !NAV_FUSION_HEADING_CALIB_ENABLE
void NavFusion_BeginHeadingCalibSession(float launch_yaw_deg) { (void)launch_yaw_deg; }
uint8 NavFusion_IsHeadingCalibSessionActive(void) { return 0u; }
uint8 NavFusion_IsHeadingCalibrating(void) { return 0u; }
uint8 NavFusion_IsHeadingAlignPending(void) { return 0u; }
uint8 NavFusion_IsHeadingCalibReady(void) { return 1u; }
uint8 NavFusion_FeedHeadingAlignSample(float cog_deg_0_360, uint8 gps_state, float imu_yaw_deg)
{
    (void)cog_deg_0_360;
    (void)gps_state;
    (void)imu_yaw_deg;
    return NAV_FUSION_HEADING_FEED_COLLECTING;
}
uint8 NavFusion_IsGpsPositionUpdateAllowed(void) { return 1u; }
float NavFusion_GetLaunchYawHoldDeg(void) { return 0.0f; }
float NavFusion_GetHeadingBiasDeg(void) { return 0.0f; }
float NavFusion_GetHoldDistM(void) { return 0.0f; }
float NavFusion_GetHeadingCogDeg(void) { return 0.0f; }
float NavFusion_GetHeadingImuRefDeg(void) { return 0.0f; }
uint8 NavFusion_GetHeadingCalibState(void) { return NAV_FUSION_CALIB_IDLE; }
uint8 NavFusion_ConsumeHeadingCalibDonePulse(void) { return 0u; }
uint8 NavFusion_ConsumeHeadingCalibFailure(void) { return 0u; }
#endif
#else /* !NAV_FUSION_ENABLE */
void NavFusion_Reset(void) {}
uint8 NavFusion_InitFromGps(double lat, double lon, float yaw_deg)
{
    (void)lat;
    (void)lon;
    (void)yaw_deg;
    return 0u;
}
void NavFusion_Predict1ms(float yaw_deg, float speed_src)
{
    (void)yaw_deg;
    (void)speed_src;
}
void NavFusion_ZeroVelocityUpdate(float speed_src) { (void)speed_src; }
void NavFusion_UpdateGps(double lat, double lon, uint8 gps_state, uint8 satellite_used)
{
    (void)lat;
    (void)lon;
    (void)gps_state;
    (void)satellite_used;
}
const NavFusionState *NavFusion_GetState(void) { return NULL; }
uint8 NavFusion_IsValid(void) { return 0u; }
void NavFusion_GetPositionLatLon(double *lat_out, double *lon_out)
{
    (void)lat_out;
    (void)lon_out;
}
void NavFusion_LatLonToLocal(double lat, double lon, float *x_m, float *y_m)
{
    (void)lat;
    (void)lon;
    (void)x_m;
    (void)y_m;
}
float NavFusion_GetMileageStepCm(void) { return -1.0f; }
void NavFusion_SyncMileageSnapshot(void) {}
void NavFusion_BeginOriginAverage(float yaw_deg) { (void)yaw_deg; }
uint8 NavFusion_FeedOriginSample(double lat, double lon, uint8 gps_state, uint8 satellite_used)
{
    (void)lat;
    (void)lon;
    (void)gps_state;
    (void)satellite_used;
    return NAV_FUSION_ORIGIN_FEED_COLLECTING;
}
uint8 NavFusion_IsOriginCalibrating(void) { return 0u; }
uint16 NavFusion_GetOriginAcceptedCount(void) { return 0u; }
uint16 NavFusion_GetOriginRejectedCount(void) { return 0u; }
uint8 NavFusion_GetOriginAvgResult(double *lat_out, double *lon_out)
{
    (void)lat_out;
    (void)lon_out;
    return 0u;
}
uint8 NavFusion_ConsumeOriginFailure(void) { return 0u; }
uint8 NavFusion_ConsumeOriginDonePulse(void) { return 0u; }
void NavFusion_BeginHeadingCalibSession(float launch_yaw_deg) { (void)launch_yaw_deg; }
uint8 NavFusion_IsHeadingCalibSessionActive(void) { return 0u; }
uint8 NavFusion_IsHeadingCalibrating(void) { return 0u; }
uint8 NavFusion_IsHeadingAlignPending(void) { return 0u; }
uint8 NavFusion_IsHeadingCalibReady(void) { return 0u; }
uint8 NavFusion_FeedHeadingAlignSample(float cog_deg_0_360, uint8 gps_state, float imu_yaw_deg)
{
    (void)cog_deg_0_360;
    (void)gps_state;
    (void)imu_yaw_deg;
    return NAV_FUSION_HEADING_FEED_COLLECTING;
}
uint8 NavFusion_IsGpsPositionUpdateAllowed(void) { return 0u; }
float NavFusion_GetLaunchYawHoldDeg(void) { return 0.0f; }
float NavFusion_GetHeadingBiasDeg(void) { return 0.0f; }
float NavFusion_GetHoldDistM(void) { return 0.0f; }
float NavFusion_GetHeadingCogDeg(void) { return 0.0f; }
float NavFusion_GetHeadingImuRefDeg(void) { return 0.0f; }
uint8 NavFusion_GetHeadingCalibState(void) { return NAV_FUSION_CALIB_IDLE; }
uint8 NavFusion_ConsumeHeadingCalibDonePulse(void) { return 0u; }
uint8 NavFusion_ConsumeHeadingCalibFailure(void) { return 0u; }
#endif /* NAV_FUSION_ENABLE */
