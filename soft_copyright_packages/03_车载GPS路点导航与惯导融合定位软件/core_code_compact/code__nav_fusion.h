#ifndef CODE_NAV_FUSION_H_
#define CODE_NAV_FUSION_H_
#include "zf_common_typedef.h"
#define NAV_FUSION_ENABLE 1u
extern uint8 g_menu_nav_fusion_enable;
static inline uint8 NavFusion_IsRuntimeEnabled(void)
{
#if NAV_FUSION_ENABLE
    return (uint8)(g_menu_nav_fusion_enable != 0u);
#else
    return 0u;
#endif
}
#define NAV_FUSION_ORIGIN_ENABLE 1u
#define NAV_FUSION_ORIGIN_SAMPLE_COUNT 50u
#define NAV_FUSION_ORIGIN_OUTLIER_M 2.0f
#define NAV_FUSION_ORIGIN_MIN_SATELLITES 10u
#define NAV_FUSION_ORIGIN_TIMEOUT_MS 10000u
#define NAV_FUSION_ORIGIN_MIN_SAMPLES 25u
#define NAV_FUSION_ORIGIN_FEED_COLLECTING 0u
#define NAV_FUSION_ORIGIN_FEED_DONE 1u
#define NAV_FUSION_ORIGIN_FEED_FAILED 2u
#define NAV_FUSION_DT_S 0.001f
#ifndef NAV_FUSION_SPEED_SCALE
#define NAV_FUSION_SPEED_SCALE 0.0037f
#endif
#define NAV_FUSION_GPS_GATE_M 3.0f
#define NAV_FUSION_GPS_GAIN 0.10f
#define NAV_FUSION_ZERO_SPEED_THRESHOLD 10.0f
#define NAV_FUSION_ZERO_SPEED_STABLE_COUNT 10u
#define NAV_FUSION_GPS_TIMEOUT_MS 3000u
#define NAV_FUSION_GPS_GOOD_STREAK 2u
#define NAV_FUSION_HEADING_CALIB_ENABLE 1u
#define NAV_FUSION_HEADING_CALIB_DISTANCE_M 5.0f
#define NAV_FUSION_HEADING_CALIB_TIMEOUT_MS 60000u
#define NAV_FUSION_HEADING_FEED_COLLECTING 0u
#define NAV_FUSION_HEADING_FEED_DONE       1u
#define NAV_FUSION_HEADING_FEED_FAILED     2u
#define NAV_FUSION_CALIB_IDLE           0u
#define NAV_FUSION_CALIB_ORIGIN         1u
#define NAV_FUSION_CALIB_STRAIGHT_HOLD  2u
#define NAV_FUSION_CALIB_HEADING_ALIGN  3u
#define NAV_FUSION_CALIB_READY          4u
#define NAV_FUSION_CALIB_FAILED         5u
#define NAV_FUSION_EARTH_RADIUS_M 6371000.0
#define NAV_FUSION_PI 3.14159265358979323846f
#define NAV_FUSION_DEG_TO_RAD(x) ((x) * (NAV_FUSION_PI / 180.0f))
#define NAV_FUSION_RAD_TO_DEG(x) ((x) * (180.0f / NAV_FUSION_PI))
typedef struct
{
    float x_m;
    float y_m;
    float yaw_deg;
    float v_mps;
    float gps_residual_m;
    float gps_weight;
    uint8 valid;
    uint8 gps_used;
} NavFusionState;
void NavFusion_Reset(void);
uint8 NavFusion_InitFromGps(double lat, double lon, float yaw_deg);
void NavFusion_Predict1ms(float yaw_deg, float speed_src);
void NavFusion_ZeroVelocityUpdate(float speed_src);
void NavFusion_UpdateGps(double lat, double lon, uint8 gps_state, uint8 satellite_used);
const NavFusionState *NavFusion_GetState(void);
uint8 NavFusion_IsValid(void);
void NavFusion_GetPositionLatLon(double *lat_out, double *lon_out);
void NavFusion_LatLonToLocal(double lat, double lon, float *x_m, float *y_m);
float NavFusion_GetMileageStepCm(void);
void NavFusion_SyncMileageSnapshot(void);
#if NAV_FUSION_ENABLE && NAV_FUSION_ORIGIN_ENABLE
void NavFusion_BeginOriginAverage(float yaw_deg);
uint8 NavFusion_FeedOriginSample(double lat, double lon, uint8 gps_state, uint8 satellite_used);
uint8 NavFusion_IsOriginCalibrating(void);
uint16 NavFusion_GetOriginAcceptedCount(void);
uint16 NavFusion_GetOriginRejectedCount(void);
uint8 NavFusion_GetOriginAvgResult(double *lat_out, double *lon_out);
uint8 NavFusion_ConsumeOriginFailure(void);
uint8 NavFusion_ConsumeOriginDonePulse(void);
#else
void NavFusion_BeginOriginAverage(float yaw_deg);
uint8 NavFusion_FeedOriginSample(double lat, double lon, uint8 gps_state, uint8 satellite_used);
uint8 NavFusion_IsOriginCalibrating(void);
uint16 NavFusion_GetOriginAcceptedCount(void);
uint16 NavFusion_GetOriginRejectedCount(void);
uint8 NavFusion_GetOriginAvgResult(double *lat_out, double *lon_out);
uint8 NavFusion_ConsumeOriginFailure(void);
uint8 NavFusion_ConsumeOriginDonePulse(void);
#endif
#if NAV_FUSION_ENABLE && NAV_FUSION_HEADING_CALIB_ENABLE
void NavFusion_BeginHeadingCalibSession(float launch_yaw_deg);
uint8 NavFusion_IsHeadingCalibSessionActive(void);
uint8 NavFusion_IsHeadingCalibrating(void);
uint8 NavFusion_IsHeadingAlignPending(void);
uint8 NavFusion_IsHeadingCalibReady(void);
uint8 NavFusion_FeedHeadingAlignSample(float cog_deg_0_360, uint8 gps_state, float imu_yaw_deg);
uint8 NavFusion_IsGpsPositionUpdateAllowed(void);
float NavFusion_GetLaunchYawHoldDeg(void);
float NavFusion_GetHeadingBiasDeg(void);
float NavFusion_GetHoldDistM(void);
float NavFusion_GetHeadingCogDeg(void);
float NavFusion_GetHeadingImuRefDeg(void);
uint8 NavFusion_GetHeadingCalibState(void);
uint8 NavFusion_ConsumeHeadingCalibDonePulse(void);
uint8 NavFusion_ConsumeHeadingCalibFailure(void);
#else
void NavFusion_BeginHeadingCalibSession(float launch_yaw_deg);
uint8 NavFusion_IsHeadingCalibSessionActive(void);
uint8 NavFusion_IsHeadingCalibrating(void);
uint8 NavFusion_IsHeadingAlignPending(void);
uint8 NavFusion_IsHeadingCalibReady(void);
uint8 NavFusion_FeedHeadingAlignSample(float cog_deg_0_360, uint8 gps_state, float imu_yaw_deg);
uint8 NavFusion_IsGpsPositionUpdateAllowed(void);
float NavFusion_GetLaunchYawHoldDeg(void);
float NavFusion_GetHeadingBiasDeg(void);
float NavFusion_GetHoldDistM(void);
float NavFusion_GetHeadingCogDeg(void);
float NavFusion_GetHeadingImuRefDeg(void);
uint8 NavFusion_GetHeadingCalibState(void);
uint8 NavFusion_ConsumeHeadingCalibDonePulse(void);
uint8 NavFusion_ConsumeHeadingCalibFailure(void);
#endif
#endif
