#ifndef CODE_MY_GPS_H_
#define CODE_MY_GPS_H_
#include "zf_common_typedef.h"
#include "navigation.h"
#define GPS_NAV_USE_FUSION_POSITION 1u
#define GPS_POINT_MAX 25u
#define GPS_NAV_ARRIVE_RADIUS_M 1.5f
#define GPS_NAV_MAX_DISTANCE_M 200.0f
#define GPS_NAV_REISSUE_YAW_DEG 3.0f
#define GPS_NAV_MIN_POINT_COUNT 1u
#define GPS_NAV_GPS_FIRST_DISTANCE_M 5.0f
typedef enum
{
    south = 0,
    north = 1,
    dong = 2,
    xi = 3,
} car_dir;
typedef enum
{
    GPS_NAV_STATE_IDLE = 0,
    GPS_NAV_STATE_RUNNING = 1,
    GPS_NAV_STATE_FINISHED = 2,
    GPS_NAV_STATE_PROTECT = 3,
} gps_nav_state_enum;
typedef enum
{
    GPS_NAV_PROTECT_NONE = 0,
    GPS_NAV_PROTECT_GPS_INVALID = 1,
    GPS_NAV_PROTECT_TARGET_INVALID = 2,
    GPS_NAV_PROTECT_DISTANCE_TOO_FAR = 3,
    GPS_NAV_PROTECT_FINISHED = 4,
    GPS_NAV_PROTECT_COG_INVALID = 5,
} gps_nav_protect_enum;
typedef enum
{
    GPS_NAV_ALIGN_WAIT = 0,
    GPS_NAV_ALIGN_TRACKING = 1u,
} gps_nav_align_state_enum;
extern car_dir car_gps_dir;
extern uint8 save_point;
extern uint8 show_point;
extern uint8 now_point;
extern uint8 tagert_point;
extern uint8 gps_point_count;
extern uint8 gps_recording_active;
extern uint32 gps_current_yuansu;
extern double latitude_point[GPS_POINT_MAX];
extern double longitude_point[GPS_POINT_MAX];
extern uint32 u32yuansu[GPS_POINT_MAX];
extern uint8 gps_nav_state;
extern uint8 gps_nav_protect_reason;
extern uint8 gps_nav_target_index;
extern double gps_nav_current_latitude;
extern double gps_nav_current_longitude;
extern double gps_nav_target_latitude;
extern double gps_nav_target_longitude;
extern float gps_nav_distance_m;
extern float gps_nav_geo_bearing_deg;
extern float gps_nav_body_target_yaw_deg;
extern float gps_nav_target_imu_yaw_deg;
extern float gps_nav_imu_yaw_deg;
extern float gps_nav_yaw_err_deg;
extern uint8 gps_nav_align_state;
extern float gps_nav_heading_bias_deg;
extern float gps_nav_gps_first_deg;
extern float gps_nav_euler_ref_at_first_deg;
extern float gps_nav_dist_from_launch_m;
extern double gps_drift_delta_lat;
extern double gps_drift_delta_lon;
extern uint8 gps_drift_corr_valid;
void GPS_ClearPoints(void);
void GPS_BeginRecord(void);
void GPS_EndRecord(void);
void GPS_ApplyLaunchSpeed(void);
void GPS_CompleteLaunchAfterOrigin(void);
void GPS_OnOriginCalibrationFailed(void);
void GPS_PointNav_Run(void);
void GPS_NavForceEndPoint(void);
void GPS_NavTryEnterElement(uint8 point_index, uint8 unified_type);
void GPS_NavOnElementDone(void);
float GPS_NavDistanceToPointM(uint8 point_index);
uint8 GPS_GetValidPointCount(void);
const char *GPS_GetElementName(uint32 element);
const char *GPS_GetNavStateName(uint8 state);
const char *GPS_GetNavProtectName(uint8 reason);
uint32 GPS_CycleCurrentElement(void);
void GPS_SavePointFromCoord(uint8 point_num, double latitude, double longitude, uint32 yuansu_num);
uint8 GPS_SaveCurrentPointFromCoord(double latitude, double longitude);
void specialpoint(uint8 point_num, uint32 yuansu_num);
void gps_first_clearerr(double first_j, double first_w, uint8 num);
void gps_first_clearerr_from_coord(double cur_j, double cur_w, double first_j, double first_w, uint8 num);
void GPS_Path_Draw(const double *lat_buf, const double *lot_buf, uint16 len, uint8 carseat_enable, uint8 dir);
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
                          uint16 height);
#endif
