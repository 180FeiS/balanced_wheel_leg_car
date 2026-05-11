#ifndef CODE_MY_GPS_H_
#define CODE_MY_GPS_H_

#include "zf_common_typedef.h"

#define GPS_POINT_MAX 25u
/* GPS 点导航参数：
 * - ARRIVE_RADIUS_M：小于该距离认为到达当前路点；
 * - MAX_DISTANCE_M：单点距离异常保护，防止坏点或定位跳变导致车辆乱跑；
 * - REISSUE_YAW_DEG：目标航向变化超过该阈值才重新下发，避免 5ms 周期反复重置转向 PID。
 */
#define GPS_NAV_ARRIVE_RADIUS_M 1.5f
#define GPS_NAV_MAX_DISTANCE_M 200.0f
#define GPS_NAV_REISSUE_YAW_DEG 3.0f
#define GPS_NAV_MIN_POINT_COUNT 1u

typedef enum
{
    south = 0, // 朝南发车
    north = 1, // 朝北发车
    dong = 2,  // 朝东发车
    xi = 3,    // 朝西发车
} car_dir;

typedef enum
{
    GPS_ELEMENT_NORMAL = 0,        // 正常点
    GPS_ELEMENT_TURNAROUND = 1,    // 折返点
    GPS_ELEMENT_END = 2,           // 终点
    GPS_ELEMENT_STEP = 3,          // 台阶
    GPS_ELEMENT_SINGLE_BRIDGE = 4, // 单边桥
    GPS_ELEMENT_BUMP = 5,          // 颠簸
    GPS_ELEMENT_GRASS = 6,         // 草坪
    GPS_ELEMENT_INS_IN = 7,        // 惯导切入
    GPS_ELEMENT_INS_OUT = 8,       // 惯导切出
    GPS_ELEMENT_COUNT = 9,
} gps_element_enum;

typedef enum
{
    GPS_NAV_STATE_IDLE = 0,     // 未启动 GPS 点导航
    GPS_NAV_STATE_RUNNING = 1,  // 正在根据 GPS 点请求目标航向
    GPS_NAV_STATE_FINISHED = 2, // 最后一个路点到达，已停车
    GPS_NAV_STATE_PROTECT = 3,  // GPS/目标/距离异常，已停车保护
} gps_nav_state_enum;

typedef enum
{
    GPS_NAV_PROTECT_NONE = 0,
    GPS_NAV_PROTECT_GPS_INVALID = 1,
    GPS_NAV_PROTECT_TARGET_INVALID = 2,
    GPS_NAV_PROTECT_DISTANCE_TOO_FAR = 3,
    GPS_NAV_PROTECT_FINISHED = 4,
} gps_nav_protect_enum;

extern car_dir car_gps_dir;

extern uint8 save_point;   // 下一个保存点序号
extern uint8 show_point;   // 当前显示/查看点序号
extern uint8 now_point;    // 当前寻迹点
extern uint8 tagert_point; // 目标寻迹点，保留源工程命名
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

void GPS_ClearPoints(void);
void GPS_BeginRecord(void);
void GPS_EndRecord(void);
/* GPS 页面 Idle 下 KEY3 发车入口：切换 GPS 航向模式、装载发车速度并锁存发车瞬间 IMU yaw。 */
void GPS_ApplyLaunchSpeed(void);
/* 5ms 软任务入口：只在 NAV_HEADING_MODE_GPS 下计算目标点航向并请求现有转向闭环。 */
void GPS_PointNav_Run(void);
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

#endif /* CODE_MY_GPS_H_ */
