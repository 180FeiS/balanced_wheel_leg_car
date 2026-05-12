#ifndef CODE_MY_GPS_H_
#define CODE_MY_GPS_H_

#include "zf_common_typedef.h"

/*
 * GPS 路点导航使用说明（摘要）：
 * 1) 录点：KEY1 开始录点，录点态下 KEY3 依次保存路点；第一次保存的点为 index0，用作「录径参考起点」。
 * 2) 存盘：KEY2 将路点写入 flash（数组为录点时的原始经纬度）。
 * 3) 发车：KEY3 进入导航；若 GPS 有效，计算漂移修正量
 *    delta = 发车时 GNSS − latitude_point[0]/longitude_point[0]，导航目标使用「路点 + delta」（与 gps_first_clearerr 同号约定）。
 * 4) 航向标定：发车后先保持「目标航向 = 发车锁存 IMU yaw」直线行驶，避免起步阶段路况干扰；当相对发车点位移 ≥
 *    GPS_NAV_GPS_FIRST_DISTANCE_M（默认 3 m）时，读取 RMC 的 gnss.direction（COG，真北 0–360°）映射为 GPS_first，
 *    计算 gps_nav_heading_bias_deg = Wrap180(GPS_first − euler_angle.yaw)，再按路点地理方位追迹。与坐标漂移修正相互独立。
 * 5) 勿与 gps_first_clearerr* 同时对同一次运行做「改表 + 运行时加 delta」，否则双重平移。
 */

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
/* 相对发车锁存经纬度累计位移达到该值后，用 RMC 的 gnss.direction（COG）作 GPS_first 与 IMU 对齐。 */
#define GPS_NAV_GPS_FIRST_DISTANCE_M 3.0f

typedef enum
{
    south = 0, // 朝南发车
    north = 1, // 朝北发车
    dong = 2,  // 朝东发车
    xi = 3,    // 朝西发车
} car_dir;
/* car_dir / car_gps_dir 仅用于路径小地图绘制朝向等 UI，不参与 GPS 点导航航向换算。 */

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
    GPS_NAV_PROTECT_COG_INVALID = 5, /* 达标定距离后 RMC 无有效 COG（如 gnss.state==0），无法锁 GPS_first */
} gps_nav_protect_enum;

typedef enum
{
    GPS_NAV_ALIGN_WAIT = 0,      /* 位移未到 GPS_NAV_GPS_FIRST_DISTANCE_M：目标航向=发车锁存 yaw */
    GPS_NAV_ALIGN_TRACKING = 1u, /* 已用 COG 锁过 bias，按「当前→路点」地理方位闭环 */
} gps_nav_align_state_enum;

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
extern uint8 gps_nav_align_state;     /* gps_nav_align_state_enum，供屏显/双核调试 */
extern float gps_nav_heading_bias_deg; /* 3 m 处 Wrap180(gps_first−imu)；追点 target_imu=Wrap180(地理方位−bias) */
extern float gps_nav_gps_first_deg;    /* ±180°，RMC COG 映射值，与 subject2 的 GPS_first 同义；WAIT 段为 0 */
extern float gps_nav_dist_from_launch_m; /* 当前距发车锁存点的位移（屏显 Lm），仅调试 */
/* 录点 index0 相对本次发车 GNSS 的经纬平移（度）；effective 时导航目标为 路点 + delta，不改 flash 数组 */
extern double gps_drift_delta_lat;
extern double gps_drift_delta_lon;
extern uint8 gps_drift_corr_valid; /* 1：本次发车已锁定 delta（需首点与发车定位均有效） */

void GPS_ClearPoints(void);
void GPS_BeginRecord(void);
void GPS_EndRecord(void);
/* KEY3 发车：切 NAV_HEADING_MODE_GPS、装 run_launch_speed、锁存 IMU yaw 与起点经纬；先直行至 GPS_NAV_GPS_FIRST_DISTANCE_M 再用 COG 锁 bias。 */
void GPS_ApplyLaunchSpeed(void);
/* 5ms 软任务：GPS 模式下先标定偏置再追点，经 steer_request_target_yaw 登记绝对航向（与惯导回放分离）。 */
void GPS_PointNav_Run(void);
uint8 GPS_GetValidPointCount(void);
const char *GPS_GetElementName(uint32 element);
const char *GPS_GetNavStateName(uint8 state);
const char *GPS_GetNavProtectName(uint8 reason);
uint32 GPS_CycleCurrentElement(void);
void GPS_SavePointFromCoord(uint8 point_num, double latitude, double longitude, uint32 yuansu_num);
uint8 GPS_SaveCurrentPointFromCoord(double latitude, double longitude);

void specialpoint(uint8 point_num, uint32 yuansu_num);
/* 将录点数组整表平移到「当前 GNSS 相对参考点」坐标系；与发车自动漂移修正互斥，勿与 RUNNING 时 GPS 导航叠用。 */
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
