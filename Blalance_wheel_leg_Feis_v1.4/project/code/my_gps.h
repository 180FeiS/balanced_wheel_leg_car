#ifndef CODE_MY_GPS_H_
#define CODE_MY_GPS_H_

#include "zf_common_typedef.h"

#define GPS_POINT_MAX 25u

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

void GPS_ClearPoints(void);
void GPS_BeginRecord(void);
void GPS_EndRecord(void);
void GPS_ApplyLaunchSpeed(void);
void GPS_PointNav_Run(void);
uint8 GPS_GetValidPointCount(void);
const char *GPS_GetElementName(uint32 element);
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
