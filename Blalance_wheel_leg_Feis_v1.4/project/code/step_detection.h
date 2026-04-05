#ifndef CODE_STEP_DETECTION_H_
#define CODE_STEP_DETECTION_H_

#include "zf_common_headfile.h"

#define STEP_HEIGHT_MM     50      // 台阶高度 50mm
#define STEP_LENGTH_MM     500     // 台阶长度 500mm
#define STEP_WIDTH_MM      500     // 台阶宽度 500mm

#define CAMERA_HEIGHT_MM  200     // 摄像头安装高度（mm），根据实际调整
#define CAMERA_ANGLE_DEG  30      // 摄像头俯仰角度（度），根据实际调整
#define FOCAL_LENGTH_MM   2.5     // 摄像头焦距（mm），根据实际调整
#define PIXEL_SIZE_MM     0.004   // 像素尺寸（mm），根据实际调整

#define MIN_STEP_HEIGHT_PIX   5   // 最小台阶高度（像素）
#define MAX_STEP_HEIGHT_PIX   150 // 最大台阶高度（像素）

typedef struct {
    uint8 detected;           // 是否检测到台阶
    uint16 step_row;          // 台阶底部边缘行坐标
    uint16 step_top_row;      // 台阶顶部边缘行坐标
    uint16 step_height_pix;   // 台阶高度（像素）
    float distance_mm;        // 台阶距离（mm）
    float distance_cm;        // 台阶距离（cm）
} step_info_t;

extern step_info_t step_data;

void step_detection_init(void);
uint8 step_detect(void);
float calculate_step_distance(uint16 step_height_pix);
void step_reset_distance_tracking(void);

#endif /* CODE_STEP_DETECTION_H_ */
