/*********************************************************************************************************************
 * @file    image.h
 * @brief   TC387 camera 模块迁入：1/2 压缩 `image_two_value`、大津、二值化、边线/最长白列、软件自动曝光等。
 *
 * @note
 *   - MT9V03X_H、MT9V03X_W 须为偶数；压缩尺寸为 `IMAGE_COMPRESS_H/W`。
 *   - 遍历/识别建议以 `image_two_value` 为坐标系（与同尺寸二值图一致）。
 *   - `step_detection.c` 若仍用全场 `mt9v03x_image`，与菜单压缩显示可能不一致，见 UI 注释。
 *********************************************************************************************************************/
#ifndef PROJECT_CODE_IMAGE_H_
#define PROJECT_CODE_IMAGE_H_

#include "zf_common_typedef.h"
#include "zf_device_mt9v03x.h"

#if ((MT9V03X_H) & 1u) || ((MT9V03X_W) & 1u)
#error "MT9V03X_H/W must be even for 1/2 compress"
#endif

#define IMAGE_COMPRESS_H    (MT9V03X_H / 2)
#define IMAGE_COMPRESS_W    (MT9V03X_W / 2)

#define IMAGE_BIN_BLACK     ((uint8)0x00)
#define IMAGE_BIN_WHITE     ((uint8)0xff)
/** TC：`IMG_BLACK` / `IMG_WHITE`，与二值化占位一致 */
#define IMG_BLACK           IMAGE_BIN_BLACK
#define IMG_WHITE           IMAGE_BIN_WHITE

/** 当前写入摄像头的曝光时间；与 TC `Camera_exposure` 对应，由 `mt9v03x_set_exposure_time` 下发 */
extern uint16 image_camera_exposure;

/** 压缩灰度 + 二值/巡线共用缓冲（TC `image_two_value`） */
extern uint8 image_two_value[IMAGE_COMPRESS_H][IMAGE_COMPRESS_W]
    __attribute__((aligned(4)));

extern int    int_test_printf;
extern int    hd_threshold;
extern float  Cammer_Err;
extern int    end_line;
extern int    white_sum;
extern int    Threshold;
extern int    test_printf_light;

extern volatile int Search_Stop_Line;
extern volatile int Left_Line[IMAGE_COMPRESS_H];
extern volatile int Right_Line[IMAGE_COMPRESS_H];
extern volatile int Mid_Line[IMAGE_COMPRESS_H];
extern volatile int Boundry_Start_Left;
extern volatile int Boundry_Start_Right;
extern volatile int Left_Lost_Time;
extern volatile int Right_Lost_Time;
extern volatile int Both_Lost_Time;
extern volatile int Road_Wide[IMAGE_COMPRESS_H];
extern volatile int White_Column[IMAGE_COMPRESS_W];

extern int Longest_White_Column_Left[2];
extern int Longest_White_Column_Right[2];

void    image_photo_compress        (const uint8 *src_full_row0);
/** 与 `image_photo_compress` 等价，旧代码/菜单仍可用此名 */
void    image_gray_compress_from_full(const uint8 *src_full_row0);

uint8   image_otsu_threshold        (const uint8 *image, uint16 col, uint16 row);
/** 对当前 `image_two_value` 做大津，结果写入 `Threshold` 可选由调用方读取 */
uint8   image_otsu_on_process_buf   (void);

void    image_binarize_buffer       (uint8 *buf, uint16 col, uint16 row,
                                     int threshold, uint8 black, uint8 white);
/** 就地二值化，仅处理 `image_two_value`（TC `Image_Binarization`） */
void    image_binarization_inplace    (int threshold);

void    hd_whitemax                 (int bw_Threshold);
void    camera_huidu                (int bw_Threshold);
void    Longest_White_Column        (void);

float   Err_Sum                     (void);
float   Err_bx_Sum                  (void);

/**
 * @brief TC `v_iftc_camera_autoexposure`：在压缩 ROI 上统计亮度闭环调节 `image_camera_exposure`。
 * @note 不修改 `mt9v03x_finish_flag`（与台阶检测等共用场中断时的约定）；单次调用内有界迭代。
 */
void    image_camera_auto_exposure  (void);

#endif /* PROJECT_CODE_IMAGE_H_ */
