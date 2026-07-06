#ifndef PROJECT_CODE_IMAGE_H_
#define PROJECT_CODE_IMAGE_H_
#include "zf_common_typedef.h"
#include "zf_device_mt9v03x.h"
#if ((MT9V03X_H) & 1u) || ((MT9V03X_W) & 1u)
#endif
#define IMAGE_COMPRESS_H    (MT9V03X_H / 2)
#define IMAGE_COMPRESS_W    (MT9V03X_W / 2)
#define IMAGE_BIN_BLACK     ((uint8)0x00)
#define IMAGE_BIN_WHITE     ((uint8)0xff)
#define IMG_BLACK           IMAGE_BIN_BLACK
#define IMG_WHITE           IMAGE_BIN_WHITE
extern uint16 image_camera_exposure;
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
typedef enum
{
    IMAGE_AE_IDLE = 0,
    IMAGE_AE_RUNNING,
    IMAGE_AE_DONE,
    IMAGE_AE_FAILED,
} image_ae_state_enum;
void    image_photo_compress        (const uint8 *src_full_row0);
void    image_gray_compress_from_full(const uint8 *src_full_row0);
uint8   image_otsu_threshold        (const uint8 *image, uint16 col, uint16 row);
uint8   image_otsu_on_process_buf   (void);
void    image_binarize_buffer       (uint8 *buf, uint16 col, uint16 row,
                                     int threshold, uint8 black, uint8 white);
void    image_binarization_inplace    (int threshold);
void    hd_whitemax                 (int bw_Threshold);
void    camera_huidu                (int bw_Threshold);
void    Longest_White_Column        (void);
float   Err_Sum                     (void);
float   Err_bx_Sum                  (void);
uint8   image_get_err_weight        (int row);
void    image_camera_exposure_flash_read(void);
void    image_camera_exposure_flash_write(void);
void    image_ae_session_arm(void);
void    image_ae_session_poll(void);
uint8   image_ae_session_is_active(void);
image_ae_state_enum image_ae_session_get_state(void);
uint8   image_ae_session_consume_done_and_save(void);
void    image_camera_auto_exposure  (void);
#ifndef IMAGE_BRIDGE_WHITE_BLOB_ENABLE
#define IMAGE_BRIDGE_WHITE_BLOB_ENABLE  1u
#endif
#ifndef IMAGE_WHITE_BLOB_VALIDATE_ENABLE
#define IMAGE_WHITE_BLOB_VALIDATE_ENABLE  0u
#endif
#ifndef IMAGE_WHITE_BLOB_ENABLE
#define IMAGE_WHITE_BLOB_ENABLE         IMAGE_WHITE_BLOB_VALIDATE_ENABLE
#endif
#if (IMAGE_BRIDGE_WHITE_BLOB_ENABLE || IMAGE_WHITE_BLOB_VALIDATE_ENABLE)
#define IMAGE_WHITE_BLOB_ANY_ENABLE     1u
#else
#define IMAGE_WHITE_BLOB_ANY_ENABLE     0u
#endif
#define IMAGE_WHITE_BLOB_ROI_ROW_START    0
#define IMAGE_WHITE_BLOB_DETECT_ROW_END   ((int)IMAGE_COMPRESS_H - 10)
#define IMAGE_WHITE_BLOB_COL_MARGIN       8
#define IMAGE_WHITE_BLOB_MIN_AREA         60
#define IMAGE_WHITE_BLOB_MIN_MEAN_GRAY    28
#define IMAGE_WHITE_BLOB_THRESH_OFFSET    (-5)
#define IMAGE_WHITE_BLOB_SWITCH_ROW_MIN   (((int)IMAGE_COMPRESS_H * 2) / 3)
#define IMAGE_WHITE_BLOB_SWITCH_AREA_MIN  200
#define IMAGE_WHITE_BLOB_SWITCH_DEBOUNCE    4u
#define IMAGE_WHITE_BLOB_YAW_K_PIXEL        0.5f
#define IMAGE_WHITE_BLOB_YAW_MAX_OFFSET_DEG 30.0f
#define IMAGE_WHITE_BLOB_YAW_KP           IMAGE_WHITE_BLOB_YAW_K_PIXEL
#define IMAGE_WHITE_BLOB_YAW_MAX_DELTA    IMAGE_WHITE_BLOB_YAW_MAX_OFFSET_DEG
#define IMAGE_WHITE_BLOB_YAW_DEADBAND_DEG 0.1f
#define IMAGE_VISION_MODE_BLOB            1u
#define IMAGE_VISION_MODE_MIDLINE         2u
typedef struct
{
    float center_err;
    int   cx;
    int   cy;
    int   area;
    int   bottom_row;
    int   top_row;
    int   bbox_w;
    int   bbox_h;
    uint8 track_valid;
} image_white_blob_result_t;
#if IMAGE_WHITE_BLOB_ANY_ENABLE
void image_white_blob_detect(image_white_blob_result_t *out);
#endif
#if IMAGE_BRIDGE_WHITE_BLOB_ENABLE
void image_bridge_blob_process_frame(int bw_threshold);
#endif
#if IMAGE_WHITE_BLOB_VALIDATE_ENABLE
void image_vision_guidance_process_frame(int bw_threshold);
void image_vision_guidance_reset(void);
void image_vision_guidance_apply_yaw(void);
void image_white_blob_apply_yaw(void);
void image_midline_apply_yaw(void);
#endif
#endif
