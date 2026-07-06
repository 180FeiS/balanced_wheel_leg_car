/*********************************************************************************************************************
 * @file    image.h
 * @brief   TC387 camera 模块迁入：1/2 压缩 `image_two_value`、大津、二值化、边线/最长白列、软件自动曝光等。
 *
 * @note
 *   - MT9V03X_H、MT9V03X_W 须为偶数；压缩尺寸为 `IMAGE_COMPRESS_H/W`。
 *   - 遍历/识别建议以 `image_two_value` 为坐标系（与同尺寸二值图一致）。
 *   - 主循环在 Debug→Image（pos 2.1*）不调用 step_detect，AE 独占 mt9v03x_finish_flag。
 *   - step_detect() 仅在惯导 ENTER_STAIR 激活（dualcore stair_enter_active）时由 CM7_1 调用。
 *   - 进入 Debug→Image（pos 2.1）时 arm AE 会话；收敛后写 Flash 页 49；上电 camera_init 读回。
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

typedef enum
{
    IMAGE_AE_IDLE = 0,
    IMAGE_AE_RUNNING,
    IMAGE_AE_DONE,
    IMAGE_AE_FAILED,
} image_ae_state_enum;

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
/** 行加权表，row 越界返回 0 */
uint8   image_get_err_weight        (int row);

/** Flash 页 49：上电读 / AE 收敛后写 */
void    image_camera_exposure_flash_read(void);
void    image_camera_exposure_flash_write(void);

/** 进入 Image 菜单区（pos 2.1*）时 arm；主循环 poll；DONE/FAILED 后 consume 写 Flash */
void    image_ae_session_arm(void);
void    image_ae_session_poll(void);
uint8   image_ae_session_is_active(void);
image_ae_state_enum image_ae_session_get_state(void);
/** 返回 1：已写 Flash 并回到 IDLE */
uint8   image_ae_session_consume_done_and_save(void);

/** 阻塞版（调试用）；菜单路径请用 session API */
void    image_camera_auto_exposure  (void);

/*--------------------------------------------------------------------------------------------------------------------
 * 上半 ROI 最大白连通域引导（室外验证：朝远处白色目标区修正 yaw，进入后可切 single_bridge 中线）
 *-------------------------------------------------------------------------------------------------------------------*/

/** 1=启用白连通域检测与 yaw 引导；验证期常开，正式比赛改 0 */
#ifndef IMAGE_WHITE_BLOB_ENABLE
#define IMAGE_WHITE_BLOB_ENABLE         1u
#endif

/** 连通域扫描行范围 [START, END)：近底留 10 行给车体/畸变 */
#define IMAGE_WHITE_BLOB_ROI_ROW_START    0
#define IMAGE_WHITE_BLOB_DETECT_ROW_END   ((int)IMAGE_COMPRESS_H - 10)

/** 左右忽略列数，避开镜头暗角 */
#define IMAGE_WHITE_BLOB_COL_MARGIN       8

/** 最小白块像素数，低于此 track_valid=0 */
#define IMAGE_WHITE_BLOB_MIN_AREA         60

/** 大津阈值偏移；室外光强时可微调（负值更严、正值更松） */
#define IMAGE_WHITE_BLOB_THRESH_OFFSET    (-5)

/** 白块→中线：最大连通域最靠下行 >= 该值（压缩图行，约 2/3 幅高） */
#define IMAGE_WHITE_BLOB_SWITCH_ROW_MIN   (((int)IMAGE_COMPRESS_H * 2) / 3)

/** 白块→中线：面积需明显大于远处小目标，低于 MIN_AREA 的误检 */
#define IMAGE_WHITE_BLOB_SWITCH_AREA_MIN  200

/** 白块→中线：上述条件连续满足的帧数，防抖 */
#define IMAGE_WHITE_BLOB_SWITCH_DEBOUNCE    4u

/** 横向误差 → 每帧相对 yaw 增量（deg），方向反了可改符号；中线模式复用 */
#define IMAGE_WHITE_BLOB_YAW_KP           0.06f

/** 单帧 yaw 修正上限（deg），防止猛打方向 */
#define IMAGE_WHITE_BLOB_YAW_MAX_DELTA    2.5f

/** yaw 死区（deg），抑制抖动 */
#define IMAGE_WHITE_BLOB_YAW_DEADBAND_DEG 0.3f

/** dualcore vision_guidance_mode 取值：白块目标引导 */
#define IMAGE_VISION_MODE_BLOB            1u
/** dualcore vision_guidance_mode 取值：左右中寻线 */
#define IMAGE_VISION_MODE_MIDLINE         2u

typedef struct
{
    float center_err;   /**< 图像中心 - 白块质心 x；正=目标在右，需向右修正 yaw */
    int   cx;           /**< 最大白连通域质心列（压缩图坐标） */
    int   cy;           /**< 最大白连通域质心行 */
    int   area;         /**< 最大白连通域像素数 */
    int   bottom_row;   /**< 最大白连通域最靠下行，用于判定是否进入中下部 */
    int   top_row;      /**< 最大白连通域最靠上行 */
    int   bbox_w;       /**< 包围盒宽度（列） */
    int   bbox_h;       /**< 包围盒高度（行） */
    uint8 track_valid;  /**< 1=area>=MIN_AREA 且检测成功 */
} image_white_blob_result_t;

/** CM7_1：压缩→二值化→全幅可用区最大白连通域；需先有新帧 mt9v03x_image */
void image_white_blob_detect(image_white_blob_result_t *out);

#if IMAGE_WHITE_BLOB_ENABLE
/**
 * CM7_1 主循环：白块/中线状态机（每帧调用一次，调用方须已置 mt9v03x_finish_flag=0）。
 * bw_threshold：中线模式传给 single_bridge_gray_diff_track 的差比和阈值。
 */
void image_vision_guidance_process_frame(int bw_threshold);

/** 离开视觉路径或进 Image 菜单 AE 时复位为白块模式 */
void image_vision_guidance_reset(void);

/** CM7_0 主循环：按 dualcore vision_guidance_mode 选择 blob 或中线 yaw 修正 */
void image_vision_guidance_apply_yaw(void);

/** CM7_0：仅白块通道 steer_request_relative_yaw（由 apply_yaw 内部调用） */
void image_white_blob_apply_yaw(void);

/** CM7_0：仅中线通道 steer_request_relative_yaw（由 apply_yaw 内部调用） */
void image_midline_apply_yaw(void);
#endif

#endif /* PROJECT_CODE_IMAGE_H_ */
