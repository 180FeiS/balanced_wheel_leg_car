/*********************************************************************************************************************
 * @file    image.c
 * @brief   TC387 camera.c 迁入：压缩、大津、二值、边线、最长白列、误差与自动曝光（见 image.h）。
 *********************************************************************************************************************/
#include "image.h"
#include "zf_driver_flash.h"
#include <stdlib.h>
#include <string.h>

uint16 image_camera_exposure = (uint16)MT9V03X_EXP_TIME_DEF;

uint8 image_two_value[IMAGE_COMPRESS_H][IMAGE_COMPRESS_W]
    __attribute__((aligned(4)));

int    int_test_printf;
int    hd_threshold;
float  Cammer_Err;
int    end_line;
int    white_sum;
int    Threshold;
int    test_printf_light;

volatile int Search_Stop_Line;
volatile int Left_Line[IMAGE_COMPRESS_H];
volatile int Right_Line[IMAGE_COMPRESS_H];
volatile int Mid_Line[IMAGE_COMPRESS_H];
volatile int Boundry_Start_Left;
volatile int Boundry_Start_Right;
volatile int Left_Lost_Time;
volatile int Right_Lost_Time;
volatile int Both_Lost_Time;
volatile int Road_Wide[IMAGE_COMPRESS_H];
volatile int White_Column[IMAGE_COMPRESS_W];

int Longest_White_Column_Left[2];
int Longest_White_Column_Right[2];

static int Left_Lost_Flag[IMAGE_COMPRESS_H];
static int Right_Lost_Flag[IMAGE_COMPRESS_H];

#if (IMAGE_COMPRESS_H != 60)
#error "Err_Sum 加权表依 TC 为 60 行；若改 MT9V03X_H 请重填 image_err_weight[]"
#endif

/** TC 原 `Weight[]`：行权重，与 `IMAGE_COMPRESS_H==60` 一致 */
static const uint8 image_err_weight[IMAGE_COMPRESS_H] =
{
    1, 1, 1, 1, 1,      1, 1, 1, 1, 1,
    1, 1, 1, 1, 1,      1, 1, 1, 1, 1,
    1, 4, 5, 6, 7,      9,11,13,15,17,
    19,20,20,19,17,     15,13,11, 9, 7,
    5, 3, 1, 1, 1,      1, 1, 1, 1, 1,
    1, 1, 1, 1, 1,      1, 1, 1, 1, 1,
};

/*--------------------------------------------------------------------------------------------------------------------
 * @brief  TC `photo_compress`：image_two_value[i][j] = raw[2i][2j]
 *-------------------------------------------------------------------------------------------------------------------*/
void image_photo_compress(const uint8 *src_full_row0)
{
    uint16 i;
    uint16 j;

    for (i = 0; i < IMAGE_COMPRESS_H; i++)
    {
        const uint8 *row = src_full_row0 + (uint32)(i * 2u) * (uint32)MT9V03X_W;

        for (j = 0; j < IMAGE_COMPRESS_W; j++)
        {
            image_two_value[i][j] = row[(uint32)j * 2u];
        }
    }
}

void image_gray_compress_from_full(const uint8 *src_full_row0)
{
    image_photo_compress(src_full_row0);
}

uint8 image_otsu_on_process_buf(void)
{
    return image_otsu_threshold(image_two_value[0], IMAGE_COMPRESS_W, IMAGE_COMPRESS_H);
}

void image_binarization_inplace(int threshold)
{
    image_binarize_buffer(image_two_value[0], IMAGE_COMPRESS_W, IMAGE_COMPRESS_H,
                          threshold, IMG_BLACK, IMG_WHITE);
}

uint8 image_otsu_threshold(const uint8 *image, uint16 col, uint16 row)
{
#define IMG_OTSU_GRAY_LEVELS (256)
    uint16 img_w = col;
    uint16 img_h = row;
    int32  hist[IMG_OTSU_GRAY_LEVELS];
    int32  x;
    uint16 y;
    const uint8 *data = image;
    uint32 amount = 0;
    uint32 pixel_back = 0;
    uint32 pixel_integral_back = 0;
    uint32 pixel_integral = 0;
    int32  pixel_integral_fore = 0;
    int32  pixel_fore = 0;
    double omega_back;
    double omega_fore;
    double micro_back;
    double micro_fore;
    double sigma_b;
    double sigma = 0;
    uint8  min_value = 0;
    uint8  max_value = 0;
    uint8  threshold = 0;

    for (x = 0; x < IMG_OTSU_GRAY_LEVELS; x++)
    {
        hist[x] = 0;
    }

    for (y = 0; y < img_h; y++)
    {
        for (x = 0; x < (int32)img_w; x++)
        {
            hist[(int)data[(uint32)y * img_w + (uint32)x]]++;
        }
    }

    min_value = 0;
    while ((min_value < 255u) && (hist[min_value] == 0))
    {
        min_value++;
    }
    max_value = 255u;
    while ((max_value > min_value) && (hist[max_value] == 0))
    {
        max_value--;
    }

    if (max_value == min_value)
    {
        return max_value;
    }
    if ((uint16)min_value + 1u == (uint16)max_value)
    {
        return min_value;
    }

    for (y = min_value; y <= max_value; y++)
    {
        amount += (uint32)hist[y];
    }

    pixel_integral = 0;
    for (y = min_value; y <= max_value; y++)
    {
        pixel_integral += (uint32)hist[y] * y;
    }

    pixel_back = 0;
    pixel_integral_back = 0;
    sigma_b = -1.0;
    for (y = min_value; y < max_value; y++)
    {
        pixel_back += (uint32)hist[y];
        pixel_fore = (int32)((int64)amount - (int64)pixel_back);

        omega_back = (double)pixel_back / (double)amount;
        omega_fore = (double)pixel_fore / (double)amount;

        pixel_integral_back += (uint32)hist[y] * y;
        pixel_integral_fore = (int32)((int64)pixel_integral - (int64)pixel_integral_back);

        if (pixel_back == 0u)
        {
            continue;
        }
        if (pixel_fore == 0)
        {
            break;
        }

        micro_back = (double)pixel_integral_back / (double)pixel_back;
        micro_fore = (double)pixel_integral_fore / (double)pixel_fore;

        sigma = omega_back * omega_fore * (micro_back - micro_fore) * (micro_back - micro_fore);

        if (sigma > sigma_b)
        {
            sigma_b = sigma;
            threshold = (uint8)y;
        }
    }

    return threshold;
#undef IMG_OTSU_GRAY_LEVELS
}

void image_binarize_buffer(uint8 *buf, uint16 col, uint16 row,
                           int threshold, uint8 black, uint8 white)
{
    uint16 i;
    uint16 j;

    for (i = 0; i < row; i++)
    {
        for (j = 0; j < col; j++)
        {
            if ((int32)buf[(uint32)i * col + (uint32)j] >= threshold)
            {
                buf[(uint32)i * col + (uint32)j] = white;
            }
            else
            {
                buf[(uint32)i * col + (uint32)j] = black;
            }
        }
    }
}

/*--------------------------------------------------------------------------------------------------------------------
 * @brief  TC `hd_whitemax`：最亮列、截止行、左右边（差比和）；已修正 `hd_white_sum` 累加前未清零。
 *-------------------------------------------------------------------------------------------------------------------*/
void hd_whitemax(int bw_Threshold)
{
    int i;
    int j;

    int hd_white_sum = 0;

    int differences_a;
    int differences_b;
    int differences_sum;

    int current_max;
    int current_sum;
    int maxline;

    maxline = 0;
    current_max = 0;
    current_sum = 0;

    for (j = 10; j < (int)IMAGE_COMPRESS_W - 10; j++)
    {
        current_sum = 0;
        for (i = 10; i < (int)IMAGE_COMPRESS_H - 10; i++)
        {
            current_sum += (int)image_two_value[i][j];
        }
        if (current_sum > current_max)
        {
            current_max = current_sum;
            maxline = j;
            int_test_printf = j;
        }
    }

    for (i = (int)IMAGE_COMPRESS_H - 10; i > 10; i--)
    {
        differences_a = (int)image_two_value[i][maxline];
        differences_b = (int)image_two_value[i - 1][maxline];
        if ((differences_a + differences_b) != 0)
        {
            differences_sum = abs(differences_a - differences_b) * 100 / (differences_a + differences_b);
        }
        else
        {
            differences_sum = 0;
        }

        if (differences_sum > bw_Threshold)
        {
            end_line = i;
            break;
        }
    }

    hd_white_sum = 0;
    for (i = (int)IMAGE_COMPRESS_H - 10; i > (int)IMAGE_COMPRESS_H - 20; i--)
    {
        hd_white_sum += (int)image_two_value[i][maxline];
        white_sum = hd_white_sum;
    }
    if (white_sum > 2300)
    {
        end_line = 50;
    }

    hd_white_sum = 0;

    for (i = 10; i < (int)IMAGE_COMPRESS_H - 5; i++)
    {
        for (j = maxline; j > 0; j--)
        {
            differences_a = (int)image_two_value[i][j];
            differences_b = (int)image_two_value[i][j - 1];
            if ((differences_a + differences_b) != 0)
            {
                differences_sum = abs(differences_a - differences_b) * 100 / (differences_a + differences_b);
            }
            else
            {
                differences_sum = 0;
            }

            if (differences_sum > bw_Threshold)
            {
                Left_Line[i] = j;
                break;
            }
        }

        for (j = maxline; j < (int)IMAGE_COMPRESS_W - 1; j++)
        {
            differences_a = (int)image_two_value[i][j];
            differences_b = (int)image_two_value[i][j + 1];
            if ((differences_a + differences_b) != 0)
            {
                differences_sum = abs(differences_a - differences_b) * 100 / (differences_a + differences_b);
            }
            else
            {
                differences_sum = 0;
            }

            if (differences_sum >= bw_Threshold)
            {
                Right_Line[i] = j;
                break;
            }
        }
    }
}

/*--------------------------------------------------------------------------------------------------------------------
 * @brief  TC `camera_huidu`：按黑白跳变阈值扫左右边。
 *-------------------------------------------------------------------------------------------------------------------*/
void camera_huidu(int bw_Threshold)
{
    int i;
    int j;
    int differences_sum;
    int differences_a;
    int differences_b;

    for (i = 10; i < (int)IMAGE_COMPRESS_H - 10; i++)
    {
        for (j = 10; j < (int)IMAGE_COMPRESS_W - 10; j++)
        {
            differences_a = (int)image_two_value[i][j];
            differences_b = (int)image_two_value[i][j - 1];
            if ((differences_a + differences_b) != 0)
            {
                differences_sum = abs(differences_a - differences_b) * 1000 / (differences_a + differences_b);
            }
            else
            {
                differences_sum = 0;
            }
            if (differences_sum > bw_Threshold)
            {
                Left_Line[i] = j;
                break;
            }
        }

        for (j = (int)IMAGE_COMPRESS_W - 10; j > 10; j--)
        {
            differences_a = (int)image_two_value[i][j];
            differences_b = (int)image_two_value[i][j + 1];
            if ((differences_a + differences_b) != 0)
            {
                differences_sum = abs(differences_a - differences_b) * 1000 / (differences_a + differences_b);
            }
            else
            {
                differences_sum = 0;
            }

            if (differences_sum >= bw_Threshold)
            {
                Right_Line[i] = j;
                break;
            }
        }
    }
}

/*--------------------------------------------------------------------------------------------------------------------
 * @brief  TC `Longest_White_Column`，输入需为二值图（IMG_WHITE / IMG_BLACK）。
 *-------------------------------------------------------------------------------------------------------------------*/
void Longest_White_Column(void)
{
    int i;
    int j;
    int start_column = 20;
    int end_column = (int)IMAGE_COMPRESS_W - 20;
    int left_border = 0;
    int right_border = 0;

    Longest_White_Column_Left[0]  = 0;
    Longest_White_Column_Left[1]  = 0;
    Longest_White_Column_Right[0] = 0;
    Longest_White_Column_Right[1] = 0;
    Right_Lost_Time = 0;
    Left_Lost_Time = 0;
    Boundry_Start_Left = 0;
    Boundry_Start_Right = 0;
    Both_Lost_Time = 0;

    for (i = 0; i <= (int)IMAGE_COMPRESS_H - 1; i++)
    {
        Right_Lost_Flag[i] = 0;
        Left_Lost_Flag[i] = 0;
        Left_Line[i] = 0;
        Right_Line[i] = (int)IMAGE_COMPRESS_W - 1;
    }

    for (i = 0; i <= (int)IMAGE_COMPRESS_W - 1; i++)
    {
        White_Column[i] = 0;
    }

    for (j = start_column; j <= end_column; j++)
    {
        for (i = (int)IMAGE_COMPRESS_H - 1; i >= 0; i--)
        {
            if (image_two_value[i][j] == IMG_WHITE)
            {
                White_Column[j]++;
            }
        }
    }

    Longest_White_Column_Left[0] = 0;
    for (i = start_column; i <= end_column; i++)
    {
        if (Longest_White_Column_Left[0] < White_Column[i])
        {
            Longest_White_Column_Left[0] = White_Column[i];
            Longest_White_Column_Left[1] = i;
        }
    }

    Longest_White_Column_Right[0] = 0;
    for (i = end_column; i >= start_column; i--)
    {
        if (Longest_White_Column_Right[0] < White_Column[i])
        {
            Longest_White_Column_Right[0] = White_Column[i];
            Longest_White_Column_Right[1] = i;
        }
    }

    Search_Stop_Line = Longest_White_Column_Left[0];

    if (Search_Stop_Line <= 0)
    {
        return;
    }

    for (i = (int)IMAGE_COMPRESS_H - 1; i >= (int)IMAGE_COMPRESS_H - Search_Stop_Line; i--)
    {
        for (j = Longest_White_Column_Right[1]; j <= (int)IMAGE_COMPRESS_W - 1 - 2; j++)
        {
            if (image_two_value[i][j] == IMG_WHITE
                && image_two_value[i][j + 1] == IMG_BLACK
                && image_two_value[i][j + 2] == IMG_BLACK)
            {
                right_border = j;
                Right_Lost_Flag[i] = 0;
                break;
            }
            else if (j >= (int)IMAGE_COMPRESS_W - 1 - 2)
            {
                right_border = j;
                Right_Lost_Flag[i] = 1;
                break;
            }
        }
        for (j = Longest_White_Column_Left[1]; j >= 2; j--)
        {
            if (image_two_value[i][j] == IMG_WHITE
                && image_two_value[i][j - 1] == IMG_BLACK
                && image_two_value[i][j - 2] == IMG_BLACK)
            {
                left_border = j;
                Left_Lost_Flag[i] = 0;
                break;
            }
            else if (j <= 2)
            {
                left_border = j;
                Left_Lost_Flag[i] = 1;
                break;
            }
        }
        Left_Line[i]  = left_border;
        Right_Line[i] = right_border;
    }

    for (i = (int)IMAGE_COMPRESS_H - 1; i >= 0; i--)
    {
        if (Left_Lost_Flag[i] == 1)
        {
            Left_Lost_Time++;
        }
        if (Right_Lost_Flag[i] == 1)
        {
            Right_Lost_Time++;
        }
        if ((Left_Lost_Flag[i] == 1) && (Right_Lost_Flag[i] == 1))
        {
            Both_Lost_Time++;
        }
        if ((Boundry_Start_Left == 0) && (Left_Lost_Flag[i] != 1))
        {
            Boundry_Start_Left = i;
        }
        if ((Boundry_Start_Right == 0) && (Right_Lost_Flag[i] != 1))
        {
            Boundry_Start_Right = i;
        }
        Road_Wide[i] = Right_Line[i] - Left_Line[i];
    }
}

float Err_Sum(void)
{
    int   i;
    float err = 0.0f;
    int   begin_line;

    if (end_line < 8)
    {
        begin_line = 0;
    }
    else
    {
        begin_line = end_line - 8;
    }

    for (i = end_line; i >= begin_line; i--)
    {
        if ((i >= 0) && (i < (int)IMAGE_COMPRESS_H))
        {
            err += (float)((int)IMAGE_COMPRESS_W / 2
                           - (((int)Left_Line[i] + (int)Right_Line[i]) >> 1))
                    * (float)image_err_weight[i];
        }
    }
    err /= 8.0f;

    return err;
}

uint8 image_get_err_weight(int row)
{
    if (row < 0 || row >= (int)IMAGE_COMPRESS_H)
    {
        return 0u;
    }
    return image_err_weight[row];
}

float Err_bx_Sum(void)
{
    int   i;
    float err = 0.0f;

    /** 原 TC 用 W 作循环上界易越界；此处按行维 `IMAGE_COMPRESS_H` */
    for (i = (int)IMAGE_COMPRESS_H - 10; i >= 10; i--)
    {
        err += (float)((int)IMAGE_COMPRESS_W / 2
                       - (((int)Left_Line[i] + (int)Right_Line[i]) >> 1))
                * (float)image_err_weight[i];
    }
    err /= 40.0f;

    return err;
}

/*--------------------------------------------------------------------------------------------------------------------
 * Flash 页 49：摄像头曝光（CM7_1 读写；与导航页 46~48 独立）
 *-------------------------------------------------------------------------------------------------------------------*/
#define IMAGE_CAMERA_EXP_PAGE     49u
#define IMAGE_CAMERA_EXP_MAGIC      0x494D4745u /* "IMGE" */
#define IMAGE_CAMERA_EXP_VERSION    1u

static uint32 image_camera_exp_checksum(uint32 exposure_raw)
{
    return IMAGE_CAMERA_EXP_MAGIC ^ IMAGE_CAMERA_EXP_VERSION ^ exposure_raw;
}

void image_camera_exposure_flash_read(void)
{
    uint32 magic = 0;
    uint32 version = 0;
    uint32 checksum = 0;
    uint32 exposure_raw = 0;

    if (!flash_check(0, IMAGE_CAMERA_EXP_PAGE))
    {
        return;
    }

    flash_buffer_clear();
    flash_read_page_to_buffer(0, IMAGE_CAMERA_EXP_PAGE, FLASH_PAGE_LENGTH);
    magic = flash_union_buffer[0].uint32_type;
    version = flash_union_buffer[1].uint32_type;
    checksum = flash_union_buffer[2].uint32_type;
    exposure_raw = flash_union_buffer[3].uint32_type;

    if ((magic != IMAGE_CAMERA_EXP_MAGIC) ||
        (version != IMAGE_CAMERA_EXP_VERSION) ||
        (checksum != image_camera_exp_checksum(exposure_raw)))
    {
        flash_buffer_clear();
        return;
    }

    image_camera_exposure = (uint16)(exposure_raw & 0xFFFFu);
    flash_buffer_clear();
}

void image_camera_exposure_flash_write(void)
{
    uint32 exposure_raw = (uint32)image_camera_exposure;

    flash_buffer_clear();
    flash_union_buffer[0].uint32_type = IMAGE_CAMERA_EXP_MAGIC;
    flash_union_buffer[1].uint32_type = IMAGE_CAMERA_EXP_VERSION;
    flash_union_buffer[2].uint32_type = image_camera_exp_checksum(exposure_raw);
    flash_union_buffer[3].uint32_type = exposure_raw;

    if (flash_check(0, IMAGE_CAMERA_EXP_PAGE))
    {
        flash_erase_page(0, IMAGE_CAMERA_EXP_PAGE);
    }
    flash_write_page_from_buffer(0, IMAGE_CAMERA_EXP_PAGE, FLASH_PAGE_LENGTH);
    flash_buffer_clear();
}

/*--------------------------------------------------------------------------------------------------------------------
 * AE 会话：每次改曝光后等非阻塞等帧，再统计 ROI；收敛或失败后由 consume 写 Flash。
 *-------------------------------------------------------------------------------------------------------------------*/
#define IMAGE_AE_LO                 380000
#define IMAGE_AE_HI                 410000
#define IMAGE_AE_TARGET             395000
#define IMAGE_AE_MAX_ITER           48u
#define IMAGE_AE_FRAME_WAIT_LOOPS   2000u
#define IMAGE_AE_IN_RANGE_STREAK    2u
#define IMAGE_AE_STUCK_LIMIT        3u

typedef enum
{
    AE_SUB_MEASURE = 0,
    AE_SUB_WAIT_FRAME,
} image_ae_sub_step_enum;

static image_ae_state_enum s_ae_state = IMAGE_AE_IDLE;
static image_ae_sub_step_enum s_ae_sub = AE_SUB_MEASURE;
static uint32 s_ae_iter = 0u;
static uint32 s_ae_frame_wait_loops = 0u;
static uint8  s_ae_in_range_streak = 0u;
static uint8  s_ae_stuck_count = 0u;

/** 按 ROI 亮度与目标比估算下一曝光（近似线性），避免每步 ±1 导致 128 步远不够 */
static uint16 image_ae_compute_next_exposure(int camera_light, uint16 cur_exp)
{
    uint32 new_exp;

    if (camera_light <= 0)
    {
        new_exp = (uint32)cur_exp + 64u;
    }
    else
    {
        new_exp = ((uint32)cur_exp * (uint32)IMAGE_AE_TARGET) / (uint32)camera_light;
        if (new_exp == (uint32)cur_exp)
        {
            if (camera_light < IMAGE_AE_LO)
            {
                new_exp = (uint32)cur_exp + 1u;
            }
            else
            {
                new_exp = (uint32)cur_exp - 1u;
            }
        }
    }

    if (new_exp > 65535u)
    {
        new_exp = 65535u;
    }
    return (uint16)new_exp;
}

static int image_ae_measure_roi_light(void)
{
    int camera_light = 0;
    int i;
    int j;

    image_photo_compress(mt9v03x_image[0]);
    for (j = 10; j < (int)IMAGE_COMPRESS_W - 10; j++)
    {
        for (i = 10; i < (int)IMAGE_COMPRESS_H - 10; i++)
        {
            camera_light += (int)image_two_value[i][j];
        }
    }
    test_printf_light = camera_light;
    return camera_light;
}

void image_ae_session_arm(void)
{
    if (s_ae_state == IMAGE_AE_IDLE)
    {
        s_ae_state = IMAGE_AE_RUNNING;
        /* 先等新帧再测量，避免用进入菜单前的旧图误判 DONE */
        s_ae_sub = AE_SUB_WAIT_FRAME;
        s_ae_iter = 0u;
        s_ae_frame_wait_loops = 0u;
        s_ae_in_range_streak = 0u;
        s_ae_stuck_count = 0u;
    }
}

void image_ae_session_poll(void)
{
    int camera_light;

    if (s_ae_state != IMAGE_AE_RUNNING)
    {
        return;
    }

    if (s_ae_sub == AE_SUB_WAIT_FRAME)
    {
        if (mt9v03x_finish_flag != 0u)
        {
            mt9v03x_finish_flag = 0u;
            s_ae_sub = AE_SUB_MEASURE;
            s_ae_frame_wait_loops = 0u;
        }
        else
        {
            s_ae_frame_wait_loops++;
            if (s_ae_frame_wait_loops > IMAGE_AE_FRAME_WAIT_LOOPS)
            {
                /* 超时仍用当前缓冲试测，避免因偶发丢 flag 整段结束 */
                s_ae_sub = AE_SUB_MEASURE;
                s_ae_frame_wait_loops = 0u;
            }
        }
        return;
    }

    camera_light = image_ae_measure_roi_light();

    if ((camera_light >= IMAGE_AE_LO) && (camera_light <= IMAGE_AE_HI))
    {
        s_ae_in_range_streak++;
        if (s_ae_in_range_streak >= IMAGE_AE_IN_RANGE_STREAK)
        {
            s_ae_state = IMAGE_AE_DONE;
            return;
        }
        s_ae_sub = AE_SUB_WAIT_FRAME;
        s_ae_frame_wait_loops = 0u;
        return;
    }

    s_ae_in_range_streak = 0u;

    s_ae_iter++;
    if (s_ae_iter >= IMAGE_AE_MAX_ITER)
    {
        s_ae_state = IMAGE_AE_FAILED;
        return;
    }

    {
        uint16 next_exp = image_ae_compute_next_exposure(camera_light, image_camera_exposure);

        if (next_exp == image_camera_exposure)
        {
            s_ae_stuck_count++;
            if (s_ae_stuck_count >= IMAGE_AE_STUCK_LIMIT)
            {
                s_ae_state = IMAGE_AE_FAILED;
                return;
            }
        }
        else
        {
            s_ae_stuck_count = 0u;
        }

        image_camera_exposure = next_exp;
        (void)mt9v03x_set_exposure_time(image_camera_exposure);
    }

    s_ae_sub = AE_SUB_WAIT_FRAME;
    s_ae_frame_wait_loops = 0u;
}

uint8 image_ae_session_is_active(void)
{
    return (uint8)(s_ae_state == IMAGE_AE_RUNNING);
}

image_ae_state_enum image_ae_session_get_state(void)
{
    return s_ae_state;
}

uint8 image_ae_session_consume_done_and_save(void)
{
    if (s_ae_state == IMAGE_AE_DONE)
    {
        image_camera_exposure_flash_write();
        s_ae_state = IMAGE_AE_IDLE;
        return 1u;
    }
    if (s_ae_state == IMAGE_AE_FAILED)
    {
        /* 未收敛不写 Flash，避免把半成品曝光当成有效值持久化 */
        s_ae_state = IMAGE_AE_IDLE;
        return 1u;
    }
    return 0u;
}

void image_camera_auto_exposure(void)
{
    image_ae_session_arm();
    while (s_ae_state == IMAGE_AE_RUNNING)
    {
        image_ae_session_poll();
    }
    (void)image_ae_session_consume_done_and_save();
}

/*--------------------------------------------------------------------------------------------------------------------
 * 白连通域检测（桥区/验证共用）与验证状态机（CM7_1 检测，CM7_0 验证 apply_yaw）
 *-------------------------------------------------------------------------------------------------------------------*/

#if defined(CY_CORE_CM7_1) && IMAGE_WHITE_BLOB_ANY_ENABLE

#include "dualcore_shared.h"

/** BFS 队列容量：全幅可用区约 50×78 像素，留余量 */
#define IMAGE_WHITE_BLOB_Q_MAX  (4096)

static uint8 s_blob_visited[IMAGE_COMPRESS_H][IMAGE_COMPRESS_W];

static uint8 image_white_blob_in_roi(int row, int col, int row_end, int col_lo, int col_hi)
{
    if (row < IMAGE_WHITE_BLOB_ROI_ROW_START || row >= row_end)
    {
        return 0u;
    }
    if (col < col_lo || col >= col_hi)
    {
        return 0u;
    }
    return (uint8)(image_two_value[row][col] == IMG_WHITE);
}

/**
 * 4 邻域 flood-fill：统计面积、质心、包围盒（bottom_row 用于白块→中线切换）。
 */
static void image_white_blob_measure_component(int seed_r, int seed_c,
                                               int row_end, int col_lo, int col_hi,
                                               int *area_out, int *sum_x_out, int *sum_y_out,
                                               int *top_row_out, int *bottom_row_out,
                                               int *bbox_w_out, int *bbox_h_out)
{
    static int16 q_r[IMAGE_WHITE_BLOB_Q_MAX];
    static int16 q_c[IMAGE_WHITE_BLOB_Q_MAX];
    int head = 0;
    int tail = 0;
    int area = 0;
    int32 sum_x = 0;
    int32 sum_y = 0;
    int min_r = seed_r;
    int max_r = seed_r;
    int min_c = seed_c;
    int max_c = seed_c;

    q_r[tail] = (int16)seed_r;
    q_c[tail] = (int16)seed_c;
    tail++;
    s_blob_visited[seed_r][seed_c] = 1u;

    while (head < tail)
    {
        int r = (int)q_r[head];
        int c = (int)q_c[head];
        head++;

        area++;
        sum_x += c;
        sum_y += r;
        if (r < min_r)
        {
            min_r = r;
        }
        if (r > max_r)
        {
            max_r = r;
        }
        if (c < min_c)
        {
            min_c = c;
        }
        if (c > max_c)
        {
            max_c = c;
        }

        if (r > IMAGE_WHITE_BLOB_ROI_ROW_START &&
            image_white_blob_in_roi(r - 1, c, row_end, col_lo, col_hi) &&
            s_blob_visited[r - 1][c] == 0u)
        {
            if (tail < IMAGE_WHITE_BLOB_Q_MAX)
            {
                s_blob_visited[r - 1][c] = 1u;
                q_r[tail] = (int16)(r - 1);
                q_c[tail] = (int16)c;
                tail++;
            }
        }
        if (r + 1 < row_end &&
            image_white_blob_in_roi(r + 1, c, row_end, col_lo, col_hi) &&
            s_blob_visited[r + 1][c] == 0u)
        {
            if (tail < IMAGE_WHITE_BLOB_Q_MAX)
            {
                s_blob_visited[r + 1][c] = 1u;
                q_r[tail] = (int16)(r + 1);
                q_c[tail] = (int16)c;
                tail++;
            }
        }
        if (c > col_lo &&
            image_white_blob_in_roi(r, c - 1, row_end, col_lo, col_hi) &&
            s_blob_visited[r][c - 1] == 0u)
        {
            if (tail < IMAGE_WHITE_BLOB_Q_MAX)
            {
                s_blob_visited[r][c - 1] = 1u;
                q_r[tail] = (int16)r;
                q_c[tail] = (int16)(c - 1);
                tail++;
            }
        }
        if (c + 1 < col_hi &&
            image_white_blob_in_roi(r, c + 1, row_end, col_lo, col_hi) &&
            s_blob_visited[r][c + 1] == 0u)
        {
            if (tail < IMAGE_WHITE_BLOB_Q_MAX)
            {
                s_blob_visited[r][c + 1] = 1u;
                q_r[tail] = (int16)r;
                q_c[tail] = (int16)(c + 1);
                tail++;
            }
        }
    }

    *area_out = area;
    *sum_x_out = (int)sum_x;
    *sum_y_out = (int)sum_y;
    *top_row_out = min_r;
    *bottom_row_out = max_r;
    *bbox_w_out = max_c - min_c + 1;
    *bbox_h_out = max_r - min_r + 1;
}

void image_white_blob_detect(image_white_blob_result_t *out)
{
    int row;
    int col;
    int row_end = IMAGE_WHITE_BLOB_DETECT_ROW_END;
    int col_lo = IMAGE_WHITE_BLOB_COL_MARGIN;
    int col_hi = (int)IMAGE_COMPRESS_W - IMAGE_WHITE_BLOB_COL_MARGIN;
    int best_area = 0;
    int best_sum_x = 0;
    int best_sum_y = 0;
    int best_top = 0;
    int best_bottom = 0;
    int best_bbox_w = 0;
    int best_bbox_h = 0;
    int best_cx = (int)IMAGE_COMPRESS_W / 2;
    int best_cy = row_end / 2;
    uint8 track_valid = 0u;
    float center_err = 0.0f;
    int th;
    int gray_sum = 0;
    int gray_count = 0;
    int mean_gray = 0;
    int roi_pixels = 0;

    if (row_end > (int)IMAGE_COMPRESS_H)
    {
        row_end = (int)IMAGE_COMPRESS_H;
    }
    if (col_hi <= col_lo)
    {
        col_lo = 0;
        col_hi = (int)IMAGE_COMPRESS_W;
    }

    image_photo_compress(mt9v03x_image[0]);

    for (row = IMAGE_WHITE_BLOB_ROI_ROW_START; row < row_end; row++)
    {
        for (col = col_lo; col < col_hi; col++)
        {
            gray_sum += (int)image_two_value[row][col];
            gray_count++;
        }
    }
    mean_gray = (gray_count > 0) ? (gray_sum / gray_count) : 0;
    roi_pixels = gray_count;

    Threshold = (int)image_otsu_on_process_buf();
    th = Threshold + IMAGE_WHITE_BLOB_THRESH_OFFSET;
    if (th < 0)
    {
        th = 0;
    }
    if (th > 255)
    {
        th = 255;
    }
    image_binarization_inplace(th);

    for (row = IMAGE_WHITE_BLOB_ROI_ROW_START; row < row_end; row++)
    {
        for (col = 0; col < (int)IMAGE_COMPRESS_W; col++)
        {
            s_blob_visited[row][col] = 0u;
        }
    }

    for (row = IMAGE_WHITE_BLOB_ROI_ROW_START; row < row_end; row++)
    {
        for (col = col_lo; col < col_hi; col++)
        {
            int area;
            int sum_x;
            int sum_y;
            int top_r;
            int bottom_r;
            int bbox_w;
            int bbox_h;

            if (s_blob_visited[row][col] != 0u)
            {
                continue;
            }
            if (image_two_value[row][col] != IMG_WHITE)
            {
                continue;
            }

            image_white_blob_measure_component(row, col, row_end, col_lo, col_hi,
                                               &area, &sum_x, &sum_y,
                                               &top_r, &bottom_r, &bbox_w, &bbox_h);
            if (area > best_area)
            {
                best_area = area;
                best_sum_x = sum_x;
                best_sum_y = sum_y;
                best_top = top_r;
                best_bottom = bottom_r;
                best_bbox_w = bbox_w;
                best_bbox_h = bbox_h;
            }
        }
    }

    if (best_area >= IMAGE_WHITE_BLOB_MIN_AREA)
    {
        track_valid = 1u;
        best_cx = best_sum_x / best_area;
        best_cy = best_sum_y / best_area;
        center_err = (float)(((int)IMAGE_COMPRESS_W / 2) - best_cx);
    }

    if (mean_gray < IMAGE_WHITE_BLOB_MIN_MEAN_GRAY)
    {
        track_valid = 0u;
        best_area = 0;
        center_err = 0.0f;
    }
    else if ((roi_pixels > 0) && (best_area > (roi_pixels * 4) / 5))
    {
        /* 连通域占 ROI 过大，多为暗场 Otsu 整幅误分割 */
        track_valid = 0u;
        best_area = 0;
        center_err = 0.0f;
    }

    Cammer_Err = center_err;

    if (out != NULL)
    {
        out->center_err = center_err;
        out->cx = best_cx;
        out->cy = best_cy;
        out->area = best_area;
        out->top_row = best_top;
        out->bottom_row = best_bottom;
        out->bbox_w = best_bbox_w;
        out->bbox_h = best_bbox_h;
        out->track_valid = track_valid;
    }
}

#endif /* CY_CORE_CM7_1 && IMAGE_WHITE_BLOB_ANY_ENABLE */

#if defined(CY_CORE_CM7_1) && IMAGE_WHITE_BLOB_VALIDATE_ENABLE

#include "single_bridge.h"
#include "dualcore_shared.h"

static uint8 s_vision_mode = IMAGE_VISION_MODE_BLOB;
static uint8 s_switch_debounce = 0u;
static uint8 s_midline_lost_debounce = 0u;

/** 白块下探到中下部且面积足够大，满足切换中线模式条件（单帧） */
static uint8 image_white_blob_ready_for_midline(const image_white_blob_result_t *blob)
{
    if (blob == NULL || blob->track_valid == 0u)
    {
        return 0u;
    }
    if (blob->bottom_row < IMAGE_WHITE_BLOB_SWITCH_ROW_MIN)
    {
        return 0u;
    }
    if (blob->area < IMAGE_WHITE_BLOB_SWITCH_AREA_MIN)
    {
        return 0u;
    }
    return 1u;
}

static void image_vision_guidance_publish_blob(const image_white_blob_result_t *blob, uint32 *seq)
{
    (*seq)++;
    dualcore_white_blob_publish(blob->center_err, blob->track_valid, *seq);
    dualcore_bridge_vision_publish_inactive();
}

static void image_vision_guidance_publish_midline(const single_bridge_track_t *track, uint32 *seq)
{
    (*seq)++;
    dualcore_bridge_vision_publish(track->center_err, track->track_valid, *seq);
    dualcore_white_blob_publish_inactive();
}

void image_vision_guidance_reset(void)
{
    s_vision_mode = IMAGE_VISION_MODE_BLOB;
    s_switch_debounce = 0u;
    s_midline_lost_debounce = 0u;
}

void image_vision_guidance_process_frame(int bw_threshold)
{
    static uint32 s_blob_frame_seq;
    static uint32 s_midline_frame_seq;
    image_white_blob_result_t blob;
    single_bridge_track_t track;
    int th = bw_threshold;

    if (th <= 0)
    {
        th = SINGLE_BRIDGE_DIFF_TH_DEFAULT;
    }

    if (s_vision_mode == IMAGE_VISION_MODE_BLOB)
    {
        image_white_blob_detect(&blob);

        if (image_white_blob_ready_for_midline(&blob))
        {
            if (s_switch_debounce < 255u)
            {
                s_switch_debounce++;
            }
        }
        else
        {
            s_switch_debounce = 0u;
        }

        if (s_switch_debounce >= IMAGE_WHITE_BLOB_SWITCH_DEBOUNCE)
        {
            s_vision_mode = IMAGE_VISION_MODE_MIDLINE;
            s_switch_debounce = 0u;
            s_midline_lost_debounce = 0u;

            single_bridge_gray_diff_track(th, &track);
            if (track.track_valid == 0u)
            {
                s_vision_mode = IMAGE_VISION_MODE_BLOB;
                image_vision_guidance_publish_blob(&blob, &s_blob_frame_seq);
            }
            else
            {
                image_vision_guidance_publish_midline(&track, &s_midline_frame_seq);
            }
        }
        else
        {
            image_vision_guidance_publish_blob(&blob, &s_blob_frame_seq);
        }
    }
    else
    {
        single_bridge_gray_diff_track(th, &track);

        if (track.track_valid == 0u)
        {
            if (s_midline_lost_debounce < 255u)
            {
                s_midline_lost_debounce++;
            }
        }
        else
        {
            s_midline_lost_debounce = 0u;
        }

        if (s_midline_lost_debounce >= SINGLE_BRIDGE_LOST_FALLBACK_DEBOUNCE)
        {
            s_vision_mode = IMAGE_VISION_MODE_BLOB;
            s_midline_lost_debounce = 0u;
            s_switch_debounce = 0u;
            image_white_blob_detect(&blob);
            image_vision_guidance_publish_blob(&blob, &s_blob_frame_seq);
        }
        else
        {
            image_vision_guidance_publish_midline(&track, &s_midline_frame_seq);
        }
    }
}

#endif /* CY_CORE_CM7_1 && IMAGE_WHITE_BLOB_VALIDATE_ENABLE */

#if defined(CY_CORE_CM7_1) && IMAGE_BRIDGE_WHITE_BLOB_ENABLE

#include "dualcore_shared.h"

/**
 * 单边桥回放：每帧最大白连通域 → dualcore blob 通道（全程 BLOB，不切中线）。
 * 调用方须在 mt9v03x_finish_flag 置位后调用，并在本函数返回前清零 finish_flag。
 */
void image_bridge_blob_process_frame(int bw_threshold)
{
    static uint32 s_bridge_blob_frame_seq;
    image_white_blob_result_t blob;

    (void)bw_threshold;

    image_white_blob_detect(&blob);
    s_bridge_blob_frame_seq++;
    dualcore_white_blob_publish(blob.center_err, blob.track_valid, s_bridge_blob_frame_seq);
    dualcore_bridge_vision_publish_inactive();
}

#endif /* CY_CORE_CM7_1 && IMAGE_BRIDGE_WHITE_BLOB_ENABLE */

#if defined(CY_CORE_CM7_0) && IMAGE_WHITE_BLOB_VALIDATE_ENABLE

#include "control.h"
#include "navigation.h"
#include "dualcore_shared.h"
#include <math.h>

static float image_vision_guidance_clipf(float v, float lo, float hi)
{
    if (v < lo)
    {
        return lo;
    }
    if (v > hi)
    {
        return hi;
    }
    return v;
}

/** 主循环 yaw 修正：center_err(像素) → 绝对航向目标偏移，经 steer_request_target_yaw 交给 1ms 航向双环 */
static void image_vision_guidance_apply_yaw_from_err(float center_err, uint8 track_valid, uint8 fresh)
{
    float target_offset_deg;
    float target_yaw_deg;

    if (fresh == 0u || track_valid == 0u)
    {
        return;
    }
    if (spin_enable != 0u || Motor_Runaway_Latch != 0u)
    {
        return;
    }
    /* 回放态默认禁视觉 yaw；桥区白块引导时 Bridge_Zone_Active 放行 */
    if (N.Nag_SystemRun_Index == 3u && N.Bridge_Zone_Active == 0u)
    {
        return;
    }

    target_offset_deg = center_err * IMAGE_WHITE_BLOB_YAW_K_PIXEL;
    target_offset_deg = image_vision_guidance_clipf(target_offset_deg,
                                                    -IMAGE_WHITE_BLOB_YAW_MAX_OFFSET_DEG,
                                                    IMAGE_WHITE_BLOB_YAW_MAX_OFFSET_DEG);
    if (fabsf(target_offset_deg) < IMAGE_WHITE_BLOB_YAW_DEADBAND_DEG)
    {
        return;
    }

    target_yaw_deg = (float)euler_angle.yaw + target_offset_deg;
    steer_request_target_yaw(target_yaw_deg);
}

void image_white_blob_apply_yaw(void)
{
    float center_err = 0.0f;
    uint8 track_valid = 0u;
    uint8 fresh = 0u;

    dualcore_white_blob_pull(&center_err, &track_valid, &fresh);
    image_vision_guidance_apply_yaw_from_err(center_err, track_valid, fresh);
}

void image_midline_apply_yaw(void)
{
    float center_err = 0.0f;
    uint8 track_valid = 0u;
    uint8 fresh = 0u;

    dualcore_bridge_vision_pull(&center_err, &track_valid, &fresh);
    image_vision_guidance_apply_yaw_from_err(center_err, track_valid, fresh);
}

void image_vision_guidance_apply_yaw(void)
{
    uint8 mode = dualcore_vision_guidance_pull_mode();

    if (mode == IMAGE_VISION_MODE_BLOB)
    {
        image_white_blob_apply_yaw();
    }
    else if (mode == IMAGE_VISION_MODE_MIDLINE)
    {
        image_midline_apply_yaw();
    }
}

#endif /* CY_CORE_CM7_0 && IMAGE_WHITE_BLOB_VALIDATE_ENABLE */

#if defined(CY_CORE_CM7_1) && IMAGE_DARK_LINE_VALIDATE_ENABLE

#include "zf_common_headfile.h"
#include "dualcore_shared.h"

typedef enum
{
    IMAGE_DARK_LINE_STATE_OUTSIDE = 0,
    IMAGE_DARK_LINE_STATE_INSIDE  = 1,
} image_dark_line_state_enum;

static image_dark_line_state_enum s_dark_line_state = IMAGE_DARK_LINE_STATE_OUTSIDE;
static uint8 s_dark_line_enter_debounce = 0u;
static uint8 s_dark_line_exit_debounce = 0u;
static uint16 s_dark_line_inside_hold_frames = 0u;
static image_dark_line_debug_state_t s_dark_line_debug;
/** 二值化前压缩灰度快照，仅供 Bumpy 调试页显示（算法仍用二值图） */
static uint8 s_dark_line_gray_view[IMAGE_COMPRESS_H][IMAGE_COMPRESS_W];

/**
 * 统计 ROI 白色像素占比（0~1）。
 * 入口/出口为深色地面时 white_ratio 低；进入白底元素后升高。
 */
static float image_dark_line_measure_white_ratio(int row_start, int row_end,
                                                 int col_lo, int col_hi)
{
    int row;
    int col;
    int white_cnt = 0;
    int total = 0;

    for (row = row_start; row < row_end; row++)
    {
        for (col = col_lo; col < col_hi; col++)
        {
            total++;
            if (image_two_value[row][col] == IMG_WHITE)
            {
                white_cnt++;
            }
        }
    }
    if (total <= 0)
    {
        return 0.0f;
    }
    return (float)white_cnt / (float)total;
}

/** 单行内找最宽黑条；宽度在 [MIN, MAX] 内返回 1 */
static uint8 image_dark_line_find_widest_black_run(int row, int col_lo, int col_hi,
                                                   int *out_left, int *out_right)
{
    int col;
    int run_left = -1;
    int best_width = 0;
    int best_left = 0;
    int best_right = 0;

    for (col = col_lo; col < col_hi; col++)
    {
        if (image_two_value[row][col] == IMG_BLACK)
        {
            if (run_left < 0)
            {
                run_left = col;
            }
        }
        else if (run_left >= 0)
        {
            int width = col - run_left;
            if (width > best_width)
            {
                best_width = width;
                best_left = run_left;
                best_right = col - 1;
            }
            run_left = -1;
        }
    }
    if (run_left >= 0)
    {
        int width = col_hi - run_left;
        if (width > best_width)
        {
            best_width = width;
            best_left = run_left;
            best_right = col_hi - 1;
        }
    }

    if (best_width < IMAGE_DARK_LINE_MIN_STRIPE_WIDTH ||
        best_width > IMAGE_DARK_LINE_MAX_STRIPE_WIDTH)
    {
        return 0u;
    }
    *out_left = best_left;
    *out_right = best_right;
    return 1u;
}

/**
 * 元素内：多行黑条中心 + 线性拟合斜率修正。
 * 横条在画面中应近似水平；斜率偏离 0 时折算为等效 center_err 修正 yaw。
 * 若 dbg 非空，写入各行中心点与拟合量供 IPS 叠加。
 */
static void image_dark_line_track_black_center(int row_start, int row_end,
                                               int col_lo, int col_hi,
                                               float *center_err, int *line_cx,
                                               int *avg_width, uint8 *track_valid,
                                               image_dark_line_debug_state_t *dbg)
{
    int row;
    int valid_rows = 0;
    int sum_cx = 0;
    int sum_width = 0;
    int img_center = (int)IMAGE_COMPRESS_W / 2;
    int pts_row[64];
    int pts_cx[64];
    int pts_n = 0;
    int weight_sum = 0;

    if (dbg != NULL)
    {
        dbg->pts_count = 0u;
        dbg->slope_x1000 = 0;
        dbg->mean_row = 0;
        dbg->mean_cx = img_center;
    }

    if (center_err != NULL)
    {
        *center_err = 0.0f;
    }
    if (line_cx != NULL)
    {
        *line_cx = img_center;
    }
    if (avg_width != NULL)
    {
        *avg_width = 0;
    }
    if (track_valid != NULL)
    {
        *track_valid = 0u;
    }

    for (row = row_start; row < row_end; row++)
    {
        int left;
        int right;
        int cx;
        int weight;

        if (image_dark_line_find_widest_black_run(row, col_lo, col_hi, &left, &right) == 0u)
        {
            continue;
        }

        cx = (left + right) / 2;
        weight = row - row_start + 1;
        valid_rows++;
        sum_cx += cx * weight;
        sum_width += (right - left + 1);
        weight_sum += weight;

        if (pts_n < 64)
        {
            pts_row[pts_n] = row;
            pts_cx[pts_n] = cx;
            pts_n++;
        }
    }

    if (valid_rows < (int)IMAGE_DARK_LINE_MIN_TRACK_ROWS || weight_sum <= 0)
    {
        return;
    }

    {
        int mean_cx = sum_cx / weight_sum;
        int mean_row = 0;
        int sum_r = 0;
        int sum_c = 0;
        int sum_rc = 0;
        int sum_rr = 0;
        int denom;
        int slope_x1000 = 0;
        int mid_row = (row_start + row_end) / 2;
        int predicted_cx;
        float lateral_err;
        int i;

        if (line_cx != NULL)
        {
            *line_cx = mean_cx;
        }
        if (avg_width != NULL)
        {
            *avg_width = sum_width / valid_rows;
        }

        for (i = 0; i < pts_n; i++)
        {
            sum_r += pts_row[i];
            sum_c += pts_cx[i];
            sum_rc += pts_row[i] * pts_cx[i];
            sum_rr += pts_row[i] * pts_row[i];
        }
        mean_row = sum_r / pts_n;

        denom = pts_n * sum_rr - sum_r * sum_r;
        if (denom != 0)
        {
            slope_x1000 = ((pts_n * sum_rc - sum_r * sum_c) * 1000) / denom;
        }

        predicted_cx = mean_cx + (slope_x1000 * (mid_row - mean_row)) / 1000;
        lateral_err = (float)(img_center - predicted_cx);

        if (center_err != NULL)
        {
            *center_err = lateral_err;
        }
        if (track_valid != NULL)
        {
            *track_valid = 1u;
        }

        if (dbg != NULL)
        {
            uint8 copy_n = (uint8)pts_n;
            if (copy_n > IMAGE_DARK_LINE_DEBUG_PTS_MAX)
            {
                copy_n = IMAGE_DARK_LINE_DEBUG_PTS_MAX;
            }
            dbg->pts_count = copy_n;
            for (i = 0; i < (int)copy_n; i++)
            {
                dbg->pts[i].row = pts_row[i];
                dbg->pts[i].cx = pts_cx[i];
            }
            dbg->slope_x1000 = slope_x1000;
            dbg->mean_row = mean_row;
            dbg->mean_cx = mean_cx;
            dbg->line_cx = mean_cx;
            dbg->black_line_width = (avg_width != NULL) ? *avg_width : (sum_width / valid_rows);
            dbg->center_err = lateral_err;
            dbg->track_valid = 1u;
        }
    }
}

/** 白底出现/消失状态机；更新 result 的 element_active 与 enter/exit_pulse */
static void image_dark_line_apply_state_machine(image_dark_line_result_t *result)
{
    uint8 white_enter;
    uint8 white_exit;

    if (result == NULL)
    {
        return;
    }

    white_enter = (uint8)(result->white_ratio * 100.0f >= (float)IMAGE_DARK_LINE_WHITE_ENTER_RATIO_PCT);
    white_exit = (uint8)(result->white_ratio * 100.0f <= (float)IMAGE_DARK_LINE_WHITE_EXIT_RATIO_PCT);
    result->enter_pulse = 0u;
    result->exit_pulse = 0u;

    if (s_dark_line_state == IMAGE_DARK_LINE_STATE_OUTSIDE)
    {
        if (white_enter != 0u)
        {
            if (s_dark_line_enter_debounce < 255u)
            {
                s_dark_line_enter_debounce++;
            }
        }
        else
        {
            s_dark_line_enter_debounce = 0u;
        }

        if (s_dark_line_enter_debounce >= IMAGE_DARK_LINE_ENTER_DEBOUNCE_FRAMES)
        {
            s_dark_line_state = IMAGE_DARK_LINE_STATE_INSIDE;
            s_dark_line_enter_debounce = 0u;
            s_dark_line_exit_debounce = 0u;
            s_dark_line_inside_hold_frames = 0u;
            result->enter_pulse = 1u;
        }
    }
    else
    {
        if (s_dark_line_inside_hold_frames < 0xFFFFu)
        {
            s_dark_line_inside_hold_frames++;
        }

        if (white_exit != 0u &&
            s_dark_line_inside_hold_frames >= IMAGE_DARK_LINE_MIN_INSIDE_HOLD_FRAMES)
        {
            if (s_dark_line_exit_debounce < 255u)
            {
                s_dark_line_exit_debounce++;
            }
        }
        else
        {
            s_dark_line_exit_debounce = 0u;
        }

        if (s_dark_line_exit_debounce >= IMAGE_DARK_LINE_EXIT_DEBOUNCE_FRAMES)
        {
            s_dark_line_state = IMAGE_DARK_LINE_STATE_OUTSIDE;
            s_dark_line_exit_debounce = 0u;
            s_dark_line_enter_debounce = 0u;
            s_dark_line_inside_hold_frames = 0u;
            result->exit_pulse = 1u;
            result->track_valid = 0u;
            result->center_err = 0.0f;
        }
    }

    result->element_active = (uint8)(s_dark_line_state == IMAGE_DARK_LINE_STATE_INSIDE);
}

/** 压缩→二值→白底比例→（元素内）黑条中心；save_gray_view=1 时保留二值化前灰度供调试显示 */
static void image_dark_line_detect(image_dark_line_result_t *out, uint8 save_gray_view)
{
    int row_end = IMAGE_DARK_LINE_ROI_ROW_END;
    int col_lo = IMAGE_DARK_LINE_COL_MARGIN;
    int col_hi = (int)IMAGE_COMPRESS_W - IMAGE_DARK_LINE_COL_MARGIN;
    int row;
    int col;
    int gray_sum = 0;
    int gray_count = 0;
    int mean_gray;
    int th;
    float white_ratio;
    float center_err = 0.0f;
    int line_cx = (int)IMAGE_COMPRESS_W / 2;
    int avg_width = 0;
    uint8 track_valid = 0u;

    if (out == NULL)
    {
        return;
    }

    if (row_end > (int)IMAGE_COMPRESS_H)
    {
        row_end = (int)IMAGE_COMPRESS_H;
    }
    if (col_hi <= col_lo)
    {
        col_lo = 0;
        col_hi = (int)IMAGE_COMPRESS_W;
    }

    image_photo_compress(mt9v03x_image[0]);

    if (save_gray_view != 0u)
    {
        memcpy(s_dark_line_gray_view, image_two_value, sizeof(s_dark_line_gray_view));
    }

    for (row = IMAGE_DARK_LINE_ROI_ROW_START; row < row_end; row++)
    {
        for (col = col_lo; col < col_hi; col++)
        {
            gray_sum += (int)image_two_value[row][col];
            gray_count++;
        }
    }
    mean_gray = (gray_count > 0) ? (gray_sum / gray_count) : 0;

    Threshold = (int)image_otsu_on_process_buf();
    th = Threshold + IMAGE_DARK_LINE_THRESH_OFFSET;
    if (th < 0)
    {
        th = 0;
    }
    if (th > 255)
    {
        th = 255;
    }
    image_binarization_inplace(th);

    white_ratio = image_dark_line_measure_white_ratio(IMAGE_DARK_LINE_ROI_ROW_START,
                                                      row_end, col_lo, col_hi);

    if (mean_gray >= IMAGE_DARK_LINE_MIN_MEAN_GRAY &&
        s_dark_line_state == IMAGE_DARK_LINE_STATE_INSIDE)
    {
        image_dark_line_track_black_center(IMAGE_DARK_LINE_TRACK_ROW_START,
                                           IMAGE_DARK_LINE_TRACK_ROW_END,
                                           col_lo, col_hi,
                                           &center_err, &line_cx, &avg_width, &track_valid,
                                           &s_dark_line_debug);
    }
    else
    {
        s_dark_line_debug.pts_count = 0u;
        s_dark_line_debug.track_valid = 0u;
        s_dark_line_debug.center_err = 0.0f;
        s_dark_line_debug.slope_x1000 = 0;
    }

    s_dark_line_debug.white_ratio = white_ratio;
    s_dark_line_debug.element_active = (uint8)(s_dark_line_state == IMAGE_DARK_LINE_STATE_INSIDE);

    out->center_err = center_err;
    out->line_cx = line_cx;
    out->black_line_width = avg_width;
    out->white_ratio = white_ratio;
    out->track_valid = track_valid;
    out->element_active = (uint8)(s_dark_line_state == IMAGE_DARK_LINE_STATE_INSIDE);
    out->enter_pulse = 0u;
    out->exit_pulse = 0u;
}

void image_dark_line_reset(void)
{
    s_dark_line_state = IMAGE_DARK_LINE_STATE_OUTSIDE;
    s_dark_line_enter_debounce = 0u;
    s_dark_line_exit_debounce = 0u;
    s_dark_line_inside_hold_frames = 0u;
    memset(&s_dark_line_debug, 0, sizeof(s_dark_line_debug));
}

/**
 * CM7_1 主循环：白底黑线元素状态机 + dualcore 发布。
 * 边界判据为白底出现/消失（入口出口均为深色，不靠黑色判定进出）。
 */
void image_dark_line_process_frame(int bw_threshold)
{
    static uint32 s_dark_line_frame_seq;
    image_dark_line_result_t result;

    (void)bw_threshold;

    image_dark_line_detect(&result, 0u);
    image_dark_line_apply_state_machine(&result);

    Cammer_Err = result.center_err;
    s_dark_line_debug.element_active = result.element_active;

    s_dark_line_frame_seq++;
    dualcore_dark_line_publish(result.center_err,
                               result.track_valid,
                               result.element_active,
                               result.enter_pulse,
                               result.exit_pulse,
                               result.white_ratio,
                               s_dark_line_frame_seq);
    dualcore_white_blob_publish_inactive();
    dualcore_bridge_vision_publish_inactive();
}

/** 压缩图坐标 → IPS 显示坐标（裁剪到屏幕与显示区内，避免 ips200_draw_line 断言） */
static int image_dark_line_clip_i(int v, int lo, int hi)
{
    if (v < lo)
    {
        return lo;
    }
    if (v > hi)
    {
        return hi;
    }
    return v;
}

static int image_dark_line_map_x(int col, int disp_x, int disp_w)
{
    extern uint16 ips200_width_max;
    int x = disp_x + (col * disp_w) / (int)IMAGE_COMPRESS_W;
    int x_hi = disp_x + disp_w - 1;
    if (x_hi > (int)ips200_width_max - 1)
    {
        x_hi = (int)ips200_width_max - 1;
    }
    return image_dark_line_clip_i(x, disp_x, x_hi);
}

static int image_dark_line_map_y(int row, int disp_y, int disp_h)
{
    extern uint16 ips200_height_max;
    int y = disp_y + (row * disp_h) / (int)IMAGE_COMPRESS_H;
    int y_hi = disp_y + disp_h - 1;
    if (y_hi > (int)ips200_height_max - 1)
    {
        y_hi = (int)ips200_height_max - 1;
    }
    return image_dark_line_clip_i(y, disp_y, y_hi);
}

static int image_dark_line_clamp_col(int col)
{
    if (col < 0)
    {
        return 0;
    }
    if (col >= (int)IMAGE_COMPRESS_W)
    {
        return (int)IMAGE_COMPRESS_W - 1;
    }
    return col;
}

/** 叠加：黄折线=各行黑条中心；绿竖线=图像中心；紫线=拟合回归线；白线=跟踪 ROI */
void image_dark_line_draw_center_overlay(int disp_x, int disp_y, int disp_w, int disp_h)
{
    int i;
    int prev_x = -1;
    int prev_y = 0;
    int center_col = (int)IMAGE_COMPRESS_W / 2;
    int y_top;
    int y_bot;
    int fit_x0;
    int fit_y0;
    int fit_x1;
    int fit_y1;
    int row0 = IMAGE_DARK_LINE_TRACK_ROW_START;
    int row1 = IMAGE_DARK_LINE_TRACK_ROW_END - 1;

    if (s_dark_line_debug.element_active == 0u)
    {
        return;
    }

    y_top = image_dark_line_map_y(row0, disp_y, disp_h);
    y_bot = image_dark_line_map_y(row1, disp_y, disp_h);
    ips200_draw_line(disp_x, y_top, disp_x + disp_w - 1, y_top, RGB565_WHITE);
    ips200_draw_line(disp_x, y_bot, disp_x + disp_w - 1, y_bot, RGB565_WHITE);

    ips200_draw_line(image_dark_line_map_x(center_col, disp_x, disp_w), disp_y,
                     image_dark_line_map_x(center_col, disp_x, disp_w), disp_y + disp_h - 1,
                     RGB565_GREEN);

    for (i = 0; i < (int)s_dark_line_debug.pts_count; i++)
    {
        int cur_x = image_dark_line_map_x(s_dark_line_debug.pts[i].cx, disp_x, disp_w);
        int cur_y = image_dark_line_map_y(s_dark_line_debug.pts[i].row, disp_y, disp_h);

        if (prev_x >= 0)
        {
            ips200_draw_line(prev_x, prev_y, cur_x, cur_y, (uint16)0xFFE0u);
        }
        else
        {
            ips200_draw_point(cur_x, cur_y, (uint16)0xFFE0u);
        }
        prev_x = cur_x;
        prev_y = cur_y;
    }

    if (s_dark_line_debug.pts_count >= 2u)
    {
        int cx0 = s_dark_line_debug.mean_cx +
                  (s_dark_line_debug.slope_x1000 * (row0 - s_dark_line_debug.mean_row)) / 1000;
        int cx1 = s_dark_line_debug.mean_cx +
                  (s_dark_line_debug.slope_x1000 * (row1 - s_dark_line_debug.mean_row)) / 1000;

        cx0 = image_dark_line_clamp_col(cx0);
        cx1 = image_dark_line_clamp_col(cx1);
        fit_x0 = image_dark_line_map_x(cx0, disp_x, disp_w);
        fit_y0 = image_dark_line_map_y(row0, disp_y, disp_h);
        fit_x1 = image_dark_line_map_x(cx1, disp_x, disp_w);
        fit_y1 = image_dark_line_map_y(row1, disp_y, disp_h);
        ips200_draw_line(fit_x0, fit_y0, fit_x1, fit_y1, RGB565_PURPLE);
    }
}

/**
 * Bumpy 调试页：压缩二值图 + INSIDE 时中心点折线叠加。
 * 复用实车状态机但不写 dualcore，避免菜单调试影响 CM7_0 控车。
 */
void image_dark_line_debug_show(int disp_x, int disp_y, int bw_threshold)
{
    image_dark_line_result_t result;

    (void)bw_threshold;

    image_dark_line_detect(&result, 1u);
    image_dark_line_apply_state_machine(&result);

    s_dark_line_debug.white_ratio = result.white_ratio;
    s_dark_line_debug.element_active = result.element_active;
    s_dark_line_debug.track_valid = result.track_valid;
    if (result.element_active == 0u)
    {
        s_dark_line_debug.pts_count = 0u;
        s_dark_line_debug.center_err = 0.0f;
        s_dark_line_debug.slope_x1000 = 0;
    }

    Cammer_Err = result.center_err;

    ips200_show_gray_image(disp_x, disp_y, s_dark_line_gray_view[0],
                           IMAGE_COMPRESS_W, IMAGE_COMPRESS_H,
                           MT9V03X_W, MT9V03X_H, 0);

    if (result.element_active != 0u)
    {
        image_dark_line_draw_center_overlay(disp_x, disp_y, MT9V03X_W, MT9V03X_H);
    }
}

float image_dark_line_get_center_err(void)
{
    return s_dark_line_debug.center_err;
}

uint8 image_dark_line_get_track_valid(void)
{
    return s_dark_line_debug.track_valid;
}

uint8 image_dark_line_get_pts_count(void)
{
    return s_dark_line_debug.pts_count;
}

float image_dark_line_get_white_ratio(void)
{
    return s_dark_line_debug.white_ratio;
}

uint8 image_dark_line_get_element_active(void)
{
    return s_dark_line_debug.element_active;
}

int image_dark_line_get_slope_x1000(void)
{
    return s_dark_line_debug.slope_x1000;
}

#endif /* CY_CORE_CM7_1 && IMAGE_DARK_LINE_VALIDATE_ENABLE */

#if defined(CY_CORE_CM7_0) && IMAGE_DARK_LINE_VALIDATE_ENABLE

#include "control.h"
#include "navigation.h"
#include "dualcore_shared.h"
#include "init.h"
#include <math.h>

static float image_dark_line_clipf(float v, float lo, float hi)
{
    if (v < lo)
    {
        return lo;
    }
    if (v > hi)
    {
        return hi;
    }
    return v;
}

/** 元素内黑线横向误差 → 绝对航向目标，经 steer_request_target_yaw 交给 1ms 航向双环 */
static void image_dark_line_apply_yaw_from_snapshot(const dualcore_dark_line_snapshot_t *snap)
{
    float target_offset_deg;
    float target_yaw_deg;

    if (snap == NULL)
    {
        return;
    }
    if (snap->fresh == 0u || snap->track_valid == 0u || snap->element_active == 0u)
    {
        return;
    }
    if (spin_enable != 0u || Motor_Runaway_Latch != 0u)
    {
        return;
    }
    /* 回放态默认禁视觉 yaw */
    if (N.Nag_SystemRun_Index == 3u)
    {
        return;
    }

    target_offset_deg = snap->center_err * IMAGE_DARK_LINE_YAW_K_PIXEL;
    target_offset_deg = image_dark_line_clipf(target_offset_deg,
                                              -IMAGE_DARK_LINE_YAW_MAX_OFFSET_DEG,
                                              IMAGE_DARK_LINE_YAW_MAX_OFFSET_DEG);
    if (fabsf(target_offset_deg) < IMAGE_DARK_LINE_YAW_DEADBAND_DEG)
    {
        return;
    }

    target_yaw_deg = (float)euler_angle.yaw + target_offset_deg;
    steer_request_target_yaw(target_yaw_deg);
}

void image_dark_line_apply_yaw(void)
{
    dualcore_dark_line_snapshot_t snap;

    dualcore_dark_line_pull_snapshot(&snap);

    if (snap.enter_pulse != 0u)
    {
        buzzer_beep_request(BRIDGE_BEEP_MS);
    }
    if (snap.exit_pulse != 0u)
    {
        buzzer_beep_request(BRIDGE_BEEP_MS);
    }

    image_dark_line_apply_yaw_from_snapshot(&snap);
}

#endif /* CY_CORE_CM7_0 && IMAGE_DARK_LINE_VALIDATE_ENABLE */
