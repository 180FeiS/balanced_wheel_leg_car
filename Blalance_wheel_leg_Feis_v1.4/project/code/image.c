/*********************************************************************************************************************
 * @file    image.c
 * @brief   TC387 camera.c 迁入：压缩、大津、二值、边线、最长白列、误差与自动曝光（见 image.h）。
 *********************************************************************************************************************/
#include "image.h"
#include <stdlib.h>

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
 * @brief  TC `v_iftc_camera_autoexposure`：压缩 ROI 亮度驱动曝光；有界迭代，不操作 `mt9v03x_finish_flag`。
 *         门限 380000/410000 对应原 TC 中心 ROI 标定，改分辨率需重标。
 *-------------------------------------------------------------------------------------------------------------------*/
void image_camera_auto_exposure(void)
{
    int         camera_light;
    int         i;
    int         j;
    uint32      k;
    const int   ae_lo = 380000;
    const int   ae_hi = 410000;
    const uint32 max_iter = 128u;

    for (k = 0u; k < max_iter; k++)
    {
        image_photo_compress(mt9v03x_image[0]);

        camera_light = 0;
        for (j = 10; j < (int)IMAGE_COMPRESS_W - 10; j++)
        {
            for (i = 10; i < (int)IMAGE_COMPRESS_H - 10; i++)
            {
                camera_light += (int)image_two_value[i][j];
            }
        }

        if (camera_light < ae_lo)
        {
            if (image_camera_exposure < 65535u)
            {
                image_camera_exposure++;
            }
            (void)mt9v03x_set_exposure_time(image_camera_exposure);
        }
        else if (camera_light > ae_hi)
        {
            if (image_camera_exposure > 0u)
            {
                image_camera_exposure--;
            }
            (void)mt9v03x_set_exposure_time(image_camera_exposure);
        }
        else
        {
            break;
        }
    }

    test_printf_light = camera_light;
}
