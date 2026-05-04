/*********************************************************************************************************************
 * @file    image.c
 * @brief   图像 1/2 压缩、大津阈值、缓冲区二值化实现；接口说明见 image.h。
 *********************************************************************************************************************/
#include "image.h"

/** 压缩灰度图实体定义（与 image.h extern 对应）；按 4 字节对齐存储 */
uint8 image_gray_compress[IMAGE_COMPRESS_H][IMAGE_COMPRESS_W]
    __attribute__((aligned(4)));

/*--------------------------------------------------------------------------------------------------------------------
 * @brief  抽样规则：`image_gray_compress[i][j] = full[2*i][2*j]`（偶行、偶列）。
 * @note   `src_full_row0` 步长按 MT9V03X_W 递增行。
 *-------------------------------------------------------------------------------------------------------------------*/
void image_gray_compress_from_full(const uint8 *src_full_row0)
{
    uint16 i;
    uint16 j;

    for (i = 0; i < IMAGE_COMPRESS_H; i++)
    {
        const uint8 *row = src_full_row0 + (uint32)(i * 2u) * (uint32)MT9V03X_W;

        for (j = 0; j < IMAGE_COMPRESS_W; j++)
        {
            image_gray_compress[i][j] = row[(uint32)j * 2u];
        }
    }
}

/*--------------------------------------------------------------------------------------------------------------------
 * @brief  大津法：对直方图在 [min_value, max_value] 上枚举阈值 t，取类间方差 σ_B² 最大时的 t。
 * @note   变量命名与常见教材对应：ω 为类概率，μ 为类均值；此处「前/后」两类随 t 滑动更新。
 *-------------------------------------------------------------------------------------------------------------------*/
uint8 image_otsu_threshold(const uint8 *image, uint16 col, uint16 row)
{
#define IMG_OTSU_GRAY_LEVELS (256)
    uint16 img_w = col;
    uint16 img_h = row;
    int32 hist[IMG_OTSU_GRAY_LEVELS];   /* 256 档灰度直方图，hist[g]=灰度 g 的像素个数 */
    int32 x;
    uint16 y;
    const uint8 *data = image;
    uint32 amount = 0;                  /* 有效灰度区间内像素总数 Σ hist */
    uint32 pixel_back = 0;              /* 当前阈值一侧（背景侧）像素累计个数 */
    uint32 pixel_integral_back = 0;     /* 同上类的灰度值总和 Σ g·N_g */
    uint32 pixel_integral = 0;        /* 全图总灰度和，用于求另一侧 */
    int32 pixel_integral_fore = 0;    /* 前景侧灰度和 = pixel_integral - pixel_integral_back */
    int32 pixel_fore = 0;             /* 前景侧像素数 */
    double omega_back;                 /* 背景类占全图比例 ω0 */
    double omega_fore;                 /* 前景类占全图比例 ω1 */
    double micro_back;                 /* 背景类均值 μ0 */
    double micro_fore;                 /* 前景类均值 μ1 */
    double sigma_b;                   /* 当前最大的类间方差 g=ω0·ω1·(μ0-μ1)² */
    double sigma = 0;
    uint8 min_value = 0;
    uint8 max_value = 0;
    uint8 threshold = 0;

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

/*--------------------------------------------------------------------------------------------------------------------
 * @brief  按固定阈值将灰度压成两档；下标 `i * col + j` 表示第 i 行第 j 列。
 *-------------------------------------------------------------------------------------------------------------------*/
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
