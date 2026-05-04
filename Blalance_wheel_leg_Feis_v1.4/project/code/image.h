/*********************************************************************************************************************
 * @file    image.h
 * @brief   摄像头灰度图 1/2 抽样压缩、大津法（Otsu）阈值、缓冲区二值化；思路来自 TC387 camera 模块，与全场 raw 分离缓冲。
 *
 * @note
 *   - MT9V03X_H、MT9V03X_W 须为偶数：每 2×2 像素取左上角一点，等效长宽各减半。
 *   - 二值化请使用独立缓冲区（如压缩图的副本），勿直接覆盖仍用于显示或上游算法的 `mt9v03x_image`。
 *********************************************************************************************************************/
#ifndef PROJECT_CODE_IMAGE_H_
#define PROJECT_CODE_IMAGE_H_

#include "zf_common_typedef.h"
#include "zf_device_mt9v03x.h"

#if ((MT9V03X_H) & 1u) || ((MT9V03X_W) & 1u)
#error "MT9V03X_H/W must be even for 1/2 compress"
#endif

/** 压缩图行数（高度），为全场 MT9V03X_H 的 1/2，单位：像素行 */
#define IMAGE_COMPRESS_H    (MT9V03X_H / 2)
/** 压缩图列数（宽度），为全场 MT9V03X_W 的 1/2，单位：像素列 */
#define IMAGE_COMPRESS_W    (MT9V03X_W / 2)

/** 二值化后「黑」侧写入的灰度值，固定 0x00 */
#define IMAGE_BIN_BLACK     ((uint8)0x00)
/** 二值化后「白」侧写入的灰度值，固定 0xff */
#define IMAGE_BIN_WHITE     ((uint8)0xff)

/** 1/2 抽样后的灰度图；4 字节对齐，利于与裸机习惯一致 */
extern uint8 image_gray_compress[IMAGE_COMPRESS_H][IMAGE_COMPRESS_W]
    __attribute__((aligned(4)));

/**
 * @brief  从全场采集缓冲做 1/2 行列抽样，结果写入 `image_gray_compress`。
 * @param  src_full_row0 全场图像第 0 行首地址，通常为 `mt9v03x_image[0]`；
 *                       行宽为 MT9V03X_W，共 MT9V03X_H 行，布局与 `zf_device_mt9v03x` 一致。
 */
void image_gray_compress_from_full(const uint8 *src_full_row0);

/**
 * @brief  大津法计算灰度分割阈值（最大化类间方差）。
 * @param  image 灰度数据，行优先（row-major）：第 (r,c) 像素下标为 `r * col + c`。
 * @param  col   图像宽度（列数），单位：像素。
 * @param  row   图像高度（行数），单位：像素。
 * @return       最优阈值，范围 0~255；若仅单色或两灰度等退化情形，返回约定值便于直接使用。
 */
uint8 image_otsu_threshold(const uint8 *image, uint16 col, uint16 row);

/**
 * @brief  对缓冲区就地二值化（不读写 `mt9v03x_image`）。
 * @param  buf       灰度缓冲区，行优先：第 (r,c) 为 `buf[r * col + c]`。
 * @param  col       宽度（列数）。
 * @param  row       高度（行数）。
 * @param  threshold 分割阈值：`buf[i] >= threshold` 写 `white`，否则写 `black`。
 * @param  black     低于阈值时写入的值，通常为 IMAGE_BIN_BLACK。
 * @param  white     不低于阈值时写入的值，通常为 IMAGE_BIN_WHITE。
 */
void image_binarize_buffer(uint8 *buf, uint16 col, uint16 row,
                           int threshold, uint8 black, uint8 white);

#endif /* PROJECT_CODE_IMAGE_H_ */
