#ifndef PROJECT_CODE_SINGLE_BRIDGE_H_
#define PROJECT_CODE_SINGLE_BRIDGE_H_

#include "zf_common_typedef.h"

/** hd_whitemax 差比和阈值默认（×100 尺度） */
#define SINGLE_BRIDGE_DIFF_TH_DEFAULT       18

/** 压缩图近底前瞻起始行（IMAGE_COMPRESS_H==60 时约图像下方） */
#define SINGLE_BRIDGE_PROSPECT_START_DEFAULT 50

/** 参与加权的中线最少有效行数，低于此 track_valid=0 */
#define SINGLE_BRIDGE_MIN_VALID_ROWS        6u

/** Mid_Line[] 无效行标记 */
#define SINGLE_BRIDGE_MID_INVALID           (-1)

typedef struct
{
    float err_sum;
    float center_err;
    int   end_line;
    int   maxline;
    uint8 track_valid;
    uint8 valid_row_count;
} single_bridge_track_t;

void single_bridge_compute_midline(void);
float single_bridge_center_error(int prospect_start);
uint8 single_bridge_track_valid(uint8 min_rows);
uint8 single_bridge_mid_valid(int i);

/** 压缩灰度 + hd_whitemax 差比和寻边 + 中线/误差，结果写入 Left/Right/Mid_Line */
void single_bridge_gray_diff_track(int bw_threshold, single_bridge_track_t *out);

void single_bridge_draw_edges_overlay(int disp_x, int disp_y,
                                      int disp_w, int disp_h);
void single_bridge_draw_midline_overlay(int disp_x, int disp_y,
                                        int disp_w, int disp_h);
void single_bridge_draw_track_overlay(int disp_x, int disp_y,
                                      int disp_w, int disp_h);

/** GUI 调试：track + 灰度图 + 红/绿/蓝三线（不参与惯导控车） */
void single_bridge_debug_show(int disp_x, int disp_y, int bw_threshold);

/** 最近一次 gray_diff_track 结果（Debug 页读取） */
float single_bridge_get_center_err(void);
uint8 single_bridge_get_track_valid(void);
uint8 single_bridge_get_valid_row_count(void);

#endif /* PROJECT_CODE_SINGLE_BRIDGE_H_ */
