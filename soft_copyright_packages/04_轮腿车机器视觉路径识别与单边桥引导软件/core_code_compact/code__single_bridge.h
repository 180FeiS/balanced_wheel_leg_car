#ifndef PROJECT_CODE_SINGLE_BRIDGE_H_
#define PROJECT_CODE_SINGLE_BRIDGE_H_
#include "zf_common_typedef.h"
/** hd_whitemax 差比和阈值默认（×100 尺度） */
#define SINGLE_BRIDGE_DIFF_TH_DEFAULT       18
/** 压缩图近底前瞻起始行（IMAGE_COMPRESS_H==60 时约图像下方） */
#define SINGLE_BRIDGE_PROSPECT_START_DEFAULT 50
/** 参与加权的中线最少有效行数，低于此 track_valid=0；亦用于中线失效回退白块 */
#define SINGLE_BRIDGE_MIN_VALID_ROWS        6u
/** 中线 track_valid 连续失效达到该帧数后，主循环状态机退回白块模式（见 image.c） */
#define SINGLE_BRIDGE_LOST_FALLBACK_DEBOUNCE  4u
/** Mid_Line[] 无效行标记 */
#define SINGLE_BRIDGE_MID_INVALID           (-1)
/** 方案 A：近底路宽 + 贴边检测（压缩图像素，实车标定） */
#define SINGLE_BRIDGE_WIDTH_MAX_DEFAULT      48
#define SINGLE_BRIDGE_WIDTH_EXIT_MIN_DEFAULT 58
#define SINGLE_BRIDGE_EDGE_PIN_TOL           2
#define SINGLE_BRIDGE_DETECT_ROW_START       45
#define SINGLE_BRIDGE_DETECT_ROW_END         58
#define SINGLE_BRIDGE_ENTER_DEBOUNCE         4u
#define SINGLE_BRIDGE_EXIT_DEBOUNCE          4u
#define SINGLE_BRIDGE_MIN_PIN_ROWS           4u
typedef enum
{
    SINGLE_BRIDGE_SIDE_UNKNOWN = 0u,
    SINGLE_BRIDGE_SIDE_LEFT  = 1u, /* 左桥：右线贴边 */
    SINGLE_BRIDGE_SIDE_RIGHT = 2u, /* 右桥：左线贴边 */
} single_bridge_side_t;
typedef struct
{
    float err_sum;
    float center_err;
    int   end_line;
    int   maxline;
    uint8 track_valid;
    uint8 valid_row_count;
} single_bridge_track_t;
typedef struct
{
    float road_w_avg;
    uint8 pin_left;
    uint8 pin_right;
    uint8 side;
    uint8 enter_ready;
    uint8 exit_ready;
    uint8 enter_confirmed;
    uint8 exit_confirmed;
    uint8 enter_count;
    uint8 exit_count;
} single_bridge_detect_t;
void single_bridge_compute_midline(void);
float single_bridge_center_error(int prospect_start);
uint8 single_bridge_track_valid(uint8 min_rows);
uint8 single_bridge_mid_valid(int i);
/** 压缩灰度 + hd_whitemax 差比和寻边 + 中线/误差，结果写入 Left/Right/Mid_Line */
void single_bridge_gray_diff_track(int bw_threshold, single_bridge_track_t *out);
/** 方案 A 进/出桥检测；on_bridge=1 时累计出桥 debounce，否则累计进桥 debounce */
void single_bridge_detect_update(single_bridge_detect_t *out, uint8 on_bridge);
void single_bridge_detect_reset(void);
void single_bridge_draw_edges_overlay(int disp_x, int disp_y,
                                      int disp_w, int disp_h);
void single_bridge_draw_midline_overlay(int disp_x, int disp_y,
                                        int disp_w, int disp_h);
void single_bridge_draw_track_overlay(int disp_x, int disp_y,
                                      int disp_w, int disp_h);
/** GUI 调试：track + 灰度图 + 红/绿/蓝三线（不参与惯导控车） */
void single_bridge_debug_show(int disp_x, int disp_y, int bw_threshold);
/** 最近一次 gray_diff_track / detect 结果（Debug 页读取） */
float single_bridge_get_center_err(void);
uint8 single_bridge_get_track_valid(void);
uint8 single_bridge_get_valid_row_count(void);
float single_bridge_get_road_w_avg(void);
uint8 single_bridge_get_pin_left(void);
uint8 single_bridge_get_pin_right(void);
uint8 single_bridge_get_enter_ready(void);
uint8 single_bridge_get_exit_ready(void);
int   single_bridge_get_width_max(void);
#endif /* PROJECT_CODE_SINGLE_BRIDGE_H_ */
