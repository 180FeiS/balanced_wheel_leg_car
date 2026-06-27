#ifndef PROJECT_CODE_SINGLE_BRIDGE_H_
#define PROJECT_CODE_SINGLE_BRIDGE_H_

#include "zf_common_typedef.h"

/** hd_whitemax 差比和阈值默认（×100 尺度） */
#define SINGLE_BRIDGE_DIFF_TH_DEFAULT  18

typedef struct
{
    float err_sum;
    int   end_line;
    int   maxline;
} single_bridge_track_t;

/** 压缩灰度 + hd_whitemax 差比和寻边，结果写入 Left_Line / Right_Line */
void single_bridge_gray_diff_track(int bw_threshold, single_bridge_track_t *out);

/** 在已显示灰度图区域上叠加左右边线（屏幕坐标） */
void single_bridge_draw_edges_overlay(int disp_x, int disp_y,
                                      int disp_w, int disp_h);

/** GUI 调试：track + 显示压缩灰度图 + 叠加边线 */
void single_bridge_debug_show(int disp_x, int disp_y, int bw_threshold);

#endif /* PROJECT_CODE_SINGLE_BRIDGE_H_ */
