/*********************************************************************************************************************
 * @file    single_bridge.c
 * @brief   单边桥：灰度差比和寻边 + 中线合成/误差 + IPS200 三线叠加。
 *********************************************************************************************************************/
#include "zf_common_headfile.h"
#include "image.h"
#include "single_bridge.h"

static float  s_last_center_err;
static uint8  s_last_track_valid;
static uint8  s_last_valid_row_count;
static uint8  s_last_valid_row_count_raw;

static int single_bridge_map_x(int j, int disp_x, int disp_w)
{
    return disp_x + (j * disp_w) / (int)IMAGE_COMPRESS_W;
}

static int single_bridge_map_y(int i, int disp_y, int disp_h)
{
    return disp_y + (i * disp_h) / (int)IMAGE_COMPRESS_H;
}

static uint8 single_bridge_left_valid(int i, int left)
{
    if (left <= 0)
    {
        return 0u;
    }
    if (left >= (int)IMAGE_COMPRESS_W - 1)
    {
        return 0u;
    }
    if (i < 10 || i >= (int)IMAGE_COMPRESS_H - 5)
    {
        return 0u;
    }
    return 1u;
}

static uint8 single_bridge_right_valid(int i, int right)
{
    if (right <= 0)
    {
        return 0u;
    }
    if (right >= (int)IMAGE_COMPRESS_W - 1)
    {
        return 0u;
    }
    if (i < 10 || i >= (int)IMAGE_COMPRESS_H - 5)
    {
        return 0u;
    }
    return 1u;
}

uint8 single_bridge_mid_valid(int i)
{
    if (i < 10 || i >= (int)IMAGE_COMPRESS_H - 5)
    {
        return 0u;
    }
    if ((int)Mid_Line[i] == SINGLE_BRIDGE_MID_INVALID)
    {
        return 0u;
    }
    return single_bridge_left_valid(i, (int)Left_Line[i]) &&
           single_bridge_right_valid(i, (int)Right_Line[i]);
}

void single_bridge_compute_midline(void)
{
    int i;

    for (i = 0; i < (int)IMAGE_COMPRESS_H; i++)
    {
        Mid_Line[i] = SINGLE_BRIDGE_MID_INVALID;
    }

    for (i = 10; i < (int)IMAGE_COMPRESS_H - 5; i++)
    {
        if (single_bridge_left_valid(i, (int)Left_Line[i]) &&
            single_bridge_right_valid(i, (int)Right_Line[i]))
        {
            Mid_Line[i] = (((int)Left_Line[i] + (int)Right_Line[i]) >> 1);
        }
    }
}

float single_bridge_center_error(int prospect_start)
{
    int   i;
    int   start_row;
    int32 sum = 0;

    s_last_valid_row_count_raw = 0u;

    start_row = prospect_start;
    if (start_row < 10)
    {
        start_row = 10;
    }
    if (end_line > start_row)
    {
        start_row = end_line - 8;
    }
    if (start_row < 10)
    {
        start_row = 10;
    }
    if (start_row > (int)IMAGE_COMPRESS_H - 6)
    {
        start_row = (int)IMAGE_COMPRESS_H - 6;
    }

    for (i = start_row; i < (int)IMAGE_COMPRESS_H - 5; i++)
    {
        if (!single_bridge_mid_valid(i))
        {
            continue;
        }
        s_last_valid_row_count_raw++;
        sum += (int32)image_get_err_weight(i) *
               (int32)(((int)IMAGE_COMPRESS_W / 2) - (int)Mid_Line[i]);
    }

    s_last_valid_row_count = s_last_valid_row_count_raw;
    return -(float)sum;
}

uint8 single_bridge_track_valid(uint8 min_rows)
{
    if (s_last_valid_row_count_raw < min_rows)
    {
        return 0u;
    }
    if (end_line < 10 || end_line >= (int)IMAGE_COMPRESS_H - 5)
    {
        return 0u;
    }
    return 1u;
}

float single_bridge_get_center_err(void)
{
    return s_last_center_err;
}

uint8 single_bridge_get_track_valid(void)
{
    return s_last_track_valid;
}

uint8 single_bridge_get_valid_row_count(void)
{
    return s_last_valid_row_count;
}

void single_bridge_gray_diff_track(int bw_threshold, single_bridge_track_t *out)
{
    int th = bw_threshold;

    if (th <= 0)
    {
        th = SINGLE_BRIDGE_DIFF_TH_DEFAULT;
    }
    hd_threshold = th;

    image_photo_compress(mt9v03x_image[0]);
    hd_whitemax(th);
    single_bridge_compute_midline();

    s_last_center_err = single_bridge_center_error(SINGLE_BRIDGE_PROSPECT_START_DEFAULT);
    s_last_track_valid = single_bridge_track_valid(SINGLE_BRIDGE_MIN_VALID_ROWS);
    Cammer_Err = s_last_center_err;

    if (out != NULL)
    {
        out->err_sum = s_last_center_err;
        out->center_err = s_last_center_err;
        out->end_line = end_line;
        out->maxline = int_test_printf;
        out->track_valid = s_last_track_valid;
        out->valid_row_count = s_last_valid_row_count;
    }
}

void single_bridge_draw_edges_overlay(int disp_x, int disp_y,
                                      int disp_w, int disp_h)
{
    int i;
    int prev_left = -1;
    int prev_right = -1;
    int prev_left_y = 0;
    int prev_right_y = 0;
    int cur_left_x;
    int cur_left_y;
    int cur_right_x;
    int cur_right_y;

    for (i = 10; i < (int)IMAGE_COMPRESS_H - 5; i++)
    {
        if (single_bridge_left_valid(i, (int)Left_Line[i]))
        {
            cur_left_x = single_bridge_map_x((int)Left_Line[i], disp_x, disp_w);
            cur_left_y = single_bridge_map_y(i, disp_y, disp_h);

            if (prev_left >= 0)
            {
                ips200_draw_line(prev_left, prev_left_y, cur_left_x, cur_left_y, RGB565_RED);
            }
            else
            {
                ips200_draw_point(cur_left_x, cur_left_y, RGB565_RED);
            }

            prev_left = cur_left_x;
            prev_left_y = cur_left_y;
        }

        if (single_bridge_right_valid(i, (int)Right_Line[i]))
        {
            cur_right_x = single_bridge_map_x((int)Right_Line[i], disp_x, disp_w);
            cur_right_y = single_bridge_map_y(i, disp_y, disp_h);

            if (prev_right >= 0)
            {
                ips200_draw_line(prev_right, prev_right_y, cur_right_x, cur_right_y, RGB565_GREEN);
            }
            else
            {
                ips200_draw_point(cur_right_x, cur_right_y, RGB565_GREEN);
            }

            prev_right = cur_right_x;
            prev_right_y = cur_right_y;
        }
    }
}

void single_bridge_draw_midline_overlay(int disp_x, int disp_y,
                                        int disp_w, int disp_h)
{
    int i;
    int prev_mid = -1;
    int prev_mid_y = 0;
    int cur_mid_x;
    int cur_mid_y;

    for (i = 10; i < (int)IMAGE_COMPRESS_H - 5; i++)
    {
        if (!single_bridge_mid_valid(i))
        {
            prev_mid = -1;
            continue;
        }

        cur_mid_x = single_bridge_map_x((int)Mid_Line[i], disp_x, disp_w);
        cur_mid_y = single_bridge_map_y(i, disp_y, disp_h);

        if (prev_mid >= 0)
        {
            ips200_draw_line(prev_mid, prev_mid_y, cur_mid_x, cur_mid_y, RGB565_BLUE);
        }
        else
        {
            ips200_draw_point(cur_mid_x, cur_mid_y, RGB565_BLUE);
        }

        prev_mid = cur_mid_x;
        prev_mid_y = cur_mid_y;
    }
}

void single_bridge_draw_track_overlay(int disp_x, int disp_y,
                                      int disp_w, int disp_h)
{
    single_bridge_draw_edges_overlay(disp_x, disp_y, disp_w, disp_h);
    single_bridge_draw_midline_overlay(disp_x, disp_y, disp_w, disp_h);
}

void single_bridge_debug_show(int disp_x, int disp_y, int bw_threshold)
{
    single_bridge_gray_diff_track(bw_threshold, NULL);

    ips200_show_gray_image(disp_x, disp_y, image_two_value[0],
                           IMAGE_COMPRESS_W, IMAGE_COMPRESS_H,
                           MT9V03X_W, MT9V03X_H, 0);

    single_bridge_draw_track_overlay(disp_x, disp_y, MT9V03X_W, MT9V03X_H);
}
