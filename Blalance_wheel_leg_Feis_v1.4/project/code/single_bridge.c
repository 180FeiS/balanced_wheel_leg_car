/*********************************************************************************************************************
 * @file    single_bridge.c
 * @brief   单边桥：灰度差比和寻边（hd_whitemax）+ IPS200 边线叠加显示。
 *********************************************************************************************************************/
#include "zf_common_headfile.h"
#include "image.h"
#include "single_bridge.h"

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

    Cammer_Err = Err_Sum();

    if (out != NULL)
    {
        out->err_sum = Cammer_Err;
        out->end_line = end_line;
        out->maxline = int_test_printf;
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

void single_bridge_debug_show(int disp_x, int disp_y, int bw_threshold)
{
    single_bridge_gray_diff_track(bw_threshold, NULL);

    ips200_show_gray_image(disp_x, disp_y, image_two_value[0],
                           IMAGE_COMPRESS_W, IMAGE_COMPRESS_H,
                           MT9V03X_W, MT9V03X_H, 0);

    single_bridge_draw_edges_overlay(disp_x, disp_y, MT9V03X_W, MT9V03X_H);
}
