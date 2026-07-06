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

static float  s_last_road_w_avg;
static uint8  s_last_pin_left;
static uint8  s_last_pin_right;
static uint8  s_last_enter_ready;
static uint8  s_last_exit_ready;
static uint8  s_enter_debounce;
static uint8  s_exit_debounce;

static uint8 single_bridge_row_pin_left(int i)
{
    int left = (int)Left_Line[i];
    int tol = SINGLE_BRIDGE_EDGE_PIN_TOL;

    if (i < SINGLE_BRIDGE_DETECT_ROW_START || i > SINGLE_BRIDGE_DETECT_ROW_END)
    {
        return 0u;
    }
    if (left <= tol)
    {
        return 1u;
    }
    return 0u;
}

static uint8 single_bridge_row_pin_right(int i)
{
    int right = (int)Right_Line[i];
    int tol = SINGLE_BRIDGE_EDGE_PIN_TOL;
    int w = (int)IMAGE_COMPRESS_W;

    if (i < SINGLE_BRIDGE_DETECT_ROW_START || i > SINGLE_BRIDGE_DETECT_ROW_END)
    {
        return 0u;
    }
    if (right >= (w - 1 - tol))
    {
        return 1u;
    }
    return 0u;
}

static uint8 single_bridge_row_width_ok(int i, int *width_out)
{
    int left = (int)Left_Line[i];
    int right = (int)Right_Line[i];
    int tol = SINGLE_BRIDGE_EDGE_PIN_TOL;
    int w = (int)IMAGE_COMPRESS_W;
    uint8 left_ok;
    uint8 right_ok;

    if (i < SINGLE_BRIDGE_DETECT_ROW_START || i > SINGLE_BRIDGE_DETECT_ROW_END)
    {
        return 0u;
    }

    left_ok = (uint8)(left > tol && left < (w - 1 - tol));
    right_ok = (uint8)(right > tol && right < (w - 1 - tol));

    if (left_ok && right_ok)
    {
        *width_out = right - left;
        return 1u;
    }
    if (left_ok && single_bridge_row_pin_right(i))
    {
        *width_out = (w - 1) - left;
        return 1u;
    }
    if (right_ok && single_bridge_row_pin_left(i))
    {
        *width_out = right;
        return 1u;
    }
    return 0u;
}

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
    /* valid_row_count 与 end_line 共同决定中线是否可用于控车/切换判定 */
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

float single_bridge_get_road_w_avg(void)
{
    return s_last_road_w_avg;
}

uint8 single_bridge_get_pin_left(void)
{
    return s_last_pin_left;
}

uint8 single_bridge_get_pin_right(void)
{
    return s_last_pin_right;
}

uint8 single_bridge_get_enter_ready(void)
{
    return s_last_enter_ready;
}

uint8 single_bridge_get_exit_ready(void)
{
    return s_last_exit_ready;
}

int single_bridge_get_width_max(void)
{
    return SINGLE_BRIDGE_WIDTH_MAX_DEFAULT;
}

void single_bridge_detect_update(single_bridge_detect_t *out, uint8 on_bridge)
{
    int   i;
    int   width = 0;
    int   width_sum = 0;
    int   width_count = 0;
    int   pin_left_rows = 0;
    int   pin_right_rows = 0;
    uint8 enter_ready = 0u;
    uint8 exit_ready = 0u;
    uint8 enter_confirmed = 0u;
    uint8 exit_confirmed = 0u;
    uint8 both_valid_rows = 0u;

    for (i = SINGLE_BRIDGE_DETECT_ROW_START; i <= SINGLE_BRIDGE_DETECT_ROW_END; i++)
    {
        if (single_bridge_row_pin_left(i))
        {
            pin_left_rows++;
        }
        if (single_bridge_row_pin_right(i))
        {
            pin_right_rows++;
        }
        if (single_bridge_row_width_ok(i, &width))
        {
            width_sum += width;
            width_count++;
        }
        if (single_bridge_left_valid(i, (int)Left_Line[i]) &&
            single_bridge_right_valid(i, (int)Right_Line[i]) &&
            !single_bridge_row_pin_left(i) &&
            !single_bridge_row_pin_right(i))
        {
            both_valid_rows++;
        }
    }

    if (width_count > 0)
    {
        s_last_road_w_avg = (float)width_sum / (float)width_count;
    }
    else
    {
        s_last_road_w_avg = (float)IMAGE_COMPRESS_W;
    }

    s_last_pin_left = (uint8)(pin_left_rows >= (int)SINGLE_BRIDGE_MIN_PIN_ROWS);
    s_last_pin_right = (uint8)(pin_right_rows >= (int)SINGLE_BRIDGE_MIN_PIN_ROWS);

    if (s_last_pin_right && !s_last_pin_left)
    {
        /* 右贴边 → 左桥 */
    }
    else if (s_last_pin_left && !s_last_pin_right)
    {
        /* 左贴边 → 右桥 */
    }

    if (!on_bridge)
    {
        if (s_last_road_w_avg < (float)SINGLE_BRIDGE_WIDTH_MAX_DEFAULT &&
            (s_last_pin_left || s_last_pin_right))
        {
            enter_ready = 1u;
        }
        if (enter_ready)
        {
            if (s_enter_debounce < 255u)
            {
                s_enter_debounce++;
            }
        }
        else
        {
            s_enter_debounce = 0u;
        }
        s_exit_debounce = 0u;

        if (s_enter_debounce >= SINGLE_BRIDGE_ENTER_DEBOUNCE)
        {
            enter_confirmed = 1u;
            s_enter_debounce = 0u;
        }
    }
    else
    {
        if (s_last_road_w_avg > (float)SINGLE_BRIDGE_WIDTH_EXIT_MIN_DEFAULT &&
            both_valid_rows >= (int)SINGLE_BRIDGE_MIN_PIN_ROWS)
        {
            exit_ready = 1u;
        }
        if (exit_ready)
        {
            if (s_exit_debounce < 255u)
            {
                s_exit_debounce++;
            }
        }
        else
        {
            s_exit_debounce = 0u;
        }
        s_enter_debounce = 0u;

        if (s_exit_debounce >= SINGLE_BRIDGE_EXIT_DEBOUNCE)
        {
            exit_confirmed = 1u;
            s_exit_debounce = 0u;
        }
    }

    s_last_enter_ready = enter_ready;
    s_last_exit_ready = exit_ready;

    if (out != NULL)
    {
        out->road_w_avg = s_last_road_w_avg;
        out->pin_left = s_last_pin_left;
        out->pin_right = s_last_pin_right;
        out->side = SINGLE_BRIDGE_SIDE_UNKNOWN;
        if (s_last_pin_right && !s_last_pin_left)
        {
            out->side = SINGLE_BRIDGE_SIDE_LEFT;
        }
        else if (s_last_pin_left && !s_last_pin_right)
        {
            out->side = SINGLE_BRIDGE_SIDE_RIGHT;
        }
        out->enter_ready = enter_ready;
        out->exit_ready = exit_ready;
        out->enter_confirmed = enter_confirmed;
        out->exit_confirmed = exit_confirmed;
        out->enter_count = s_enter_debounce;
        out->exit_count = s_exit_debounce;
    }
}

void single_bridge_detect_reset(void)
{
    s_enter_debounce = 0u;
    s_exit_debounce = 0u;
}

void single_bridge_gray_diff_track(int bw_threshold, single_bridge_track_t *out)
{
    int th = bw_threshold;

    if (th <= 0)
    {
        th = SINGLE_BRIDGE_DIFF_TH_DEFAULT;
    }
    hd_threshold = th;

    /* 灰度差比和寻左右边 → 中线 → center_err；供中线模式与 Debug 页 */
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
    single_bridge_track_t track;
    single_bridge_detect_t detect;

    single_bridge_gray_diff_track(bw_threshold, &track);
    single_bridge_detect_update(&detect, 0u);

    ips200_show_gray_image(disp_x, disp_y, image_two_value[0],
                           IMAGE_COMPRESS_W, IMAGE_COMPRESS_H,
                           MT9V03X_W, MT9V03X_H, 0);

    single_bridge_draw_track_overlay(disp_x, disp_y, MT9V03X_W, MT9V03X_H);
}
