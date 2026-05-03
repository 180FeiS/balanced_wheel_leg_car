#include "zf_common_headfile.h"

step_info_t step_data = {0};

/* 每处理完一帧摄像头数据后由 step_vofa_snapshot 更新，供 step_debug_send_to_vofa 发送 */
static float step_vofa_top_row;
static float step_vofa_bottom_row;
static float step_vofa_height_pix;
static float step_vofa_dist_raw_mm;
static float step_vofa_dist_filt_mm;
/* VOFA CH5：视觉跳跃影子（前置进度 / armed；仅 CM7_1 且启用 VISUAL_JUMP 时每圈更新；否则 VOFA 侧发 0） */
static float step_vofa_jump_dbg;

static void step_vofa_snapshot(int top, int bot, float hpix, float raw_mm, uint8 frame_ok)
{
    step_vofa_top_row = (top < 0) ? 0.0f : (float)top;
    step_vofa_bottom_row = (bot < 0) ? 0.0f : (float)bot;
    step_vofa_height_pix = hpix;
    step_vofa_dist_raw_mm = raw_mm;
    step_vofa_dist_filt_mm = step_data.distance_mm;
    (void)frame_ok;
}

static uint8 image_binary[MT9V03X_H][MT9V03X_W];
static uint16 edge_histogram[MT9V03X_H];
/* 软阈值：白→蓝时强阈值下过阈列数不足，用较低阈值再建一条直方图（梯度仍用 abs，与亮暗方向无关） */
static uint16 edge_histogram_soft[MT9V03X_H];
static uint8 edge_hist_soft_is_distinct; /* 1：软直方图与强不同，级联里才值得再搜 */
static uint8 threshold;

#define HISTORY_SIZE 10
static float distance_history[HISTORY_SIZE ] = {0};
static uint16 height_history[HISTORY_SIZE] = {0};
static uint8 history_index = 0;
static int8 detect_counter = 0;
static uint8 last_valid_height = 0;
static float last_valid_distance = 0;
static float min_distance_recorded = 9999.0f;
static uint8 fail_counter = 0;

/* 像素高中值：削弱单帧边缘行跳变对 calculate_step_distance 的影响 */
static uint16 height_med_fifo[STEP_HEIGHT_MED_WIN];
static uint8 height_med_cnt;

static void height_median_reset(void)
{
    height_med_cnt = 0;
    for (int i = 0; i < STEP_HEIGHT_MED_WIN; i++)
        height_med_fifo[i] = 0;
}

/* 推入本帧像素高，返回当前窗口内中值；未满窗口时返回已有数据的中值（更快可用） */
static uint16 push_median_height(uint16 new_h)
{
    if (height_med_cnt < STEP_HEIGHT_MED_WIN)
        height_med_fifo[height_med_cnt++] = new_h;
    else
    {
        for (int i = 0; i < STEP_HEIGHT_MED_WIN - 1; i++)
            height_med_fifo[i] = height_med_fifo[i + 1];
        height_med_fifo[STEP_HEIGHT_MED_WIN - 1] = new_h;
    }

    uint16 tmp[STEP_HEIGHT_MED_WIN];
    uint8 n = height_med_cnt;
    for (uint8 i = 0; i < n; i++)
        tmp[i] = height_med_fifo[i];
    for (uint8 a = 0; a + 1 < n; a++)
        for (uint8 b = a + 1; b < n; b++)
            if (tmp[b] < tmp[a])
            {
                uint16 s = tmp[a];
                tmp[a] = tmp[b];
                tmp[b] = s;
            }
    return tmp[n / 2];
}

void step_detection_init(void)
{
    step_data.detected = 0;
    step_data.step_row = 0;
    step_data.step_top_row = 0;
    step_data.step_height_pix = 0;
    step_data.distance_mm = 0.0f;
    step_data.distance_cm = 0.0f;
    step_data.bottom_row_raw = 0;

    for (int i = 0; i < HISTORY_SIZE; i++)
    {
        distance_history[i] = 0;
        height_history[i] = 0;
    }
    history_index = 0;
    detect_counter = 0;
    last_valid_height = 0;
    last_valid_distance = 0;
    min_distance_recorded = 9999.0f;
    fail_counter = 0;
    height_median_reset();
}

static void compute_horizontal_edge_histogram(void)
{
    const int d = STEP_EDGE_ROW_DELTA;
    const int hard = STEP_EDGE_GRAD_THRESH;
    const int use_soft = (STEP_EDGE_GRAD_SOFT_THRESH > 0 && STEP_EDGE_GRAD_SOFT_THRESH < hard);
    const int soft = use_soft ? STEP_EDGE_GRAD_SOFT_THRESH : hard;

    for (int i = d; i < MT9V03X_H - d; i++)
    {
        edge_histogram[i] = 0;
        edge_histogram_soft[i] = 0;
    }
    for (int i = d; i < MT9V03X_H - d; i++)
    {
        for (int j = 10; j < MT9V03X_W - 10; j++)
        {
            int gradient = abs((int)mt9v03x_image[i + d][j] - (int)mt9v03x_image[i - d][j]);
            if (gradient > hard)
                edge_histogram[i]++;
            if (gradient > soft)
                edge_histogram_soft[i]++;
        }
    }
    if (!use_soft)
    {
        for (int i = d; i < MT9V03X_H - d; i++)
            edge_histogram_soft[i] = edge_histogram[i];
        edge_hist_soft_is_distinct = 0;
    }
    else
        edge_hist_soft_is_distinct = 1;
}

static int find_strong_edge_row_hist(const uint16 *hist, int start_row, int end_row, int min_edges)
{
    int best_row = -1;
    uint16 max_edges = 0;

    for (int i = start_row; i < end_row; i++)
    {
        if (hist[i] > max_edges && hist[i] >= min_edges)
        {
            int is_local_max = 1;
            for (int k = -2; k <= 2; k++)
            {
                if (k != 0 && (i + k) >= start_row && (i + k) < end_row)
                {
                    if (hist[i + k] > hist[i])
                    {
                        is_local_max = 0;
                        break;
                    }
                }
            }

            if (is_local_max)
            {
                max_edges = hist[i];
                best_row = i;
            }
        }
    }

    return best_row;
}

/* 同一 ROI 内：强直方图严/宽 min_edges → 软直方图严/宽，用于白台面→蓝台阶等弱边 */
static int find_edge_row_cascade(const uint16 *hist_hard, const uint16 *hist_soft, int start, int end)
{
    const int min_strict = MT9V03X_W / STEP_EDGE_MIN_DIV;
#if STEP_BOTTOM_MIN_DIV_FALLBACK > 0
    const int min_loose = (STEP_BOTTOM_MIN_DIV_FALLBACK != STEP_EDGE_MIN_DIV)
                              ? (MT9V03X_W / STEP_BOTTOM_MIN_DIV_FALLBACK)
                              : min_strict;
#else
    const int min_loose = min_strict; /* FALLBACK 为 0 时不做宽 min 二次搜 */
#endif
    int row = find_strong_edge_row_hist(hist_hard, start, end, min_strict);
    if (row < 0 && min_loose != min_strict)
        row = find_strong_edge_row_hist(hist_hard, start, end, min_loose);
    if (row < 0 && edge_hist_soft_is_distinct)
    {
        row = find_strong_edge_row_hist(hist_soft, start, end, min_strict);
        if (row < 0 && min_loose != min_strict)
            row = find_strong_edge_row_hist(hist_soft, start, end, min_loose);
    }
    return row;
}

static int find_step_bottom_edge(void)
{
    const int end = MT9V03X_H - 10;
    const int start = MT9V03X_H / STEP_BOTTOM_ROW_START_DIV;
    return find_edge_row_cascade(edge_histogram, edge_histogram_soft, start, end);
}

static int find_step_top_edge(int bottom_row)
{
    int end = bottom_row - 5;
    int start = MT9V03X_H / 3;
    if (start >= end)
        start = 10;
    if (start >= end)
        start = 5;
    return find_edge_row_cascade(edge_histogram, edge_histogram_soft, start, end);
}

static float get_filtered_distance(float new_distance)
{
    distance_history[history_index] = new_distance;
    height_history[history_index] = step_data.step_height_pix;
    history_index = (history_index + 1) % HISTORY_SIZE;
    
    float sum = 0;
    uint8 count = 0;
    for (int i = 0; i < HISTORY_SIZE; i++)
    {
        if (distance_history[i] > 0)
        {
            sum += distance_history[i];
            count++;
        }
    }
    
    if (count >= 3)
        return sum / count;
    else if (last_valid_distance > 0)
        return last_valid_distance;
    else
        return new_distance;
}

float calculate_step_distance(uint16 step_height_pix)
{
    if (step_height_pix == 0) return 0.0f;
    
    float distance = (STEP_HEIGHT_MM * FOCAL_LENGTH_MM) / (step_height_pix * PIXEL_SIZE_MM);
    
    distance = distance * cos(CAMERA_ANGLE_DEG * 3.14159f / 180.0f);
    
    distance = distance - CAMERA_HEIGHT_MM * sin(CAMERA_ANGLE_DEG * 3.14159f / 180.0f);
    
    if (distance < 0) distance = 0;
    
    return distance;
}

/* 处理一帧 MT9V034：更新边缘直方图、测距与 step_data；末尾写入 bottom_row_raw 供普通模式视觉跳跃等使用 */
uint8 step_detect(void)
{
    if (mt9v03x_finish_flag)
    {
        mt9v03x_finish_flag = 0;

        /* 供本帧末尾 VOFA 快照：失败路径也会带上能算出来的边缘与像素高 */
        int dbg_top = -1;
        int dbg_bot = -1;
        uint16 dbg_h = 0;
        float dbg_raw = 0.0f;

        compute_horizontal_edge_histogram();
        
        int step_bottom = find_step_bottom_edge();
        
        if (step_bottom > 0)
        {
            dbg_bot = step_bottom;
            int step_top = find_step_top_edge(step_bottom);
            
            if (step_top > 0 && step_bottom > step_top)
            {
                dbg_top = step_top;
                uint16 height_pix = (uint16)(step_bottom - step_top);
                dbg_h = height_pix;
                
                if (height_pix >= MIN_STEP_HEIGHT_PIX && height_pix <= MAX_STEP_HEIGHT_PIX)
                {
                    uint16 stable_h = push_median_height(height_pix);
                    float new_distance = calculate_step_distance(stable_h);
                    dbg_raw = new_distance;
                    
                    uint8 is_valid = 1;
                    
                    if (last_valid_distance > 0)
                    {
                        if (new_distance > last_valid_distance + 50.0f)
                        {
                            is_valid = 0;
                        }
                        
                        if (min_distance_recorded < 9999.0f && new_distance > min_distance_recorded + 100.0f)
                        {
                            is_valid = 0;
                        }
                    }
                    
                    if (is_valid)
                    {
                        if (new_distance < min_distance_recorded)
                        {
                            min_distance_recorded = new_distance;
                        }
                        
                        step_data.step_row = (uint16)step_bottom;
                        step_data.step_top_row = (uint16)step_top;
                        step_data.step_height_pix = stable_h;
                        step_data.distance_mm = get_filtered_distance(new_distance);
                        step_data.distance_cm = step_data.distance_mm / 10.0f;
                        
                        last_valid_height = (uint8)stable_h;
                        last_valid_distance = step_data.distance_mm;
                        
                        detect_counter++;
                        fail_counter = 0;
                        if (detect_counter > 15) detect_counter = 15;
                        
                        if (detect_counter >= 3)
                        {
                            step_data.detected = 1;
                        }
                        dbg_h = stable_h;
                        step_data.bottom_row_raw = (dbg_bot >= 0) ? (uint16)dbg_bot : 0u;
                        step_vofa_snapshot(dbg_top, dbg_bot, (float)dbg_h, dbg_raw, 1);
                        return 1;
                    }
                }
            }
        }
        
        fail_counter++;
        detect_counter--;
        if (detect_counter < 0) detect_counter = 0;
        
        if (fail_counter > 30 || detect_counter < 2)
        {
            step_data.detected = 0;
        }
        
        if (fail_counter > 50)
        {
            min_distance_recorded = 9999.0f;
            last_valid_distance = 0;
            last_valid_height = 0;
        }
        else if (last_valid_distance > 0)
        {
            step_data.step_height_pix = last_valid_height;
            step_data.distance_mm = last_valid_distance;
            step_data.distance_cm = last_valid_distance / 10.0f;
        }
        
        step_data.bottom_row_raw = (dbg_bot >= 0) ? (uint16)dbg_bot : 0u;
        step_vofa_snapshot(dbg_top, dbg_bot, (float)dbg_h, dbg_raw, 0);
        return 0;
    }
    return 0;
}

void step_reset_distance_tracking(void)
{
    min_distance_recorded = 9999.0f;
    last_valid_distance = 0;
    last_valid_height = 0;
    fail_counter = 0;
    height_median_reset();
}

/*---------------------------------------------------------------------------
 * 视觉自动跳跃状态机（见 step_detection.h 中 VISUAL_JUMP_* 宏说明）
 *---------------------------------------------------------------------------*/
#if !LEG_DEBUG_MODE && DUALCORE_UI_ON_CM7_1 && VISUAL_JUMP_AUTO_ENABLE

#if VISUAL_JUMP_POST_JUMP_COOLDOWN_5MS_TICKS > 0u
static volatile uint16 step_vjump_post_cooldown_ticks_remaining;
#endif

/* pit0_ch0 每 1ms 累计：bottom_row_raw > ARM_MIN 时递增，否则清零 */
static volatile uint16 step_vjump_arm_ms;
static volatile uint8 step_vjump_armed;

#if defined(CY_CORE_CM7_1)
/* VOFA CH5 影子：不受 jump_allowed/jump_active 门控；反映前置毫秒进度与 armed / 接近触发 */
static void step_vjump_vofa_shadow_tick(uint16 curr)
{
    const float arm_denom = (VISUAL_JUMP_ARM_TIME_MS > 0u) ? (float)VISUAL_JUMP_ARM_TIME_MS : 1.0f;
    uint16 ms = step_vjump_arm_ms;
    uint8 armed = step_vjump_armed;

    float out;
    if (armed != 0u)
        out = 1.0f + ((curr > VISUAL_JUMP_TRIGGER_THRESHOLD) ? 0.2f : 0.0f);
    else
    {
        out = (float)ms / arm_denom;
        if (out > 1.0f)
            out = 1.0f;
    }
    step_vofa_jump_dbg = out;
}
#endif /* CY_CORE_CM7_1 */

static void step_vjump_reset_arm_prereq(void)
{
    step_vjump_arm_ms = 0u;
    step_vjump_armed = 0u;
}

/* 上一拍 dualcore snapshot 的 jump_active（与 jump_flag 同步） */
static uint8 step_vjump_prev_jump_active;
static uint8 step_vjump_done_count;
/* 1：已达 VISUAL_JUMP_MAX_COUNT，不再自动发跳 */
static uint8 step_vjump_lockout;

void step_visual_jump_after_step(void)
{
#if defined(CY_CORE_CM7_1)
    step_vjump_vofa_shadow_tick(step_data.bottom_row_raw);
#endif
    dualcore_ctrl_to_ui_t dcj;
    dualcore_ctrl_to_ui_pull(&dcj);

    /* jump_active 上升沿：清空前置状态 */
    if (step_vjump_prev_jump_active == 0u && dcj.jump_active != 0u)
        step_vjump_reset_arm_prereq();

#if VISUAL_JUMP_POST_JUMP_COOLDOWN_5MS_TICKS > 0u
    if (step_vjump_prev_jump_active != 0u && dcj.jump_active == 0u)
        step_vjump_post_cooldown_ticks_remaining = (uint16)VISUAL_JUMP_POST_JUMP_COOLDOWN_5MS_TICKS;
#endif

    step_vjump_prev_jump_active = (dcj.jump_active != 0u) ? 1u : 0u;

    uint16 curr = step_data.bottom_row_raw;

    if (step_vjump_lockout != 0u)
        return;

    if (dcj.jump_allowed == 0u)
    {
        step_vjump_reset_arm_prereq();
        return;
    }

    if (dcj.jump_active != 0u)
    {
        step_vjump_reset_arm_prereq();
        return;
    }

#if VISUAL_JUMP_POST_JUMP_COOLDOWN_5MS_TICKS > 0u
    if (step_vjump_post_cooldown_ticks_remaining > 0u)
    {
        step_vjump_reset_arm_prereq();
        return;
    }
#endif

    if (step_vjump_armed != 0u && curr > VISUAL_JUMP_TRIGGER_THRESHOLD)
    {
        (void)dualcore_ui_cmd_push(DUALCORE_UI_CMD_JUMP, 0, 0.0f);
        step_vjump_done_count++;
        if (step_vjump_done_count >= VISUAL_JUMP_MAX_COUNT)
            step_vjump_lockout = 1u;
        step_vjump_reset_arm_prereq();
    }
}

#else

void step_visual_jump_after_step(void)
{
}

#endif /* !LEG_DEBUG_MODE && DUALCORE_UI_ON_CM7_1 && VISUAL_JUMP_AUTO_ENABLE */

#if defined(CY_CORE_CM7_1)
void step_visual_jump_post_jump_cooldown_on_cm7_1_1ms(void)
{
    /*
     * 不用 PIT_CH1：其与 CM7_0 的 TCPWM0->GRP[2].CNT[1]/leg_control 冲突（见 init.c CM7_1 注释）。
     * 每 1ms：视觉跳跃前置毫秒累计（bottom_row_raw）。
     * VISUAL_JUMP_POST_JUMP_COOLDOWN_5MS_TICKS：仍为「每 5ms 减一档」语义，内部 ÷5 后递减落地冷却。
     */
#if !LEG_DEBUG_MODE && DUALCORE_UI_ON_CM7_1 && VISUAL_JUMP_AUTO_ENABLE
    {
        uint16 bot = step_data.bottom_row_raw;
        if (bot > VISUAL_JUMP_ARM_MIN_THRESHOLD)
        {
            uint16 ms = (uint16)(step_vjump_arm_ms + 1u);
            if (ms > VISUAL_JUMP_ARM_TIME_MS)
                ms = VISUAL_JUMP_ARM_TIME_MS;
            step_vjump_arm_ms = ms;
            if (step_vjump_arm_ms >= VISUAL_JUMP_ARM_TIME_MS)
                step_vjump_armed = 1u;
        }
        else
        {
            step_vjump_arm_ms = 0u;
            step_vjump_armed = 0u;
        }
    }
#if VISUAL_JUMP_POST_JUMP_COOLDOWN_5MS_TICKS > 0u
    static uint8 ms_agg_for_vjump_cd;
    ms_agg_for_vjump_cd++;
    if (ms_agg_for_vjump_cd >= 5u)
    {
        ms_agg_for_vjump_cd = 0u;
        uint16 r = step_vjump_post_cooldown_ticks_remaining;
        if (r > 0u)
            step_vjump_post_cooldown_ticks_remaining = (uint16)(r - 1u);
    }
#endif
#endif /* !LEG_DEBUG_MODE && DUALCORE_UI_ON_CM7_1 && VISUAL_JUMP_AUTO_ENABLE */
}
#endif /* CY_CORE_CM7_1 */

void step_debug_send_to_vofa(void)
{
#if defined(CY_CORE_CM7_1) && !LEG_DEBUG_MODE && DUALCORE_UI_ON_CM7_1 && VISUAL_JUMP_AUTO_ENABLE
    float ch5_jump_dbg = step_vofa_jump_dbg;
#else
    float ch5_jump_dbg = 0.0f;
#endif
    /* JustFloat：6 个 float + 帧尾；通道含义见 step_detection.h 顶部注释 */
    SendDataStreamToVOFA(6,
                         step_vofa_top_row,
                         step_vofa_bottom_row,
                         step_vofa_height_pix,
                         step_vofa_dist_raw_mm,
                         step_vofa_dist_filt_mm,
                         ch5_jump_dbg);
}
