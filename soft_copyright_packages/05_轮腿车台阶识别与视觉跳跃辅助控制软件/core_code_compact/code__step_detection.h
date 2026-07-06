#ifndef CODE_STEP_DETECTION_H_
#define CODE_STEP_DETECTION_H_
#include "zf_common_headfile.h"
#define STEP_HEIGHT_MM     50
#define STEP_LENGTH_MM     500
#define STEP_WIDTH_MM      500
#define CAMERA_HEIGHT_MM  250
#define CAMERA_ANGLE_DEG  30
#define FOCAL_LENGTH_MM   2.3
#define PIXEL_SIZE_MM      0.006
#define MIN_STEP_HEIGHT_PIX   5
#define MAX_STEP_HEIGHT_PIX   60
#define STEP_EDGE_ROW_DELTA    2
#define STEP_EDGE_GRAD_THRESH  50
#define STEP_EDGE_GRAD_SOFT_THRESH 25
#define STEP_EDGE_MIN_DIV      3
#define STEP_HEIGHT_MED_WIN    5
#define STEP_BOTTOM_ROW_START_DIV 2
#ifndef STEP_BOTTOM_MIN_DIV_FALLBACK
#define STEP_BOTTOM_MIN_DIV_FALLBACK 4
#endif
typedef struct {
    uint8 detected;
    uint16 step_row;
    uint16 step_top_row;
    uint16 step_height_pix;
    float distance_mm;
    float distance_cm;
    uint16 bottom_row_raw;
} step_info_t;
extern step_info_t step_data;
void step_detection_init(void);
uint8 step_detect(void);
float calculate_step_distance(uint16 step_height_pix);
void step_reset_distance_tracking(void);
#ifndef STEP_DEBUG_USE_VOFA
#define STEP_DEBUG_USE_VOFA 0
#endif
#ifndef VISUAL_JUMP_AUTO_ENABLE
#define VISUAL_JUMP_AUTO_ENABLE 1u
#endif
#ifndef VISUAL_JUMP_TRIGGER_THRESHOLD
#define VISUAL_JUMP_TRIGGER_THRESHOLD 106u
#endif
#ifndef VISUAL_JUMP_TRIGGER_THRESHOLD_2ND
#define VISUAL_JUMP_TRIGGER_THRESHOLD_2ND 106u
#endif
#ifndef VISUAL_JUMP_TRIGGER_THRESHOLD_3RD
#define VISUAL_JUMP_TRIGGER_THRESHOLD_3RD 106u
#endif
#ifndef VISUAL_JUMP_ARM_MIN_THRESHOLD
#define VISUAL_JUMP_ARM_MIN_THRESHOLD 50u
#endif
#ifndef VISUAL_JUMP_ARM_TIME_MS
#define VISUAL_JUMP_ARM_TIME_MS 40u
#endif
#ifndef VISUAL_JUMP_FALLBACK_ZERO_HOLD_MS
#define VISUAL_JUMP_FALLBACK_ZERO_HOLD_MS 10u
#endif
#ifndef VISUAL_JUMP_MAX_COUNT
#define VISUAL_JUMP_MAX_COUNT 3u
#endif
#ifndef VISUAL_JUMP_POST_JUMP_COOLDOWN_5MS_TICKS
#ifdef VISUAL_JUMP_POST_JUMP_COOLDOWN_LOOPS
#define VISUAL_JUMP_POST_JUMP_COOLDOWN_5MS_TICKS VISUAL_JUMP_POST_JUMP_COOLDOWN_LOOPS
#else
#define VISUAL_JUMP_POST_JUMP_COOLDOWN_5MS_TICKS 0u
#endif
#endif
void step_visual_jump_after_step(void);
#if defined(CY_CORE_CM7_1)
void step_visual_jump_post_jump_cooldown_on_cm7_1_1ms(void);
#endif
void step_debug_send_to_vofa(void);
#endif
