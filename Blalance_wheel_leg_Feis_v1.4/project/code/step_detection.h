#ifndef CODE_STEP_DETECTION_H_
#define CODE_STEP_DETECTION_H_

#include "zf_common_headfile.h"

#define STEP_HEIGHT_MM     50      // 台阶高度 50mm
#define STEP_LENGTH_MM     500     // 台阶长度 500mm
#define STEP_WIDTH_MM      500     // 台阶宽度 500mm

#define CAMERA_HEIGHT_MM  250     // 摄像头安装高度（mm），根据实际调整
#define CAMERA_ANGLE_DEG  30      // 摄像头俯仰角度（度），根据实际调整
#define FOCAL_LENGTH_MM   2.3     // 摄像头焦距（mm），根据实际调整
#define PIXEL_SIZE_MM      0.006   // 像素尺寸（mm），根据实际调整

#define MIN_STEP_HEIGHT_PIX   5   // 最小台阶高度（像素）
#define MAX_STEP_HEIGHT_PIX   60 // 最大台阶高度（像素）

/* 边缘检测：CH2/CH3 抖动时优先调大 GRAD_THRESH 或 MIN_DIV；检不到台阶则反向调 */
#define STEP_EDGE_ROW_DELTA    2   /* 纵向梯度用 i±此行距，越小边缘定位越细 */
#define STEP_EDGE_GRAD_THRESH  50  /* 灰度差阈值，场地亮/反光强可调到 60~80 */
/* 软阈值直方图：白台面→蓝台阶时过曝/对比弱，强阈值下 edge 计数不够；须小于 GRAD_THRESH；0=不建软直方图，看不到二级台阶，减小这个 */
#define STEP_EDGE_GRAD_SOFT_THRESH 25
#define STEP_EDGE_MIN_DIV      3   /* 水平边缘最少计数 = MT9V03X_W / 本值，越小越严 */
#define STEP_HEIGHT_MED_WIN    5   /* 像素高中值滤波窗口（进入测距公式前） */

/* 下沿纵向 ROI：仅搜 [H/STEP_BOTTOM_ROW_START_DIV , H-10)。除数越大起始行越靠上，越利于“站在白台面看下一级”（边常出现在画面中上部）；除数过小易等同只搜下半幅导致 CH1 恒为 0 */
#define STEP_BOTTOM_ROW_START_DIV 2//2
/* 主 min_edges 未检出时，用 W/本值 再搜同一 ROI 一次；0=关闭 */
#ifndef STEP_BOTTOM_MIN_DIV_FALLBACK
#define STEP_BOTTOM_MIN_DIV_FALLBACK 4
#endif

typedef struct {
    uint8 detected;           // 是否检测到台阶
    uint16 step_row;          // 台阶底部边缘行坐标（通过测距校验后写入，失败时可能保持旧值）
    uint16 step_top_row;      // 台阶顶部边缘行坐标
    uint16 step_height_pix;   // 台阶高度（像素）
    float distance_mm;        // 台阶距离（mm）
    float distance_cm;        // 台阶距离（cm）
    /* 本帧原始台阶下沿行号：与 VOFA CH1（step_bottom_row）一致；find_step_bottom_edge()>0 时写入行号，否则为 0。
     * 供视觉自动跳跃（阈值判定，见 VISUAL_JUMP_*）等逻辑使用，不等同于滤波后的测距结果。仅在新摄像头帧被 step_detect 处理时更新。 */
    uint16 bottom_row_raw;
} step_info_t;

extern step_info_t step_data;

void step_detection_init(void);
uint8 step_detect(void);
float calculate_step_distance(uint16 step_height_pix);
void step_reset_distance_tracking(void);

/*
 * 台阶测距 VOFA 调试（JustFloat 6 通道，与 SendDataStreamToVOFA 顺序一致）：
 *
 *  CH0  step_top_row      台阶上沿在图像中的行号（约小越靠画面上方）；未检出时为 0
 *  CH1  step_bottom_row  台阶下沿行号；未检出时为 0（与 step_data.bottom_row_raw 同语义，每处理一帧更新）
 *  CH2  step_height_pix  用于测距的像素高（经 STEP_HEIGHT_MED_WIN 帧中值）；与距离公式直接相关
 *  CH3  dist_raw_mm       本帧由 calculate_step_distance 算出的原始距离（未做 10 帧均值）
 *  CH4  dist_filt_mm     与 step_data.distance_mm 一致（滤波后或失败时保持的上次有效值）
 *  CH5  jump_shadow       CM7_1 且启用视觉跳跃时：前置毫秒进度 ~[0,1]；armed 后置约 1.0；若 bottom_row_raw 大于当前跳主触发阈值，或 fallback_ready／丢边零保持进程中，可加约 0~0.2 至合计约 1.2；否则 ≤1.0
 *
 * 在 VOFA 里如何判断问题出在谁：
 *  1) CH2 波动大、CH3 跟着跳 → 边缘检测不稳或光照变化，优先改梯度阈值/ROI/对 step_height 做中值滤波
 *  2) CH2 很稳，但 CH3 与卷尺真值差一截 → step_detection.h 里 CAMERA_HEIGHT_MM / ANGLE / FOCAL / PIXEL_SIZE 标定不对
 *  3) CH3 稳，CH4 仍飘 → get_filtered_distance 均值把野值拉偏，可改为中值或收紧野值剔除
 *  4) CH0/CH1 突然跳到无关行、CH2 异常大/小 → 可能检到地布纹理或坡道边缘；白台看下一级 CH1 常为 0 时增大 STEP_BOTTOM_ROW_START_DIV 或 FALLBACK
 *  5) 蓝→白能检、白→蓝不能：多为过曝/弱对比而非梯度方向(abs 已对称)；略降 STEP_EDGE_GRAD_SOFT_THRESH 或 STEP_EDGE_GRAD_THRESH
 *
 * 使用：将 STEP_DEBUG_USE_VOFA 置 1，main_cm7_1 主循环只发台阶 6 路；置 0 则发 vofa_send_nav_from_dualcore_snapshot()。
 *       导航 VOFA 时 STEP_DEBUG_USE_VOFA=0，菜单 / 上位机「n」循环 nag_vofa_group：0=IMU 1=速度 2=融合 3=里程纠偏 4=自旋（见 vofa.h）。
 */
#ifndef STEP_DEBUG_USE_VOFA
#define STEP_DEBUG_USE_VOFA 0 /* 1=主循环发台阶 6 路；0=保持原导航 VOFA */
#endif

/*---------------------------------------------------------------------------
 * 视觉自动跳跃（bottom_row_raw + 前置毫秒 + 主触发阈值 / 丢边保底；经双核发往 CM7_0）
 *
 * 仅在 !LEG_DEBUG_MODE && DUALCORE_UI_ON_CM7_1 && VISUAL_JUMP_AUTO_ENABLE 时
 * step_visual_jump_after_step() 内有实际逻辑；否则为空实现（含 CM7_0 工程链接）。
 *
 * 【惯导门控】CM7_1 主循环仅在 dualcore ctrl.stair_enter_active==1（ENTER_STAIR 接管）
 * 时调用 step_detect() 与本函数；EXIT_STAIR 或元素结束后 stair_enter_active 下降沿会复位视觉跳计数。
 *
 * 判定量：step_data.bottom_row_raw（与 VOFA CH1 同语义）。
 *
 * 【前置】在 pit0_ch0 每 1ms ISR（step_visual_jump_post_jump_cooldown_on_cm7_1_1ms）中：
 *   仅当 bottom_row_raw > VISUAL_JUMP_ARM_MIN_THRESHOLD 时递增「强项 streak」毫秒；
 *   未维持强项时每拍清零该 streak（step_vjump_arm_ms），不因 bot==0 或弱值而擅自清除 step_vjump_armed，
 *   否则会使得「丢边后连续 bot==0 再保持 FALLBACK_ZERO_HOLD_MS」永远无法成立。
 *   streak 达 VISUAL_JUMP_ARM_TIME_MS 后置 step_vjump_armed（前置完成），允许主循环判跳。
 *
 * 【主触发】前置完成后在主循环：（阈值按已成功跳跃次数 VISUAL_JUMP_TRIGGER_THRESHOLD / _2ND / _3RD）
 *   bottom_row_raw > 当前跳的触发阈值 且 armed → dualcore_ui_cmd_push(JUMP)；无需先变 0。
 *
 * 【丢边保底】不依赖大于主触发阈值：主循环若 armed、本帧 curr==0、上一帧 prev > ARM_MIN，
 *   则置 fallback_pending；ISR 在 pending 期间须 bottom_row_raw 连续 ==0 累计满
 *   VISUAL_JUMP_FALLBACK_ZERO_HOLD_MS 后置 fallback_ready，主循环再发跳。
 *   余项下沿再次出现（bot>ARM_MIN）时 ISR 作废 pending，防止假丢边误跳。
 *
 * 触发成功或互锁发生时清零前置／丢边，需重新攒前置才可再次判定。
 *
 * 与跳跃互锁：dualcore_ctrl_to_ui.jump_active==1（CM7_0 的 jump_flag）时不发跳；
 * jump_active 上升沿时清空内部前置状态；jump_allowed==0、落地冷却内同样清空前置状态。
 * jump_active 落地后可选 VISUAL_JUMP_POST_JUMP_COOLDOWN_5MS_TICKS 内仍不发跳（CM7_1：
 * 同一 1ms ISR 内先做前置毫秒累计，再每 5ms 递减冷却一档；时长仍为 N×5ms，
 * 不占 PIT_CH1，避免与 CM7_0 leg_control 争用 TCPWM CNT[1]）。
 * 发往 CM7_0 的台阶快照见 dualcore_vision_publish_after_step：jump_active 时 step 故意置为全 0 无效帧。
 *---------------------------------------------------------------------------*/
#ifndef VISUAL_JUMP_AUTO_ENABLE
#define VISUAL_JUMP_AUTO_ENABLE 1u /* 0=关闭自动跳跃 */
#endif
#ifndef VISUAL_JUMP_TRIGGER_THRESHOLD
#define VISUAL_JUMP_TRIGGER_THRESHOLD 106u /* 第 1 跳：前置完成后 bottom_row_raw 大于本值则发跳 */
#endif
#ifndef VISUAL_JUMP_TRIGGER_THRESHOLD_2ND
#define VISUAL_JUMP_TRIGGER_THRESHOLD_2ND 106u /* 第 2 跳专用触发阈值 */
#endif
#ifndef VISUAL_JUMP_TRIGGER_THRESHOLD_3RD
#define VISUAL_JUMP_TRIGGER_THRESHOLD_3RD 106u /* 第 3 跳及以后（与默认 106 一致，可按场布覆盖）*/
#endif
#ifndef VISUAL_JUMP_ARM_MIN_THRESHOLD
#define VISUAL_JUMP_ARM_MIN_THRESHOLD 50u /* ISR：用于「强项 streak」阈值；主循环 latch 保底要求上一帧 prev_b 大于本值 */
#endif
#ifndef VISUAL_JUMP_ARM_TIME_MS
#define VISUAL_JUMP_ARM_TIME_MS 40u /* ISR：bottom>ARM_MIN 连续本毫秒后置 armed（前置完成） */
#endif
#ifndef VISUAL_JUMP_FALLBACK_ZERO_HOLD_MS
#define VISUAL_JUMP_FALLBACK_ZERO_HOLD_MS 10u /* ISR：fallback_pending 时须 bot==0 连续本毫秒才置 fallback_ready（与主阈值无关） */
#endif
#ifndef VISUAL_JUMP_MAX_COUNT
#define VISUAL_JUMP_MAX_COUNT 3u /* 成功投递跳跃命令次数上限，满后 lockout 直至复位 */
#endif
/* jump_active 从 1 变 0 后，再经 N 个「等效 5ms」节拍内禁止下沿触发（递减由 CM7_1 pit0_ch0×5 软件分频驱动）；时长 = N×5ms；0=关闭。
 * 若在工程中曾用 VISUAL_JUMP_POST_JUMP_COOLDOWN_LOOPS，可继续定义该宏，未定义 VISUAL_JUMP_POST_JUMP_COOLDOWN_5MS_TICKS 时会映射过来。 */
#ifndef VISUAL_JUMP_POST_JUMP_COOLDOWN_5MS_TICKS
#ifdef VISUAL_JUMP_POST_JUMP_COOLDOWN_LOOPS
#define VISUAL_JUMP_POST_JUMP_COOLDOWN_5MS_TICKS VISUAL_JUMP_POST_JUMP_COOLDOWN_LOOPS
#else
#define VISUAL_JUMP_POST_JUMP_COOLDOWN_5MS_TICKS 0u
#endif
#endif

/* CM7_1：在 step_detect() 之后调用；门控后主触发或 fallback_ready → dualcore_ui_cmd_push(JUMP) */
void step_visual_jump_after_step(void);
#if defined(CY_CORE_CM7_1)
/* CM7_1：pit0_ch0_isr（1ms）：强项 streak→armed；pending 下 bot==0 零保持；勿在 bot==0 无脑清 armed。另含落地冷却÷5（若配置了 POST_JUMP）。勿用 PIT_CH1（与 CM7_0 leg_control 同源）。 */
void step_visual_jump_post_jump_cooldown_on_cm7_1_1ms(void);
#endif /* CY_CORE_CM7_1 */

void step_debug_send_to_vofa(void);

/** 菜单调试：对当前 mt9v03x_image 做下沿检测，不改动滤波/计数/finish_flag；未检出返回 -1 */
int step_debug_find_bottom_row(void);

#if defined(CY_CORE_CM7_1)
/** 在压缩灰度预览上叠加下沿水平线（bottom_row<0 时不绘制） */
void step_debug_draw_bottom_overlay(int disp_x, int disp_y, int disp_w, int disp_h, int bottom_row);
/** 台阶 Image 调试页：压缩显示 + 下沿红线叠加，并更新 step_data.bottom_row_raw */
void step_debug_show(int disp_x, int disp_y);
#endif /* CY_CORE_CM7_1 */

#endif /* CODE_STEP_DETECTION_H_ */
