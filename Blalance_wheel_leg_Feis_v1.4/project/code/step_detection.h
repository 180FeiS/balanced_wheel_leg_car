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
/* 软阈值直方图：白台面→蓝台阶时过曝/对比弱，强阈值下 edge 计数不够；须小于 GRAD_THRESH；0=不建软直方图 */
#define STEP_EDGE_GRAD_SOFT_THRESH 32
#define STEP_EDGE_MIN_DIV      3   /* 水平边缘最少计数 = MT9V03X_W / 本值，越大越严 */
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
     * 供普通模式“下沿消失触发跳跃”等逻辑使用，不等同于滤波后的测距结果。仅在新摄像头帧被 step_detect 处理时更新。 */
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
 *  CH5  flags             整数化信息：frame_ok*10 + detected；frame_ok=本帧是否通过有效测距更新(0/1)，detected=累计确认标志
 *
 * 在 VOFA 里如何判断问题出在谁：
 *  1) CH2 波动大、CH3 跟着跳 → 边缘检测不稳或光照变化，优先改梯度阈值/ROI/对 step_height 做中值滤波
 *  2) CH2 很稳，但 CH3 与卷尺真值差一截 → step_detection.h 里 CAMERA_HEIGHT_MM / ANGLE / FOCAL / PIXEL_SIZE 标定不对
 *  3) CH3 稳，CH4 仍飘 → get_filtered_distance 均值把野值拉偏，可改为中值或收紧野值剔除
 *  4) CH0/CH1 突然跳到无关行、CH2 异常大/小 → 可能检到地布纹理或坡道边缘；白台看下一级 CH1 常为 0 时增大 STEP_BOTTOM_ROW_START_DIV 或 FALLBACK
 *  5) 蓝→白能检、白→蓝不能：多为过曝/弱对比而非梯度方向(abs 已对称)；略降 STEP_EDGE_GRAD_SOFT_THRESH 或 STEP_EDGE_GRAD_THRESH
 *
 * 使用：将 STEP_DEBUG_USE_VOFA 置 1，main_cm7_1.c 中 cm71_vofa_main_loop_tx_dispatch 只发本组 6 路（不再发导航快照帧），避免两帧混叠。
 */
#ifndef STEP_DEBUG_USE_VOFA
#define STEP_DEBUG_USE_VOFA 1 /* 1=主循环发台阶 6 路；0=保持原导航 VOFA */
#endif

/*---------------------------------------------------------------------------
 * 视觉自动跳跃（台阶下沿消失触发，经双核命令发往 CM7_0）
 *
 * 仅在 !LEG_DEBUG_MODE && DUALCORE_UI_ON_CM7_1 && VISUAL_JUMP_AUTO_ENABLE 时
 * step_visual_jump_after_step() 内有实际逻辑；否则为空实现（含 CM7_0 工程链接）。
 * 触发量：step_data.bottom_row_raw（与 VOFA CH1 同语义）；条件为上一拍 >0 且当前为 0，
 * 再连续保持 0 共 VISUAL_JUMP_ZERO_CONFIRM_FRAMES 次（下降沿当帧计第 1 次）。
 *
 * 与跳跃互锁：dualcore_ctrl_to_ui.jump_active==1（CM7_0 的 jump_flag）时不发跳、不起动沿确认；
 * 上升沿时清空内部沿状态；jump_active 落地后可选 VISUAL_JUMP_POST_JUMP_COOLDOWN_5MS_TICKS 内仍不发跳（由 pit0_ch1 每 5ms 递减，时长 = N×5ms）。
 * 发往 CM7_0 的台阶快照见 dualcore_vision_publish_after_step：jump_active 时 step 故意置为全 0 无效帧。
 *---------------------------------------------------------------------------*/
#ifndef VISUAL_JUMP_AUTO_ENABLE
#define VISUAL_JUMP_AUTO_ENABLE 1u /* 0=关闭自动跳跃 */
#endif
#ifndef VISUAL_JUMP_ZERO_CONFIRM_FRAMES
#define VISUAL_JUMP_ZERO_CONFIRM_FRAMES 5u /* 连续为 0 的确认次数，抑制单帧丢边 2 */
#endif
#ifndef VISUAL_JUMP_MAX_COUNT
#define VISUAL_JUMP_MAX_COUNT 3u /* 成功投递跳跃命令次数上限，满后 lockout 直至复位 */
#endif
/* jump_active 从 1 变 0 后，再经 N 个 5ms 节拍（pit0_ch1）内禁止下沿触发；冷却时长 = N×5ms；0=关闭。
 * 若在工程中曾用 VISUAL_JUMP_POST_JUMP_COOLDOWN_LOOPS，可继续定义该宏，未定义 VISUAL_JUMP_POST_JUMP_COOLDOWN_5MS_TICKS 时会映射过来。 */
#ifndef VISUAL_JUMP_POST_JUMP_COOLDOWN_5MS_TICKS
#ifdef VISUAL_JUMP_POST_JUMP_COOLDOWN_LOOPS
#define VISUAL_JUMP_POST_JUMP_COOLDOWN_5MS_TICKS VISUAL_JUMP_POST_JUMP_COOLDOWN_LOOPS
#else
#define VISUAL_JUMP_POST_JUMP_COOLDOWN_5MS_TICKS 0u
#endif
#endif

/* CM7_1：在 step_detect() 之后调用；读 dualcore_ctrl_to_ui 的 jump_allowed / jump_active，满足时 dualcore_ui_cmd_push(JUMP) */
void step_visual_jump_after_step(void);
/* CM7_1：pit0_ch1_isr（5ms）内调用；递减落地冷却计数，与 VISUAL_JUMP_POST_JUMP_COOLDOWN_5MS_TICKS 配套 */
void step_visual_jump_pit_ch1_5ms_tick(void);

void step_debug_send_to_vofa(void);

#endif /* CODE_STEP_DETECTION_H_ */
