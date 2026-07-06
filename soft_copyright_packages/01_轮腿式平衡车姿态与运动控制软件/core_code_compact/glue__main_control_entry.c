/*
 * 轮腿式平衡车姿态与运动控制软件 — CM7_0 主入口（软著 01 专用摘录）
 * 自 main_cm7_0.c 抽取控制核启动链，去除 GPS/融合/视觉/菜单等非本模块逻辑。
 */
#include "zf_common_headfile.h"
int main(void)
{
    clock_init(SYSTEM_CLOCK_250M);
    debug_init();
    all_init_cm7_0_control(); /* IMU/舵机/电机/PID/EKF/PIT */
    while (true)
    {
        /* 双核：消费 UI 下发的跳跃/调速/自旋等命令（经 dualcore 适配层） */
        dualcore_ui_cmd_consume_all();
        remote_lora_apply_validate_motor();
    }
}
