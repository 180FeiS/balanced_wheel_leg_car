/*
 * 机器视觉路径识别软件 — CM7_1 主循环摘录（软著 04）
 */
#include "zf_common_headfile.h"
#include "image.h"
#include "single_bridge.h"
#include "dualcore_shared.h"
int main(void)
{
    clock_init(SYSTEM_CLOCK_250M);
    debug_info_init();
    all_init_cm7_1_ui();
    while (true)
    {
        dualcore_ctrl_to_ui_t ctrl;
        dualcore_ctrl_to_ui_pull(&ctrl);
        if (ctrl.bridge_zone_active && mt9v03x_finish_flag)
        {
            mt9v03x_finish_flag = 0u;
            image_bridge_blob_process_frame(hd_threshold);
        }
        else if (ctrl.bridge_detect_arm && mt9v03x_finish_flag)
        {
            single_bridge_track_t track;
            single_bridge_detect_t detect;
            mt9v03x_finish_flag = 0u;
            single_bridge_gray_diff_track(hd_threshold, &track);
            single_bridge_detect_update(&detect, ctrl.bridge_zone_active);
            dualcore_bridge_vision_publish_detect(track.center_err, track.track_valid,
                detect.road_w_avg, detect.pin_left, detect.pin_right,
                detect.enter_ready, detect.exit_ready,
                detect.enter_confirmed, detect.exit_confirmed,
                (uint8)detect.side, 0u);
        }
        image_ae_session_poll();
    }
}
