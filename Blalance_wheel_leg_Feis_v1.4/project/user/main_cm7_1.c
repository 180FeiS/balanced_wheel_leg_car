/*********************************************************************************************************************
* CYT4BB Opensourec Library 即（ CYT4BB 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是 CYT4BB 开源库的一部分
*
* CYT4BB 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
*
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
*
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
*
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
*
* 文件名称          main_cm7_1
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          IAR 9.40.1
* 适用平台          CYT4BB
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者                备注
* 2024-1-4       pudding            first version
********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "step_detection.h"
#include "vofa.h"
#include "image.h"
#include "Menu.h"
#include "dualcore_shared.h"
#include "single_bridge.h"

/*-------------------------------------------------------------------------------------------------------------------
 * CM7_1 主循环每圈最多发一帧 VOFA（JustFloat 经 wireless_uart）。
 * - 无线模块仅在本核初始化(all_init_cm7_1_ui→wireless_uart_init)，故上发集中在此处。
 * - STEP_DEBUG_USE_VOFA==1：台阶调试 6 路(step_debug_send_to_vofa)；==0：双核快照(vofa_send_nav_from_dualcore_snapshot)。
 * - 快照分组由 Nag_Vofa_Group 决定（菜单 n / 上位机切组），共 NAG_VOFA_GROUP_COUNT 组：
 *   0=IMU 姿态，1=速度目标/实测，2=GPS+惯导融合（见 vofa.h）。
 * - 导航快照来自 CM7_0 dualcore_ctrl_to_ui_publish；与本轮 step_detect 之间可能差一拍主循环，属正常。
 *-------------------------------------------------------------------------------------------------------------------*/
static void cm71_vofa_main_loop_tx_dispatch(void)
{
#if STEP_DEBUG_USE_VOFA
  step_debug_send_to_vofa();
#else
  vofa_send_nav_from_dualcore_snapshot();
#endif
}

// 打开新的工程或者工程移动了位置务必执行以下操作
// 第一步 关闭上面所有打开的文件
// 第二步 project->clean  等待下方进度条走完


// **************************** 代码区域 ****************************

int main(void)
{
    clock_init(SYSTEM_CLOCK_250M); 	// 时钟配置及系统初始化<务必保留>
    debug_info_init();                  // 调试串口信息初始化

    all_init_cm7_1_ui();
    /* LORA 默认 UART_1 与 wireless_uart 同口：后初始化覆盖 RX 回调；分路时请改 zf_device_lora3a22.h 宏 */
    remote_lora_init();

    /* 板载键扫描与 menu_key_capture_event 在 cm7_1_isr pit0_ch2(10ms) 中，此处只消费队列并刷新菜单/界面。 */
    while(true)
    {
        dualcore_ctrl_to_ui_t ctrl;
        selectMenu_Key();
        Menu_UpdateImageAeArm();
        dualcore_ctrl_to_ui_pull(&ctrl);
        ui_pull_ctrl_snapshot();
        /* 台阶检测：仅惯导 ENTER_STAIR 激活时跑；Image 菜单 AE 模式仍独占摄像头 */
        if (!MenuIsImageSectionPage() && (ctrl.stair_enter_active != 0u))
        {
            step_detect();
            step_visual_jump_after_step();
#if IMAGE_WHITE_BLOB_ENABLE
            image_vision_guidance_reset();
#endif
        }
#if IMAGE_WHITE_BLOB_ENABLE
        /*
         * 室外验证视觉状态机（台阶之后、惯导 bridge_detect_arm 之前）：
         * - WHITE_BLOB：上半区最大白连通域 → dualcore blob 通道
         * - MIDLINE：single_bridge 左右中寻线 → dualcore bridge 通道
         * 切换/回退逻辑见 image_vision_guidance_process_frame()。
         */
        else if (!MenuIsImageSectionPage())
        {
            if (mt9v03x_finish_flag != 0u)
            {
                mt9v03x_finish_flag = 0u;
                image_vision_guidance_process_frame(hd_threshold);
            }
        }
#endif
        /* 单边桥视觉：预区 arm 或桥上 detect；IMAGE_WHITE_BLOB_ENABLE=0 时启用 */
        else if (!MenuIsImageSectionPage() && (ctrl.bridge_detect_arm != 0u))
        {
            static uint32 bridge_frame_seq;
            single_bridge_track_t track;
            single_bridge_detect_t detect;

            if (mt9v03x_finish_flag != 0u)
            {
                mt9v03x_finish_flag = 0u;
                single_bridge_gray_diff_track(hd_threshold, &track);
                single_bridge_detect_update(&detect, ctrl.bridge_zone_active);
                bridge_frame_seq++;
                dualcore_bridge_vision_publish_detect(track.center_err,
                                                        track.track_valid,
                                                        detect.road_w_avg,
                                                        detect.pin_left,
                                                        detect.pin_right,
                                                        detect.enter_ready,
                                                        detect.exit_ready,
                                                        detect.enter_confirmed,
                                                        detect.exit_confirmed,
                                                        (uint8)detect.side,
                                                        bridge_frame_seq);
            }
        }
        else if (!MenuIsImageSectionPage())
        {
            single_bridge_detect_reset();
            dualcore_bridge_vision_publish_inactive();
#if IMAGE_WHITE_BLOB_ENABLE
            dualcore_white_blob_publish_inactive();
            image_vision_guidance_reset();
#endif
        }
        image_ae_session_poll();
        selectMenu();
        (void)image_ae_session_consume_done_and_save();

        remote_lora_update_from_driver_and_publish();

        static uint32 step_frame_seq;
        step_frame_seq++;
        dualcore_vision_publish_after_step(step_frame_seq);

        if (ctrl.menu_vofa_enable != 0u)
        {
            cm71_vofa_main_loop_tx_dispatch();
        }
    }
}

// **************************** 代码区域 ****************************
