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

/*-------------------------------------------------------------------------------------------------------------------
 * CM7_1 主循环每圈最多发一帧 VOFA（JustFloat 经 wireless_uart）。
 * - 无线模块仅在本核初始化(all_init_cm7_1_ui→wireless_uart_init)，故上发集中在此处。
 * - STEP_DEBUG_USE_VOFA==1：台阶调试 6 路(step_debug_send_to_vofa)；==0：双核快照(vofa_send_nav_from_dualcore_snapshot)。
 * - 快照分组由 Nag_Vofa_Group 决定（菜单 n / 上位机切组），共 NAG_VOFA_GROUP_COUNT 组：
 *   0~5 惯导等，6/7 GPS，8 转向，9 元素调速（speed_target_effective/car_speed/Run_index/Event_Active_Type）。
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
        selectMenu_Key();
        Menu_UpdateImageAeArm();
        /* Debug→Image（2.1*）只做曝光/阈值调试，不跑台阶检测，避免与 AE 抢 mt9v03x_finish_flag */
        if (!MenuIsImageSectionPage())
        {
            step_detect();
        }
        image_ae_session_poll();
        selectMenu();
        (void)image_ae_session_consume_done_and_save();
        ui_pull_ctrl_snapshot();

        remote_lora_update_from_driver_and_publish();

        //step_visual_jump_after_step(); /* 视觉自动跳跃：见 step_detection.c / VISUAL_JUMP_* */
        static uint32 step_frame_seq;
        step_frame_seq++;
        dualcore_vision_publish_after_step(step_frame_seq);

        //cm71_vofa_main_loop_tx_dispatch(); /* 详见 static 函数注释 */
    }
}

// **************************** 代码区域 ****************************
