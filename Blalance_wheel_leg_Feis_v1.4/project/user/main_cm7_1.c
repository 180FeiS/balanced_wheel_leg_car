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

#if !LEG_DEBUG_MODE
#ifndef VISUAL_JUMP_AUTO_ENABLE
#define VISUAL_JUMP_AUTO_ENABLE 1u
#endif
#ifndef VISUAL_JUMP_ZERO_CONFIRM_FRAMES
#define VISUAL_JUMP_ZERO_CONFIRM_FRAMES 2u
#endif
#ifndef VISUAL_JUMP_MAX_COUNT
#define VISUAL_JUMP_MAX_COUNT 3u
#endif

#if VISUAL_JUMP_AUTO_ENABLE
static uint16 visual_jump_prev_bottom_raw;
static uint8 visual_jump_in_zero_confirm;
static uint8 visual_jump_zero_confirm_cnt;
static uint8 visual_jump_done_count;
static uint8 visual_jump_lockout;

static void visual_jump_after_step_cm71(void)
{
  uint16 curr = step_data.bottom_row_raw;
  dualcore_ctrl_to_ui_t dcj;
  dualcore_ctrl_to_ui_pull(&dcj);

  if (visual_jump_lockout != 0u)
  {
    visual_jump_prev_bottom_raw = curr;
    return;
  }

  if (dcj.jump_allowed == 0u)
  {
    visual_jump_in_zero_confirm = 0u;
    visual_jump_zero_confirm_cnt = 0u;
    visual_jump_prev_bottom_raw = curr;
    return;
  }

  if (dcj.jump_active != 0u)
  {
    visual_jump_in_zero_confirm = 0u;
    visual_jump_zero_confirm_cnt = 0u;
    visual_jump_prev_bottom_raw = curr;
    return;
  }

  if (visual_jump_in_zero_confirm != 0u)
  {
    if (curr != 0u)
    {
      visual_jump_in_zero_confirm = 0u;
      visual_jump_zero_confirm_cnt = 0u;
    }
    else
    {
      visual_jump_zero_confirm_cnt++;
      if (visual_jump_zero_confirm_cnt >= VISUAL_JUMP_ZERO_CONFIRM_FRAMES)
      {
        (void)dualcore_ui_cmd_push(DUALCORE_UI_CMD_JUMP, 0, 0.0f);
        visual_jump_done_count++;
        if (visual_jump_done_count >= VISUAL_JUMP_MAX_COUNT)
          visual_jump_lockout = 1u;
        visual_jump_in_zero_confirm = 0u;
        visual_jump_zero_confirm_cnt = 0u;
      }
    }
  }
  else if (visual_jump_prev_bottom_raw > 0u && curr == 0u)
  {
    visual_jump_in_zero_confirm = 1u;
    visual_jump_zero_confirm_cnt = 1u;
    if (VISUAL_JUMP_ZERO_CONFIRM_FRAMES <= 1u)
    {
      (void)dualcore_ui_cmd_push(DUALCORE_UI_CMD_JUMP, 0, 0.0f);
      visual_jump_done_count++;
      if (visual_jump_done_count >= VISUAL_JUMP_MAX_COUNT)
        visual_jump_lockout = 1u;
      visual_jump_in_zero_confirm = 0u;
      visual_jump_zero_confirm_cnt = 0u;
    }
  }

  visual_jump_prev_bottom_raw = curr;
}
#endif
#endif /* !LEG_DEBUG_MODE */

// 打开新的工程或者工程移动了位置务必执行以下操作
// 第一步 关闭上面所有打开的文件
// 第二步 project->clean  等待下方进度条走完

// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设

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
        selectMenu();
        ui_pull_ctrl_snapshot();

        remote_lora_update_from_driver_and_publish();

        step_detect();
        static uint32 step_frame_seq;
        step_frame_seq++;
        dualcore_vision_publish_after_step(step_frame_seq);

        /* VOFA：无线模块仅在本核初始化(all_init_cm7_1_ui→wireless_uart_init)，故上发只在此处。
         * STEP_DEBUG_USE_VOFA=1：发台阶 6 路(step_debug_send_to_vofa)，=0：发惯导快照(vofa_send_nav_from_dualcore_snapshot)。
         * 快照来自 CM7_0 的 dualcore_ctrl_to_ui_publish，与本轮 step 之间可能差一拍主循环，属正常。 */
#if STEP_DEBUG_USE_VOFA
        step_debug_send_to_vofa();
#else
        vofa_send_nav_from_dualcore_snapshot();
#endif

#if !LEG_DEBUG_MODE && VISUAL_JUMP_AUTO_ENABLE
        //visual_jump_after_step_cm71();
#endif
    }
}

// **************************** 代码区域 ****************************
