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
* （项目）        ——                双核：本文件为 CM7_1（UI/感知）主循环，见下方「双核分工说明」
********************************************************************************************************************/

/*********************************************************************************************************************
 * 【双核分工说明 — 哪些任务在 CM7_1】
 *
 * CM7_0（控制核）：IMU/EKF、导航 Nag_System、PID、电机 PWM、跳跃状态机 jump_control、拨码 dip_switch_motor_sync
 *   等与实时控制相关的逻辑；通过 dualcore_ctrl_to_ui_publish() 把状态快照写给 CM7_1 显示。
 *
 * CM7_1（本核）：从原单核 main 软任务 / 外设中迁出、且与控制核解耦的部分，主要包括：
 *   1) 摄像头 + IPS200 + 台阶检测：camera_init_ips200 / step_detect / step_data（见 init.c all_init_cm7_1_ui）
 *   2) 菜单与五向键：menu_key_capture_event、selectMenu_Key、selectMenu（命令经 dualcore_ui_cmd_push 到 0 核执行）
 *   3) 界面刷新数据：ui_pull_ctrl_snapshot() 从共享区拉取控制核快照再画 GUI（UI.c）
 *   4) 无线 VOFA：wireless_uart_init（init），SendDataStreamToVOFA / ReadDataFromPc（vofa.c），UART1 在 cm7_1_isr
 *   5) 视觉结果下发：dualcore_vision_publish_after_step() 把 step_data 写入 g_dualcore_blob.vision 供 0 核拉取
 *   6) （可选）遥控：remote_control_init/task，与 UART1 共用，与 VOFA 二选一时注意 REMOTE_CONTROL_ENABLE
 *   7) （可选）视觉自动跳跃：visual_jump_after_step_cm71() 通过 DUALCORE_UI_CMD_JUMP 通知 0 核置 jump_flag
 *
 * 共享与协议：project/code/dualcore_shared.h、dualcore_shared.c（地址 DUALCORE_SHARED_PHYS_ADDR）。
 ********************************************************************************************************************/

#include "zf_common_headfile.h"


#if !LEG_DEBUG_MODE
/* 视觉自动跳跃：与原先 CM7_0 main 软任务里逻辑一致，迁到 1 核后改经命令队列触发跳跃（非直接写 jump_flag） */
#ifndef VISUAL_JUMP_AUTO_ENABLE
#define VISUAL_JUMP_AUTO_ENABLE 1u       /* 1：启用；工程预处理里可改为 0 关闭 */
#endif
#ifndef VISUAL_JUMP_ZERO_CONFIRM_FRAMES
#define VISUAL_JUMP_ZERO_CONFIRM_FRAMES 2u  /* 底行由有边沿变 0 后，需连续为 0 的 step_detect 次数再触发 */
#endif
#ifndef VISUAL_JUMP_MAX_COUNT
#define VISUAL_JUMP_MAX_COUNT 3u         /* 成功下发跳跃命令的次数上限，达到后 visual_jump_lockout 禁止直到复位 */
#endif

#if VISUAL_JUMP_AUTO_ENABLE
static uint16 visual_jump_prev_bottom_raw;   /* 上一帧 step_data.bottom_row_raw，用于边沿检测 */
static uint8 visual_jump_in_zero_confirm;   /* 1：已看到高→0，正在数连续为 0 的帧数 */
static uint8 visual_jump_zero_confirm_cnt; /* 当前连续为 0 的计数 */
static uint8 visual_jump_done_count;       /* 已成功 push JUMP 命令的次数 */
static uint8 visual_jump_lockout;          /* 1：已达 VISUAL_JUMP_MAX_COUNT，不再触发 */

/*-------------------------------------------------------------------------------------------------------------------
 * 函数简介     每帧台阶检测后调用：根据底行消失沿 + 连续 0 确认，向 CM7_0 发送跳跃命令
 * 参数说明     无（读 step_data、读共享快照 dcj）
 * 备注         门控：dcj.jump_allowed（0 核 jump_is_allowed）、dcj.jump_active（0 核 jump_flag 进行中）；
 *              真正置位 jump_flag 在 CM7_0 的 dualcore_ui_cmd_consume_all → DUALCORE_UI_CMD_JUMP
-------------------------------------------------------------------------------------------------------------------*/
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

// **************************** CM7_1 主循环（UI / 感知核）****************************

/*-------------------------------------------------------------------------------------------------------------------
 * 函数简介     CM7_1 入口：只做本核外设与用户交互；不向 CM7_0 全局直接写控制量（走 dualcore_ui_cmd_push）
 * 备注         all_init_cm7_1_ui 定义见 init.c：摄像头/屏/菜单/按键/无线，无 IMU/电机/PIT 控制定时器
-------------------------------------------------------------------------------------------------------------------*/
int main(void)
{
    clock_init(SYSTEM_CLOCK_250M); 	// 时钟配置及系统初始化<务必保留>
    debug_info_init();                  // 本核调试串口（与 CM7_0 的 debug_init 独立）

    /* 摄像头 IPS200、台阶初始化、无线 VOFA、按键、MenuInit；pit_flag=0 不在本核开控制用 PIT */
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
        dualcore_vision_publish_after_step(step_frame_seq); /* 拷贝 step_data → g_dualcore_blob.vision + 缓存同步 */

#if !LEG_DEBUG_MODE && VISUAL_JUMP_AUTO_ENABLE
        //visual_jump_after_step_cm71();
#endif
    }
}

// **************************** 代码区域 ****************************
