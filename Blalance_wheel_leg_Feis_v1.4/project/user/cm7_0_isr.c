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
 * 文件名称          cm7_0_isr
 * 公司名称          成都逐飞科技有限公司
 * 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
 * 开发环境          IAR 9.40.1
 * 适用平台          CYT4BB
 * 店铺链接          https://seekfree.taobao.com/
 *
 * 修改记录
 * 日期              作者                备注
 * 2024-1-9      pudding            first version
 * 2024-5-14     pudding            新增12个pit周期中断 增加部分注释说明
 ********************************************************************************************************************/

#include "zf_common_headfile.h"

vuint8 task_5ms_nav_pending = 0;
vuint8 task_10ms_menu_key_pending = 0;
vuint8 task_20ms_menu_pending = 0;
vuint8 task_10ms_step_pending = 0;

/* ISR 侧统一用这个函数累加软任务计数。
 * 以后如果新增软任务，优先复用这里，不要在中断里直接写复杂逻辑。
 * 使用计数而不是单 bit 标志，能避免主循环偶尔来不及处理时直接丢任务。
 */
static void task_pending_push(vuint8 *task_pending)
{
    if (*task_pending < 0xFF)
    {
        (*task_pending)++;
    }
}

// **************************** PIT中断函数 ****************************
void pit0_ch0_isr() // 定时器通道 0 周期中断服务函数
{
    pit_isr_flag_clear(PIT_CH0); // 1ms
    EKF_UpData();
    EKF_V_UPData();
    /* 导航当前固定在 1ms 中断里运行：
     * 1. navigation.h 中 Nag_Sample_Dt 必须与这里保持一致；
     * 2. 不要再在 main 或 5ms 软任务里重复调用 Nag_System()，否则里程会被重复积分；
     * 3. 若以后真的迁到 5ms 软任务，必须同时修改 Nag_Sample_Dt 和实车标定系数。
     */
    Nag_System();
    /* 绝对航向请求统一在这里消费：
     * 1. 非自旋时：在 pid_ctrl_Run() 前执行一次 steer_set_target_yaw()；
     * 2. 自旋时：只保留最新目标并延迟，避免请求式转向打断 spin_task_start()；
     * 3. 元素锁航向模块也在这里前置补登请求，但仍复用同一套 pending 消费链路。
     */
     if(Nag_Debug_Speed_Bypass_Enable)
     {
        steer_request_target_yaw(steer_yaw_request_deg);
     }
    if (Nag_HeadingHold_ShouldRequest())
    {
        steer_request_target_yaw(Nag_HeadingHold_GetTargetYaw());
    }
    if (steer_yaw_request_pending)
    {
        if (spin_enable)
        {
            steer_yaw_delayed_by_spin = 1;
        }
        else
        {
            steer_set_target_yaw(steer_yaw_request_deg);
            steer_yaw_request_pending = 0;
            steer_yaw_delayed_by_spin = 0;
        }
    }
    pid_ctrl_Run();
    Left_Motor_Pwm = -motor_value.receive_left_speed_data;
    Right_Motor_Pwm = motor_value.receive_right_speed_data;
}

void pit0_ch1_isr() // 定时器通道 1 周期中断服务函数
{
    pit_isr_flag_clear(PIT_CH1);// 5ms 横滚/俯仰腿控制（与俯仰角5ms同频）
    leg_control();
    /* 导航函数可能走到 Flash/慢路径，因此这里只挂任务，真正执行放到主循环。 */
    task_pending_push(&task_5ms_nav_pending);
}

void pit0_ch2_isr() // 定时器通道 2 周期中断服务函数
{
    pit_isr_flag_clear(PIT_CH2); // 10ms
    /* key_init(10) 要求约每 10ms 调用一次 key_scanner；若仅随主循环调用，主循环慢时短按无法累计到 KEY_MAX_SHOCK_PERIOD。 */
    key_scanner();
    /* 短按事件先在这里缓存，避免主循环忙时被下一次 key_scanner() 覆盖掉；真正的菜单切换和显示刷新仍放主循环。 */
    menu_key_capture_event();
    task_pending_push(&task_10ms_menu_key_pending);
}

void pit0_ch10_isr() // 定时器通道 10 周期中断服务函数
{
    pit_isr_flag_clear(PIT_CH10); // 20ms 跳跃/菜单（leg_control已移至5ms）
    jump_control();
    /* 跳跃控制保留在 ISR，菜单解析迁到主循环。 */
    task_pending_push(&task_20ms_menu_pending);
    
    Left_Motor_Speed = -motor_value.receive_left_speed_data;
    Right_Motor_Speed = motor_value.receive_right_speed_data;
    car_speed = (Left_Motor_Speed + Right_Motor_Speed) / 2;

}

void pit0_ch11_isr() // 定时器通道 11 周期中断服务函数
{
    pit_isr_flag_clear(PIT_CH11); // 10ms
    /* 台阶检测会遍历图像，耗时不稳定，因此只挂任务。 */
    task_pending_push(&task_10ms_step_pending);
   
}

void pit0_ch12_isr() // 定时器通道 12 周期中断服务函数
{
    pit_isr_flag_clear(PIT_CH12);
}

void pit0_ch13_isr() // 定时器通道 13 周期中断服务函数
{
    pit_isr_flag_clear(PIT_CH13);
}

void pit0_ch14_isr() // 定时器通道 14 周期中断服务函数
{
    pit_isr_flag_clear(PIT_CH14);
}

void pit0_ch15_isr() // 定时器通道 15 周期中断服务函数
{
    pit_isr_flag_clear(PIT_CH15);
}

void pit0_ch16_isr() // 定时器通道 16 周期中断服务函数
{
    pit_isr_flag_clear(PIT_CH16);
}

void pit0_ch17_isr() // 定时器通道 17 周期中断服务函数
{
    pit_isr_flag_clear(PIT_CH17);
}

void pit0_ch18_isr() // 定时器通道 18 周期中断服务函数
{
    pit_isr_flag_clear(PIT_CH18);
}

void pit0_ch19_isr() // 定时器通道 19 周期中断服务函数
{
    pit_isr_flag_clear(PIT_CH19);
}

void pit0_ch20_isr() // 定时器通道 20 周期中断服务函数
{
    pit_isr_flag_clear(PIT_CH20);
}

void pit0_ch21_isr() // 定时器通道 21 周期中断服务函数
{
    pit_isr_flag_clear(PIT_CH21);
    tsl1401_collect_pit_handler();
}
// **************************** PIT中断函数 ****************************

// **************************** 外部中断函数 ****************************
void gpio_0_exti_isr() // 外部 GPIO_0 中断服务函数
{
}

void gpio_1_exti_isr() // 外部 GPIO_1 中断服务函数
{
    if (exti_flag_get(P01_0)) // 示例P1_0端口外部中断判断
    {
    }
    if (exti_flag_get(P01_1))
    {
    }
}

void gpio_2_exti_isr() // 外部 GPIO_2 中断服务函数
{
    if (exti_flag_get(P02_0))
    {
    }
    if (exti_flag_get(P02_4))
    {
    }
}

void gpio_3_exti_isr() // 外部 GPIO_3 中断服务函数
{
}

void gpio_4_exti_isr() // 外部 GPIO_4 中断服务函数
{
}

void gpio_5_exti_isr() // 外部 GPIO_5 中断服务函数
{
}

void gpio_6_exti_isr() // 外部 GPIO_6 中断服务函数
{
}

void gpio_7_exti_isr() // 外部 GPIO_7 中断服务函数
{
}

void gpio_8_exti_isr() // 外部 GPIO_8 中断服务函数
{
}

void gpio_9_exti_isr() // 外部 GPIO_9 中断服务函数
{
}

void gpio_10_exti_isr() // 外部 GPIO_10 中断服务函数
{
}

void gpio_11_exti_isr() // 外部 GPIO_11 中断服务函数
{
}

void gpio_12_exti_isr() // 外部 GPIO_12 中断服务函数
{
}

void gpio_13_exti_isr() // 外部 GPIO_13 中断服务函数
{
}

void gpio_14_exti_isr() // 外部 GPIO_14 中断服务函数
{
}

void gpio_15_exti_isr() // 外部 GPIO_15 中断服务函数
{
}

void gpio_16_exti_isr() // 外部 GPIO_16 中断服务函数
{
}

void gpio_17_exti_isr() // 外部 GPIO_17 中断服务函数
{
}

void gpio_18_exti_isr() // 外部 GPIO_18 中断服务函数
{
}

void gpio_19_exti_isr() // 外部 GPIO_19 中断服务函数
{
}

void gpio_20_exti_isr() // 外部 GPIO_20 中断服务函数
{
}

void gpio_21_exti_isr() // 外部 GPIO_21 中断服务函数
{
}

void gpio_22_exti_isr() // 外部 GPIO_22 中断服务函数
{
}

void gpio_23_exti_isr() // 外部 GPIO_23 中断服务函数
{
}
// **************************** 外部中断函数 ****************************

//// **************************** DMA中断函数 ****************************
// void dma_event_callback(void* callback_arg, cyhal_dma_event_t event)
//{
//     CY_UNUSED_PARAMETER(event);
//
//
//
//
// }
//  **************************** DMA中断函数 ****************************

// **************************** 串口中断函数 ****************************
// 串口0默认作为调试串口
void uart0_isr(void)
{
    if (Cy_SCB_GetRxInterruptMask(get_scb_module(UART_0)) & CY_SCB_UART_RX_NOT_EMPTY) // 串口0接收中断
    {
        Cy_SCB_ClearRxInterrupt(get_scb_module(UART_0), CY_SCB_UART_RX_NOT_EMPTY); // 清除接收中断标志位

#if DEBUG_UART_USE_INTERRUPT       // 如果开启 debug 串口中断
        debug_interrupr_handler(); // 调用 debug 串口接收处理函数 数据会被 debug 环形缓冲区读取
#endif                             // 如果修改了 DEBUG_UART_INDEX 那这段代码需要放到对应的串口中断去
    }
    else if (Cy_SCB_GetTxInterruptMask(get_scb_module(UART_0)) & CY_SCB_UART_TX_DONE) // 串口0发送中断
    {
        Cy_SCB_ClearTxInterrupt(get_scb_module(UART_0), CY_SCB_UART_TX_DONE); // 清除接收中断标志位
    }
}

void uart1_isr(void)
{
    if (Cy_SCB_GetRxInterruptMask(get_scb_module(UART_1)) & CY_SCB_UART_RX_NOT_EMPTY) // 串口1接收中断
    {
        Cy_SCB_ClearRxInterrupt(get_scb_module(UART_1), CY_SCB_UART_RX_NOT_EMPTY); // 清除接收中断标志位

        wireless_module_uart_handler();
    }
    else if (Cy_SCB_GetTxInterruptMask(get_scb_module(UART_1)) & CY_SCB_UART_TX_DONE) // 串口1发送中断
    {
        Cy_SCB_ClearTxInterrupt(get_scb_module(UART_1), CY_SCB_UART_TX_DONE); // 清除接收中断标志位
    }
}

void uart2_isr(void)
{
    if (Cy_SCB_GetRxInterruptMask(get_scb_module(UART_2)) & CY_SCB_UART_RX_NOT_EMPTY) // 串口2接收中断
    {
        Cy_SCB_ClearRxInterrupt(get_scb_module(UART_2), CY_SCB_UART_RX_NOT_EMPTY); // 清除接收中断标志位

        uart_control_callback();
    }
    else if (Cy_SCB_GetTxInterruptMask(get_scb_module(UART_2)) & CY_SCB_UART_TX_DONE) // 串口2发送中断
    {
        Cy_SCB_ClearTxInterrupt(get_scb_module(UART_2), CY_SCB_UART_TX_DONE); // 清除接收中断标志位
    }
}

void uart3_isr(void)
{
    if (Cy_SCB_GetRxInterruptMask(get_scb_module(UART_3)) & CY_SCB_UART_RX_NOT_EMPTY) // 串口3接收中断
    {
        Cy_SCB_ClearRxInterrupt(get_scb_module(UART_3), CY_SCB_UART_RX_NOT_EMPTY); // 清除接收中断标志位
    }
    else if (Cy_SCB_GetTxInterruptMask(get_scb_module(UART_3)) & CY_SCB_UART_TX_DONE) // 串口3发送中断
    {
        Cy_SCB_ClearTxInterrupt(get_scb_module(UART_3), CY_SCB_UART_TX_DONE); // 清除接收中断标志位
    }
}

void uart4_isr(void)
{

    if (Cy_SCB_GetRxInterruptMask(get_scb_module(UART_4)) & CY_SCB_UART_RX_NOT_EMPTY) // 串口4接收中断
    {
        Cy_SCB_ClearRxInterrupt(get_scb_module(UART_4), CY_SCB_UART_RX_NOT_EMPTY); // 清除接收中断标志位

        uart_control_callback();
    }
    else if (Cy_SCB_GetTxInterruptMask(get_scb_module(UART_4)) & CY_SCB_UART_TX_DONE) // 串口4发送中断
    {
        Cy_SCB_ClearTxInterrupt(get_scb_module(UART_4), CY_SCB_UART_TX_DONE); // 清除接收中断标志位
    }
}
// **************************** 串口中断函数 ****************************