#ifndef CODE_TASK_SCHEDULE_H_
#define CODE_TASK_SCHEDULE_H_

#include "zf_common_typedef.h"

/*
 * 软任务调度说明
 *
 * 当前工程采用“短中断 + 主循环执行软任务”的混合式调度：
 * 1. PIT 中断里只保留必须的硬实时控制。
 * 2. 菜单、导航慢路径、图像处理等非硬实时任务只在中断里累加 pending 计数。
 * 3. 主循环通过 run_soft_tasks() 读取 pending 计数并真正执行对应函数。
 *
 * 以后如果要新增一个软任务，按下面 3 步改：
 * 1. 在这里新增一个 pending 变量声明，例如：
 *      extern vuint8 task_100ms_log_pending;
 * 2. 在 cm7_0_isr.c 中定义该变量，并在对应周期中断里调用：
 *      task_pending_push(&task_100ms_log_pending);
 * 3. 在 main_cm7_0.c 的 run_soft_tasks() 中增加：
 *      if (task_pending_take(&task_100ms_log_pending)) { log_task(); }
 *
 * 约定：
 * - 名字使用 task_xxx_pending 风格，表示“还有多少次待执行”。
 * - 计数由 ISR 累加、由主循环递减。
 * - 新增到 ISR 的内容应尽量只做置位/计数，不要直接放耗时逻辑。
 */
extern vuint8 task_5ms_nav_pending;
extern vuint8 task_10ms_menu_key_pending;
extern vuint8 task_20ms_menu_pending;
extern vuint8 task_10ms_step_pending;

#endif /* CODE_TASK_SCHEDULE_H_ */
