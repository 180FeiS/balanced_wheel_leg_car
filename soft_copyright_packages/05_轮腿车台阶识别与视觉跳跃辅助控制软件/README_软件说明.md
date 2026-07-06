# 05 轮腿车台阶识别与视觉跳跃辅助控制软件

## 功能概述
- 台阶上下沿检测与像素距离换算
- ENTER_STAIR 门控下的 step_detect
- 视觉触发跳跃 dualcore_ui_cmd_push(JUMP)
- 跳后冷却 1ms 递减

## 入口
step_detection_init → main 循环 step_detect / step_visual_jump_after_step
cm7_1 pit0_ch0: step_visual_jump_post_jump_cooldown_on_cm7_1_1ms
