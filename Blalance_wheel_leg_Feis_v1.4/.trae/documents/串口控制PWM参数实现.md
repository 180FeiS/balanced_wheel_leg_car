# 串口控制PWM参数实现计划

## 1. 问题分析
- 当前代码中只有pwm_ph1和pwm_ph4两个PWM参数，需要添加pwm_ph2和pwm_ph3
- 串口数据处理在vofa.c的ReadDataFromPc函数中
- 电机控制逻辑在control.c的left_leg_control和right_leg_control函数中

## 2. 实现步骤

### 2.1 扩展串口指令解析
- 修改vofa.c中的ReadDataFromPc函数，添加对PWM参数控制指令的解析
- 定义指令格式：如"p1+"表示pwm_ph1增加10，"p1-"表示pwm_ph1减少10
- 同样为pwm_ph2、pwm_ph3、pwm_ph4添加类似指令

### 2.2 添加PWM参数变量
- 在control.c中添加全局变量pwm_ph1、pwm_ph2、pwm_ph3、pwm_ph4
- 初始化为0

### 2.3 修改电机控制逻辑
- 修改left_leg_control和right_leg_control函数
- 不再通过servo_control_table计算PWM值，而是直接使用全局PWM变量
- 添加PWM参数的边界检查，确保值在合理范围内

### 2.4 实现返回值功能
- 在处理完PWM参数调整后，通过串口发送SERVO1_MID与pwm_ph4的和值
- 使用现有的wireless_uart_send_buffer或uart_write_buffer函数

### 2.5 测试和验证
- 确保指令解析正确
- 确保PWM参数边界检查有效
- 确保返回值计算准确

## 3. 技术要点
- 串口指令格式设计：简单明了，易于解析
- PWM参数边界检查：防止值过大或过小导致电机损坏
- 返回值计算：确保SERVO1_MID与pwm_ph4的和值正确计算和发送
- 代码兼容性：确保修改不会影响现有功能

## 4. 预期结果
- 能够通过串口发送指令控制四个PWM参数的增减
- 执行完参数调整后，能够通过串口收到SERVO1_MID与pwm_ph4的和值
- 系统能够正常运行，不受修改影响