---
name: 里程纠偏计划
overview: 在不改动导航事件框架的前提下，为纯惯导回放增加基于左右轮速度与 IMU 角速度的打滑检测和里程纠偏，优先修正转弯时单轮空转导致的 Run_index 超前问题。第一版不处理台阶开环跳跃腾空，也不启用 IMU 加速度积分，只预留扩展接口和注释。
todos:
  - id: inspect-wheel-signals
    content: 确认 navigation 层可直接访问的左右轮速度与 gyro_z 的单位、符号和换算关系
    status: pending
  - id: design-slip-state
    content: 在 navigation.h 设计打滑状态枚举、参数宏和 Nag 结构体新增字段，并补充中文注释
    status: pending
  - id: integrate-corrected-step
    content: 在 navigation.c 增加打滑检测/中心速度重建函数，并接入 Nag_GetMileageStep()
    status: pending
  - id: reset-and-debug
    content: 补齐初始化复位和调试输出规划，确保新增状态不会跨模式残留
    status: pending
isProject: false
---

# 两轮平衡车里程纠偏改动计划

## 目标与边界
- 目标：提高纯惯导回放时 `Run_index` 的推进可信度，减少转弯单轮空转/轻微腾空造成的里程超前。
- 第一版只处理普通行驶和转弯阶段的单轮打滑/空转，不处理台阶处开环跳跃腾空。
- 第一版不启用 IMU 加速度积分，只在接口与注释中预留后续扩展位。
- 重要函数和关键参数补充中文注释，尤其说明判据来源、单位、调参方向和不适用场景。

## 关键接入点
- 主接入点放在 `D:/Gitee/balanced_wheel_leg_car/Blalance_wheel_leg_Feis_v1.4/project/code/navigation.c` 的 `Nag_GetMileageStep()` 前后逻辑，统一输出“纠偏后的里程步长”，避免在多处直接改 `Run_index`。
- 保持 `Run_Nag_GPS()` 的推进流程不变，即继续走“`Mileage_All += step` -> 每 `Nag_Set_mileage` 推进 1 个 `Run_index`”这条主链。
- 参数、状态量和函数声明集中放在 `D:/Gitee/balanced_wheel_leg_car/Blalance_wheel_leg_Feis_v1.4/project/code/navigation.h`，便于后续 VOFA 观察和迭代调参。

## 方案设计
### 1. 增加打滑检测状态
- 在 `Nag` 结构体中新增“里程纠偏”相关状态，建议包含：
  - 当前纠偏后中心速度
  - 左/右轮可信重建中心速度
  - 打滑状态枚举（正常、左轮打滑、右轮打滑、双侧不可信）
  - 进入/退出打滑的连续计数器
  - 最近一次可信速度
- 状态命名与现有 `N.Event_*`、`N.Speed_Forward` 风格保持一致，避免引入独立全局变量。

### 2. 新增专门的里程纠偏函数
- 在 `navigation.c` 中新增 2~3 个静态函数，职责拆开：
  - 读取左右轮速度和 `gyro_z`，统一换算到 cm/s 或与 `car_speed` 同体系单位
  - 根据两轮速度和角速度重建车体中心速度
  - 输出本周期纠偏后的里程步长
- 推荐接口形态：
  - `Nag_UpdateSlipDetection(...)`
  - `Nag_GetCorrectedForwardSpeed(...)`
  - `Nag_GetCorrectedMileageStep(...)`
- `Nag_GetMileageStep()` 最终改为优先调用“纠偏后步长”，保留现有纯 `car_speed` 积分作为兜底路径。

### 3. 两轮车专用判据
- 采用“左右轮速度 + `gyro_z`”一致性判据，而不是只比较两个轮子的差速。
- 核心思路：利用 `gyro_z` 和轮距 `B` 分别从左轮、右轮反推中心速度，得到两个候选值；若其中一侧明显偏离而另一侧稳定，则判对应轮打滑。
- 第一版不依赖 IMU 线加速度，避免俯仰控速、重力投影和机身振动带来的误判。
- 判定逻辑加去抖：满足阈值持续若干 ms 才进入打滑；恢复也要求持续若干 ms，避免弯道边缘来回抖动。

## 参数与注释规划
- 在 `navigation.h` 新增并注释以下参数：
  - `Nag_Wheel_Track_Cm`：轮距，定义为左右驱动轮接地点对应轮平面的中心间距，工程上按左右轮中面/中心平面的横向距离测量。
  - 左右轮一致性阈值
  - 打滑进入计数阈值
  - 打滑恢复计数阈值
  - 打滑时允许的最大里程推进比例或保底速度策略
- 注释中明确说明轮距的取法：
  - 对差速运动学，使用的是左右轮“滚动中心线”的间距，也就是通常说的两个轮子中心的横向间距。
  - 轮子宽度通常不单独再加一次；若轮胎较宽，以左右轮中面作为参考面测量即可。
  - 若外八/内八安装导致接地点偏差明显，再通过实车转向半径或原地小角度圆弧试验微调。

## 代码改动范围
- `D:/Gitee/balanced_wheel_leg_car/Blalance_wheel_leg_Feis_v1.4/project/code/navigation.h`
  - 新增纠偏参数宏、状态枚举、`Nag` 结构体字段、必要的函数声明。
  - 为关键宏和状态增加中文注释，注明单位和调参方向。
  - 将 `NAG_VOFA_GROUP_COUNT` 与新增里程纠偏调试组同步扩展，避免 `Nag_Vofa_Group` 循环上限和 VOFA 实际分组不一致。
- `D:/Gitee/balanced_wheel_leg_car/Blalance_wheel_leg_Feis_v1.4/project/code/navigation.c`
  - 新增打滑检测与纠偏静态函数。
  - 在 `Nag_GetMileageStep()` 接入纠偏速度计算。
  - 在初始化流程如 `Init_Nag()` / 回放起始复位路径中清零新增状态。
  - 如有必要，补一个只读调试读取函数，方便 VOFA 或 UI 后续接入。
- `D:/Gitee/balanced_wheel_leg_car/Blalance_wheel_leg_Feis_v1.4/project/code/vofa.h`
  - 新增独立的“里程纠偏调试组”宏定义和通道说明，明确每个通道对应变量、单位和观察目的。
  - 保留现有速度组、融合组语义不变，避免老的调试流程被打断。
- `D:/Gitee/balanced_wheel_leg_car/Blalance_wheel_leg_Feis_v1.4/project/code/vofa.c`
  - 在 `vofa_send_nav_from_dualcore_snapshot()` 中新增一个专门用于里程纠偏的 `case`，单独发送打滑检测相关量，不与原速度组混发。
  - 更新文件头部分组注释，说明 0/1/2/3 四个组的用途和切换方式。
- `D:/Gitee/balanced_wheel_leg_car/Blalance_wheel_leg_Feis_v1.4/project/code/flash.c`
  - 维持 `Nag_Vofa_Group % NAG_VOFA_GROUP_COUNT` 的保存/读取逻辑与新组数一致，避免旧配置读出后越界或切组异常。
- 配置菜单相关代码（以 `Nag_Vofa_Group` / `g_menu_vofa_enable` 的现有入口为准）
  - 同步更新 config/Launch 菜单中的 VOFA 组循环与显示文案，让用户能明确切到“里程纠偏调试组”。
  - 若菜单当前只显示数字组号，则计划中一并补充对应说明注释，必要时再评估是否显示简短组名。

## 验证与调试
- 先做静态验证：保证未触发打滑时，`Nag_GetMileageStep()` 行为尽量接近当前实现。
- 实车重点观察三类场景：
  - 普通直行，不能比原方案更抖
  - 正常弯道，不能把正常差速误判为打滑
  - 单轮轻微空转/腾空，`Run_index` 推进应明显更稳
- 新增独立 VOFA 调试组，建议优先放 6 路核心量：
  - 左轮速度
  - 右轮速度
  - `gyro_z`
  - 左轮反推中心速度
  - 右轮反推中心速度
  - 打滑状态或最终纠偏中心速度
- 调试方法按“先静态、再低速、后动态”展开：
  - 上电静止：确认左右轮速度、`gyro_z`、重建中心速度都接近 0，打滑状态稳定在正常态
  - 低速直行：确认左右反推中心速度接近、纠偏输出与原始 `car_speed` 接近，不应频繁误报
  - 正常弯道：重点看左右轮速度允许差速，但左右反推中心速度仍应接近；若打滑状态频繁跳变，优先调一致性阈值和去抖计数
  - 单轮短时空转：重点看原始 `car_speed` 是否突然抬高、某一侧反推中心速度是否明显偏离、最终纠偏速度是否比原始 `car_speed` 更平稳
- 实车重点看三类量：
  - 原始量：左右轮速度、原始 `car_speed`、`gyro_z`
  - 判据量：左右反推中心速度、两侧一致性误差、打滑状态
  - 结果量：最终纠偏中心速度、`Nag_GetMileageStep()` 输出或里程累计变化趋势
- 若通道数不够，优先级建议为：左右轮速度、`gyro_z`、左右反推中心速度、最终纠偏速度；打滑状态可暂时替换其中一路或后续切第二组观察。

## 后续扩展位
- 预留第二阶段接口：当后续确认可稳定取得“去重力后的前向加速度”时，再接入短时 IMU 积分兜底。
- 第二阶段只在“双轮都不可信且持续时间较短”时启用，超时仍回退到冻结/限幅推进策略。