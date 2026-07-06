# 排除依赖 — 01 运动控制软件

## 第三方库（不提交源码）

- `Blalance_wheel_leg_Feis_v1.4/libraries/sdk/` — Infineon CYT4BB SDK
- `Blalance_wheel_leg_Feis_v1.4/libraries/zf_common/`, `zf_driver/`, `zf_device/` — 逐飞 GPL 库
- `zf_common_headfile.h` — 仅说明为聚合头文件

## 其他业务模块（接口引用）

| 模块 | 处理方式 |
|------|----------|
| navigation | `interfaces/control_port.h` 反向：`Nag_GetControlSpeedTarget` 由 02 实现 |
| dualcore_shared | 主归属 06/07；本模块仅调用 consume/apply |
| Menu | `dip_switch_motor_sync_from_hw` 主归属 06 |

## 构建产物

- `project/iar/Debug_m7_0/**`
- `*.o`, `*.pbi`, `*.map`, `*.hex`, `.ninja_*`
