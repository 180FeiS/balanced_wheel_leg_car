# GPS 坐标漂移修正（录点参考起点 ↔ 发车实测起点）

## 功能说明

比赛或长时间运行后，同一物理位置的 GNSS 读数会缓慢漂移。本工程在 **KEY3 发车** 时，用「录径时保存的第一个路点（index 0）」与「发车瞬间锁存的 GNSS」求差，得到经纬度平移量 `gps_drift_delta_lat` / `gps_drift_delta_lon`。导航时在内存中将 **目标路点加上该平移量** 再算方位与距离，**不修改 flash 中已存储的原始路点数组**。

公式与 `gps_first_clearerr_from_coord` 中 `d_w、d_j` 一致：

- `delta_lat = 发车纬度 − latitude_point[0]`
- `delta_lon = 发车经度 − longitude_point[0]`
- 导航用目标：`(latitude_point[k] + delta_lat, longitude_point[k] + delta_lon)`

**不要**在同一次运行中再调用 `gps_first_clearerr` / `gps_first_clearerr_from_coord` 去整表平移路点，否则与上述逻辑 **双重平移**。

## 与 COG 航向标定（发车后 3 m）的关系


| 机制     | 变量/宏                                                   | 作用                                             |
| ------ | ------------------------------------------------------ | ---------------------------------------------- |
| 坐标漂移修正 | `gps_drift_`*, `gps_drift_corr_valid`                  | 把录点时的「经纬坐标系」平移到发车时的卫星读数坐标系                     |
| 航向标定   | `GPS_NAV_GPS_FIRST_DISTANCE_M`，`gps_nav_gps_first_deg`，`gps_nav_heading_bias_deg` | 相对发车点 **位移 ≥ 默认 3 m** 时读取 RMC **`gnss.direction`（COG）** 作 **GPS_first**，与 **IMU yaw** 一次标定偏置；与点列平移 **独立** |


## 操作顺序（建议）

1. **KEY1**：开始录点（清空点列）。
2. 将车推到 **真实起跑线/参考点**，**第一次按 KEY3** 保存为 **index 0**（录径参考起点）。
3. 继续 KEY3 录入后续路点。
4. **KEY2**：结束并写入 flash（可选，便于下次上电加载）。
5. 比赛发车前，将车 **再次停在同一物理起跑位置**。
6. **KEY3**：发车。屏幕 GPS Debug 中 **Dv=1** 表示已成功计算漂移修正量；**dLa/dLo** 为 delta（度，通常为小量）。
7. 发车后 **前约 `GPS_NAV_GPS_FIRST_DISTANCE_M`（默认 3 m）**：目标航向保持为 **发车瞬间锁存的 IMU yaw**（屏 **Al=0**，**Lm** 为距发车点距离）。驶过该距离后读取 **COG** 锁 **GF（GPS_first）** 与 **Bias**，**Al=1（TRACKING）** 后按路点地理方位追迹。

## 验证建议

1. **静态漂移模拟**：在固定点录好 index 0 后，断电或等待数分钟，将车放回 **同一点** 再 KEY3 发车，观察 dLa/dLo 量级（典型为 `1e-5`～`1e-4` 度量级，视环境而定）。
2. **轨迹对比**：修正开启（Dv=1）时，车应更贴近当时录制的相对路径；若 index 0 未打在真实起点，整条路径会整体平移但形状仍可保持。
3. **航向**：**Lm** 增至约 3 m 后 **GF** 应有值、**Al** 切至 1；若 **Pr=CogBad**，说明该时刻 RMC 无效，需定位/天线/场地后再试。
4. **异常**：无卫星、经纬仍为 0、`gps_drift_corr_valid=0` 时，导航 **不应用** 经纬平移；仍有 IMU 目标航向，但 **COG 标定依赖 RMC**，需检查定位。

## 相关源码

- 逻辑实现：`[project/code/my_gps.c](../project/code/my_gps.c)` — `GPS_NavTryUpdateDriftCorrection`, `GPS_PointNav_Run`
- 对外变量：`[project/code/my_gps.h](../project/code/my_gps.h)`
- 双核屏/UI 观测：`[project/code/dualcore_shared.c](../project/code/dualcore_shared.c)`, `[project/code/UI.c](../project/code/UI.c)` `GUI_2_3_1`（**Al / GF / Lm**）
