#ifndef _VOFA_H_
#define _VOFA_H_

#include "zf_common_headfile.h"

void SendDataStreamToVOFA(uint32_t Count, float Data, ...);
void ReadDataFromPc(void); /* 除单字节 Menu_command 外，支持 V<数值> 扩展帧，详见 vofa.c 内注释 */
void SendDataToVofa(void);

extern uint32 WirelessUart_ReadBuff_Count;   // 读取无线串口数据长度
extern uint8 WirelessUart_ReadBuff_Data[64]; // 读取无线串口缓冲数组
extern uint8 Menu_command;                   // 菜单进行指令
extern float ReadBuf_Pid;

#if defined(CY_CORE_CM7_1)
/* 仅双核 CM7_1：读 shared ctrl 快照，经 wireless_uart 发 JustFloat（须本核已 wireless_uart_init） */
void vofa_send_nav_from_dualcore_snapshot(void);

/*
 * VOFA 组 1 — 速度调试（菜单 n 切组至 group==1）：
 *   ch1 speed_target_effective  速度环实际目标
 *   ch2 car_speed               当前实测车速
 */
#define VOFA_GROUP_SPEED_DEBUG (1u)

/*
 * VOFA 组 2 — GPS+惯导融合调试（菜单 n 切组至 2）：
 *   ch1 fusion_x_m           融合东向位移(m)
 *   ch2 fusion_y_m           融合北向位移(m)
 *   ch3 fusion_v_mps         标定后前向速度(m/s)
 *   ch4 fusion_gps_residual_m  GPS 与融合位置残差(m)，>3 常见跳点
 *   ch5 fusion_gps_weight    本帧 GPS 修正增益
 *   ch6 fusion_gps_used      1=本帧 GPS 参与修正
 *   （CM7_1 组 2 扩展）ch5/ch6 可换为 fusion_origin_accepted / fusion_origin_rejected
 */
#define VOFA_GROUP_FUSION_DEBUG (2u)

/*
 * VOFA 组 3 — 里程纠偏/打滑（菜单 n / Config 切组至 3）：
 *   ch1 odo_wheel_left_cmps   左轮前向速度 cm/s
 *   ch2 odo_wheel_right_cmps  右轮前向速度 cm/s
 *   ch3 odo_gyro_z_dps        IMU Z 轴角速度 deg/s
 *   ch4 odo_vc_from_l_cmps    左轮+gyro 反推中心速度 cm/s
 *   ch5 odo_vc_from_r_cmps    右轮+gyro 反推中心速度 cm/s
 *   ch6 odo_corr_speed_cmps   纠偏后中心速度 cm/s（打滑态见 dualcore odo_slip_state：0正常 1左 2右 3双侧）
 * 调试：直行/弯道看 ch4≈ch5；单轮空转时一侧偏离、ch6 应低于 car_speed 对应步长。
 */
#define VOFA_GROUP_ODO_SLIP_DEBUG (3u)

/*
 * VOFA 组 4 — 自旋航向闭环校正（菜单 n 切组至 4）：
 *   ch1 spin_angle_err_snap      剩余角 spin_target - spin_accum (deg)
 *   ch2 spin_accum_deg_snap      自旋累计角 (deg)
 *   ch3 spin_start_yaw_deg        起转显示航向 (deg)
 *   ch4 spin_yaw_correct_delta_deg  最近一次校正量 (deg)
 *   ch5 spin_yaw_correct_applied  1=上次 spin_finish(1) 已校正
 *   ch6 spin_yaw_correct_skipped  1=误差超门限跳过校正
 */
#define VOFA_GROUP_SPIN_YAW_DEBUG (4u)
#endif

#endif

