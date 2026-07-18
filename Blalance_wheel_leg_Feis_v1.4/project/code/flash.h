#ifndef CODE_FLASH_H_
#define CODE_FLASH_H_

/* 导航 flash 模块职责：
 * 1. 保存/读取 yaw 轨迹页；
 * 2. 保存/读取 Save_index 元数据页；
 * 3. 保存/读取元素表页（Nag_Event_Table），用于掉电后恢复元素触发点/type。
 * 4. 保存/读取 Run Launch 参数（无元素速度 + 元素速度/提前减速距离；独立参数页 47）。
 * 5. 保存/读取 Jump 参数（页 50）。
 * 6. 保存/读取 gyro_z 零偏标定（页 51）。
 * 7. 保存/读取 GPS 点位、经纬度和 GPS 专用元素表。
 * 8. 惯导三科目：每科目独立元数据/yaw/元素页（见 navigation.h NAG_SUBJECT_*）。
 */

void flash_Nag_Write(void);
void flash_Nag_Read(void);
void flash_Nag_ResetReadState(void);
/* 显式绑定惯导 Flash 槽位（1~3）；PathFix 保存时使用会话载入时的 replay_subject */
void flash_Nag_BindSubjectSlot(uint8 subject);
void flash_RunLaunchSpeed_Write(void);
void flash_RunLaunchSpeed_Read(void);
void flash_JumpParams_Write(void);
void flash_JumpParams_Read(void);
void flash_GyroBias_Write(void);
void flash_GyroBias_Read(void);
void flash_GpsPoints_Write(void);
void flash_GpsPoints_Read(void);
void flash_GpsPoints_Clear(void);

/* 仅载入惯导 yaw 轨迹到 Nav_read[]（不进入回放态）；调用前须 flash_Nag_BindSubjectSlot */
uint8 flash_Nag_LoadTrajectoryOnly(uint16 *out_save_index);
/* 将 Nav_read[0..save_index-1] 整表回写已绑定 Flash 槽；调用前须 flash_Nag_BindSubjectSlot */
uint8 flash_Nag_WriteFullPath(uint16 save_index);

#endif
