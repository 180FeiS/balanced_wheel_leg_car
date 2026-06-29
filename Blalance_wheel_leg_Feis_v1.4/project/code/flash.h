#ifndef CODE_FLASH_H_
#define CODE_FLASH_H_

/* 导航 flash 模块职责：
 * 1. 保存/读取 yaw 轨迹页；
 * 2. 保存/读取 Save_index 元数据页；
 * 3. 保存/读取元素表页（Nag_Event_Table），用于掉电后恢复元素触发点/type。
 * 4. 保存/读取 Run Launch 参数（无元素速度 + 元素速度/提前减速距离；独立参数页 47）。
 * 5. 保存/读取 Jump 参数（页 50）。
 * 6. 保存/读取 GPS 点位、经纬度和 GPS 专用元素表。
 */

void flash_Nag_Write(void);
void flash_Nag_Read(void);
void flash_Nag_ResetReadState(void);
void flash_RunLaunchSpeed_Write(void);
void flash_RunLaunchSpeed_Read(void);
void flash_JumpParams_Write(void);
void flash_JumpParams_Read(void);
void flash_GpsPoints_Write(void);
void flash_GpsPoints_Read(void);
void flash_GpsPoints_Clear(void);


#endif
