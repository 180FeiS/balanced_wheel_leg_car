#ifndef CODE_FLASH_H_
#define CODE_FLASH_H_

/* 导航 flash 模块职责：
 * 1. 保存/读取 yaw 轨迹页；
 * 2. 保存/读取 Save_index 元数据页；
 * 3. 保存/读取元素表页（Nag_Event_Table），用于掉电后恢复元素 enter/exit/type。
 */

void flash_Nag_Write(void);
void flash_Nag_Read(void);
void flash_Nag_ResetReadState(void);


#endif
