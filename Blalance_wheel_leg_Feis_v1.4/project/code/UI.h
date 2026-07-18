/*
 * @Author: bom
 * @Version: V1.0
 * @Date: 2024-12-18 17:23:09
 * @LastEditTime: 2024-12-20 22:26:58
 * @FilePath: \Balance_Car V3.0.1\project\code\UI.h
 * @Description: 
 */
/*********************************************************************************************************************
* 修改记录
* 日期              作者             版本           说明
* 2024-07-24        Bron            V1.0.0         搭建新工程
* 2024-07-27        Bron            V1.0.2         搭建了二级菜单的框架
********************************************************************************************************************/
/*********************************************************************************************************************
* 多级菜单结构描述
*********************************************************************************************************************
* 1. 测试外设模式 (GUI_1)
*    - 1_1. 测试电机
*      - 1_1_1. 电机详细信息
*    - 1_2. 测试编码器
*      - 1_2_1. 编码器详细信息
*    - 1_3. 测试摄像头
*      - 1_3_1. 摄像头详细信息
*    - 1_4. 测试陀螺仪
*      - 1_4_1. 陀螺仪详细信息
*    - 1_5. GPS
*      - 1_5_1. GPS 数据
*
 * 2. 调试模式 (GUI_2)
 *    - 2_1. Image（Debug 二级 Image 行；KEY3 进入三级列表）
 *      - 2_1_1 / 2_1_2 / 2_1_3. Step / Bridge / Bumpy 三级列表（同级切换）
 *      - 2_1_1_1. 台阶检测功能页
 *      - 2_1_2_1. 单边桥检测功能页
 *      - 2_1_3_1. 颠簸路段功能页
*    - 2_2. 速度环设置
*      - 2_2_1. 角速度环P
*      - 2_2_2. 角速度环I
*      - 2_2_3. 角度环P
*      - 2_2_4. 角速度环D
*      - 2_2_5. 速度环P
*      - 2_2_6. 速度环D
*    - 2_3. 转向环设置
*      - 2_3_1. 转向内环P
*      - 2_3_2. 转向内环D
*      - 2_3_3. 转向外环P
*      - 2_3_4. 转向外环D
*    - 2_4. 速度设置
*    - 2_5. 更新Flash参数
 *    - 2_6. PathFix 惯导路径修正（进入 2_6_1）
 *      - 2_6_1. PathFix 功能页（KEY1 +10点 KEY2/3 调 yaw KEY4 保存退出）
*
* 3. 运行模式 (GUI_3)
*    - 3    pos「3」一级 Run（与主菜单其它一级同级）
*    - GUI_3 仅限上述顶层三行；Launch/Save/Config/Jump 二级列表仅用 GUI_3_1～3_4（勿把列表画进 GUI_3）。
*    - 3_1  Launch 二级列表（进入 3_1_1）
*    - 3_2  Save（KEY3 保存 Run 参数：Launch + Config + Jump）
 *    - 3_3  Config 预配置（进入 3_3_1：KEY1 选字段 KEY2 改值）
 *      - 3_3_1. InputMode + VofaEnable + VofaGroup(0~4) + InitLeg 预配置项
*    - 3_4  Jump 跳跃参数（进入 3_4_1：KEY1 选字段 KEY2/3 ±0.5）
*      - 3_4_1. 四阶段腿长 + 四阶段时长（20ms 格）
*    - 3_5  GyroBias（进入 3_5_1）
*    - 3_6  RecSubj 录制科目（进入 3_6_1：KEY1/2 选 KEY3 确认写 Flash）
*    - 3_7  PlaySubj 回放科目（进入 3_7_1：KEY1/2 选 KEY3 确认写 Flash）
*      - 3_1_1. Launch 参数子页（15 字段：速度/距离/BumpSec 等，无底部 K 提示）
*********************************************************************************************************************/
#ifndef __UI_H__
#define __UI_H__

#include "zf_common_typedef.h"

#define PENCOLOR    RGB565_GREEN
#define BGCOLOR     RGB565_BLACK

#define ROW_1   0       //0*16
#define ROW_2   16      //1*16
#define ROW_3   32      //2*16
#define ROW_4   48      //3*16
#define ROW_5   64      //4*16
#define ROW_6   80      //5*16
#define ROW_7   96      //6*16
#define ROW_8   112     //7*16
#define ROW_9   128     //8*16
#define ROW_10  144     //9*16
#define ROW_11  160     //10*16
#define ROW_12  176     //11*16
#define ROW_13  192     //12*16
#define ROW_14  208     //13*16
#define ROW_15  224     //14*16
#define ROW_16  240     //15*16
#define ROW_17  256     //16*16
#define ROW_18  272     //17*16
#define ROW_19  288     //18*16
#define ROW_20  304     //19*16

extern void GUI_1();
extern void ACT_1();

extern void GUI_2();
extern void ACT_2();

extern void GUI_3();
extern void ACT_3();

extern void GUI_1_1();
extern void ACT_1_1();

extern void GUI_1_2();
extern void ACT_1_2();  

extern void GUI_1_3();
extern void ACT_1_3();

extern void GUI_1_4();
extern void ACT_1_4();

extern void GUI_1_5();
extern void ACT_1_5();

extern void GUI_2_1();
extern void ACT_2_1();

extern void GUI_2_2();
extern void ACT_2_2();

extern void GUI_2_3();
extern void ACT_2_3();

extern void GUI_2_4();
extern void ACT_2_4();

extern void GUI_2_5();
extern void ACT_2_5();

extern void GUI_2_6();
extern void ACT_2_6();

extern void GUI_2_6_1();
extern void ACT_2_6_1();

extern void GUI_3_1();
extern void ACT_3_1();

extern void GUI_3_1_1();
extern void ACT_3_1_1();

extern void GUI_3_2();
extern void ACT_3_2();

extern void GUI_3_3();
extern void ACT_3_3();

extern void GUI_3_3_1();
extern void ACT_3_3_1();

extern void GUI_3_4();
extern void ACT_3_4();

extern void GUI_3_4_1();
extern void ACT_3_4_1();

extern void GUI_3_5();
extern void ACT_3_5();

extern void GUI_3_5_1();
extern void ACT_3_5_1();

extern void GUI_3_6();
extern void ACT_3_6();

extern void GUI_3_6_1();
extern void ACT_3_6_1();

extern void GUI_3_7();
extern void ACT_3_7();

extern void GUI_3_7_1();
extern void ACT_3_7_1();

extern void GUI_1_1_1();
extern void ACT_1_1_1();

extern void GUI_1_2_1();
extern void ACT_1_2_1();

extern void GUI_1_3_1();
extern void ACT_1_3_1();

extern void GUI_1_4_1();
extern void ACT_1_4_1();

extern void GUI_1_5_1();
extern void ACT_1_5_1();

extern void GUI_2_1_1();
extern void ACT_2_1_1();

extern void GUI_2_1_2();
extern void ACT_2_1_2();

extern void GUI_2_1_3();
extern void ACT_2_1_3();

extern void GUI_2_1_1_1();
extern void ACT_2_1_1_1();

extern void GUI_2_1_2_1();
extern void ACT_2_1_2_1();

extern void GUI_2_1_3_1();
extern void ACT_2_1_3_1();

extern void GUI_2_2_1();
extern void ACT_2_2_1();

extern void GUI_2_2_2();
extern void ACT_2_2_2();

extern void GUI_2_2_3();
extern void ACT_2_2_3();

extern void GUI_2_2_4();
extern void ACT_2_2_4();

extern void GUI_2_2_5();
extern void ACT_2_2_5();

extern void GUI_2_2_6();
extern void ACT_2_2_6();

extern void GUI_2_3_1();
extern void ACT_2_3_1();

extern void GUI_2_3_2();
extern void ACT_2_3_2();

extern void GUI_2_3_3();
extern void ACT_2_3_3();

extern void GUI_2_3_4();
extern void ACT_2_3_4();

/* CM7_1：在刷新菜单 GUI 前调用，从共享区拉取控制核快照供显示。CM7_0 为空操作。 */
void ui_pull_ctrl_snapshot(void);

#endif
