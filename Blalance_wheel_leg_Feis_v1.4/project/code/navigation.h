/*
 * nagivation.h
 *
 *  Created on: 2024年10月16日
 *      Author: Monst
 */

#ifndef _NAVIGATION_H_
#define _NAVIGATION_H_


//*********************用户宏定义****************************//
#define MaxSize 500    //flash存储数组页数

#define Read_MaxSize 10000//导航读取数组，1w个应该是够了

//存储范围 <0 - 47>
#define Nag_End_Page 1      //flash结束页数
#define Nag_Start_Page 45   //flash开始页数

/* 速度积分式里程计参数：
 * 1. Nag_Set_mileage 表示每隔多少“距离单位”记录/回放一次 yaw，默认按 cm 理解；
 * 2. Nag_Speed_Source 为当前选用的前向速度源，默认先用 car_speed 快速打通流程；
 * 3. Nag_Speed_To_Mileage_Scale 用来把速度源换算成“距离单位/秒”，需要实车标定；
 * 4. Nag_Speed_Deadband 用于抑制静止抖动造成的里程累计。
 */
#define Nag_Set_mileage 5.0f            //每隔 5cm 记录一次 yaw
#define Nag_Prev 200                    //前瞻
#define Nag_Yaw euler_angle.yaw         //航向角度取偏航角
#define Nag_Sample_Dt 0.005f            //Nag_System 运行周期，单位 s
#define Nag_Speed_Source car_speed      //默认优先使用车体平均速度
#define Nag_Speed_To_Mileage_Scale 0.37f //速度单位到 cm/s 的换算系数，需标定
#define Nag_Speed_Deadband 1.0f         //速度死区，抑制静止噪声
#define Nag_Reissue_Error 3.5f          //转向收敛后若再次偏离该角度，则重新下发目标 yaw
//********************************************************//

typedef struct{
       float Final_Out; //导航输出
       float Mileage_All;   //里程计累加
       float Mileage_Step;  //本周期位移增量
       float Mileage_Debug_Total; //调试用累计总路程
       float Speed_Forward; //当前用于积分的前向速度
       float Angle_Run; //取偏航角
       float Requested_Target_Yaw; //最近一次下发给 steer 的目标航向
       bool Nag_Stop_f; //走到终点flag
       uint8 Target_Request_Valid; //目标航向是否已下发
       uint8 Flash_read_f;//走到终点读取flag
       uint16 size; //导航数组大小，通过计数
       uint16 Run_index;
       uint16 Save_count;
       uint16 Save_index;//保存flag
       uint8 Save_state;
       uint8 End_f;//终点flag
       //flash相关参数
       uint8 Flash_page_index;//flash页索引
       uint8 Flash_Save_Page_Index;//flash保存页索引
       uint8 Nag_SystemRun_Index;   //导航执行索引
       //临时未使用参数
       int Prev_mile[Nag_Prev]; //前包
}Nag;

extern Nag N;   //导航相关的结构体，用户开放参数
extern int32 Nav_read[Read_MaxSize];//每5cm的点，1000个点50m
extern uint8 Nag_Vofa_Group; //VOFA 调试组切换
void Nag_Run(); //偏航角控制总函数
void Run_Nag_GPS();//偏航角读取

void NagFlashRead();   //Flash读取目标点数组
void Run_Nag_Save();    //偏航角读取保存
void Nag_Read();    //偏航角读取总函数
//****************************//
void Init_Nag();    //偏航角初始化，flash缓冲区初始化，索引初始化
void Nag_Begin_Record(void); //开始录制前复位运行态
void Nag_Begin_Replay(void); //开始复现前复位运行态
void Nag_Request_Stop_Record(void); //录制结束请求
float Nag_GetDebugReadYaw(void); //安全读取当前回放目标 yaw
void Nag_System();  //偏航角函数的封装，包装进中断小
#endif /* _NAVIGATION_H_ */
