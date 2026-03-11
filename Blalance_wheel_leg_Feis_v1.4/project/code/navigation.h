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

#define Nag_Set_mileage 2100 //里程计//5cm
#define Nag_Prev 200    //前包
#define Nag_Yaw angle_Z //航向角度取偏航角

#define L_Mileage motor_value.receive_left_speed_data   //左轮里程计
#define R_Mileage motor_value.receive_right_speed_data //右轮里程计
//********************************************************//

typedef struct{
       float Final_Out; //导航输出
       float Mileage_All;   //里程计累加
       float Angle_Run; //取偏航角
       bool Nag_Stop_f; //走到终点flag
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
void Nag_Run(); //偏航角控制总函数
void Run_Nag_GPS();//偏航角读取

void NagFlashRead();   //Flash读取目标点数组
void Run_Nag_Save();    //偏航角读取保存
void Nag_Read();    //偏航角读取总函数
//****************************//
void Init_Nag();    //偏航角初始化，flash缓冲区初始化，索引初始化
void Nag_System();  //偏航角函数的封装，包装进中断小
#endif /* _NAVIGATION_H_ */
