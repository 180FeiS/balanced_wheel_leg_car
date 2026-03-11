/*
 * navigation.c
 *
 *  Created on: 2024年10月16日
 *      Author: Monst
 *
 *
 */

#include "zf_common_headfile.h"
#include "navigation.h"

int32 Nav_read[Read_MaxSize];//每5cm的点，1000个点50m
Nag N;
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读取偏航角的里程函数
// 参数说明     读取偏航角的里程函数，通过切换N.End_f来切换里程
// 返回参数     void
// 使用示例     用户自行调用
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void Nag_Read()
{
        switch(N.End_f)
        {
            case 0:Run_Nag_Save();  //默认执行保存
                break;
            case 1:;
                    flash_Nag_Write();  //写入最后一页，确保falsh存储
                    N.End_f++;
                    break;
            case 2://Buzzer_check(500);   //蜂鸣器确认执行
                    N.End_f++;  //结束里程
                    break;
        }
}
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     计算导航偏航角
// 参数说明     N.Final_Out为最终生成的偏航角
// 返回参数     void
// 使用示例     用户自行调用
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void Nag_Run()
{
    Run_Nag_GPS();  //偏航角读取函数
 if(N.Nag_Stop_f) //终点停止
 {
    N.Final_Out=0;
    return;
  }
    N.Final_Out=angle_Z-N.Angle_Run;
    
}
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     偏航角保存
// 参数说明     读取YAW存储flash写入
// 返回参数     void
// 使用示例     用户自行调用
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void Run_Nag_Save()
{
    N.Mileage_All+=(R_Mileage+L_Mileage)/2.0;//里程计取数，取左右两轮的平均值，使用更平滑的缓冲区总平均值
    if(N.size > MaxSize)//当数组超过一页的flash大小时，写入一次，防止重复写入
    {
        flash_Nag_Write();
        N.size=0;   //将数组大小为0，下一次从新开始取
        N.Flash_page_index--;   //flash页索引减小
        zf_assert(N.Flash_page_index > Nag_End_Page);//防止越界保护
    }

    if(N.Mileage_All >= Nag_Set_mileage)    //当里程超过设定值时
    {
       int32 Save=(int32)(Nag_Yaw*100); //取偏航角放大100倍，避免使用Float类型存储
       flash_union_buffer[N.size++].int32_type = Save;  //将偏航角写入缓冲区

       N.Save_index++;


       if(N.Mileage_All > 0) N.Mileage_All -= Nag_Set_mileage;//里程计累加//保存到flash
       else N.Mileage_All += Nag_Set_mileage;//反向
    }

}
// 偏航角读取
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     偏航角读取
// 参数说明     读取flash存储YAW
// 返回参数     void
// 使用示例     用户自行调用
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void Run_Nag_GPS()
{
    N.Mileage_All+=(int)((R_Mileage+L_Mileage)/2.0);//里程计取数，取左右两轮的平均值，使用更平滑的缓冲区总平均值
    uint16 prospect=0;
    if(N.Mileage_All >= Nag_Set_mileage)
    {
    if(N.Run_index> N.Save_index-2)
    {
        N.Nag_Stop_f++;
        return;
    }
       N.Run_index++;//增加需要跑的圈数，直接从后往前，值为0.
    
       prospect=N.Run_index ;//前包
       if(prospect >N.Save_index-2)  prospect=N.Save_index-2;//越界保护
       N.Angle_Run = (Nav_read[prospect]/100.0f);  
       if(N.Mileage_All > 0) N.Mileage_All -= Nag_Set_mileage;//里程计累加//保存到flash
       else N.Mileage_All += Nag_Set_mileage;   //反向
    }


}
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     导航参数初始化
// 返回参数     void
// 使用示例     导航执行前开始
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void Init_Nag()
{
    memset(&N, 0, sizeof(N));
    N.Flash_page_index=Nag_Start_Page;
    flash_buffer_clear();
}
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     封装执行函数
// 参数说明     index           索引
// 参数说明     type            类型值
// 返回参数     void
// 使用示例     在中断里调用
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void Nag_System(){
    //偏航角
    if(!N.Nag_SystemRun_Index || N.Nag_Stop_f )  return;

    switch(N.Nag_SystemRun_Index)
    {
       case 1 : Nag_Read();    //1是读取
            break;
      case 3: Nag_Run();
            break;
    }
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     第一次自动读取，只读取一次
// 参数说明     index           索引
// 参数说明     type            类型值
// 返回参数     void
// 使用示例     在主循环直接调用，demo中直接显示
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void NagFlashRead(){
  if(N.Save_state) return;
  flash_Nag_Read();
  uint8 page_trun=0;
  
  for(int index=0;index <= N.Save_index;index++)
  {
    if(index >= N.Save_index)
    {
        N.Save_state=1;
        break;
    }
    int temp_index=index-(500*page_trun);
    if(temp_index >MaxSize)    //当超过设定的flsh大小时
    {
        N.Flash_page_index--;   //页索引减小
        page_trun++;
        flash_Nag_Read(); //重新读取
    }
     Nav_read[index]= flash_union_buffer[index-(500*page_trun)].int32_type;
  }
  N.Nag_SystemRun_Index++;
}
