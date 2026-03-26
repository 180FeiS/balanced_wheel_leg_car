/*
 * self_math.c
 *
 *  Created on: 2025年2月8日
 *      Author: 33845
 */
#include "zf_common_headfile.h"

int depart_i=0;
int16 Flag_time=0;


void system_depart(uint32 x)          //计时器  放中断中ms级和us级
{
    depart_i++;
    if(depart_i>x)
    {
      Flag_time=1;
      depart_i=0;
    }
}

void Gather(double a[],double x)      //经纬采集
{
      for(uint8 i = 39;i >= 1;i--)
    {
        a[i] = a[i - 1];
    }
    a[0] = x;
}




double double_abs(double x)                //取绝对值
{
  if(x>=0)
  {
    x=x;
  }
  else
  {
    x=-x;
  }
  return x;
}




double age_change_180(double age)
{
  if(age>180)
    age=age-360;
  else
    age=age;
  return age;
}


double age_180_180(double age)
{
  if(age>180)
  {
    age=age-360;
  }
  else if(age<-180)
  {
    age=age+360;
  }
  else
  {
    age=age;
  }
  return age;
}


void bubbleSort(double arr[], int n)        //冒泡排序
{
    for (int i = 0; i < n-1; i++)
    {
        for (int j = 0; j < n-i-1; j++)
        {
            if (arr[j] > arr[j+1])
            {
                double temp = arr[j];
                arr[j] = arr[j+1];
                arr[j+1] = temp;
            }
        }
    }
}

// 求有效值的平均值
double calculateMean(double arr[], int n)
{
    // 去掉六个最小值和六个最大值
    double sum = 0;
    for (int i = 6; i < n-6; i++)
    {
        sum += arr[i];
    }
    return sum / (n-12);
}

