/*
 * GPS_deal.c
 *
 *  Created on: 2025年2月8日
 *      Author: 33845
 */
#include "zf_common_headfile.h"


int16 GPS_num=1;
double lat=0;
double lon=0;
double latErr=0;
double lonErr=0;
double GPS_angle=0;
double GPS_angle_revise=0;
int16 lora_receive_num=0;
double GPS_Distance=0;
double GPS_Direction=0;


void GPS_data(void)
{
  //lat=GPSS[0][lora_receive_num]+latErr;         //目标位置的纬度值
  //lon=GPSS[1][lora_receive_num]+lonErr;         //目标位置的经度值
  //lat=GPSS[0][1]+latErr;
  //lon=GPSS[1][1]+lonErr;

  lat=GPSS[0][GPS_num]+latErr;
  lon=GPSS[1][GPS_num]+lonErr;

  GPS_angle=age_change_180(get_two_points_azimuth(gnss.latitude, gnss.longitude, lat, lon));   //获取车当前位置和目标位置的夹角
  GPS_Distance=get_two_points_distance(gnss.latitude, gnss.longitude, lat, lon);                   //获取车当前位置和目标位置的距离
  //if(GPS_angle<-172||GPS_angle>172)
  //{
    //GPS_angle=175;
  //}
  


  if(gnss.direction>180)
  {
    GPS_Direction=gnss.direction-360;
  }
  else
  {
    GPS_Direction=gnss.direction;
  }


}

