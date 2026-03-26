/*
 * GPS_flash.c
 *
 *  Created on: 2025骞�2鏈�8鏃�
 *      Author: 33845
 */
#include "zf_common_headfile.h"

uint16 gps_point=0;
int16 GPS_Show_Num=0;
uint8 GPS_cun_flag=0;
int16 Correct_GPS_num=0;
uint8 Correct_wei_flag=0;
uint8 Correct_jing_flag=0;
double Correct_wei=0;
double Correct_jing=0;
double GPSS[2][100]={0};
static int TG=0;
double jing=0;
double wei=0;
double j[nun_shuzu]={0};
double w[nun_shuzu]={0};
float GPS_flash=0;







void gps_write(void)         //閲�
{                                                    // 娓呯┖缂撳啿鍖�
    bubbleSort(j, nun_shuzu);
    bubbleSort(w, nun_shuzu);
    jing=calculateMean(j, nun_shuzu)*100;
    wei=calculateMean(w, nun_shuzu)*100;
    flash_union_buffer[gps_point].uint16_type=(uint16)(wei);
    gps_point++;
    flash_union_buffer[gps_point].float_type=(float)(wei-(uint16)(wei));
    gps_point++;             // 鎿﹂櫎杩欎竴椤�
    flash_union_buffer[gps_point].uint32_type=(uint16)(jing);
    gps_point++;
    flash_union_buffer[gps_point].float_type=(float)(jing-(uint16)(jing));
    gps_point++;

    flash_union_buffer[1000].int16_type=gps_point/4;




}


void gps_cun(void)               //瀛�
{
      if(flash_check(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX))
         {
               flash_erase_page(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX);
         }
      flash_write_page_from_buffer(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX, 60);
   for(int i=0;i<nun_shuzu;i++)
    {
      j[i]=0;
      w[i]=0;
    }
}

/*

void gps_read(void)
{
  flash_buffer_clear();
  flash_read_page_to_buffer(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX);
  GPS_Show_Num=flash_union_buffer[1000].int16_type;
  for(int i=0;i<60;i=i+2)
  {
    GPSS[0][TG]=(flash_union_buffer[i].uint32_type+flash_union_buffer[i+1].float_type)*0.01;
    i=i+2;
    GPSS[1][TG]=(flash_union_buffer[i].uint32_type+flash_union_buffer[i+1].float_type)*0.01;
    TG++;
  }

}            */



void gps_read(void)
{
    flash_buffer_clear();
    flash_write_page_from_buffer(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX, 60);
    GPS_Show_Num=flash_union_buffer[1000].int16_type;
    for(int i=0;i<60;i=i+4)
    {
        // 纬锟斤拷锟截癸拷
        double lat_integer = flash_union_buffer[i].uint16_type;
        double lat_fraction = flash_union_buffer[i+1].float_type;
        GPSS[0][TG] = (lat_integer + lat_fraction) * 0.01;

        // 锟斤拷锟斤拷锟截癸拷
        double lon_integer = flash_union_buffer[i+2].uint16_type;
        double lon_fraction = flash_union_buffer[i+3].float_type;
        GPSS[1][TG] = (lon_integer + lon_fraction) * 0.01;

        TG++;
    }
}




void GPS_Adjust_Write(void)
{
    flash_union_buffer[1000].int16_type = GPS_Show_Num;

    for(int i = 0; i < 15; i++)
    {
        int base = i * 4;


        double lat_val = GPSS[0][i] * 100;
        flash_union_buffer[base].uint16_type = (uint16_t)lat_val;
        flash_union_buffer[base+1].float_type = lat_val - (uint16_t)lat_val;


        double lon_val = GPSS[1][i] * 100;
        flash_union_buffer[base+2].uint16_type = (uint16_t)lon_val;
        flash_union_buffer[base+3].float_type = lon_val - (uint16_t)lon_val;
    }
}


void GPS_Adjust_Cun(void)
{
    if(flash_check(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX))
       {
          flash_erase_page(FLASH_PAGE_INDEX, FLASH_PAGE_INDEX);
       }
       flash_write_page_from_buffer(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX, 60);
}



























