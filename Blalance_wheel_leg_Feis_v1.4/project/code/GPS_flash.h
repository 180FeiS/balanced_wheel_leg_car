/*
 * GPS_flash.h
 *
 *  Created on: 2025年2月8日
 *      Author: 33845
 */

#ifndef CODE_GPS_FLASH_H_
#define CODE_GPS_FLASH_H_

#define FLASH_SECTION_INDEX       (0)                                 // 存储数据用的扇区
#define FLASH_PAGE_INDEX          (0)                                // 存储数据用的页码 倒数第一个页码
#define nun_shuzu 40

void Gps_flash();
void Data_Cun();
void Data_Read();
extern void gps_write(void);
extern void gps_read(void);
extern void gps_cun(void);
extern void Correct_GPS_write(void);
extern void Correct_GPS_cun(void);
extern void Correct_GPS_read(void);
extern void GPS_Adjust_Write(void);
extern void GPS_Adjust_Cun(void);
void GPS_Read(void);


extern double GPSS[2][100];
extern uint16 gps_point;
extern double jing;
extern double wei;
extern double j[nun_shuzu];
extern double w[nun_shuzu];
extern int16 GPS_Show_Num;
extern uint8 GPS_cun_flag;
extern int16 Correct_GPS_num;
extern uint8 Correct_wei_flag;
extern uint8 Correct_jing_flag;


extern float Flag_Distance;//开始减速的距离GPS距离，稍微大一点给减速留出空间
extern float Flag_Distance1;//进入声音的距离声音强度,能够识别即可





#endif /* CODE_GPS_FLASH_H_ */
