/*
 * GPS_deal.h
 *
 *  Created on: 2025Äê2ÔÂ8ÈÕ
 *      Author: 33845
 */

#ifndef CODE_GPS_DEAL_H_
#define CODE_GPS_DEAL_H_

extern int16 GPS_num;
extern double lat;
extern double lon;
extern double latErr;
extern double lonErr;
extern double GPS_angle;
extern double GPS_angle_revise;
extern int16 lora_receive_num;
extern double GPS_Distance;
extern double GPS_Direction;

void GPS_data(void);


#endif /* CODE_GPS_DEAL_H_ */
