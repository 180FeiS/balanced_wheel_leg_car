/*
 * self_math.h
 *
 *  Created on: 2025年2月8日
 *      Author: 33845
 */

#ifndef CODE_SELF_MATH_H_
#define CODE_SELF_MATH_H_

extern int depart_i;
extern int16 Flag_time;



void Gather(double a[],double x);
double double_abs(double x) ;
extern double age_change_180(double age);
extern double age_180_180(double age);
void bubbleSort(double arr[], int n);
extern double calculateMean(double arr[], int n) ;
void system_depart(uint32 x) ;         //计时器



#endif /* CODE_SELF_MATH_H_ */
