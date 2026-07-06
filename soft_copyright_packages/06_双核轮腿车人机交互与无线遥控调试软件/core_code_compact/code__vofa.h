#ifndef _VOFA_H_
#define _VOFA_H_
#include "zf_common_headfile.h"
void SendDataStreamToVOFA(uint32_t Count, float Data, ...);
void ReadDataFromPc(void); /* �����ֽ� Menu_command �⣬֧�� V<��ֵ> ��չ֡����� vofa.c ��ע�� */
void SendDataToVofa(void);
extern uint32 WirelessUart_ReadBuff_Count;   // ��ȡ���ߴ������ݳ���
extern uint8 WirelessUart_ReadBuff_Data[64]; // ��ȡ���ߴ��ڻ�������
extern uint8 Menu_command;                   // �˵�����ָ��
extern float ReadBuf_Pid;
#if defined(CY_CORE_CM7_1)
/* ��˫�� CM7_1���� shared ctrl ���գ��� wireless_uart �� JustFloat���뱾���� wireless_uart_init�� */
void vofa_send_nav_from_dualcore_snapshot(void);
/*
 * VOFA �� 1 �� �ٶȵ��ԣ��˵� n ������ group==1����
 *   ch1 speed_target_effective  �ٶȻ�ʵ��Ŀ��
 *   ch2 car_speed               ��ǰʵ�⳵��
 */
#define VOFA_GROUP_SPEED_DEBUG (1u)
/*
 * VOFA �� 2 �� GPS+�ߵ��ںϵ��ԣ��˵� n ������ 2����
 *   ch1 fusion_x_m           �ں϶���λ��(m)
 *   ch2 fusion_y_m           �ںϱ���λ��(m)
 *   ch3 fusion_v_mps         �궨��ǰ���ٶ�(m/s)
 *   ch4 fusion_gps_residual_m  GPS ���ں�λ�òв�(m)��>3 ��������
 *   ch5 fusion_gps_weight    ��֡ GPS ��������
 *   ch6 fusion_gps_used      1=��֡ GPS ��������
 *   ��CM7_1 �� 2 ��չ��ch5/ch6 �ɻ�Ϊ fusion_origin_accepted / fusion_origin_rejected
 */
#define VOFA_GROUP_FUSION_DEBUG (2u)
/*
 * VOFA �� 3 �� ��̾�ƫ/�򻬣��˵� n / Config ������ 3����
 *   ch1 odo_wheel_left_cmps   ����ǰ���ٶ� cm/s
 *   ch2 odo_wheel_right_cmps  ����ǰ���ٶ� cm/s
 *   ch3 odo_gyro_z_dps        IMU Z ����ٶ� deg/s
 *   ch4 odo_vc_from_l_cmps    ����+gyro ���������ٶ� cm/s
 *   ch5 odo_vc_from_r_cmps    ����+gyro ���������ٶ� cm/s
 *   ch6 odo_corr_speed_cmps   ��ƫ�������ٶ� cm/s����̬�� dualcore odo_slip_state��0���� 1�� 2�� 3˫�ࣩ
 * ���ԣ�ֱ��/����� ch4��ch5�����ֿ�תʱһ��ƫ�롢ch6 Ӧ���� car_speed ��Ӧ������
 */
#define VOFA_GROUP_ODO_SLIP_DEBUG (3u)
#endif
#endif
