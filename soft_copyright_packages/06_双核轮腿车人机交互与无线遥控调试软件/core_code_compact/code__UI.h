/*
 * @Author: bom
 * @Version: V1.0
 * @Date: 2024-12-18 17:23:09
 * @LastEditTime: 2024-12-20 22:26:58
 * @FilePath: \Balance_Car V3.0.1\project\code\UI.h
 * @Description:
 */
/*********************************************************************************************************************
* �޸ļ�¼
* ����              ����             �汾           ˵��
* 2024-07-24        Bron            V1.0.0         ��¹���
* 2024-07-27        Bron            V1.0.2         ��˶����˵��Ŀ��
********************************************************************************************************************/
/*********************************************************************************************************************
* �༶�˵��ṹ����
*********************************************************************************************************************
* 1. ��������ģʽ (GUI_1)
*    - 1_1. ���Ե��
*      - 1_1_1. �����ϸ��Ϣ
*    - 1_2. ���Ա�����
*      - 1_2_1. ��������ϸ��Ϣ
*    - 1_3. ��������ͷ
*      - 1_3_1. ����ͷ��ϸ��Ϣ
*    - 1_4. ����������
*      - 1_4_1. ��������ϸ��Ϣ
*    - 1_5. GPS
*      - 1_5_1. GPS ����
*
 * 2. ����ģʽ (GUI_2)
 *    - 2_1. Image��Debug ���� Image �У�KEY3 ���������б���
 *      - 2_1_1 / 2_1_2 / 2_1_3. Step / Bridge / Bumpy �����б���ͬ���л���
 *      - 2_1_1_1. ̨�׼�⹦��ҳ
 *      - 2_1_2_1. �����ż�⹦��ҳ
 *      - 2_1_3_1. ����·�ι���ҳ
*    - 2_2. �ٶȻ�����
*      - 2_2_1. ���ٶȻ�P
*      - 2_2_2. ���ٶȻ�I
*      - 2_2_3. �ǶȻ�P
*      - 2_2_4. ���ٶȻ�D
*      - 2_2_5. �ٶȻ�P
*      - 2_2_6. �ٶȻ�D
*    - 2_3. ת������
*      - 2_3_1. ת���ڻ�P
*      - 2_3_2. ת���ڻ�D
*      - 2_3_3. ת���⻷P
*      - 2_3_4. ת���⻷D
*    - 2_4. �ٶ�����
*    - 2_5. ����Flash����
*    - 2_6. ���FLASH������
*
* 3. ����ģʽ (GUI_3)
*    - 3    pos��3��һ�� Run�������˵�����һ��ͬ����
*    - GUI_3 ���������������У�Launch/Save/Config/Jump �����б����� GUI_3_1��3_4������б����� GUI_3����
*    - 3_1  Launch �����б������� 3_1_1��
*    - 3_2  Save��KEY3 ���� Run ������Launch + Config + Jump��
 *    - 3_3  Config Ԥ���ã����� 3_3_1��KEY1 ѡ�ֶ� KEY2 ��ֵ��
 *      - 3_3_1. InputMode + VofaEnable + VofaGroup Ԥ������
*    - 3_4  Jump ��Ծ���������� 3_4_1��KEY1 ѡ�ֶ� KEY2/3 ��0.5��
*      - 3_4_1. �Ľ׶��ȳ� + �Ľ׶�ʱ����20ms ��
*      - 3_1_1. Launch ������ҳ���ٶ�/�ƶ����룩
*********************************************************************************************************************/
#ifndef __UI_H__
#define __UI_H__
#include "zf_common_typedef.h"
#define PENCOLOR    RGB565_GREEN
#define BGCOLOR     RGB565_BLACK
#define ROW_1   0       //0*16
#define ROW_2   16      //1*16
#define ROW_3   32      //2*16
#define ROW_4   48      //3*16
#define ROW_5   64      //4*16
#define ROW_6   80      //5*16
#define ROW_7   96      //6*16
#define ROW_8   112     //7*16
#define ROW_9   128     //8*16
#define ROW_10  144     //9*16
#define ROW_11  160     //10*16
#define ROW_12  176     //11*16
#define ROW_13  192     //12*16
#define ROW_14  208     //13*16
#define ROW_15  224     //14*16
#define ROW_16  240     //15*16
#define ROW_17  256     //16*16
#define ROW_18  272     //17*16
#define ROW_19  288     //18*16
#define ROW_20  304     //19*16
extern void GUI_1();
extern void ACT_1();
extern void GUI_2();
extern void ACT_2();
extern void GUI_3();
extern void ACT_3();
extern void GUI_1_1();
extern void ACT_1_1();
extern void GUI_1_2();
extern void ACT_1_2();
extern void GUI_1_3();
extern void ACT_1_3();
extern void GUI_1_4();
extern void ACT_1_4();
extern void GUI_1_5();
extern void ACT_1_5();
extern void GUI_2_1();
extern void ACT_2_1();
extern void GUI_2_2();
extern void ACT_2_2();
extern void GUI_2_3();
extern void ACT_2_3();
extern void GUI_2_4();
extern void ACT_2_4();
extern void GUI_2_5();
extern void ACT_2_5();
extern void GUI_2_6();
extern void ACT_2_6();
extern void GUI_3_1();
extern void ACT_3_1();
extern void GUI_3_1_1();
extern void ACT_3_1_1();
extern void GUI_3_2();
extern void ACT_3_2();
extern void GUI_3_3();
extern void ACT_3_3();
extern void GUI_3_3_1();
extern void ACT_3_3_1();
extern void GUI_3_4();
extern void ACT_3_4();
extern void GUI_3_4_1();
extern void ACT_3_4_1();
extern void GUI_1_1_1();
extern void ACT_1_1_1();
extern void GUI_1_2_1();
extern void ACT_1_2_1();
extern void GUI_1_3_1();
extern void ACT_1_3_1();
extern void GUI_1_4_1();
extern void ACT_1_4_1();
extern void GUI_1_5_1();
extern void ACT_1_5_1();
extern void GUI_2_1_1();
extern void ACT_2_1_1();
extern void GUI_2_1_2();
extern void ACT_2_1_2();
extern void GUI_2_1_3();
extern void ACT_2_1_3();
extern void GUI_2_1_1_1();
extern void ACT_2_1_1_1();
extern void GUI_2_1_2_1();
extern void ACT_2_1_2_1();
extern void GUI_2_1_3_1();
extern void ACT_2_1_3_1();
extern void GUI_2_2_1();
extern void ACT_2_2_1();
extern void GUI_2_2_2();
extern void ACT_2_2_2();
extern void GUI_2_2_3();
extern void ACT_2_2_3();
extern void GUI_2_2_4();
extern void ACT_2_2_4();
extern void GUI_2_2_5();
extern void ACT_2_2_5();
extern void GUI_2_2_6();
extern void ACT_2_2_6();
extern void GUI_2_3_1();
extern void ACT_2_3_1();
extern void GUI_2_3_2();
extern void ACT_2_3_2();
extern void GUI_2_3_3();
extern void ACT_2_3_3();
extern void GUI_2_3_4();
extern void ACT_2_3_4();
/* CM7_1����ˢ�²˵� GUI ǰ���ã��ӹ�������ȡ���ƺ˿��չ���ʾ��CM7_0 Ϊ�ղ����� */
void ui_pull_ctrl_snapshot(void);
#endif
