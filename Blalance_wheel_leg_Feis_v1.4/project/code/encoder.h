#ifndef _ENCODER_H_
#define _ENCODER_H_

#define ENCODER1                     	(TC_CH09_ENCODER)                      // 左编码器接口       
#define ENCODER1_QUADDEC_A             	(TC_CH09_ENCODER_CH1_P05_0)            // A相引脚                      
#define ENCODER1_QUADDEC_B            	(TC_CH09_ENCODER_CH2_P05_1)            // B相引脚                        
                                                                                
#define ENCODER2                     	(TC_CH07_ENCODER)                      // 右编码器接口   
#define ENCODER2_QUADDEC_A            	(TC_CH07_ENCODER_CH1_P02_0)            // A相引脚                  
#define ENCODER2_QUADDEC_B            	(TC_CH07_ENCODER_CH2_P02_1)            // B相引脚          

typedef struct{

  int L_Speed;
  int R_Speed;

}Motor_Inf_Typedef;
extern Motor_Inf_Typedef Mf;
void Encoder();
void EncoderInit();

#endif
