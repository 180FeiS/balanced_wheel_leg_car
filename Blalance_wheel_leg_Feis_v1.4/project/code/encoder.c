#include "zf_common_headfile.h"
#include "encoder.h"
Motor_Inf_Typedef Mf;
void EncoderInit(){
  
    encoder_dir_init(ENCODER1, ENCODER1_QUADDEC_A, ENCODER1_QUADDEC_B);       // 初始化编码器模块，设置引脚，设置滤波，设置四倍频模式
    encoder_dir_init(ENCODER2, ENCODER2_QUADDEC_A, ENCODER2_QUADDEC_B);       // 初始化编码器模块，设置引脚，设置滤波，设置四倍频模式
  
}

void Encoder(){
        Mf.L_Speed = encoder_get_count(ENCODER1);
        encoder_clear_count(ENCODER1);
        Mf.R_Speed = -encoder_get_count(ENCODER2);
        encoder_clear_count(ENCODER2);



}
