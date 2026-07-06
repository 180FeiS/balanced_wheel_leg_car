#ifndef SMALL_DRIVER_UART_CONTROL_H_
#define SMALL_DRIVER_UART_CONTROL_H_
#include "zf_common_headfile.h"
#define SMALL_DRIVER_UART (UART_2)
#define SMALL_DRIVER_BAUDRATE (460800)
#define SMALL_DRIVER_RX (UART2_TX_P10_1)
#define SMALL_DRIVER_TX (UART2_RX_P10_0)
#define MOTOR_ON (1)
#define MOTOR_OFF (0)
typedef struct
{
    uint8 send_data_buffer[7];
    uint8 receive_data_buffer[7];
    uint8 receive_data_count;
    uint8 sum_check_data;
    int16 receive_left_speed_data;
    int16 receive_right_speed_data;
} small_device_value_struct;
extern small_device_value_struct motor_value;
extern int16 Left_Motor_Pwm ;
extern int16 Right_Motor_Pwm;
extern uint8 Motor_Switch ;
extern uint8 Motor_OpenFlag ;
extern int16 Left_Motor_Speed ;
extern int16 Right_Motor_Speed ;
void uart_control_callback(void);
void small_driver_set_duty(int16 left_duty, int16 right_duty);
void small_driver_get_speed(void);
void small_driver_uart_init(void);
#endif
