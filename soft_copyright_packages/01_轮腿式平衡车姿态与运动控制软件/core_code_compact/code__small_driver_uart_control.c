#include "small_driver_uart_control.h"
small_device_value_struct motor_value;
int16 Left_Motor_Pwm = 0;
int16 Right_Motor_Pwm = 0;
uint8 Motor_Switch = 1;
uint8 Motor_OpenFlag = 0;
int16 Left_Motor_Speed = 0;
int16 Right_Motor_Speed = 0;
void uart_control_callback(void)
{
    uint8 receive_data;
    if (uart_query_byte(SMALL_DRIVER_UART, &receive_data))
    {
        if (receive_data == 0xA5 && motor_value.receive_data_buffer[0] != 0xA5)
        {
            motor_value.receive_data_count = 0;
        }
        motor_value.receive_data_buffer[motor_value.receive_data_count++] = receive_data;
        if (motor_value.receive_data_count >= 7)
        {
            if (motor_value.receive_data_buffer[0] == 0xA5)
            {
                motor_value.sum_check_data = 0;
                for (int i = 0; i < 6; i++)
                {
                    motor_value.sum_check_data += motor_value.receive_data_buffer[i];
                }
                if (motor_value.sum_check_data == motor_value.receive_data_buffer[6])
                {
                    if (motor_value.receive_data_buffer[1] == 0x02)
                    {
                        motor_value.receive_left_speed_data = (((int)motor_value.receive_data_buffer[2] << 8) | (int)motor_value.receive_data_buffer[3]);
                        motor_value.receive_right_speed_data = (((int)motor_value.receive_data_buffer[4] << 8) | (int)motor_value.receive_data_buffer[5]);
                    }
                    motor_value.receive_data_count = 0;
                    memset(motor_value.receive_data_buffer, 0, 7);
                }
                else
                {
                    motor_value.receive_data_count = 0;
                    memset(motor_value.receive_data_buffer, 0, 7);
                }
            }
            else
            {
                motor_value.receive_data_count = 0;
                memset(motor_value.receive_data_buffer, 0, 7);
            }
        }
    }
}
void small_driver_set_duty(int16 left_duty, int16 right_duty)
{
    motor_value.send_data_buffer[0] = 0xA5;
    motor_value.send_data_buffer[1] = 0X01;
    motor_value.send_data_buffer[2] = (uint8)((left_duty & 0xFF00) >> 8);
    motor_value.send_data_buffer[3] = (uint8)(left_duty & 0x00FF);
    motor_value.send_data_buffer[4] = (uint8)((right_duty & 0xFF00) >> 8);
    motor_value.send_data_buffer[5] = (uint8)(right_duty & 0x00FF);
    motor_value.send_data_buffer[6] = 0;
    for (int i = 0; i < 6; i++)
    {
        motor_value.send_data_buffer[6] += motor_value.send_data_buffer[i];
    }
    uart_write_buffer(SMALL_DRIVER_UART, motor_value.send_data_buffer, 7);
}
void small_driver_get_speed(void)
{
    motor_value.send_data_buffer[0] = 0xA5;
    motor_value.send_data_buffer[1] = 0X02;
    motor_value.send_data_buffer[2] = 0x00;
    motor_value.send_data_buffer[3] = 0x00;
    motor_value.send_data_buffer[4] = 0x00;
    motor_value.send_data_buffer[5] = 0x00;
    motor_value.send_data_buffer[6] = 0xA7;
    uart_write_buffer(SMALL_DRIVER_UART, motor_value.send_data_buffer, 7);
}
void small_driver_init(void)
{
    memset(motor_value.send_data_buffer, 0, 7);
    memset(motor_value.receive_data_buffer, 0, 7);
    motor_value.receive_data_count = 0;
    motor_value.sum_check_data = 0;
    motor_value.receive_right_speed_data = 0;
    motor_value.receive_left_speed_data = 0;
}
void small_driver_uart_init(void)
{
    uart_init(SMALL_DRIVER_UART, SMALL_DRIVER_BAUDRATE, SMALL_DRIVER_RX, SMALL_DRIVER_TX);
    uart_rx_interrupt(SMALL_DRIVER_UART, 1);
    small_driver_init();
    small_driver_set_duty(0, 0);
    small_driver_get_speed();
}
