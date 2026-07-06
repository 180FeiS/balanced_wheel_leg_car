#include "small_driver_uart_control.h"
small_device_value_struct motor_value; // ����ͨѶ�����ṹ��
int16 Left_Motor_Pwm = 0;   // �������pwm
int16 Right_Motor_Pwm = 0;  // �ҵ�����pwm
uint8 Motor_Switch = 1;     // �������
uint8 Motor_OpenFlag = 0;   // ����ϵ��־λ
int16 Left_Motor_Speed = 0; // �����ٶ�
int16 Right_Motor_Speed = 0; // �ҵ���ٶ�
//-------------------------------------------------------------------------------------------------------------------
// �������     ��ˢ���� ���ڽ��ջص�����
// ����˵��     void
// ���ز���     void
// ʹ��ʾ��     uart_control_callback(1000, -1000);
// ��ע��Ϣ     ���ڽ������յ����ٶ�����  �ú�����Ҫ�ڶ�Ӧ�Ĵ��ڽ����ж��е���
//-------------------------------------------------------------------------------------------------------------------
void uart_control_callback(void)
{
    uint8 receive_data; // ������ʱ����
    if (uart_query_byte(SMALL_DRIVER_UART, &receive_data)) // ���մ�������
    {
        if (receive_data == 0xA5 && motor_value.receive_data_buffer[0] != 0xA5) // �ж��Ƿ��յ�֡ͷ ���� ��ǰ�����������Ƿ���ȷ����֡ͷ
        {
            motor_value.receive_data_count = 0; // δ�յ�֡ͷ����δ��ȷ����֡ͷ�����½���
        }
        motor_value.receive_data_buffer[motor_value.receive_data_count++] = receive_data; // ���洮������
        if (motor_value.receive_data_count >= 7) // �ж��Ƿ���յ�ָ������������
        {
            if (motor_value.receive_data_buffer[0] == 0xA5) // �ж�֡ͷ�Ƿ���ȷ
            {
                motor_value.sum_check_data = 0; // ���У��λ����
                for (int i = 0; i < 6; i++)
                {
                    motor_value.sum_check_data += motor_value.receive_data_buffer[i]; // ���¼���У��λ
                }
                if (motor_value.sum_check_data == motor_value.receive_data_buffer[6]) // У������׼ȷ��
                {
                    if (motor_value.receive_data_buffer[1] == 0x02) // �ж��Ƿ���ȷ���յ� �ٶ���� ������
                    {
                        motor_value.receive_left_speed_data = (((int)motor_value.receive_data_buffer[2] << 8) | (int)motor_value.receive_data_buffer[3]); // ��������ת������
                        motor_value.receive_right_speed_data = (((int)motor_value.receive_data_buffer[4] << 8) | (int)motor_value.receive_data_buffer[5]); // ����Ҳ���ת������
                    }
                    motor_value.receive_data_count = 0; // �������������ֵ
                    memset(motor_value.receive_data_buffer, 0, 7); // �������������
                }
                else
                {
                    motor_value.receive_data_count = 0; // �������������ֵ
                    memset(motor_value.receive_data_buffer, 0, 7); // �������������
                }
            }
            else
            {
                motor_value.receive_data_count = 0; // �������������ֵ
                memset(motor_value.receive_data_buffer, 0, 7); // �������������
            }
        }
    }
}
//-------------------------------------------------------------------------------------------------------------------
// �������     ��ˢ���� ���õ��ռ�ձ�
// ����˵��     left_duty       �����ռ�ձ�  ��Χ -10000 ~ 10000  ����Ϊ��ת
// ����˵��     right_duty      �Ҳ���ռ�ձ�  ��Χ -10000 ~ 10000  ����Ϊ��ת
// ���ز���     void
// ʹ��ʾ��     small_driver_set_duty(1000, -1000);
// ��ע��Ϣ
//-------------------------------------------------------------------------------------------------------------------
void small_driver_set_duty(int16 left_duty, int16 right_duty)
{
    motor_value.send_data_buffer[0] = 0xA5; // ����֡ͷ
    motor_value.send_data_buffer[1] = 0X01; // ���ù�����
    motor_value.send_data_buffer[2] = (uint8)((left_duty & 0xFF00) >> 8); // ��� ���ռ�ձ� �ĸ߰�λ
    motor_value.send_data_buffer[3] = (uint8)(left_duty & 0x00FF); // ��� ���ռ�ձ� �ĵͰ�λ
    motor_value.send_data_buffer[4] = (uint8)((right_duty & 0xFF00) >> 8); // ��� �Ҳ�ռ�ձ� �ĸ߰�λ
    motor_value.send_data_buffer[5] = (uint8)(right_duty & 0x00FF); // ��� �Ҳ�ռ�ձ� �ĵͰ�λ
    motor_value.send_data_buffer[6] = 0; // ��У�����
    for (int i = 0; i < 6; i++)
    {
        motor_value.send_data_buffer[6] += motor_value.send_data_buffer[i]; // ����У��λ
    }
    uart_write_buffer(SMALL_DRIVER_UART, motor_value.send_data_buffer, 7); // ��������ռ�ձȵ� �ֽڰ� ����
}
//-------------------------------------------------------------------------------------------------------------------
// �������     ��ˢ���� ��ȡ�ٶ���Ϣ
// ����˵��     void
// ���ز���     void
// ʹ��ʾ��     small_driver_get_speed();
// ��ע��Ϣ     ���跢��һ�� ���������ڷ����ٶ���Ϣ(Ĭ��10ms)
//-------------------------------------------------------------------------------------------------------------------
void small_driver_get_speed(void)
{
    motor_value.send_data_buffer[0] = 0xA5; // ����֡ͷ
    motor_value.send_data_buffer[1] = 0X02; // ���ù�����
    motor_value.send_data_buffer[2] = 0x00; // ����λ���
    motor_value.send_data_buffer[3] = 0x00; // ����λ���
    motor_value.send_data_buffer[4] = 0x00; // ����λ���
    motor_value.send_data_buffer[5] = 0x00; // ����λ���
    motor_value.send_data_buffer[6] = 0xA7; // ����У��λ
    uart_write_buffer(SMALL_DRIVER_UART, motor_value.send_data_buffer, 7); // ���ͻ�ȡת�����ݵ� �ֽڰ� ����
}
//-------------------------------------------------------------------------------------------------------------------
// �������     ��ˢ���� ������ʼ��
// ����˵��     void
// ���ز���     void
// ʹ��ʾ��     small_driver_init();
// ��ע��Ϣ
//-------------------------------------------------------------------------------------------------------------------
void small_driver_init(void)
{
    memset(motor_value.send_data_buffer, 0, 7); // �������������
    memset(motor_value.receive_data_buffer, 0, 7); // �������������
    motor_value.receive_data_count = 0;
    motor_value.sum_check_data = 0;
    motor_value.receive_right_speed_data = 0;
    motor_value.receive_left_speed_data = 0;
}
//-------------------------------------------------------------------------------------------------------------------
// �������     ��ˢ���� ����ͨѶ��ʼ��
// ����˵��     void
// ���ز���     void
// ʹ��ʾ��     small_driver_uart_init();
// ��ע��Ϣ
//-------------------------------------------------------------------------------------------------------------------
void small_driver_uart_init(void)
{
    uart_init(SMALL_DRIVER_UART, SMALL_DRIVER_BAUDRATE, SMALL_DRIVER_RX, SMALL_DRIVER_TX); // ���ڳ�ʼ��
    uart_rx_interrupt(SMALL_DRIVER_UART, 1); // ʹ�ܴ��ڽ����ж�
    small_driver_init(); // �ṹ�������ʼ��
    small_driver_set_duty(0, 0); // ����0ռ�ձ�
    small_driver_get_speed(); // ��ȡʵʱ�ٶ�����
}
