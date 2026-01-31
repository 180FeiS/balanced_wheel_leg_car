#include "vofa.h"

const uint8_t vofa_justfloat_frame_tail[4] = {0x00, 0x00, 0x80, 0x7f};
#define Wired_Mode 1 // 0-有线串口    1-无线串口
float ReadBuf_Pid = 0;

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     将一组数据以justfloat格式发送到VOFA
// 参数说明     uart_index_enum * UARTx     串口号
// 参数说明     uint32_t Count              发送数据个数
// 参数说明     float Data                  发送数据
// 返回参数     void
// 使用示例     SendDataStreamToVOFA(1,Car_AimSpeed);
//             SendDataStreamToVOFA(3,data1,data2,data3);
// 备注信息     V1.0.0  调用时传入的数据要强转成float类型
//             V1.0.1  优先写好有线串口调试函数         2024年7月26日
//-------------------------------------------------------------------------------------------------------------------
void SendDataStreamToVOFA(uint32_t Count, float Data, ...)
{
    volatile float Temp = Data;
    va_list Args;
    va_start(Args, Data);

#if Wired_Mode
    wireless_uart_send_buffer((uint8 *)&Data, sizeof(float));
#else
    uart_write_buffer(UART_1, (uint8 *)&Data, sizeof(float));
#endif

    for (uint32_t i = 1; i < Count; i++)
    {
        Temp = va_arg(Args, double);
#if Wired_Mode
        wireless_uart_send_buffer((uint8 *)&Temp, sizeof(float));
#else
        uart_write_buffer(UART_1, (uint8 *)&Temp, sizeof(float));
#endif
    }
#if Wired_Mode
    wireless_uart_send_buffer((uint8 *)vofa_justfloat_frame_tail, 4);
#else
    uart_write_buffer(UART_1, (uint8 *)vofa_justfloat_frame_tail, 4);
#endif
    va_end(Args);
}

uint32 WirelessUart_ReadBuff_Count = 0;     // 读取无线串口数据长度
uint8 WirelessUart_ReadBuff_Data[64] = {0}; // 读取无线串口缓冲数组
uint8 Menu_command = 0;                     // 菜单进行指令
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     从上位机接受数据
// 参数说明     void
// 返回参数     void
// 使用示例     ReadDataFromPc(WirelessUart_ReadBuff_Data,WirelessUart_ReadBuff_Count);
// 备注信息     V1.0.0  调用时传入的数据要强转成float类型
//             V1.0.1  优先写好有线串口调试函数         2024年7月26日
//             V1.1.2  pid传参功能完善，需要加前缀a，可任意更改 2024年7月31日
//-------------------------------------------------------------------------------------------------------------------
void ReadDataFromPc()
{
// 无线串口读取数据
#if Wired_Mode
    WirelessUart_ReadBuff_Count = wireless_uart_read_buffer(WirelessUart_ReadBuff_Data, sizeof(WirelessUart_ReadBuff_Data));

    if (WirelessUart_ReadBuff_Count)
    {
        Menu_command = WirelessUart_ReadBuff_Data[0];
        if (WirelessUart_ReadBuff_Data[0] >= '0' && WirelessUart_ReadBuff_Data[0] <= '9')
        {
            ReadBuf_Pid = atof(&WirelessUart_ReadBuff_Data[0]);
            //Flash.Flash_state = FLASH_READ;
            //Flash.Flash_Error = FLASH_RUNNING;
        }
        if (WirelessUart_ReadBuff_Data[0] == 's')
        {
            //Car_AimSpeed = (int16)atof(&WirelessUart_ReadBuff_Data[1]);
        }

        memset(WirelessUart_ReadBuff_Data, 0, WirelessUart_ReadBuff_Count); // 清空消息区
        WirelessUart_ReadBuff_Count = 0;
    }

#else // 从Debug串口中读取数据
    WirelessUart_ReadBuff_Count = debug_read_ring_buffer(WirelessUart_ReadBuff_Data, sizeof(WirelessUart_ReadBuff_Data));
    if (WirelessUart_ReadBuff_Count)
    {
        // debug_send_buffer(WirelessUart_ReadBuff_Data,WirelessUart_ReadBuff_Count);
        debug_send_buffer(ReadPos, sizeof(ReadPos));
        Menu_command = WirelessUart_ReadBuff_Data[0];
        if (WirelessUart_ReadBuff_Data[0] >= '0' && WirelessUart_ReadBuff_Data[0] <= '9')
        {
            ReadBuf_Pid = atof(&WirelessUart_ReadBuff_Data[0]);
            //Flash_Read_State = FLASH_RUNNING;
        }
        memset(WirelessUart_ReadBuff_Data, 0, WirelessUart_ReadBuff_Count); // 清空消息区
        WirelessUart_ReadBuff_Count = 0;
    }
#endif
}
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     发送数据
// 参数说明     void
// 返回参数     void
// 使用示例     SendDataToVofa();
// 备注信息     V1.0.0
//-------------------------------------------------------------------------------------------------------------------
void SendDataToVofa()
{
    // SendDataStreamToVOFA(4,(float)Gyro.pitch,(float)0,(float)Left_Motor_Pwm,(float)Gyro.gyro_y);
}