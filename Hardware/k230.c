#include "k230.h"

// 全局存储K230串口解析后的矩形坐标与深度数据
K230_Data_t g_rect_data = {0};

/**
 * @brief 串口单字节数据解析处理函数
 * 通信帧协议格式：帧头0xAA + 10字节有效数据(x1,y1,x2,y2,z，各占2字节大端模式) + 帧尾0x55
 * @param byte 串口接收的单字节数据，逐字节送入解析状态机
 * @retval 无
 */
void K230_UART_Handler(uint8_t byte)
{
    static uint8_t state = 0;          // 状态机当前运行状态
    static uint8_t data_buf[10];       // 10字节缓存区，存放一帧的有效负载数据
    static uint8_t byte_cnt = 0;       // 当前已接收有效数据字节计数

    switch (state)
    {
        case 0: // 状态0：等待帧头 0xAA
            if (byte == 0xAA)
            {
                // 检测到帧头，切换至接收数据阶段，清空字节计数器
                state = 1;
                byte_cnt = 0;
            }
            break;

        case 1: // 状态1：接收10字节坐标深度有效数据
            data_buf[byte_cnt++] = byte;
            // 10字节数据全部接收完成，切换至校验帧尾状态
            if (byte_cnt >= 10)
            {
                state = 2;
            }
            break;

        case 2: // 状态2：校验帧尾 0x55
            if (byte == 0x55)
            {
                // 帧头+数据+帧尾完整匹配，解析大端格式16位坐标数据
                // 高字节左移8位 或 低字节，组合为int16_t数值
                g_rect_data.x1 = (int16_t)((data_buf[0] << 8) | data_buf[1]);
                g_rect_data.y1 = (int16_t)((data_buf[2] << 8) | data_buf[3]);
                g_rect_data.x2 = (int16_t)((data_buf[4] << 8) | data_buf[5]);
                g_rect_data.y2 = (int16_t)((data_buf[6] << 8) | data_buf[7]);
                g_rect_data.z  = (int16_t)((data_buf[8] << 8) | data_buf[9]);

                g_rect_data.is_updated = 1; // 标记数据更新标志，上层可读取新数据
            }
            // 无论帧尾校验成功/失败，均重置状态机，等待下一帧
            state = 0;
            byte_cnt = 0;
            break;

        default: // 异常状态复位
            state = 0;
            byte_cnt = 0;
            break;
    }
}