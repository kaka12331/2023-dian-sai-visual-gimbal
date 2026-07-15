#include "bsp_usart.h"

/**
 * @brief  初始化 USART1 及其接收中断
 * @param  baudrate: 串口波特率 (例如: 115200)
 * @retval 无
 */
void USART1_Init(uint32_t baudrate)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    // 1. 开启时钟：GPIOA 和 USART1
    // 注意：USART1 挂载在高速总线 APB2 上
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);

    // 2. 配置 GPIO 引脚
    // PA9 -> USART1_TX (复用推挽输出)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // PA10 -> USART1_RX (浮空输入或上拉输入)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; 
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // 3. 配置 USART1 参数
    USART_InitStructure.USART_BaudRate = baudrate;                                  // 波特率
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;                     // 数据位 8位
    USART_InitStructure.USART_StopBits = USART_StopBits_1;                          // 停止位 1位
    USART_InitStructure.USART_Parity = USART_Parity_No;                             // 无校验位
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // 无硬件流控
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;                 // 收发模式
    USART_Init(USART1, &USART_InitStructure);

    // 4. 配置 NVIC (嵌套向量中断控制器)
    // 注意：如果你的整个工程还没有配置过中断分组，这里配置一次即可
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); // 设置中断优先级分组为2 (2位抢占，2位响应)

    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;           // USART1 中断通道
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;   // 抢占优先级 (0~3)
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;          // 响应优先级 (0~3)
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;             // 使能中断通道
    NVIC_Init(&NVIC_InitStructure);

    // 5. 开启串口接收中断，并使能串口
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); // 开启 RXNE (接收寄存器非空) 中断
    USART_Cmd(USART1, ENABLE);                     // 使能 USART1
}

/* 备用：串口发送单字节函数 (方便调试)
void USART1_SendByte(uint8_t Byte)
{
    USART_SendData(USART1, Byte);
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET); // 等待发送完成
}
*/
