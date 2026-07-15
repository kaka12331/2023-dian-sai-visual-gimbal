#ifndef __BSP_USART_H
#define __BSP_USART_H

#include "stm32f10x.h"

// 函数声明
void USART1_Init(uint32_t baudrate);

// 如果你需要通过串口向电脑打印调试信息，可以解除下面这个函数的注释
// void USART1_SendByte(uint8_t Byte); 

#endif /* __BSP_USART_H */
