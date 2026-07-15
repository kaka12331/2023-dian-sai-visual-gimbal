#ifndef __K230_H
#define __K230R_H

#include "stm32f10x.h" 

typedef struct {
    int16_t x1;      // 靶心X
    int16_t y1;      // 靶心Y
    int16_t x2;      // 激光X
    int16_t y2;      // 激光Y
    int16_t z;       // Z轴/距离数据
    uint8_t is_updated; 
} K230_Data_t;

extern K230_Data_t g_rect_data;

void K230_UART_Handler(uint8_t byte);

#endif

