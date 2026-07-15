#ifndef __BSP_PID_H
#define __BSP_PID_H

#include "stm32f10x.h"

// PID 结构体
typedef struct {
    float Kp;           // 比例系数
    float Ki;           // 积分系数
    float Kd;           // 微分系数
    
    float error;        // 当前偏差
    float last_error;   // 上次偏差
    float integral;     // 积分累积
    
    float max_integral; // 积分限幅 (防止积分饱和)
    float max_output;   // 输出限幅 (限制每次舵机的最大转动速度)
} PID_TypeDef;

// 函数声明
void PID_Init(PID_TypeDef *pid, float p, float i, float d, float max_i, float max_out);
float PID_Calc(PID_TypeDef *pid, float target, float current);

#endif /* __BSP_PID_H */
