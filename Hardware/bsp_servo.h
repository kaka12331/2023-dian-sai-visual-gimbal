#ifndef __BSP_SERVO_H
#define __BSP_SERVO_H

#include "stm32f10x.h"

// 函数声明
void Servo_Init(void);
void Servo_SetPanAngle(float angle);  // 控制底部 270° 舵机 (0 ~ 270)
void Servo_SetTiltAngle(float angle); // 控制顶部 180° 舵机 (0 ~ 180)

#endif /* __BSP_SERVO_H */
