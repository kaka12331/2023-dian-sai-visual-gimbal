#include "bsp_servo.h"
#include "stm32f10x.h"
/**
 * @brief  初始化舵机所用的 PWM 引脚和定时器 (TIM2, PA0, PA1)
 * @param  无
 * @retval 无
 */
void Servo_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;

    // 1. 开启时钟：GPIOA 和 TIM2
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    // 2. 配置 GPIO 引脚 (PA0: TIM2_CH1, PA1: TIM2_CH2)为复用推挽输出
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // 3. 配置 TIM2 时基
    // 系统主频 72MHz。预分频器(PSC)设为 72-1，则定时器时钟为 1MHz (即1微秒跳动1次)
    // 周期(ARR)设为 20000-1，则一个完整的 PWM 周期为 20000 微秒 = 20ms (50Hz)
    TIM_TimeBaseStructure.TIM_Period = 20000 - 1; 
    TIM_TimeBaseStructure.TIM_Prescaler = 72 - 1; 
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

    // 4. 配置 TIM2 输出比较模式 (PWM1)
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

    // 初始化通道 1 (PA0 - 底部 270度)
    TIM_OCInitStructure.TIM_Pulse = 1500; // 默认停在中间 (1.5ms)
    TIM_OC1Init(TIM2, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);

    // 初始化通道 2 (PA1 - 顶部 180度)
    TIM_OCInitStructure.TIM_Pulse = 1500; // 默认停在中间 (1.5ms)
    TIM_OC2Init(TIM2, &TIM_OCInitStructure);
    TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);

    // 5. 使能定时器
    TIM_Cmd(TIM2, ENABLE);
}

/**
 * @brief  设置底部云台舵机角度 (270度舵机)
 * @param  angle: 目标角度，范围 0.0 ~ 270.0
 */
void Servo_SetPanAngle(float angle)
{
    // 限制输入范围防止舵机卡死损坏
    if (angle < 0.0f) angle = 0.0f;
    if (angle > 360.0f) angle = 360.0f;
    
    // 占空比计算：
    // 0度 -> 0.5ms -> 500
    // 270度 -> 2.5ms -> 2500
    // 差值范围为 2000
    uint16_t pulse = 500 + (uint16_t)((angle / 360.0f) * 2000.0f);
    
    TIM_SetCompare1(TIM2, pulse); // 修改 PA0 的 PWM 占空比
}

/**
 * @brief  设置顶部云台舵机角度 (180度舵机)
 * @param  angle: 目标角度，范围 0.0 ~ 180.0
 */
void Servo_SetTiltAngle(float angle)
{
    // 限制输入范围
    if (angle < 0.0f) angle = 0.0f;
    if (angle > 180.0f) angle = 180.0f;
    
    // 占空比计算：
    // 0度 -> 0.5ms -> 500
    // 180度 -> 2.5ms -> 2500
    // 差值范围为 2000
    uint16_t pulse = 500 + (uint16_t)((angle / 180.0f) * 2000.0f);
    
    TIM_SetCompare2(TIM2, pulse); // 修改 PA1 的 PWM 占空比
}
