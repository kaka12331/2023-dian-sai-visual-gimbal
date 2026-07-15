#include "bsp_pid.h"

/**
 * @brief PID 参数初始化
 */
void PID_Init(PID_TypeDef *pid, float p, float i, float d, float max_i, float max_out) {
    pid->Kp = p;
    pid->Ki = i;
    pid->Kd = d;
    pid->error = 0.0f;
    pid->last_error = 0.0f;
    pid->integral = 0.0f;
    pid->max_integral = max_i;
    pid->max_output = max_out;
}

/**
 * @brief PID 计算函数
 * @param target  目标值 (例如图像中心坐标)
 * @param current 当前值 (例如目标框中心坐标)
 * @retval 舵机角度的调整增量
 */
float PID_Calc(PID_TypeDef *pid, float target, float current) {
    float output;
    
    // 1. 计算偏差
    pid->error = target - current;
    
    // 2. 累积积分，并进行积分限幅
    pid->integral += pid->error;
    if (pid->integral > pid->max_integral) pid->integral = pid->max_integral;
    else if (pid->integral < -pid->max_integral) pid->integral = -pid->max_integral;
    
    // 3. 计算 PID 输出
    output = (pid->Kp * pid->error) + 
             (pid->Ki * pid->integral) + 
             (pid->Kd * (pid->error - pid->last_error));
             
    // 4. 更新上次偏差
    pid->last_error = pid->error;
    
    // 5. 输出限幅 (防止单次转动幅度过大，导致画面剧烈晃动丢失目标)
    if (output > pid->max_output) output = pid->max_output;
    else if (output < -pid->max_output) output = -pid->max_output;
    
    return output;
}
