#include "stm32f10x.h"
#include "bsp_servo.h"
#include "bsp_pid.h"
#include "k230.h" 
#include "OLED.h" // 确保这里包含你的OLED头文件名称 (根据你的实际文件名修改)
#include "bsp_usart.h" // 确保这里包含你的OLED头文件名称 (根据你的实际文件名修改)
// 定义全局的 PID 结构体
PID_TypeDef pid_pan;   // 控制底部左右 (Yaw)
PID_TypeDef pid_tilt;  // 控制顶部上下 (Pitch)

// 舵机当前角度变量
float current_pan_angle = 180.0f; // 初始在 360 度的中间
float current_tilt_angle = 90.0f; // 初始在 180 度的中间

// 声明外部变量 (来自你的 K230 接收文件)
extern K230_Data_t g_rect_data;

int main(void)
{
    // 1. 硬件初始化
    SystemInit();
    Servo_Init();
    OLED_Init(); // 初始化 OLED 显示屏
    USART1_Init(115200); 
    
    // 2. PID 参数初始化 
    PID_Init(&pid_pan,  0.012f, 0.0f, 0.005f, 10.0f, 2.0f); 
    PID_Init(&pid_tilt, 0.005f, 0.0f, 0.005f, 10.0f, 2.0f);

    // 3. 舵机初始回中
    Servo_SetPanAngle(current_pan_angle);
    Servo_SetTiltAngle(current_tilt_angle);

    // 4. OLED 静态UI框架初始化 (只刷一次，避免闪屏)
    OLED_Clear();
    OLED_ShowString(1, 1, "T: X=    Y=   "); // T 代表 Target (目标)
    OLED_ShowString(2, 1, "A: X=    Y=   "); // A 代表 Actual (实际)
    OLED_ShowString(3, 1, "E: X=    Y=   "); // E 代表 Error (误差)
    OLED_ShowString(4, 1, "S: P=    T=   "); // S 代表 Servo (舵机角度)

    while(1)
    {
        // 检查是否接收到 K230 发来的新一帧数据
        if (g_rect_data.is_updated == 1)
        {
            g_rect_data.is_updated = 0; // 清除更新标志位
            
            // 提取数据并转为有符号整数，方便计算误差
            int16_t actual_x = g_rect_data.x1;
            int16_t actual_y = g_rect_data.y1;
            int16_t target_x = g_rect_data.x2;
            int16_t target_y = g_rect_data.y2;
            
            // 计算误差 (目标 - 实际)
            int16_t err_x = target_x - actual_x;
            int16_t err_y = target_y - actual_y;
            
            // ========= 1. 运算与控制 =========
            float pan_adj = PID_Calc(&pid_pan, (float)target_x, (float)actual_x);
            float tilt_adj = PID_Calc(&pid_tilt, (float)target_y, (float)actual_y);
            
            // 更新舵机角度 (注意方向，如果反了修改此处的 += 或 -=)
            current_pan_angle += pan_adj;  
            current_tilt_angle += tilt_adj; 
            
            // 软件限幅
            if (current_pan_angle > 360.0f) current_pan_angle = 360.0f;
            if (current_pan_angle < 0.0f)   current_pan_angle = 0.0f;
            if (current_tilt_angle > 180.0f) current_tilt_angle = 180.0f;
            if (current_tilt_angle < 0.0f)   current_tilt_angle = 0.0f;
            
            // 驱动舵机
            Servo_SetPanAngle(current_pan_angle);
            Servo_SetTiltAngle(current_tilt_angle);
            
            // ========= 2. 刷新 OLED 屏幕 =========
            // 第1行：显示目标值 (从第6列和第13列开始写，预留3位数字)
            OLED_ShowNum(1, 6, target_x, 3);
            OLED_ShowNum(1, 13, target_y, 3);
            
            // 第2行：显示实际值
            OLED_ShowNum(2, 6, actual_x, 3);
            OLED_ShowNum(2, 13, actual_y, 3);
            
            // 第3行：显示误差值 (使用你的带符号函数)
            // 注意：ShowSignedNum 的 Length 参数不包含符号位。
            // 放在第5列，第5列显示 +/- 号，6~8列显示数字，刚好和上面对齐！
            OLED_ShowSignedNum(3, 5, err_x, 3);
            OLED_ShowSignedNum(3, 12, err_y, 3);
            
            // 第4行：显示舵机当前角度 (强转为整型显示)
            OLED_ShowNum(4, 6, (uint32_t)current_pan_angle, 3);
            OLED_ShowNum(4, 13, (uint32_t)current_tilt_angle, 3);
        }
    }
}

/**
 * @brief 串口接收中断服务函数
 */
void USART1_IRQHandler(void)
{
    if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
        uint8_t res = USART_ReceiveData(USART1);
        K230_UART_Handler(res); // 喂给状态机解析
        USART_ClearITPendingBit(USART1, USART_IT_RXNE);
    }
}
