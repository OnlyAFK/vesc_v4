#include "current_loop.h"



// 声明两个 PID 对象：一个给 D轴，一个给 Q轴
PI_Controller_t pid_id = {0};
PI_Controller_t pid_iq = {0};

void current_pi_init(void)
{
    // D轴 PID (目标是把电流控制为 0)
    pid_id.Kp = 0.07037f;       // 试探值：1V/A
    pid_id.Ki = 0.01423f;      // 试探值
    pid_id.Out_Max = 12.0f; // 母线电压 12V
    pid_id.Out_Min = -12.0f;
    pid_id.Integral = 0.0f;
    pid_id.Ref = 0.0f;      // D轴目标永远是 0 (对于普通电机)

    // Q轴 PID (目标是产生力矩)
    pid_iq.Kp = 0.07037f;      
    pid_iq.Ki = 0.01423f;
    pid_iq.Out_Max = 12.0f;
    pid_iq.Out_Min = -12.0f;
    pid_iq.Integral = 0.0f;
    pid_iq.Ref = 0.0f;      // 初始目标为0，安全
}

void current_calc(PI_Controller_t *pPI)
{
    // 1. 计算误差
    float Error = pPI->Ref - pPI->Fdb;
    
    // 2. 计算比例项 (P)
    float Up = pPI->Kp * Error;
    
    // 3. 计算积分项 (I)
    pPI->Integral += pPI->Ki * Error;
    
    // [关键] 积分抗饱和 (Clamp Integral)
    // 如果积分项太大，导致输出饱和，系统会失控。必须限制积分项的范围。
    // 这里简单的限制在输出范围内。更高级的做法是动态钳位。
    if (pPI->Integral > pPI->Out_Max) pPI->Integral = pPI->Out_Max;
    if (pPI->Integral < pPI->Out_Min) pPI->Integral = pPI->Out_Min;
    
    // 4. 计算总输出
    pPI->Out = Up + pPI->Integral;
    
    // 5. 输出限幅 (Output Saturation)
    if (pPI->Out > pPI->Out_Max) pPI->Out = pPI->Out_Max;
    if (pPI->Out < pPI->Out_Min) pPI->Out = pPI->Out_Min;
}
