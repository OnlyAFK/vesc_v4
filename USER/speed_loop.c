#include "speed_loop.h"
#include "foc_math.h" // 包含 PI 定义
#include "main.h"     // 获取 Ts (采样周期)


#define PI 3.14159265358f

Speed_PID_t pid_spd;
float speed_filter_const = 0.03f; // 滤波系数，越小滤波越强，延迟越大

// 内部变量用于差分计算
static float theta_prev = 0.0f;

void Speed_Loop_Init(void)
{
    // 初始化 PID 参数 (需要根据实际带载调试)
    pid_spd.Kp = 0.03f;       // 速度环 Kp 通常比电流环小很多
    pid_spd.Ki = 0.0003f;      
    pid_spd.Out_Max = 3.0f;   // 最大限制 5A 电流
    pid_spd.Out_Min = -3.0f;
    pid_spd.Integral = 0.0f;
    pid_spd.TargetRPM = 0.0f;
}

// 1. 计算速度 (包含过零处理和滤波)
// theta_now: 当前机械角度 (0 ~ 6.28)
void Speed_Calc(float theta_now)
{
    float delta_theta = theta_now - theta_prev;

    // --- 过零点处理 ---
    // 如果变化量大于 PI (180度)，说明跨过了 0 点
    if (delta_theta > PI)
    {
        delta_theta -= 2.0f * PI; // 例如 0.1 - 6.0 = -5.9 -> -5.9 + 6.28 = 0.38
    }
    else if (delta_theta < -PI)
    {
        delta_theta += 2.0f * PI;
    }
    
    // 保存当前角度供下次使用
    theta_prev = theta_now;

    // --- 计算瞬时速度 (rad/s) ---
    // omega = d_theta / dt
    // 假设调用频率是 20kHz (Ts = 0.00005s)
    // 这里的 Ts 必须非常准确，否则速度值不对
    float omega_inst = delta_theta / 0.00005f; 

    // 转换为 RPM: RPM = (rad/s) * 60 / (2*PI)
    float rpm_inst = omega_inst * 9.549296f;

    // --- 低通滤波 (LPF) ---
    // y[k] = alpha * x[k] + (1-alpha) * y[k-1]
    pid_spd.FilteredRPM = speed_filter_const * rpm_inst + 
                          (1.0f - speed_filter_const) * pid_spd.FilteredRPM;
    
    pid_spd.ActualRPM = pid_spd.FilteredRPM;
}

// 2. 速度环 PID 计算
void Speed_PID_Calc(void)
{
    float error = pid_spd.TargetRPM - pid_spd.ActualRPM;

    // 积分项
    pid_spd.Integral += pid_spd.Ki * error;
    
    // 积分限幅 (抗饱和) - 非常重要！
    if (pid_spd.Integral > pid_spd.Out_Max) pid_spd.Integral = pid_spd.Out_Max;
    if (pid_spd.Integral < pid_spd.Out_Min) pid_spd.Integral = pid_spd.Out_Min;

    // 比例项 + 积分项
    pid_spd.Out = pid_spd.Kp * error + pid_spd.Integral;

    // 输出限幅
    if (pid_spd.Out > pid_spd.Out_Max) pid_spd.Out = pid_spd.Out_Max;
    if (pid_spd.Out < pid_spd.Out_Min) pid_spd.Out = pid_spd.Out_Min;
}

