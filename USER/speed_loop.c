/**
 * @file    speed_loop.c
 * @brief   速度环控制器 (兼容层)
 * @note    此文件保留用于向后兼容，新代码请使用 pll.c 和 foc_core.c
 */

#include "speed_loop.h"

#define PI 3.14159265358f

/*============================================================================*/
/*                    兼容旧代码的全局变量                                      */
/*============================================================================*/

Speed_PID_t pid_spd = {0};
float speed_filter_const = 0.03f;

/* 内部 PLL 实例 */
static PLL_t compat_pll = {0};

/* 差分法内部变量 */
static float theta_prev = 0.0f;

/*============================================================================*/
/*                    兼容旧代码的函数                                          */
/*============================================================================*/

/**
 * @brief  初始化速度环
 */
void Speed_Loop_Init(void)
{
    pid_spd.Kp = 0.006f;
    pid_spd.Ki = 0.00001f;
    pid_spd.Out_Max = 3.0f;
    pid_spd.Out_Min = -3.0f;
    pid_spd.Integral = 0.0f;
    pid_spd.TargetRPM = 0.0f;
    pid_spd.ActualRPM = 0.0f;
    pid_spd.FilteredRPM = 0.0f;
    pid_spd.Out = 0.0f;
    
    /* 初始化 PLL */
    PLL_Init(&compat_pll, 200.0f, 40000.0f);
}

/**
 * @brief  速度计算 (差分法 + 低通滤波)
 */
void Speed_Calc(float theta_now)
{
    float delta_theta = theta_now - theta_prev;
    
    /* 跨零处理 */
    if (delta_theta > PI) {
        delta_theta -= 2.0f * PI;
    }
    else if (delta_theta < -PI) {
        delta_theta += 2.0f * PI;
    }
    
    theta_prev = theta_now;
    
    /* 计算角速度 */
    float omega_inst = delta_theta / 0.00005f;
    float rpm_inst = omega_inst * 9.549296f;
    
    /* 低通滤波 */
    pid_spd.FilteredRPM = speed_filter_const * rpm_inst + 
                          (1.0f - speed_filter_const) * pid_spd.FilteredRPM;
    
    pid_spd.ActualRPM = pid_spd.FilteredRPM;
}

/**
 * @brief  速度环 PID 计算
 */
void Speed_PID_Calc(void)
{
    float error = pid_spd.TargetRPM - pid_spd.ActualRPM;
    
    pid_spd.Integral += pid_spd.Ki * error;
    
    /* 积分限幅 */
    if (pid_spd.Integral > pid_spd.Out_Max) pid_spd.Integral = pid_spd.Out_Max;
    if (pid_spd.Integral < pid_spd.Out_Min) pid_spd.Integral = pid_spd.Out_Min;
    
    pid_spd.Out = pid_spd.Kp * error + pid_spd.Integral;
    
    /* 输出限幅 */
    if (pid_spd.Out > pid_spd.Out_Max) pid_spd.Out = pid_spd.Out_Max;
    if (pid_spd.Out < pid_spd.Out_Min) pid_spd.Out = pid_spd.Out_Min;
}

/**
 * @brief  PLL 速度计算
 */
void Speed_Calc_PLL(float theta_now, float Ts)
{
    PLL_Update(&compat_pll, theta_now, Ts);
    pid_spd.ActualRPM = PLL_GetSpeedRPM(&compat_pll);
}
