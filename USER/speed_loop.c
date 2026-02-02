/**
 * @file    speed_loop.c
 * @brief   ?????? (???)
 * @note    ?????????????????? pll.c ? foc_core.c
 */

#include "speed_loop.h"

#define PI 3.14159265358f

/*============================================================================*/
/*                    ??????????                                      */
/*============================================================================*/

Speed_PID_t pid_spd = {0};
float speed_filter_const = 0.03f;

/* ?? PLL ?? */
static PLL_t compat_pll = {0};

/* ??????? */
static float theta_prev = 0.0f;

/*============================================================================*/
/*                    ????????                                          */
/*============================================================================*/

/**
 * @brief  ??????
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
    
    /* ??? PLL */
    PLL_Init(&compat_pll, 200.0f, 40000.0f);
}

/**
 * @brief  ???? (??? + ????)
 */
void Speed_Calc(float theta_now)
{
    float delta_theta = theta_now - theta_prev;
    
    /* ???? */
    if (delta_theta > PI) {
        delta_theta -= 2.0f * PI;
    }
    else if (delta_theta < -PI) {
        delta_theta += 2.0f * PI;
    }
    
    theta_prev = theta_now;
    
    /* ????? */
    float omega_inst = delta_theta / 0.00005f;
    float rpm_inst = omega_inst * 9.549296f;
    
    /* ???? */
    pid_spd.FilteredRPM = speed_filter_const * rpm_inst + 
                          (1.0f - speed_filter_const) * pid_spd.FilteredRPM;
    
    pid_spd.ActualRPM = pid_spd.FilteredRPM;
}

/**
 * @brief  ??? PID ??
 */
void Speed_PID_Calc(void)
{
    float error = pid_spd.TargetRPM - pid_spd.ActualRPM;
    
    pid_spd.Integral += pid_spd.Ki * error;
    
    /* ???? */
    if (pid_spd.Integral > pid_spd.Out_Max) pid_spd.Integral = pid_spd.Out_Max;
    if (pid_spd.Integral < pid_spd.Out_Min) pid_spd.Integral = pid_spd.Out_Min;
    
    pid_spd.Out = pid_spd.Kp * error + pid_spd.Integral;
    
    /* ???? */
    if (pid_spd.Out > pid_spd.Out_Max) pid_spd.Out = pid_spd.Out_Max;
    if (pid_spd.Out < pid_spd.Out_Min) pid_spd.Out = pid_spd.Out_Min;
}

/**
 * @brief  PLL ????
 */
void Speed_Calc_PLL(float theta_now, float Ts)
{
    PLL_Update(&compat_pll, theta_now, Ts);
    pid_spd.ActualRPM = PLL_GetSpeedRPM(&compat_pll);
}
