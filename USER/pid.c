/**
 * @file    pid.c
 * @brief   通用 PID 控制器模块实现
 * @note    纯算法实现，无硬件依赖，可移植
 */

#include "pid.h"

/**
 * @brief  初始化 PID 控制器
 */
void PID_Init(PID_Controller_t *pid, float kp, float ki, float kd, 
              float out_max, float out_min)
{
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    
    pid->Ref = 0.0f;
    pid->Fdb = 0.0f;
    pid->Out = 0.0f;
    
    pid->Integral = 0.0f;
    pid->LastError = 0.0f;
    
    pid->OutMax = out_max;
    pid->OutMin = out_min;
    pid->IntegralMax = out_max;  // 默认与输出限幅相同
    pid->IntegralMin = out_min;
}

/**
 * @brief  PID 控制器复位
 */
void PID_Reset(PID_Controller_t *pid)
{
    pid->Integral = 0.0f;
    pid->LastError = 0.0f;
    pid->Out = 0.0f;
}

/**
 * @brief  限幅函数
 */
static inline float Clamp(float value, float min, float max)
{
    if (value > max) return max;
    if (value < min) return min;
    return value;
}

/**
 * @brief  PI 控制器计算 (无微分，用于电流环/速度环)
 */
float PI_Calc(PID_Controller_t *pid, float ref, float fdb)
{
    pid->Ref = ref;
    pid->Fdb = fdb;
    
    float error = ref - fdb;
    
    /* 比例项 */
    float p_out = pid->Kp * error;
    
    /* 积分项 (带抗饱和) */
    pid->Integral += pid->Ki * error;
    pid->Integral = Clamp(pid->Integral, pid->IntegralMin, pid->IntegralMax);
    
    /* 输出 */
    pid->Out = p_out + pid->Integral;
    pid->Out = Clamp(pid->Out, pid->OutMin, pid->OutMax);
    
    return pid->Out;
}

/**
 * @brief  PID 控制器计算 (完整版，含微分)
 */
float PID_Calc(PID_Controller_t *pid, float ref, float fdb)
{
    pid->Ref = ref;
    pid->Fdb = fdb;
    
    float error = ref - fdb;
    
    /* 比例项 */
    float p_out = pid->Kp * error;
    
    /* 积分项 (带抗饱和) */
    pid->Integral += pid->Ki * error;
    pid->Integral = Clamp(pid->Integral, pid->IntegralMin, pid->IntegralMax);
    
    /* 微分项 (对反馈值微分，避免目标值突变引起的微分冲击) */
    float d_out = pid->Kd * (error - pid->LastError);
    pid->LastError = error;
    
    /* 输出 */
    pid->Out = p_out + pid->Integral + d_out;
    pid->Out = Clamp(pid->Out, pid->OutMin, pid->OutMax);
    
    return pid->Out;
}

/**
 * @brief  增量式 PID 计算
 */
float PID_Calc_Incremental(PID_Controller_t *pid, float ref, float fdb)
{
    static float last_error_1 = 0.0f;
    static float last_error_2 = 0.0f;
    
    pid->Ref = ref;
    pid->Fdb = fdb;
    
    float error = ref - fdb;
    
    /* 增量计算 */
    float delta = pid->Kp * (error - last_error_1)
                + pid->Ki * error
                + pid->Kd * (error - 2.0f * last_error_1 + last_error_2);
    
    last_error_2 = last_error_1;
    last_error_1 = error;
    
    /* 累加输出 */
    pid->Out += delta;
    pid->Out = Clamp(pid->Out, pid->OutMin, pid->OutMax);
    
    return pid->Out;
}
