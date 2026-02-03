/**
 * @file    pll.c
 * @brief   PLL 锁相环速度估算模块实现
 * @note    纯算法实现，无硬件依赖，可移植
 */

#include "pll.h"

/* rad/s 转 RPM 的系数: 60 / (2*PI) = 9.5493 */
#define RAD_S_TO_RPM    9.5492965855f

/**
 * @brief  角度归一化到 [0, 2π)
 */
static inline float NormalizeAngle(float angle)
{
    while (angle >= PLL_2PI) angle -= PLL_2PI;
    while (angle < 0.0f)     angle += PLL_2PI;
    return angle;
}

/**
 * @brief  角度误差归一化到 [-π, π)
 */
static inline float NormalizeAngleError(float error)
{
    if (error > PLL_PI)  error -= PLL_2PI;
    if (error < -PLL_PI) error += PLL_2PI;
    return error;
}

/**
 * @brief  初始化 PLL
 */
void PLL_Init(PLL_t *pll, float kp, float ki)
{
    pll->Kp = kp;
    pll->Ki = ki;
    
    pll->AngleEst = 0.0f;
    pll->SpeedEst = 0.0f;
    pll->SpeedRPM = 0.0f;
    pll->SpeedRadS = 0.0f;
}

/**
 * @brief  PLL 复位
 */
void PLL_Reset(PLL_t *pll)
{
    pll->AngleEst = 0.0f;
    pll->SpeedEst = 0.0f;
    pll->SpeedRPM = 0.0f;
    pll->SpeedRadS = 0.0f;
}

/**
 * @brief  PLL 速度估算更新
 * @note   使用二阶 PLL 结构:
 *         - 角度误差 = theta_meas - theta_est
 *         - 速度更新: speed += Ki * error * dt
 *         - 角度更新: angle += (speed + Kp * error) * dt
 */
float PLL_Update(PLL_t *pll, float theta_meas, float dt)
{
    /* 计算角度误差 (归一化到 [-π, π]) */
    float angle_err = theta_meas - pll->AngleEst;
    angle_err = NormalizeAngleError(angle_err);
    
    /* 二阶 PLL 更新 */
    /* 积分环节: 速度估算 */
    pll->SpeedEst += pll->Ki * angle_err * dt;
    
    /* 角度估算更新 (含比例校正) */
    pll->AngleEst += (pll->SpeedEst + pll->Kp * angle_err) * dt;
    
    /* 角度归一化 */
    pll->AngleEst = NormalizeAngle(pll->AngleEst);
    
    /* 更新输出 */
    pll->SpeedRadS = pll->SpeedEst;
    pll->SpeedRPM = pll->SpeedEst * RAD_S_TO_RPM;
    
    return pll->SpeedRadS;
}
