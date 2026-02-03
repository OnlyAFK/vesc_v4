/**
 * @file    pll.h
 * @brief   PLL 锁相环速度估算模块
 * @note    纯算法实现，无硬件依赖，可移植
 */

#ifndef __PLL_H
#define __PLL_H

#include <stdint.h>

#define PLL_2PI     6.2831853f
#define PLL_PI      3.1415926f

/**
 * @brief PLL 锁相环结构体
 */
typedef struct {
    /* PLL 增益参数 */
    float Kp;               // 比例增益 (带宽)
    float Ki;               // 积分增益
    
    /* 状态估算 */
    float AngleEst;         // 估算角度 (rad)
    float SpeedEst;         // 估算速度 (rad/s)
    
    /* 输出 (不同单位) */
    float SpeedRPM;         // 估算速度 (RPM)
    float SpeedRadS;        // 估算速度 (rad/s)
} PLL_t;

/**
 * @brief  初始化 PLL
 * @param  pll: PLL 结构体指针
 * @param  kp: 比例增益 (典型值: 100~500)
 * @param  ki: 积分增益 (典型值: 10000~50000)
 */
void PLL_Init(PLL_t *pll, float kp, float ki);

/**
 * @brief  PLL 复位
 * @param  pll: PLL 结构体指针
 */
void PLL_Reset(PLL_t *pll);

/**
 * @brief  PLL 速度估算更新
 * @param  pll: PLL 结构体指针
 * @param  theta_meas: 测量的角度 (rad, 0~2π)
 * @param  dt: 采样周期 (s)
 * @return 估算速度 (rad/s)
 */
float PLL_Update(PLL_t *pll, float theta_meas, float dt);

/**
 * @brief  获取估算速度 (RPM)
 * @param  pll: PLL 结构体指针
 * @return 估算速度 (RPM)
 */
static inline float PLL_GetSpeedRPM(PLL_t *pll) {
    return pll->SpeedRPM;
}

/**
 * @brief  获取估算速度 (rad/s)
 * @param  pll: PLL 结构体指针
 * @return 估算速度 (rad/s)
 */
static inline float PLL_GetSpeedRadS(PLL_t *pll) {
    return pll->SpeedRadS;
}

/**
 * @brief  获取估算角度 (rad)
 * @param  pll: PLL 结构体指针
 * @return 估算角度 (rad)
 */
static inline float PLL_GetAngle(PLL_t *pll) {
    return pll->AngleEst;
}

#endif /* __PLL_H */
