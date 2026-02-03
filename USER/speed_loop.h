/**
 * @file    speed_loop.h
 * @brief   速度环控制器 (兼容层)
 * @note    此文件保留用于向后兼容，新代码请使用 pid.h, pll.h 和 foc_core.h
 */

#ifndef __SPEED_LOOP_H
#define __SPEED_LOOP_H

#include "pid.h"
#include "pll.h"

/*============================================================================*/
/*                    兼容旧代码的类型定义                                      */
/*============================================================================*/

/**
 * @brief 速度环 PID 结构体 (兼容旧版)
 */
typedef struct {
    float TargetRPM;    // 目标转速 (RPM)
    float ActualRPM;    // 实际转速 (RPM)
    float FilteredRPM;  // 滤波后的转速 (RPM)
    
    float Kp;
    float Ki;
    float Integral;
    float Out_Max;      // 输出限制 (即最大允许电流 A)
    float Out_Min;
    
    float Out;          // PID输出 (目标电流 Iq_ref)
} Speed_PID_t;

/*============================================================================*/
/*                    兼容旧代码的全局变量                                      */
/*============================================================================*/

extern Speed_PID_t pid_spd;
extern float speed_filter_const;

/*============================================================================*/
/*                    兼容旧代码的函数                                          */
/*============================================================================*/

/**
 * @brief  初始化速度环
 */
void Speed_Loop_Init(void);

/**
 * @brief  速度计算 (差分法)
 * @param  theta_mech_now: 当前机械角度 (rad)
 */
void Speed_Calc(float theta_mech_now);

/**
 * @brief  速度环 PID 计算
 */
void Speed_PID_Calc(void);

/**
 * @brief  PLL 速度计算
 * @param  theta_now: 当前机械角度 (rad)
 * @param  Ts: 采样周期 (s)
 */
void Speed_Calc_PLL(float theta_now, float Ts);

#endif /* __SPEED_LOOP_H */
