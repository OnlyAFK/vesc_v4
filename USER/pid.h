/**
 * @file    pid.h
 * @brief   通用 PID 控制器模块
 * @note    纯算法实现，无硬件依赖，可移植
 */

#ifndef __PID_H
#define __PID_H

#include <stdint.h>

/**
 * @brief 通用 PID 控制器结构体
 */
typedef struct {
    /* 增益参数 */
    float Kp;               // 比例增益
    float Ki;               // 积分增益
    float Kd;               // 微分增益 (可选)
    
    /* 输入输出 */
    float Ref;              // 目标值 (参考值)
    float Fdb;              // 反馈值 (实际值)
    float Out;              // 输出值
    
    /* 内部状态 */
    float Integral;         // 积分累加器
    float LastError;        // 上次误差 (用于微分)
    
    /* 限幅参数 */
    float OutMax;           // 输出上限
    float OutMin;           // 输出下限
    float IntegralMax;      // 积分上限 (防饱和)
    float IntegralMin;      // 积分下限
} PID_Controller_t;

/**
 * @brief  初始化 PID 控制器
 * @param  pid: PID 控制器指针
 * @param  kp: 比例增益
 * @param  ki: 积分增益
 * @param  kd: 微分增益
 * @param  out_max: 输出上限
 * @param  out_min: 输出下限
 */
void PID_Init(PID_Controller_t *pid, float kp, float ki, float kd, 
              float out_max, float out_min);

/**
 * @brief  PID 控制器复位
 * @param  pid: PID 控制器指针
 */
void PID_Reset(PID_Controller_t *pid);

/**
 * @brief  PI 控制器计算 (无微分)
 * @param  pid: PID 控制器指针
 * @param  ref: 目标值
 * @param  fdb: 反馈值
 * @return 控制器输出
 */
float PI_Calc(PID_Controller_t *pid, float ref, float fdb);

/**
 * @brief  PID 控制器计算 (完整版)
 * @param  pid: PID 控制器指针
 * @param  ref: 目标值
 * @param  fdb: 反馈值
 * @return 控制器输出
 */
float PID_Calc(PID_Controller_t *pid, float ref, float fdb);

/**
 * @brief  增量式 PID 计算
 * @param  pid: PID 控制器指针
 * @param  ref: 目标值
 * @param  fdb: 反馈值
 * @return 控制器输出增量
 */
float PID_Calc_Incremental(PID_Controller_t *pid, float ref, float fdb);

#endif /* __PID_H */
