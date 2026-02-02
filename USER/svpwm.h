/**
 * @file    svpwm.h
 * @brief   空间矢量 PWM 调制模块
 * @note    纯算法实现，无硬件依赖，可移植
 */

#ifndef __SVPWM_H
#define __SVPWM_H

#include <stdint.h>

/*============================================================================*/
/*                              数据结构                                       */
/*============================================================================*/

/**
 * @brief SVPWM 调制结构体
 */
typedef struct {
    /* 输入: αβ 坐标系电压 */
    float Alpha;            // α轴电压 (V)
    float Beta;             // β轴电压 (V)
    
    /* 输入: 系统参数 */
    float Udc;              // 直流母线电压 (V)
    uint32_t Ts;            // PWM 周期 (ARR 值)
    
    /* 输出: 扇区和 CCR 值 */
    uint8_t Sector;         // 当前扇区 (1~6)
    uint32_t CCR1;          // U相 CCR
    uint32_t CCR2;          // V相 CCR
    uint32_t CCR3;          // W相 CCR
    
    /* 内部: 矢量作用时间 */
    float T1;               // 矢量1作用时间
    float T2;               // 矢量2作用时间
} SVPWM_t;

/*============================================================================*/
/*                              函数接口                                       */
/*============================================================================*/

/**
 * @brief  SVPWM 调制计算
 * @param  svpwm: SVPWM 结构体指针
 * @note   输入: Alpha, Beta, Udc, Ts
 *         输出: Sector, CCR1, CCR2, CCR3
 */
void SVPWM_Calc(SVPWM_t *svpwm);

/**
 * @brief  SVPWM 初始化
 * @param  svpwm: SVPWM 结构体指针
 * @param  udc: 母线电压 (V)
 * @param  ts: PWM 周期 (ARR 值)
 */
void SVPWM_Init(SVPWM_t *svpwm, float udc, uint32_t ts);

/*============================================================================*/
/*                    兼容旧代码的别名 (逐步废弃)                               */
/*============================================================================*/

/* 旧结构体名称别名 */
typedef SVPWM_t svpwm_t;

/* 旧函数名称别名 */
#define svpwm_calc(p)   SVPWM_Calc(p)

/* 兼容旧成员名 (sector -> Sector) */
#define sector Sector

/* 旧全局变量声明 (兼容) */
extern SVPWM_t svpwm1_t;

#endif /* __SVPWM_H */
