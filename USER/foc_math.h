/**
 * @file    foc_math.h
 * @brief   FOC 坐标变换数学库
 * @note    纯算法实现，无硬件依赖，可移植
 */

#ifndef __FOC_MATH_H
#define __FOC_MATH_H

#include <stdint.h>

/*============================================================================*/
/*                              常量定义                                       */
/*============================================================================*/

#define FOC_PI              3.14159265358f
#define FOC_2PI             6.28318530718f
#define FOC_SQRT3           1.73205080757f
#define FOC_SQRT3_BY_2      0.86602540378f      // sqrt(3) / 2
#define FOC_ONE_BY_SQRT3    0.57735026919f      // 1 / sqrt(3)

/*============================================================================*/
/*                              数据结构                                       */
/*============================================================================*/

/**
 * @brief Clarke 变换结构体 (abc → αβ)
 */
typedef struct {
    /* 输入: 三相电流/电压 */
    float Ia;
    float Ib;
    float Ic;
    
    /* 输出: αβ 坐标系 */
    float Alpha;
    float Beta;
} Clarke_t;

/**
 * @brief Park 变换结构体 (αβ → dq)
 */
typedef struct {
    /* 输入: αβ 坐标系 */
    float Alpha;
    float Beta;
    
    /* 输入: 转子电角度 */
    float Theta;
    
    /* 输出: dq 坐标系 */
    float D;
    float Q;
} Park_t;

/**
 * @brief 逆 Park 变换结构体 (dq → αβ)
 */
typedef struct {
    /* 输入: dq 坐标系 */
    float D;
    float Q;
    
    /* 输入: 转子电角度 */
    float Theta;
    
    /* 输出: αβ 坐标系 */
    float Alpha;
    float Beta;
} InvPark_t;

/**
 * @brief 逆 Clarke 变换结构体 (αβ → abc)
 */
typedef struct {
    /* 输入: αβ 坐标系 */
    float Alpha;
    float Beta;
    
    /* 输出: 三相电压 */
    float Va;
    float Vb;
    float Vc;
} InvClarke_t;

/*============================================================================*/
/*                              函数接口                                       */
/*============================================================================*/

/**
 * @brief  Clarke 变换 (等幅值变换)
 * @param  clarke: Clarke 结构体指针
 * @note   Alpha = Ia
 *         Beta  = (Ia + 2*Ib) / sqrt(3)
 */
void Clarke_Calc(Clarke_t *clarke);

/**
 * @brief  Park 变换
 * @param  park: Park 结构体指针
 * @note   D =  Alpha * cos(θ) + Beta * sin(θ)
 *         Q = -Alpha * sin(θ) + Beta * cos(θ)
 */
void Park_Calc(Park_t *park);

/**
 * @brief  逆 Park 变换
 * @param  invpark: InvPark 结构体指针
 * @note   Alpha = D * cos(θ) - Q * sin(θ)
 *         Beta  = D * sin(θ) + Q * cos(θ)
 */
void InvPark_Calc(InvPark_t *invpark);

/**
 * @brief  逆 Clarke 变换
 * @param  invclarke: InvClarke 结构体指针
 * @note   Va = Alpha
 *         Vb = -0.5*Alpha + sqrt(3)/2*Beta
 *         Vc = -0.5*Alpha - sqrt(3)/2*Beta
 */
void InvClarke_Calc(InvClarke_t *invclarke);

/*============================================================================*/
/*                    兼容旧代码的别名 (逐步废弃)                               */
/*============================================================================*/

/* 旧结构体名称别名 */
typedef Clarke_t    clarke_t;
typedef Park_t      park_t;
typedef InvPark_t   ipark_t;
typedef InvClarke_t iclarke_t;

/* 旧函数名称别名 */
#define clarke_calc(p)      Clarke_Calc(p)
#define park_calc(p)        Park_Calc(p)
#define ipark_calc(p)       InvPark_Calc(p)
#define iclarke_calc(p)     InvClarke_Calc(p)

/* 旧全局变量声明 (兼容) */
extern Clarke_t clarke1_t;
extern Park_t park1_t;
extern InvPark_t ipark1_t;

#endif /* __FOC_MATH_H */
