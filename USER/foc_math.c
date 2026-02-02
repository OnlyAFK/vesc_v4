/**
 * @file    foc_math.c
 * @brief   FOC 坐标变换数学库实现
 * @note    纯算法实现，无硬件依赖，可移植
 */

#include "foc_math.h"
#include "arm_math.h"   /* 使用 CMSIS-DSP 优化的三角函数 */

/*============================================================================*/
/*                    兼容旧代码的全局变量 (逐步废弃)                            */
/*============================================================================*/

Clarke_t clarke1_t = {0};
Park_t park1_t = {0};
InvPark_t ipark1_t = {0};

/*============================================================================*/
/*                              函数实现                                       */
/*============================================================================*/

/**
 * @brief  Clarke 变换 (等幅值变换)
 * @note   使用两相输入重建三相:
 *         Alpha = Ia
 *         Beta  = (Ia + 2*Ib) / sqrt(3)
 */
void Clarke_Calc(Clarke_t *clarke)
{
    clarke->Alpha = clarke->Ia;
    clarke->Beta  = FOC_ONE_BY_SQRT3 * (clarke->Ia + 2.0f * clarke->Ib);
}

/**
 * @brief  Park 变换
 * @note   将静止 αβ 坐标系变换到旋转 dq 坐标系
 *         D =  Alpha * cos(θ) + Beta * sin(θ)
 *         Q = -Alpha * sin(θ) + Beta * cos(θ)
 */
void Park_Calc(Park_t *park)
{
    float sin_val, cos_val;
    
    /* 使用 CMSIS-DSP 优化的三角函数 */
    sin_val = arm_sin_f32(park->Theta);
    cos_val = arm_cos_f32(park->Theta);
    
    park->D =  park->Alpha * cos_val + park->Beta * sin_val;
    park->Q = -park->Alpha * sin_val + park->Beta * cos_val;
}

/**
 * @brief  逆 Park 变换
 * @note   将旋转 dq 坐标系变换到静止 αβ 坐标系
 *         Alpha = D * cos(θ) - Q * sin(θ)
 *         Beta  = D * sin(θ) + Q * cos(θ)
 */
void InvPark_Calc(InvPark_t *invpark)
{
    float sin_val, cos_val;
    
    /* 使用 CMSIS-DSP 优化的三角函数 */
    sin_val = arm_sin_f32(invpark->Theta);
    cos_val = arm_cos_f32(invpark->Theta);
    
    invpark->Alpha = invpark->D * cos_val - invpark->Q * sin_val;
    invpark->Beta  = invpark->D * sin_val + invpark->Q * cos_val;
}

/**
 * @brief  逆 Clarke 变换
 * @note   将 αβ 坐标系变换到 abc 三相
 *         Va = Alpha
 *         Vb = -0.5*Alpha + sqrt(3)/2*Beta
 *         Vc = -0.5*Alpha - sqrt(3)/2*Beta
 */
void InvClarke_Calc(InvClarke_t *invclarke)
{
    float term_alpha_half = invclarke->Alpha * 0.5f;
    float term_beta = invclarke->Beta * FOC_SQRT3_BY_2;
    
    invclarke->Va = invclarke->Alpha;
    invclarke->Vb = -term_alpha_half + term_beta;
    invclarke->Vc = -term_alpha_half - term_beta;
}
