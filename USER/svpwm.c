/**
 * @file    svpwm.c
 * @brief   ???? PWM ??????
 * @note    ???????????????
 */

#include "svpwm.h"

/*============================================================================*/
/*                              ????                                       */
/*============================================================================*/

#define SQRT3           1.73205080757f
#define SQRT3_BY_2      0.86602540378f

/*============================================================================*/
/*                    ?????????? (????)                            */
/*============================================================================*/

SVPWM_t svpwm1_t = {0};

/*============================================================================*/
/*                              ????                                       */
/*============================================================================*/

/**
 * @brief  SVPWM ???
 */
void SVPWM_Init(SVPWM_t *svpwm, float udc, uint32_t ts)
{
    svpwm->Udc = udc;
    svpwm->Ts = ts;
    svpwm->Alpha = 0.0f;
    svpwm->Beta = 0.0f;
    svpwm->Sector = 0;
    svpwm->CCR1 = ts / 2;
    svpwm->CCR2 = ts / 2;
    svpwm->CCR3 = ts / 2;
}

/**
 * @brief  SVPWM ????
 * @note   ????? SVPWM?????
 *         
 *         ????: A = V?, B = ?3/2·V? - 1/2·V?, C = -?3/2·V? - 1/2·V?
 *         sector = (A>0) + 2*(B>0) + 4*(C>0)
 */
void SVPWM_Calc(SVPWM_t *svpwm)
{
    float alpha = svpwm->Alpha;
    float beta = svpwm->Beta;
    float ts = (float)svpwm->Ts;
    float udc = svpwm->Udc;
    
    /*--- 1. ???? ---*/
    float A = beta;
    float B = SQRT3_BY_2 * alpha - 0.5f * beta;
    float C = -SQRT3_BY_2 * alpha - 0.5f * beta;
    
    uint8_t sector = 0;
    if (A > 0.0f) sector += 1;
    if (B > 0.0f) sector += 2;
    if (C > 0.0f) sector += 4;
    svpwm->Sector = sector;
    
    /*--- 2. ?????????? ---*/
    float temp = (SQRT3 * ts) / udc;
    float X = temp * beta;
    float Y = temp * (SQRT3_BY_2 * alpha + 0.5f * beta);
    float Z = temp * (-SQRT3_BY_2 * alpha + 0.5f * beta);
    
    float t1, t2;
    switch (sector) {
        case 3:  /* Sector 1 */
            t1 = -Z; t2 = X;  break;
        case 1:  /* Sector 2 */
            t1 = Z;  t2 = Y;  break;
        case 5:  /* Sector 3 */
            t1 = X;  t2 = -Y; break;
        case 4:  /* Sector 4 */
            t1 = -X; t2 = Z;  break;
        case 6:  /* Sector 5 */
            t1 = -Y; t2 = -Z; break;
        case 2:  /* Sector 6 */
            t1 = Y;  t2 = -X; break;
        default:
            t1 = 0.0f; t2 = 0.0f; break;
    }
    
    svpwm->T1 = t1;
    svpwm->T2 = t2;
    
    /*--- 3. ????? ---*/
    if ((t1 + t2) > ts) {
        float ratio = ts / (t1 + t2);
        t1 *= ratio;
        t2 *= ratio;
    }
    
    /*--- 4. ???????? ---*/
    float Ta = (ts + t1 + t2) * 0.5f;
    float Tb = (ts - t1 + t2) * 0.5f;
    float Tc = (ts - t1 - t2) * 0.5f;
    
    /*--- 5. ?????? CCR ---*/
    switch (sector) {
        case 3:  /* Sector 1: U > V > W */
            svpwm->CCR1 = (uint32_t)Ta;
            svpwm->CCR2 = (uint32_t)Tb;
            svpwm->CCR3 = (uint32_t)Tc;
            break;
        case 1:  /* Sector 2: V > U > W */
            svpwm->CCR1 = (uint32_t)Tb;
            svpwm->CCR2 = (uint32_t)Ta;
            svpwm->CCR3 = (uint32_t)Tc;
            break;
        case 5:  /* Sector 3: V > W > U */
            svpwm->CCR1 = (uint32_t)Tc;
            svpwm->CCR2 = (uint32_t)Ta;
            svpwm->CCR3 = (uint32_t)Tb;
            break;
        case 4:  /* Sector 4: W > V > U */
            svpwm->CCR1 = (uint32_t)Tc;
            svpwm->CCR2 = (uint32_t)Tb;
            svpwm->CCR3 = (uint32_t)Ta;
            break;
        case 6:  /* Sector 5: W > U > V */
            svpwm->CCR1 = (uint32_t)Tb;
            svpwm->CCR2 = (uint32_t)Tc;
            svpwm->CCR3 = (uint32_t)Ta;
            break;
        case 2:  /* Sector 6: U > W > V */
            svpwm->CCR1 = (uint32_t)Ta;
            svpwm->CCR2 = (uint32_t)Tc;
            svpwm->CCR3 = (uint32_t)Tb;
            break;
        default:
            svpwm->CCR1 = (uint32_t)(ts * 0.5f);
            svpwm->CCR2 = (uint32_t)(ts * 0.5f);
            svpwm->CCR3 = (uint32_t)(ts * 0.5f);
            break;
    }
}
