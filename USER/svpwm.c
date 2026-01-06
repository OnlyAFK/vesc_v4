#include "svpwm.h"

#define SQRT3_BY_2      0.86602540378f  // sqrt(3) / 2
#define SQRT3           1.73205080756f // sqrt(3)

svpwm_t svpwm1_t = {0};

void svpwm_calc(svpwm_t *pSvpwm) {
    float A = pSvpwm->Beta;
    float B = SQRT3_BY_2 * pSvpwm->Alpha - 0.5f * pSvpwm->Beta;
    float C = -SQRT3_BY_2 * pSvpwm->Alpha - 0.5f * pSvpwm->Beta;
    
    pSvpwm->sector = 0;
    if (A > 0) pSvpwm->sector += 1;
    if (B > 0) pSvpwm->sector += 2;
    if (C > 0) pSvpwm->sector += 4;
    
    // 注意：这里的 pSvpwm->Ts 必须是 ARR (4200) 20k的pwm频率
    float temp = (SQRT3 * pSvpwm->Ts) / pSvpwm->Udc; 
    float X = temp * pSvpwm->Beta;
    float Y = temp * (SQRT3_BY_2 * pSvpwm->Alpha + 0.5f * pSvpwm->Beta);
    float Z = temp * (-SQRT3_BY_2 * pSvpwm->Alpha + 0.5f * pSvpwm->Beta);
    
    float t1, t2;
    switch(pSvpwm->sector) {
        case 3: // Sector 1
            t1 = -Z; t2 = X; break;
        case 1: // Sector 2
            t1 = Z; t2 = Y; break;
        case 5: // Sector 3
            t1 = X; t2 = -Y; break;
        case 4: // Sector 4
            t1 = -X; t2 = Z; break;
        case 6: // Sector 5
            t1 = -Y; t2 = -Z; break;
        case 2: // Sector 6
            t1 = Y; t2 = -X; break;
        default: 
            t1 = 0; t2 = 0; break;
    }

    if ((t1 + t2) > pSvpwm->Ts) {
        float ratio = pSvpwm->Ts / (t1 + t2);
        t1 *= ratio;
        t2 *= ratio;
    }

    float Ta = (pSvpwm->Ts + t1 + t2) * 0.5f;
    float Tb = (pSvpwm->Ts - t1 + t2) * 0.5f;
    float Tc = (pSvpwm->Ts - t1 - t2) * 0.5f;

    switch(pSvpwm->sector) {
        case 3: // Sector 1 (U > V > W)
            pSvpwm->CCR1 = Ta;
            pSvpwm->CCR2 = Tb;
            pSvpwm->CCR3 = Tc;
            break;
            
        case 1: // Sector 2 (V > U > W)
            pSvpwm->CCR1 = Tb;
            pSvpwm->CCR2 = Ta;
            pSvpwm->CCR3 = Tc;
            break;
            
        case 5: // Sector 3 (V > W > U)
            pSvpwm->CCR1 = Tc;
            pSvpwm->CCR2 = Ta;
            pSvpwm->CCR3 = Tb;
            break;
            
        case 4: // Sector 4 (W > V > U)
            pSvpwm->CCR1 = Tc;
            pSvpwm->CCR2 = Tb;
            pSvpwm->CCR3 = Ta;
            break;
            
        case 6: // Sector 5 (W > U > V)
            pSvpwm->CCR1 = Tb;
            pSvpwm->CCR2 = Tc;
            pSvpwm->CCR3 = Ta;
            break;
            
        case 2: // Sector 6 (U > W > V)
            pSvpwm->CCR1 = Ta;
            pSvpwm->CCR2 = Tc;
            pSvpwm->CCR3 = Tb;
            break;
            
        default:
            pSvpwm->CCR1 = 0;
            pSvpwm->CCR2 = 0;
            pSvpwm->CCR3 = 0;
            break;
    }
}

void pwm_update(svpwm_t *pSvpwm) {
    TIM1->CCR1 = pSvpwm->CCR1;
    TIM1->CCR2 = pSvpwm->CCR2;
    TIM1->CCR3 = pSvpwm->CCR3;
}
