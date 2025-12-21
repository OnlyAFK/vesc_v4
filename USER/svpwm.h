#ifndef __SVPWM_H
#define __SVPWM_H

#include "stdint.h"
#include "tim.h"
typedef struct
{
    float Alpha;
    float Beta;
    
    uint8_t sector;
    
    float Udc;
    uint32_t Ts;
    
    uint32_t CCR1;
    uint32_t CCR2;
    uint32_t CCR3;
} svpwm_t;


void svpwm_calc(svpwm_t *pSvpwm);
void pwm_update(svpwm_t *pSvpwm);

extern svpwm_t svpwm1_t;

#endif
