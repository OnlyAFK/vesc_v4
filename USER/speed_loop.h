/**
 * @file    speed_loop.h
 * @brief   ?????? (???)
 * @note    ?????????????????? pid.h, pll.h ? foc_core.h
 */

#ifndef __SPEED_LOOP_H
#define __SPEED_LOOP_H

#include "pid.h"
#include "pll.h"

/*============================================================================*/
/*                    ??????????                                      */
/*============================================================================*/

/**
 * @brief ??? PID ??? (????)
 */
typedef struct {
    float TargetRPM;    // ???? (RPM)
    float ActualRPM;    // ???? (RPM)
    float FilteredRPM;  // ?????? (RPM)
    
    float Kp;
    float Ki;
    float Integral;
    float Out_Max;      // ???? (??????? A)
    float Out_Min;
    
    float Out;          // PID?? (???? Iq_ref)
} Speed_PID_t;

/*============================================================================*/
/*                    ??????????                                      */
/*============================================================================*/

extern Speed_PID_t pid_spd;
extern float speed_filter_const;

/*============================================================================*/
/*                    ????????                                          */
/*============================================================================*/

/**
 * @brief  ??????
 */
void Speed_Loop_Init(void);

/**
 * @brief  ???? (???)
 * @param  theta_mech_now: ?????? (rad)
 */
void Speed_Calc(float theta_mech_now);

/**
 * @brief  ??? PID ??
 */
void Speed_PID_Calc(void);

/**
 * @brief  PLL ????
 * @param  theta_now: ?????? (rad)
 * @param  Ts: ???? (s)
 */
void Speed_Calc_PLL(float theta_now, float Ts);

#endif /* __SPEED_LOOP_H */
