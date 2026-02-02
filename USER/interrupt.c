/**
 * @file    interrupt.c
 * @brief   ???? (????????)
 * @note    ?? FOC ???????? foc_core ??
 */

#include "interrupt.h"
#include "adc.h"
#include "foc_core.h"
#include "vofa.h"

/*============================================================================*/
/*                              ADC ????????                            */
/*============================================================================*/

/**
 * @brief  ADC ???????? (20kHz)
 * @note   ?? FOC ???????
 */
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1) {
        
        /*--- ???????? ---*/
        if (!g_Motor.CurOffset.IsCalibrated) {
            FOC_CalibrateCurrentOffset(&g_Motor);
            return;
        }
        
        /*--- FOC ????? ---*/
        FOC_ControlLoop(&g_Motor);
        
        /*--- ?? VOFA ???? ---*/
        VOFA_UpdateFromMotor(&g_Motor);
    }
}

/*============================================================================*/
/*                              VOFA ????                                  */
/*============================================================================*/

/**
 * @brief  ??????? VOFA ????
 * @param  motor: ??????
 */
void VOFA_UpdateFromMotor(Motor_t *motor)
{
    /* ???? */
    vofa_data[VOFA_CH_VD] = motor->InvPark.D;
    vofa_data[VOFA_CH_VQ] = motor->InvPark.Q;
    
    /* ?? */
    vofa_data[VOFA_CH_MECH_THETA] = motor->Encoder.MechAngle;
    
    /* SVPWM */
    vofa_data[VOFA_CH_SVPWM_ALPHA] = motor->SVPWM.Alpha;
    vofa_data[VOFA_CH_SVPWM_BETA] = motor->SVPWM.Beta;
    vofa_data[VOFA_CH_CCR1] = (float)motor->SVPWM.CCR1;
    vofa_data[VOFA_CH_CCR2] = (float)motor->SVPWM.CCR2;
    vofa_data[VOFA_CH_CCR3] = (float)motor->SVPWM.CCR3;
    vofa_data[VOFA_CH_SECTOR] = (float)motor->SVPWM.Sector;
    
    /* ???? */
    vofa_data[VOFA_CH_IU] = motor->Currents.Iu;
    vofa_data[VOFA_CH_IV] = motor->Currents.Iv;
    vofa_data[VOFA_CH_IW] = motor->Currents.Iw;
    
    /* Clarke ?? */
    vofa_data[VOFA_CH_I_ALPHA] = motor->Clarke.Alpha;
    vofa_data[VOFA_CH_I_BETA] = motor->Clarke.Beta;
    
    /* Park ?? (????) */
    vofa_data[VOFA_CH_ID] = motor->ActualId;
    vofa_data[VOFA_CH_IQ] = motor->ActualIq;
    
    /* ???? */
    vofa_data[VOFA_CH_ID_REF] = motor->TargetId;
    vofa_data[VOFA_CH_IQ_REF] = motor->TargetIq;
    
    /* ?? */
    vofa_data[VOFA_CH_POS_TARGET] = motor->PosCtrl.TargetPos;
    vofa_data[VOFA_CH_POS_ACTUAL] = motor->PosCtrl.CurrentPos;
    
    /* ?? */
    vofa_data[VOFA_CH_SPEED_ACTUAL] = motor->ActualRPM;
    vofa_data[VOFA_CH_SPEED_TARGET] = motor->TargetRPM;
}
