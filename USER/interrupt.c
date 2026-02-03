/**
 * @file    interrupt.c
 * @brief   中断处理 (简化版，只做调度)
 * @note    所有 FOC 控制逻辑已封装到 foc_core 模块
 */

#include "interrupt.h"
#include "adc.h"
#include "foc_core.h"
#include "vofa.h"

/*============================================================================*/
/*                              ADC 注入转换完成中断                            */
/*============================================================================*/

/**
 * @brief  ADC 注入转换完成回调 (20kHz)
 * @note   这是 FOC 控制的主入口点
 */
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1) {
        
        /*--- 电流偏移校准阶段 ---*/
        if (!g_Motor.CurOffset.IsCalibrated) {
            FOC_CalibrateCurrentOffset(&g_Motor);
            return;
        }
        
        /*--- FOC 主控制循环 ---*/
        FOC_ControlLoop(&g_Motor);
        
        /*--- 更新 VOFA 调试数据 ---*/
        VOFA_UpdateFromMotor(&g_Motor);
    }
}

/*============================================================================*/
/*                              VOFA 数据更新                                  */
/*============================================================================*/

/**
 * @brief  从电机对象更新 VOFA 调试数据
 * @param  motor: 电机对象指针
 */
void VOFA_UpdateFromMotor(Motor_t *motor)
{
    /* 输出电压 */
    vofa_data[VOFA_CH_VD] = motor->InvPark.D;
    vofa_data[VOFA_CH_VQ] = motor->InvPark.Q;
    
    /* 角度 */
    vofa_data[VOFA_CH_MECH_THETA] = motor->Encoder.MechAngle;
    
    /* SVPWM */
    vofa_data[VOFA_CH_SVPWM_ALPHA] = motor->SVPWM.Alpha;
    vofa_data[VOFA_CH_SVPWM_BETA] = motor->SVPWM.Beta;
    vofa_data[VOFA_CH_CCR1] = (float)motor->SVPWM.CCR1;
    vofa_data[VOFA_CH_CCR2] = (float)motor->SVPWM.CCR2;
    vofa_data[VOFA_CH_CCR3] = (float)motor->SVPWM.CCR3;
    vofa_data[VOFA_CH_SECTOR] = (float)motor->SVPWM.Sector;
    
    /* 三相电流 */
    vofa_data[VOFA_CH_IU] = motor->Currents.Iu;
    vofa_data[VOFA_CH_IV] = motor->Currents.Iv;
    vofa_data[VOFA_CH_IW] = motor->Currents.Iw;
    
    /* Clarke 输出 */
    vofa_data[VOFA_CH_I_ALPHA] = motor->Clarke.Alpha;
    vofa_data[VOFA_CH_I_BETA] = motor->Clarke.Beta;
    
    /* Park 输出 (实际电流) */
    vofa_data[VOFA_CH_ID] = motor->ActualId;
    vofa_data[VOFA_CH_IQ] = motor->ActualIq;
    
    /* 电流参考 */
    vofa_data[VOFA_CH_ID_REF] = motor->TargetId;
    vofa_data[VOFA_CH_IQ_REF] = motor->TargetIq;
    
    /* 位置 */
    vofa_data[VOFA_CH_POS_TARGET] = motor->PosCtrl.TargetPos;
    vofa_data[VOFA_CH_POS_ACTUAL] = motor->PosCtrl.CurrentPos;
    
    /* 速度 */
    vofa_data[VOFA_CH_SPEED_ACTUAL] = motor->ActualRPM;
    vofa_data[VOFA_CH_SPEED_TARGET] = motor->TargetRPM;
}
