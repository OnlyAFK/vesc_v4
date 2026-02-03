/**
 * @file    motor_hw.c
 * @brief   电机硬件抽象层 (HAL) 实现
 * @note    STM32F4 + DRV8301 + AS5047P 硬件平台
 */

#include "motor_hw.h"
#include "main.h"
#include "tim.h"
#include "adc.h"
#include "spi.h"
#include "gpio.h"
#include <math.h>

/*============================================================================*/
/*                              私有变量                                       */
/*============================================================================*/

/* 编码器 SPI 通信缓冲区 */
static uint16_t encoder_tx_buffer = 0x3FFF;     // 读取角度命令
static uint16_t encoder_rx_buffer = 0;

/* 电流偏移校准 */
#define CALIBRATION_SAMPLES     1000

/*============================================================================*/
/*                              常量定义                                       */
/*============================================================================*/

#define TWO_PI      6.2831853f
#define RAW_TO_RAD  (TWO_PI / 16384.0f)     // 14-bit 转弧度

/*============================================================================*/
/*                              函数实现                                       */
/*============================================================================*/

/**
 * @brief  硬件层初始化
 */
void MotorHW_Init(void)
{
    /* 清除 SPI 溢出标志 */
    __HAL_SPI_CLEAR_OVRFLAG(&hspi1);
    
    /* 编码器片选拉高 (空闲) */
    HAL_GPIO_WritePin(AS5047P_NSS_GPIO_Port, AS5047P_NSS_Pin, GPIO_PIN_SET);
}

/**
 * @brief  使能电机驱动
 */
void MotorHW_EnableDriver(void)
{
    HAL_GPIO_WritePin(DRV8301_EN_GATE_GPIO_Port, DRV8301_EN_GATE_Pin, GPIO_PIN_SET);
}

/**
 * @brief  禁用电机驱动
 */
void MotorHW_DisableDriver(void)
{
    HAL_GPIO_WritePin(DRV8301_EN_GATE_GPIO_Port, DRV8301_EN_GATE_Pin, GPIO_PIN_RESET);
}

/**
 * @brief  设置三相 PWM 占空比
 */
void MotorHW_SetPWM(uint32_t ccr_u, uint32_t ccr_v, uint32_t ccr_w)
{
    TIM1->CCR1 = ccr_u;
    TIM1->CCR2 = ccr_v;
    TIM1->CCR3 = ccr_w;
}

/**
 * @brief  设置所有 PWM 为 50% (刹车/停止)
 */
void MotorHW_SetPWMBrake(void)
{
    uint32_t mid = HW_PWM_PERIOD / 2;
    TIM1->CCR1 = mid;
    TIM1->CCR2 = mid;
    TIM1->CCR3 = mid;
}

/**
 * @brief  读取三相电流 ADC 原始值
 */
void MotorHW_GetCurrentADC(uint32_t *adc_u, uint32_t *adc_v, uint32_t *adc_w)
{
    *adc_u = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
    *adc_v = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);
    *adc_w = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_3);
}

/**
 * @brief  将 ADC 值转换为电流 (A)
 */
float MotorHW_ADCToCurrent(uint32_t adc_value, float offset)
{
    return ((float)adc_value - offset) * HW_CURRENT_SCALE;
}

/**
 * @brief  读取三相电流 (已转换)
 */
void MotorHW_GetCurrents(PhaseCurrents_t *currents, const CurrentOffset_t *offset)
{
    uint32_t adc_u, adc_v, adc_w;
    MotorHW_GetCurrentADC(&adc_u, &adc_v, &adc_w);
    
    currents->Iu = MotorHW_ADCToCurrent(adc_u, offset->OffsetU);
    currents->Iv = MotorHW_ADCToCurrent(adc_v, offset->OffsetV);
    /* Iw 由基尔霍夫定律计算: Iu + Iv + Iw = 0 */
    currents->Iw = -currents->Iu - currents->Iv;
}

/**
 * @brief  启动编码器 DMA 读取
 */
void MotorHW_StartEncoderRead(void)
{
    HAL_GPIO_WritePin(AS5047P_NSS_GPIO_Port, AS5047P_NSS_Pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive_DMA(&hspi1, (uint8_t*)&encoder_tx_buffer, 
                                 (uint8_t*)&encoder_rx_buffer, 1);
}

/**
 * @brief  角度归一化到 [0, 2π)
 */
static inline float NormalizeAngle(float angle)
{
    return fmodf(angle + TWO_PI, TWO_PI);
}

/**
 * @brief  处理编码器数据
 */
void MotorHW_ProcessEncoderData(EncoderData_t *encoder)
{
    /* 拉高片选，结束传输 */
    HAL_GPIO_WritePin(AS5047P_NSS_GPIO_Port, AS5047P_NSS_Pin, GPIO_PIN_SET);
    
    /* 提取 14-bit 角度值 */
    encoder->RawAngle = encoder_rx_buffer & 0x3FFF;
    
    /* 计算机械角度 */
    encoder->MechAngle = encoder->Direction * encoder->RawAngle * RAW_TO_RAD;
    encoder->MechAngle = NormalizeAngle(encoder->MechAngle);
    
    /* 计算电角度: θe = (θm - offset) * Pp */
    encoder->ElecAngle = (encoder->MechAngle - HW_ENCODER_ZERO_OFFSET) * HW_MOTOR_POLE_PAIRS;
    encoder->ElecAngle = NormalizeAngle(encoder->ElecAngle);
}

/**
 * @brief  电流偏移校准
 */
uint8_t MotorHW_CalibrateCurrentOffset(CurrentOffset_t *offset, 
                                        uint32_t adc_u, uint32_t adc_v, uint32_t adc_w)
{
    static uint32_t sum_u = 0, sum_v = 0, sum_w = 0;
    static uint32_t sample_count = 0;
    
    if (offset->IsCalibrated) {
        return 1;  /* 已完成 */
    }
    
    if (sample_count < CALIBRATION_SAMPLES) {
        sum_u += adc_u;
        sum_v += adc_v;
        sum_w += adc_w;
        sample_count++;
        return 0;  /* 校准中 */
    }
    else {
        /* 计算平均值作为偏移 */
        offset->OffsetU = (float)sum_u / (float)CALIBRATION_SAMPLES;
        offset->OffsetV = (float)sum_v / (float)CALIBRATION_SAMPLES;
        offset->OffsetW = (float)sum_w / (float)CALIBRATION_SAMPLES;
        offset->IsCalibrated = 1;
        
        /* 重置累加器 (以备下次使用) */
        sum_u = sum_v = sum_w = 0;
        sample_count = 0;
        
        return 1;  /* 校准完成 */
    }
}

/**
 * @brief  调试 GPIO 设置
 */
void MotorHW_DebugPin(uint8_t state)
{
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
