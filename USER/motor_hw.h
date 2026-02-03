/**
 * @file    motor_hw.h
 * @brief   电机硬件抽象层 (HAL)
 * @note    封装所有硬件相关操作，便于移植
 */

#ifndef __MOTOR_HW_H
#define __MOTOR_HW_H

#include <stdint.h>

/*============================================================================*/
/*                              硬件配置参数                                   */
/*============================================================================*/

/* PWM 配置 */
#define HW_PWM_PERIOD           4200        // TIM1 ARR (20kHz @ 168MHz)
#define HW_PWM_DEADTIME         200         // 死区时间 (约1.2us)
#define HW_PWM_FREQ_HZ          20000       // PWM 频率

/* ADC 电流采样配置 */
#define HW_ADC_RESOLUTION       4096.0f     // 12-bit
#define HW_ADC_VREF             3.3f        // 参考电压
#define HW_OPAMP_GAIN           20.0f       // AD8418A 增益
#define HW_SHUNT_RESISTANCE     0.005f      // 采样电阻 (5mΩ)

/* 电流转换系数: A/LSB = Vref / (Gain * Rshunt * ADC_Max) */
#define HW_CURRENT_SCALE        (HW_ADC_VREF / (HW_OPAMP_GAIN * HW_SHUNT_RESISTANCE * HW_ADC_RESOLUTION))

/* 编码器配置 */
#define HW_ENCODER_CPR          16384       // AS5047P 14-bit (每转计数)
#define HW_MOTOR_POLE_PAIRS     7           // 电机极对数
#define HW_ENCODER_ZERO_OFFSET  0.386563f   // 零点偏移 (rad)

/* 采样周期 */
#define HW_CONTROL_PERIOD_S     0.00005f    // 控制周期 (50us = 20kHz)
#define HW_SPEED_LOOP_DIV       20          // 速度环分频 (1kHz)
#define HW_POS_LOOP_DIV         20          // 位置环分频 (1kHz)

/*============================================================================*/
/*                              数据结构                                       */
/*============================================================================*/

/**
 * @brief 三相电流结构体
 */
typedef struct {
    float Iu;           // U相电流 (A)
    float Iv;           // V相电流 (A)
    float Iw;           // W相电流 (A)
} PhaseCurrents_t;

/**
 * @brief 编码器数据结构体
 */
typedef struct {
    uint16_t RawAngle;      // 原始角度值 (0~16383)
    float MechAngle;        // 机械角度 (rad, 0~2π)
    float ElecAngle;        // 电角度 (rad, 0~2π)
    int8_t Direction;       // 方向 (+1 或 -1)
} EncoderData_t;

/**
 * @brief 电流偏移校准结构体
 */
typedef struct {
    float OffsetU;          // U相偏移
    float OffsetV;          // V相偏移
    float OffsetW;          // W相偏移
    uint8_t IsCalibrated;   // 校准完成标志
} CurrentOffset_t;

/*============================================================================*/
/*                              函数接口                                       */
/*============================================================================*/

/**
 * @brief  硬件层初始化
 */
void MotorHW_Init(void);

/**
 * @brief  使能电机驱动 (DRV8301 EN_GATE)
 */
void MotorHW_EnableDriver(void);

/**
 * @brief  禁用电机驱动
 */
void MotorHW_DisableDriver(void);

/**
 * @brief  设置三相 PWM 占空比
 * @param  ccr_u: U相 CCR值 (0 ~ HW_PWM_PERIOD)
 * @param  ccr_v: V相 CCR值
 * @param  ccr_w: W相 CCR值
 */
void MotorHW_SetPWM(uint32_t ccr_u, uint32_t ccr_v, uint32_t ccr_w);

/**
 * @brief  设置所有 PWM 为 50% (刹车/停止)
 */
void MotorHW_SetPWMBrake(void);

/**
 * @brief  读取三相电流 ADC 原始值
 * @param  adc_u: U相 ADC 值指针
 * @param  adc_v: V相 ADC 值指针
 * @param  adc_w: W相 ADC 值指针
 */
void MotorHW_GetCurrentADC(uint32_t *adc_u, uint32_t *adc_v, uint32_t *adc_w);

/**
 * @brief  将 ADC 值转换为电流 (A)
 * @param  adc_value: ADC 原始值
 * @param  offset: 零点偏移
 * @return 电流值 (A)
 */
float MotorHW_ADCToCurrent(uint32_t adc_value, float offset);

/**
 * @brief  读取三相电流 (已转换)
 * @param  currents: 电流结构体指针
 * @param  offset: 偏移校准数据指针
 */
void MotorHW_GetCurrents(PhaseCurrents_t *currents, const CurrentOffset_t *offset);

/**
 * @brief  启动编码器 DMA 读取
 */
void MotorHW_StartEncoderRead(void);

/**
 * @brief  处理编码器数据 (在 SPI DMA 回调中调用)
 * @param  encoder: 编码器数据结构体指针
 */
void MotorHW_ProcessEncoderData(EncoderData_t *encoder);

/**
 * @brief  电流偏移校准 (累加一次采样)
 * @param  offset: 偏移结构体指针
 * @param  adc_u: U相 ADC 值
 * @param  adc_v: V相 ADC 值
 * @param  adc_w: W相 ADC 值
 * @return 1=校准完成, 0=校准中
 */
uint8_t MotorHW_CalibrateCurrentOffset(CurrentOffset_t *offset, 
                                        uint32_t adc_u, uint32_t adc_v, uint32_t adc_w);

/**
 * @brief  调试 GPIO 设置 (用于测量中断时间)
 * @param  state: 1=高电平, 0=低电平
 */
void MotorHW_DebugPin(uint8_t state);

#endif /* __MOTOR_HW_H */
