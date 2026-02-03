/**
 * @file    as5047p.c
 * @brief   AS5047P 磁编码器驱动 (兼容层)
 * @note    此文件保留用于向后兼容，新代码请使用 motor_hw.c
 */

#include "as5047p.h"
#include "spi.h"
#include "main.h"
#include <math.h>

/*============================================================================*/
/*                               配置参数                                      */
/*============================================================================*/

#define POLE_PAIRS   7u             // 电机极对数
#define RAW_MAX      16384.0f       // 14-bit 最大码值+1
#define ZERO_OFFSET  0.386563f      // 零点偏移 (rad)
#define TWO_PI       6.2831853f
#define PI           3.1415926f

/*============================================================================*/
/*                    兼容旧代码的全局变量                                      */
/*============================================================================*/

extern SPI_HandleTypeDef hspi1;

uint16_t tx_buffer = 0x3FFF;        // 读取角度命令
uint16_t rx_buffer = 0;
uint16_t angle_raw = 0;
float elec_theta = 0.0f;
float mech_theta = 0.0f;
int direction_flag = 1;

/*============================================================================*/
/*                               内部函数                                      */
/*============================================================================*/

/**
 * @brief  角度归一化到 [0, 2π)
 */
static inline float _norm_angle(float x)
{
    return fmodf(x + TWO_PI, TWO_PI);
}

/*============================================================================*/
/*                    兼容旧代码的函数                                          */
/*============================================================================*/

/**
 * @brief  启动 DMA 读取编码器
 */
void as5047p_read_dma_start(void)
{
    HAL_GPIO_WritePin(AS5047P_NSS_GPIO_Port, AS5047P_NSS_Pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive_DMA(&hspi1, (uint8_t*)&tx_buffer, (uint8_t*)&rx_buffer, 1);
}

/**
 * @brief  处理编码器数据
 */
void as5047_data_process(void)
{
    /* 拉高片选 */
    HAL_GPIO_WritePin(AS5047P_NSS_GPIO_Port, AS5047P_NSS_Pin, GPIO_PIN_SET);
    
    /* 提取 14-bit 角度 */
    angle_raw = rx_buffer & 0x3FFF;
    
    /* 计算机械角度 */
    mech_theta = direction_flag * angle_raw * (TWO_PI / RAW_MAX);
    mech_theta = _norm_angle(mech_theta);
    
    /* 计算电角度 */
    elec_theta = (mech_theta - ZERO_OFFSET) * POLE_PAIRS;
    elec_theta = _norm_angle(elec_theta);
}
