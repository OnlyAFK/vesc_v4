/**
 * @file    as5047p.c
 * @brief   AS5047P ?????? (???)
 * @note    ?????????????????? motor_hw.c
 */

#include "as5047p.h"
#include "spi.h"
#include "main.h"
#include <math.h>

/*============================================================================*/
/*                              ????                                       */
/*============================================================================*/

#define POLE_PAIRS   7u             // ?????
#define RAW_MAX      16384.0f       // 14-bit ????+1
#define ZERO_OFFSET  0.386563f      // ???? (rad)
#define TWO_PI       6.2831853f
#define PI           3.1415926f

/*============================================================================*/
/*                    ??????????                                      */
/*============================================================================*/

extern SPI_HandleTypeDef hspi1;

uint16_t tx_buffer = 0x3FFF;        // ??????
uint16_t rx_buffer = 0;
uint16_t angle_raw = 0;
float elec_theta = 0.0f;
float mech_theta = 0.0f;
int direction_flag = 1;

/*============================================================================*/
/*                              ????                                       */
/*============================================================================*/

/**
 * @brief  ?????? [0, 2?)
 */
static inline float _norm_angle(float x)
{
    return fmodf(x + TWO_PI, TWO_PI);
}

/*============================================================================*/
/*                    ????????                                          */
/*============================================================================*/

/**
 * @brief  ?? DMA ?????
 */
void as5047p_read_dma_start(void)
{
    HAL_GPIO_WritePin(AS5047P_NSS_GPIO_Port, AS5047P_NSS_Pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive_DMA(&hspi1, (uint8_t*)&tx_buffer, (uint8_t*)&rx_buffer, 1);
}

/**
 * @brief  ???????
 */
void as5047_data_process(void)
{
    /* ???? */
    HAL_GPIO_WritePin(AS5047P_NSS_GPIO_Port, AS5047P_NSS_Pin, GPIO_PIN_SET);
    
    /* ?? 14-bit ?? */
    angle_raw = rx_buffer & 0x3FFF;
    
    /* ?????? */
    mech_theta = direction_flag * angle_raw * (TWO_PI / RAW_MAX);
    mech_theta = _norm_angle(mech_theta);
    
    /* ????? */
    elec_theta = (mech_theta - ZERO_OFFSET) * POLE_PAIRS;
    elec_theta = _norm_angle(elec_theta);
}
