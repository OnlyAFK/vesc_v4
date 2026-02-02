/**
 * @file    vofa.h
 * @brief   VOFA+ 调试数据发送模块
 */

#ifndef __VOFA_H
#define __VOFA_H

#include <stdint.h>

/*============================================================================*/
/*                              配置                                          */
/*============================================================================*/

#define VOFA_CHANNEL_CNT    30      // 最大通道数

/*============================================================================*/
/*                              通道定义                                       */
/*============================================================================*/

/* 便于理解的通道定义 */
enum {
    VOFA_CH_VD = 0,             // 0: d轴输出电压
    VOFA_CH_VQ,                 // 1: q轴输出电压
    VOFA_CH_MECH_THETA,         // 2: 机械角度
    VOFA_CH_SVPWM_ALPHA,        // 3: SVPWM Alpha
    VOFA_CH_SVPWM_BETA,         // 4: SVPWM Beta
    VOFA_CH_CCR1,               // 5: CCR1
    VOFA_CH_CCR2,               // 6: CCR2
    VOFA_CH_CCR3,               // 7: CCR3
    VOFA_CH_SECTOR,             // 8: SVPWM 扇区
    VOFA_CH_IU,                 // 9: U相电流
    VOFA_CH_IV,                 // 10: V相电流
    VOFA_CH_IW,                 // 11: W相电流
    VOFA_CH_I_ALPHA,            // 12: Alpha电流
    VOFA_CH_I_BETA,             // 13: Beta电流
    VOFA_CH_ID,                 // 14: d轴电流
    VOFA_CH_IQ,                 // 15: q轴电流
    VOFA_CH_ID_REF,             // 16: d轴目标电流
    VOFA_CH_IQ_REF,             // 17: q轴目标电流
    VOFA_CH_POS_TARGET,         // 18: 目标位置
    VOFA_CH_POS_ACTUAL,         // 19: 实际位置
    VOFA_CH_DEBUG,              // 20: 调试变量
    VOFA_CH_SPEED_ACTUAL,       // 21: 实际转速
    VOFA_CH_SPEED_TARGET,       // 22: 目标转速
};

/*============================================================================*/
/*                              数据                                          */
/*============================================================================*/

extern float vofa_data[VOFA_CHANNEL_CNT];

/*============================================================================*/
/*                              函数接口                                       */
/*============================================================================*/

/**
 * @brief  发送 JustFloat 格式数据
 * @param  data_ptr: 数据指针
 * @param  count: 通道数
 */
void VOFA_Send_JustFloat(float *data_ptr, uint8_t count);

/**
 * @brief  更新单个通道数据
 * @param  channel: 通道号
 * @param  value: 数据值
 */
static inline void VOFA_SetChannel(uint8_t channel, float value) {
    if (channel < VOFA_CHANNEL_CNT) {
        vofa_data[channel] = value;
    }
}

#endif /* __VOFA_H */
