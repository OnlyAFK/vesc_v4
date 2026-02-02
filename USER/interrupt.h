/**
 * @file    interrupt.h
 * @brief   中断处理模块头文件
 */

#ifndef __INTERRUPT_H
#define __INTERRUPT_H

#include <stdint.h>
#include "foc_core.h"

/**
 * @brief  从电机对象更新 VOFA 调试数据
 * @param  motor: 电机对象指针
 */
void VOFA_UpdateFromMotor(Motor_t *motor);

#endif /* __INTERRUPT_H */
