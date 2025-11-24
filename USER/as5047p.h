#ifndef __AS5047P_H
#define __AS5047P_H

#include "main.h"

// 配置参数
#define AS5047P_READ_ANGLE_CMD  0x3FFF  // 读取角度寄存器命令
//#define AS5047P_CS_PORT         GPIOA
//#define AS5047P_CS_PIN          GPIO_PIN_4

// 数据结构
typedef struct {
    uint16_t raw_angle;     // 原始角度值 (0-16383)
    float angle_deg;        // 角度 [0, 360)
    float angle_rad;        // 弧度 [0, 2π)
    uint8_t is_valid;       // 数据有效标志
} AS5047P_Data_t;

// 函数接口
void AS5047P_Init(void);
uint8_t AS5047P_StartDMA_Read(void);
void AS5047P_ProcessData(void);
uint16_t AS5047P_GetRawAngle(void);
float AS5047P_GetAngleDeg(void);

extern AS5047P_Data_t encoder_data;

#endif
