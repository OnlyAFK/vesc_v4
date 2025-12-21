#ifndef __AS5047P_H
#define __AS5047P_H
#include "stdint.h"
void as5047p_read_dma_start(void);
void as5047_data_process(void);
extern uint16_t rx_buffer;
extern uint16_t angle_raw;
extern float elec_theta;
extern float mech_theta;
#endif



























//#ifndef __AS5047P_H
//#define __AS5047P_H

//#include "main.h"
//#include "spi.h"

//// AS5047P 寄存器命令
//#define AS5047P_CMD_READ_ANGLE  0x3FFF  // 读取角度 (14-bit)

//typedef struct {
//    uint16_t raw_angle;   // 14-bit 原始角度值
//    float angle_deg;      // 角度 (度)
//    float angle_rad;      // 角度 (弧度)
//    uint8_t is_valid;     // 数据有效性 (1=有效, 0=无效)
//} AS5047P_Data_t;

//// 初始化 AS5047P
//void AS5047P_Init(void);

//// 启动 DMA 读取 (非阻塞)
//uint8_t AS5047P_StartDMA_Read(void);

//// 处理 DMA 接收完成 (在 SPI 回调中调用)
//void AS5047P_ProcessData(void);

//// 获取原始角度值
//uint16_t AS5047P_GetRawAngle(void);

//// 获取角度 (度)
//float AS5047P_GetAngleDeg(void);


//#endif
