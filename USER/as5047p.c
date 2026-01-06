#include "as5047p.h"
#include "spi.h"
#include "arm_math.h"
#include "main.h"

extern SPI_HandleTypeDef hspi1; 

uint16_t tx_buffer = 0x3FFF; 
uint16_t rx_buffer = 0;
uint16_t angle_raw = 0;

#define POLE_PAIRS   7u          
#define RAW_MAX      16384.0f    // 14-bit 最大码值+1
#define ZERO_OFFSET  0.386563f
#define TWO_PI       6.2831853f
float elec_theta;
float mech_theta;
int direction_flag = 1.0;

static inline float _norm_angle(float x) {
    return fmodf(x + 2.0f*PI, 2.0f*PI);
}

void as5047p_read_dma_start(void) {
    HAL_GPIO_WritePin(AS5047P_NSS_GPIO_Port, AS5047P_NSS_Pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive_DMA(&hspi1, (uint8_t*)&tx_buffer, (uint8_t*)&rx_buffer, 1);
}

void as5047_data_process(void) {
    HAL_GPIO_WritePin(AS5047P_NSS_GPIO_Port, AS5047P_NSS_Pin, GPIO_PIN_SET);
    angle_raw = rx_buffer & 0x3FFF;
    mech_theta = direction_flag * angle_raw * (2.0f * PI / RAW_MAX);
    mech_theta = _norm_angle(mech_theta);
    elec_theta = (mech_theta - ZERO_OFFSET) * POLE_PAIRS;
    elec_theta = _norm_angle(elec_theta);
}


























//#include "as5047p.h"
//#include "spi.h"
//#include "arm_math.h"
//#include "main.h"

//// 外部 SPI 句柄
//extern SPI_HandleTypeDef hspi1;

//// 片选引脚 (根据你的硬件修改)
//#define AS5047P_NSS_GPIO_Port  GPIOA
//#define AS5047P_NSS_Pin        GPIO_PIN_4

//// 静态变量 (16-bit 模式)
//static uint16_t tx_cmd = AS5047P_CMD_READ_ANGLE; // 发送命令 (16-bit)
//static uint16_t rx_data = 0;                     // 接收数据 (16-bit)
//static volatile uint8_t dma_flag = 1;            // DMA 状态标志

//AS5047P_Data_t encoder_data = {0}; // 编码器数据

//// 片选控制函数
//static void AS5047P_Select(void) {
//    HAL_GPIO_WritePin(AS5047P_NSS_GPIO_Port, AS5047P_NSS_Pin, GPIO_PIN_RESET);
//}

//static void AS5047P_Deselect(void) {
//    HAL_GPIO_WritePin(AS5047P_NSS_GPIO_Port, AS5047P_NSS_Pin, GPIO_PIN_SET);
//}

//// 初始化 AS5047P
//void AS5047P_Init(void) {
//    // 配置 NSS 为推挽输出 (确保片选正常)
////    GPIO_InitTypeDef GPIO_InitStruct = {0};
////    GPIO_InitStruct.Pin = AS5047P_NSS_Pin;
////    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
////    GPIO_InitStruct.Pull = GPIO_NOPULL;
////    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
////    HAL_GPIO_Init(AS5047P_NSS_GPIO_Port, &GPIO_InitStruct);
//    
//    // 默认拉高片选
//    AS5047P_Deselect();
//    
//    // 确保 SPI1 已使能 (HAL_SPI_Init 已使能)
//    __HAL_SPI_ENABLE(&hspi1);
////    __HAL_DMA_ENABLE(&hdma2);
//}

//// 启动 DMA 读取
//uint8_t AS5047P_StartDMA_Read(void) {
//    AS5047P_Select();
////    HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)&tx_cmd, (uint8_t*)&tx_cmd, 2, 100);
//    HAL_SPI_TransmitReceive_DMA(&hspi1, (uint8_t*)&tx_cmd, (uint8_t*)&rx_data, 1);
//    AS5047P_Deselect();
//    
//    
////    if (!dma_flag) return 0; // DMA 正在传输中
////    dma_flag = 0;
////    AS5047P_Select();
////    
////    // 关键：16-bit 模式 + 传入字节指针 + Size=2 (字节)
////    if (HAL_SPI_TransmitReceive_DMA(&hspi1, (uint8_t*)&tx_cmd, (uint8_t*)&rx_data, 2) != HAL_OK) {
////        AS5047P_Deselect();
////        dma_flag = 1;
////        return 0;
////    }
//    return 1;
//}

//// DMA 接收完成回调 (在 HAL_SPI_TxRxCpltCallback 中调用)
//void AS5047P_ProcessData(void) {
//    AS5047P_Deselect();
//    
//    // 检查错误标志 (D15)
////    if (rx_data & 0x4000) {
////        encoder_data.is_valid = 0;
////    } else {
//        // 提取 14-bit 角度 (D13-D0)
//        encoder_data.raw_angle = rx_data & 0x3FFF;
//        encoder_data.is_valid = 1;
//        
//        // 转换为角度 (度)
//        encoder_data.angle_deg = encoder_data.raw_angle * 360.0f / 16384.0f;
//        
//        // 转换为弧度 (可选)
//        encoder_data.angle_rad = encoder_data.raw_angle * 2.0f * 3.1415926535f / 16384.0f;
////    }
//    dma_flag = 1; // 标记 DMA 就绪
//}

//// 获取原始角度
//uint16_t AS5047P_GetRawAngle(void) {
//    return encoder_data.raw_angle;
//}

//// 获取角度 (度)
//float AS5047P_GetAngleDeg(void) {
//    return encoder_data.angle_deg;
//}


