#include "as5047p.h"
#include "spi.h"
#include "arm_math.h"
#include "main.h"

extern SPI_HandleTypeDef hspi1;

// DMA缓冲区
static uint16_t tx_buf[2] = {0x3FFF, 0x3FFF};  // 发送命令0x3FFF
static uint16_t rx_buf[2] = {0, 0};
static volatile uint8_t dma_flag = 1;     // DMA就绪标志
AS5047P_Data_t encoder_data = {0};

// 片选控制
static void AS5047P_Select(void) {
    HAL_GPIO_WritePin(AS5047P_NSS_GPIO_Port, AS5047P_NSS_Pin, GPIO_PIN_RESET);
}

static void AS5047P_Deselect(void) {
    HAL_GPIO_WritePin(AS5047P_NSS_GPIO_Port, AS5047P_NSS_Pin, GPIO_PIN_SET);
}

// 初始化
void AS5047P_Init(void)
{
    __HAL_SPI_ENABLE(&hspi1);  // 使能SPI
}

// 启动DMA读取（非阻塞）
uint8_t AS5047P_StartDMA_Read(void)
{
    if(!dma_flag) return 0;  // DMA正在传输
    
    dma_flag = 0;
    AS5047P_Select();
    uint16_t cmd = 0x3FFF;
		uint16_t val = 0;
    // 启动SPI+DMA收发（发送2字节命令，接收2字节数据）
    if(HAL_SPI_TransmitReceive_DMA(&hspi1, (uint8_t*)tx_buf, (uint8_t *)rx_buf, 2) != HAL_OK) {
        AS5047P_Deselect();
        dma_flag = 1;
        return 0;
    }
    
    return 1;
}

// DMA完成后调用（在中断回调或主循环中）
void AS5047P_ProcessData(void)
{
    AS5047P_Deselect();
    
    // 合并接收到的数据
    uint16_t raw_data = (rx_buf[0] << 8) | rx_buf[1];
    
    // 检查错误标志位 (D15)
    if(raw_data & 0x4000) {
        encoder_data.is_valid = 0;
        dma_flag = 1;
        return;
    }
    
    // 提取14位角度值 (D13-D0)
    encoder_data.raw_angle = raw_data & 0x3FFF;
    encoder_data.is_valid = 1;
    
    // 转换为物理量
    encoder_data.angle_deg = encoder_data.raw_angle * 360.0f / 16384.0f;
    encoder_data.angle_rad = encoder_data.raw_angle * 2.0f * PI / 16384.0f;
    
    dma_flag = 1;  // 标记DMA就绪
}

// 获取数据
uint16_t AS5047P_GetRawAngle(void) {
    return encoder_data.raw_angle;
}

float AS5047P_GetAngleDeg(void) {
    return encoder_data.angle_deg;
}
