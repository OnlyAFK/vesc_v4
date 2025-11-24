#ifndef __USB_VCP_H
#define __USB_VCP_H

#include "main.h"
#include "usbd_cdc_if.h"
// 初始化（MX_USB_DEVICE_Init已在main.c中调用）
void VCP_Init(void);

// 阻塞式发送数据
void VCP_SendData(uint8_t* data, uint16_t length);

// 格式化打印（类似printf）
void VCP_Printf(const char* format, ...);

// 检查USB是否连接
uint8_t VCP_IsConnected(void);

#endif
