#include "usb_vcp.h"
#include "usb_device.h"
#include <stdarg.h>
#include <string.h>

extern USBD_HandleTypeDef hUsbDeviceFS;

// 发送缓冲区
static uint8_t tx_buffer[256];

void VCP_Init(void)
{
    // USB初始化已在main.c中调用
    // 这里可以添加用户初始化代码
}

void VCP_SendData(uint8_t* data, uint16_t length)
{
	
    // 等待USB配置完成
    while(hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED)
    {
        HAL_Delay(1);
    }
    
    // 发送数据（循环直到成功）
    while(CDC_Transmit_FS(data, length) != USBD_OK)
    {
        HAL_Delay(1);
    }
}

void VCP_Printf(const char* format, ...)
{
    va_list args;
    va_start(args, format);
    uint16_t len = vsnprintf((char*)tx_buffer, sizeof(tx_buffer), format, args);
    va_end(args);
    
    VCP_SendData(tx_buffer, len);
}

uint8_t VCP_IsConnected(void)
{
    return (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED);
}
