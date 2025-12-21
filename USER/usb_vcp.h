#ifndef __USB_VCP_H
#define __USB_VCP_H

#include "main.h"
#include "usbd_cdc_if.h"

void VCP_Init(void);
void VCP_SendData(uint8_t* data, uint16_t length);
void VCP_Printf(const char* format, ...);
uint8_t VCP_IsConnected(void);

#endif
