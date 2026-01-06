#include "vofa.h" 
#include "usb_vcp.h"

const uint8_t VOFA_TAIL[4] = {0x00, 0x00, 0x80, 0x7F};
float vofa_data[VOFA_CHANNEL_CNT] = {0};

void VOFA_Send_JustFloat(float *data_ptr, uint8_t count) {
    VCP_SendData((uint8_t*)data_ptr, count * sizeof(float));
    VCP_SendData((uint8_t*)VOFA_TAIL, 4);
}
