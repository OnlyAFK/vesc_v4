#include "vofa.h" 
#include "usb_vcp.h"

const uint8_t VOFA_TAIL[4] = {0x00, 0x00, 0x80, 0x7F};
float vofa_data[VOFA_CHANNEL_CNT] = {0};

// 2. 发送函数
void VOFA_Send_JustFloat(float *data_ptr, uint8_t count)
{
    // A. 发送数据部分 (每个 float 4 字节)
    // 直接将 float 数组指针强转为 uint8_t* 进行发送
    VCP_SendData((uint8_t*)data_ptr, count * sizeof(float));
    // B. 发送帧尾 (4 字节)
    VCP_SendData((uint8_t*)VOFA_TAIL, 4);
}
