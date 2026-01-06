#ifndef __VOFA_H
#define __VOFA_H

#include "stdint.h"
#define VOFA_CHANNEL_CNT 30
extern float vofa_data[];
void VOFA_Send_JustFloat(float *data_ptr, uint8_t count);

#endif

