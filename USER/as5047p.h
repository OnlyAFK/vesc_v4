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
