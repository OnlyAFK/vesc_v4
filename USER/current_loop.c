#include "current_loop.h"

PI_Controller_t pid_id = {0};
PI_Controller_t pid_iq = {0};

void current_pi_init(void) {
    pid_id.Kp = 0.07037f;       
    pid_id.Ki = 0.01423f;      
    pid_id.Out_Max = 12.0f; 
    pid_id.Out_Min = -12.0f;
    pid_id.Integral = 0.0f;
    pid_id.Ref = 0.0f;      

    pid_iq.Kp = 0.07037f;      
    pid_iq.Ki = 0.01423f;
    pid_iq.Out_Max = 12.0f;
    pid_iq.Out_Min = -12.0f;
    pid_iq.Integral = 0.0f;
    pid_iq.Ref = 0.0f;      
}

void current_calc(PI_Controller_t *pPI) {
    float Error = pPI->Ref - pPI->Fdb;
    float Up = pPI->Kp * Error;
    pPI->Integral += pPI->Ki * Error;
    if (pPI->Integral > pPI->Out_Max) pPI->Integral = pPI->Out_Max;
    if (pPI->Integral < pPI->Out_Min) pPI->Integral = pPI->Out_Min;
    pPI->Out = Up + pPI->Integral;
    if (pPI->Out > pPI->Out_Max) pPI->Out = pPI->Out_Max;
    if (pPI->Out < pPI->Out_Min) pPI->Out = pPI->Out_Min;
}
