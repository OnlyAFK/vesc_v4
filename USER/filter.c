#include "filter.h"

/**
 * @brief ????????
 * @param lpf   ???????
 * @param alpha ???? (0.0 - 1.0)
 */
void LPF_Init(LPF_Handle_t *lpf, float alpha) {
    if (alpha > 1.0f) alpha = 1.0f;
    if (alpha < 0.0f) alpha = 0.0f;
    
    lpf->alpha = alpha;
    lpf->out_prev = 0.0f; // ??????
}

/**
 * @brief ??????
 * @param lpf   ???????
 * @param input ??????? (Raw Value)
 * @return      ?????
 * ??: Y(n) = alpha * X(n) + (1 - alpha) * Y(n-1)
 */
float LPF_Apply(LPF_Handle_t *lpf, float input) {
    float output;
    
    // ????
    output = (lpf->alpha * input) + ((1.0f - lpf->alpha) * lpf->out_prev);
    
    // ??????????
    lpf->out_prev = output;
    
    return output;
}
