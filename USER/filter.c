#include "filter.h"

/**
 * @brief 初始化低通滤波器
 * @param lpf   滤波器句柄指针
 * @param alpha 滤波系数 (0.0 - 1.0)
 */
void LPF_Init(LPF_Handle_t *lpf, float alpha) {
    if (alpha > 1.0f) alpha = 1.0f;
    if (alpha < 0.0f) alpha = 0.0f;
    
    lpf->alpha = alpha;
    lpf->out_prev = 0.0f; // 初始记忆清零
}

/**
 * @brief 计算滤波输出
 * @param lpf   滤波器句柄指针
 * @param input 当前原始输入值 (Raw Value)
 * @return      滤波后的值
 * 公式: Y(n) = alpha * X(n) + (1 - alpha) * Y(n-1)
 */
float LPF_Apply(LPF_Handle_t *lpf, float input) {
    float output;
    
    // 计算公式
    output = (lpf->alpha * input) + ((1.0f - lpf->alpha) * lpf->out_prev);
    
    // 更新记忆，供下次使用
    lpf->out_prev = output;
    
    return output;
}
