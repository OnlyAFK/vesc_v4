#ifndef __FILTER_H
#define __FILTER_H

typedef struct {
    float alpha;        // 滤波系数 (0 < alpha < 1)
                        // alpha 越小，滤波越强，延迟越大
    float out_prev;     // 上一次的输出值 (记忆项)
} LPF_Handle_t;

// 函数声明
void LPF_Init(LPF_Handle_t *lpf, float alpha);
float LPF_Apply(LPF_Handle_t *lpf, float input);

#endif
