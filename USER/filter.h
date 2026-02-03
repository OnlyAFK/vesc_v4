#ifndef __FILTER_H
#define __FILTER_H

typedef struct {
    float alpha;        // ???? (0 < alpha < 1)
                        // alpha ????????????
    float out_prev;     // ??????? (???)
} LPF_Handle_t;

// ????
void LPF_Init(LPF_Handle_t *lpf, float alpha);
float LPF_Apply(LPF_Handle_t *lpf, float input);

#endif
