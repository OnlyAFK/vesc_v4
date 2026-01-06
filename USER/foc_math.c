#include "foc_math.h"
#include "arm_math.h" 

#define ONE_BY_SQRT3    0.57735026919f  // 1 / sqrt(3)
#define SQRT3_BY_2      0.86602540378f  // sqrt(3) / 2
#define SQRT3           1.73205080756f // sqrt(3)

ipark_t ipark1_t = {0};
clarke_t clarke1_t = {0};
park_t park1_t = {0};

void clarke_calc(clarke_t *pClarke) {
    pClarke->Alpha = pClarke->Ia;
    pClarke->Beta  = ONE_BY_SQRT3 * (pClarke->Ia + 2.0f * pClarke->Ib);
}

void park_calc(park_t *pPark) {
    float sin_val = 0.0f;
    float cos_val = 0.0f;
    
    sin_val = arm_sin_f32(pPark->theta);
    cos_val = arm_cos_f32(pPark->theta);
    
    pPark->D =  pPark->Alpha * cos_val + pPark->Beta * sin_val;
    pPark->Q = -pPark->Alpha * sin_val + pPark->Beta * cos_val;
}

void ipark_calc(ipark_t *pIPark) {
    float sin_val = 0.0f;
    float cos_val = 0.0f;
    
    sin_val = arm_sin_f32(pIPark->theta);
    cos_val = arm_cos_f32(pIPark->theta);
    
    pIPark->Alpha = pIPark->D * cos_val - pIPark->Q * sin_val;
    pIPark->Beta  = pIPark->D * sin_val + pIPark->Q * cos_val;
}

void iclarke_calc(iclarke_t *pIClarke) {
    float term_beta = pIClarke->Beta * SQRT3_BY_2;  // sqrt(3)*0.5f*Beta
    float term_alpha_half = pIClarke->Alpha * 0.5f; // 0.5f*Alpha

    pIClarke->Va = pIClarke->Alpha;
    pIClarke->Vb = -term_alpha_half + term_beta;
    pIClarke->Vc = -term_alpha_half - term_beta;
}
