#ifndef _FOC_MATH_H
#define _FOC_MATH_H

typedef struct
{
    float Ia;
    float Ib;
    float Ic;

    float Alpha;
    float Beta;
} clarke_t;

typedef struct
{
    float Alpha;
    float Beta;
    
    float D;
    float Q;
    
    float theta;
} park_t;

typedef struct
{
    float D;
    float Q;
    
    float Alpha;
    float Beta;
    
    float theta;
} ipark_t;

typedef struct
{
    float Alpha;
    float Beta;
    
    float Va;
    float Vb;
    float Vc;
} iclarke_t;

void clarke_calc(clarke_t *pClarke);
void park_calc(park_t *pPark);
void ipark_calc(ipark_t *pIPark);
void iclarke_calc(iclarke_t *pIClarke);

extern ipark_t ipark1_t;
extern clarke_t clarke1_t;
extern park_t park1_t;
#endif
