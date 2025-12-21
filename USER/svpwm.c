#include "svpwm.h"


#define SQRT3_BY_2      0.86602540378f  // sqrt(3) / 2
#define SQRT3           1.73205080756f // sqrt(3)

svpwm_t svpwm1_t = {0};

// svpwm算法步骤 1.扇区判断 2.计算两个矢量的作用时长 3.作用时长转化为CCR值

//void svpwm_calc(svpwm_t *pSvpwm)
//{
//    #define c0To60Degree 	    3
//    #define c60To120Degree		1
//    #define c120To180Degree 	5
//    #define c180To240Degree 	4
//    #define c240To300Degree		6
//    #define c300To360Degree		2
////    #define MIN_PULSE_WIDTH 	500//输出侧采样，没有最小脉宽
//    float A = pSvpwm->Beta;
//    float B = SQRT3_BY_2*pSvpwm->Alpha - 0.5*pSvpwm->Beta;
//    float C = -SQRT3_BY_2*pSvpwm->Alpha - 0.5*pSvpwm->Beta;
//    
//    pSvpwm->sector = 0;
//    if(A > 0) pSvpwm->sector += 1;
//    if(B > 0) pSvpwm->sector += 2;
//    if(C > 0) pSvpwm->sector += 4;
//    
//    float X,Y,Z,temp;
//    temp = (SQRT3*pSvpwm->Ts) / pSvpwm->Udc; // sqrt(3)*Ts/Udc
//    X = temp*pSvpwm->Beta;                                    // temp * Alpha
//    Y = temp*(SQRT3_BY_2*pSvpwm->Alpha + 0.5f*pSvpwm->Beta);  // temp*(sqrt(3)/2*Alpha+0.5*Beta) 
//    Z = temp*(-SQRT3_BY_2*pSvpwm->Alpha + 0.5f*pSvpwm->Beta); // temp*(-sqrt(3)/2*Alpha+0.5*Beta) 
//    
//    float Tx,Ty;
//    switch(pSvpwm->sector)
//    {
//        case c0To60Degree:
//            Tx = -Z; Ty = X;
//            break;
//        case c60To120Degree:
//            Tx = Z;  Ty = Y;
//            break;
//        case c120To180Degree:
//            Tx = X;  Ty = -Y;
//            break;
//        case c180To240Degree:
//            Tx = -X; Ty = Z;
//            break;
//        case c240To300Degree:
//            Tx = -Y; Ty = -Z;
//            break;
//        case c300To360Degree:
//            Tx = Y;  Ty = -X;
//            break;
//        default:
//            break;
//    }   

//    switch(pSvpwm->sector)
//    {
//        case c0To60Degree:
//            pSvpwm->CCR1 = 0.25f*(pSvpwm->Ts-Tx-Ty);
//            pSvpwm->CCR2 = 0.25f*(pSvpwm->Ts+Tx-Ty);
//            pSvpwm->CCR3 = 0.25f*(pSvpwm->Ts+Tx+Ty);
//            break;
//        case c60To120Degree:
//            pSvpwm->CCR1 = 0.25f*(pSvpwm->Ts+Tx-Ty);
//            pSvpwm->CCR2 = 0.25f*(pSvpwm->Ts-Tx-Ty);
//            pSvpwm->CCR3 = 0.25f*(pSvpwm->Ts+Tx+Ty);
//            break;
//        case c120To180Degree:
//            pSvpwm->CCR1 = 0.25f*(pSvpwm->Ts+Tx+Ty);
//            pSvpwm->CCR2 = 0.25f*(pSvpwm->Ts-Tx-Ty);
//            pSvpwm->CCR3 = 0.25f*(pSvpwm->Ts+Tx-Ty);
//            break;
//        case c180To240Degree:
//            pSvpwm->CCR1 = 0.25f*(pSvpwm->Ts+Tx+Ty);
//            pSvpwm->CCR2 = 0.25f*(pSvpwm->Ts+Tx-Ty);
//            pSvpwm->CCR3 = 0.25f*(pSvpwm->Ts-Tx-Ty);
//            break;
//        case c240To300Degree:
//            pSvpwm->CCR1 = 0.25f*(pSvpwm->Ts+Tx-Ty);
//            pSvpwm->CCR2 = 0.25f*(pSvpwm->Ts+Tx+Ty);
//            pSvpwm->CCR3 = 0.25f*(pSvpwm->Ts-Tx-Ty);
//            break;
//        case c300To360Degree:
//            pSvpwm->CCR1 = 0.25f*(pSvpwm->Ts-Tx-Ty);
//            pSvpwm->CCR2 = 0.25f*(pSvpwm->Ts+Tx+Ty);
//            pSvpwm->CCR3 = 0.25f*(pSvpwm->Ts+Tx-Ty);
//            break;
//        default:
//            break;
//    }
//}

void svpwm_calc(svpwm_t *pSvpwm)
{
    float A = pSvpwm->Beta;
    float B = SQRT3_BY_2 * pSvpwm->Alpha - 0.5f * pSvpwm->Beta;
    float C = -SQRT3_BY_2 * pSvpwm->Alpha - 0.5f * pSvpwm->Beta;
    
    pSvpwm->sector = 0;
    if (A > 0) pSvpwm->sector += 1;
    if (B > 0) pSvpwm->sector += 2;
    if (C > 0) pSvpwm->sector += 4;
    
    // 注意：这里的 pSvpwm->Ts 必须是 ARR (4200) 20k的pwm频率
    float temp = (SQRT3 * pSvpwm->Ts) / pSvpwm->Udc; 
    float X = temp * pSvpwm->Beta;
    float Y = temp * (SQRT3_BY_2 * pSvpwm->Alpha + 0.5f * pSvpwm->Beta);
    float Z = temp * (-SQRT3_BY_2 * pSvpwm->Alpha + 0.5f * pSvpwm->Beta);
    
    float t1, t2;
    switch(pSvpwm->sector)
    {
        case 3: // Sector 1
            t1 = -Z; t2 = X; break;
        case 1: // Sector 2
            t1 = Z; t2 = Y; break;
        case 5: // Sector 3
            t1 = X; t2 = -Y; break;
        case 4: // Sector 4
            t1 = -X; t2 = Z; break;
        case 6: // Sector 5
            t1 = -Y; t2 = -Z; break;
        case 2: // Sector 6
            t1 = Y; t2 = -X; break;
        default: 
            t1 = 0; t2 = 0; break;
    }

    if ((t1 + t2) > pSvpwm->Ts)
    {
        float ratio = pSvpwm->Ts / (t1 + t2);
        t1 *= ratio;
        t2 *= ratio;
    }

    float Ta = (pSvpwm->Ts + t1 + t2) * 0.5f;
    float Tb = (pSvpwm->Ts - t1 + t2) * 0.5f;
    float Tc = (pSvpwm->Ts - t1 - t2) * 0.5f;

    switch(pSvpwm->sector)
    {
        case 3: // Sector 1 (U > V > W)
            pSvpwm->CCR1 = Ta;
            pSvpwm->CCR2 = Tb;
            pSvpwm->CCR3 = Tc;
            break;
            
        case 1: // Sector 2 (V > U > W)
            pSvpwm->CCR1 = Tb;
            pSvpwm->CCR2 = Ta;
            pSvpwm->CCR3 = Tc;
            break;
            
        case 5: // Sector 3 (V > W > U)
            pSvpwm->CCR1 = Tc;
            pSvpwm->CCR2 = Ta;
            pSvpwm->CCR3 = Tb;
            break;
            
        case 4: // Sector 4 (W > V > U)
            pSvpwm->CCR1 = Tc;
            pSvpwm->CCR2 = Tb;
            pSvpwm->CCR3 = Ta;
            break;
            
        case 6: // Sector 5 (W > U > V)
            pSvpwm->CCR1 = Tb;
            pSvpwm->CCR2 = Tc;
            pSvpwm->CCR3 = Ta;
            break;
            
        case 2: // Sector 6 (U > W > V)
            pSvpwm->CCR1 = Ta;
            pSvpwm->CCR2 = Tc;
            pSvpwm->CCR3 = Tb;
            break;
            
        default:
            pSvpwm->CCR1 = 0;
            pSvpwm->CCR2 = 0;
            pSvpwm->CCR3 = 0;
            break;
    }
}

void pwm_update(svpwm_t *pSvpwm)
{
    TIM1->CCR1 = pSvpwm->CCR1;
    TIM1->CCR2 = pSvpwm->CCR2;
    TIM1->CCR3 = pSvpwm->CCR3;
}
