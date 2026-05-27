/*
 * ipark.c
 *反 Park 变换
 *
 *  Created on: 2026年3月9日
 *      Author: L.YF
 */

#include "ipark.h"

/*
 * 反 Park 变换计算函数
 * 公式:
 * V_alpha =  Vd * cos(θ) - Vq * sin(θ)
 * V_beta  =  Vq * cos(θ) + Vd * sin(θ)
 */
void ipark_calc(IPARK *p)
{
    p->Alpha = (p->Ds * p->Cos) - (p->Qs * p->Sin);
    p->Beta  = (p->Qs * p->Cos) + (p->Ds * p->Sin);
}

//**********************************************************************
// End of file
//**********************************************************************
