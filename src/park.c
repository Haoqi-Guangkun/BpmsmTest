/*
 * park.c
 *
 *  Created on: 2026年3月9日
 *      Author: L.YF
 */

#include "park.h"

/*
 * Park 变换计算函数
 * 公式:
 * d =  Alpha * cos(θ) + Beta * sin(θ)
 * q = -Alpha * sin(θ) + Beta * cos(θ)
 */
void park_calc(PARK *p)
{
    // F28335 的 FPU 硬件可以在单周期内完成浮点乘加运算 (MACF32)
    p->Ds = (p->Alpha * p->Cos) + (p->Beta * p->Sin);
    p->Qs = (p->Beta  * p->Cos) - (p->Alpha * p->Sin);
}

//**********************************************************************
// End of file
//**********************************************************************
