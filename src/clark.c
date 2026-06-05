/*
 * clark.c
 *
 *  Created on: 2026年3月9日
 *      Author: L.YF
 */

#include "clark.h"
#include <math.h>

/*
 * Clarke 变换计算函数
 * 公式 (等幅值变换):
 * Alpha = As
 * Beta  = (As + 2*Bs) / sqrt(3) = As*(1/sqrt(3)) + Bs*(2/sqrt(3))
 *
 * 1/sqrt(3) ≈ 0.577350269
 * 2/sqrt(3) ≈ 1.154700538
 */
void clark_calc(CLARK *p)
{
    p->Alpha = p->As;

    // 常数后面的'f'能保证F28335编译器使用单精度 FPU 硬件指令，而不是耗时的 double 软件库
    p->Beta  = (p->As * 0.577350269f) + (p->Bs * 1.154700538f);
}

//**********************************************************************
// End of file
//**********************************************************************


