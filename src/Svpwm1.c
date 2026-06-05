/*
 * Svpwm.c
 *
 *  Created on: 2026年2月4日
 *      Author: L.YF
 */

#include "Svpwm.h"
#include "DSP2833x_EPwm_defines.h"

void Init_Svpwm_Gpio(void)
{
    EALLOW;

   // --- 开启 GPIO 时钟 ---
   // (通常在 InitSysCtrl 中已开启，为保险再次确认)
   // SysCtrlRegs.PCLKCR3.bit.GPIOINENCLK = 1;

   // --- 配置 GPIO 引脚复用 (Mux) ---
   // 0:GPIO, 1:EPWM, 2:Reserved, 3:Reserved

   // GPAMUX1 寄存器控制 GPIO0 - GPIO15
   // 配置 GPIO0 - GPIO11 为 EPWM 功能 (置 1)
   GpioCtrlRegs.GPAMUX1.bit.GPIO0  = 1; // EPWM1A
   GpioCtrlRegs.GPAMUX1.bit.GPIO1  = 1; // EPWM1B
   GpioCtrlRegs.GPAMUX1.bit.GPIO2  = 1; // EPWM2A
   GpioCtrlRegs.GPAMUX1.bit.GPIO3  = 1; // EPWM2B
   GpioCtrlRegs.GPAMUX1.bit.GPIO4  = 1; // EPWM3A
   GpioCtrlRegs.GPAMUX1.bit.GPIO5  = 1; // EPWM3B
   GpioCtrlRegs.GPAMUX1.bit.GPIO6  = 1; // EPWM4A
   GpioCtrlRegs.GPAMUX1.bit.GPIO7  = 1; // EPWM4B
   GpioCtrlRegs.GPAMUX1.bit.GPIO8  = 1; // EPWM5A
   GpioCtrlRegs.GPAMUX1.bit.GPIO9  = 1; // EPWM5B
   GpioCtrlRegs.GPAMUX1.bit.GPIO10 = 1; // EPWM6A
   GpioCtrlRegs.GPAMUX1.bit.GPIO11 = 1; // EPWM6B

   EDIS;
}

void Init_Svpwm_Module(void)
{
    // 配置 EPWM1/2/3 为中心对齐、互补、带死区
    // 以 EPWM1 为例，2和3雷同，为节省篇幅只写关键配置

    // --- EPWM1 ---
    EPwm1Regs.TBPRD = PWM_PRD;
    EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;
    EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;
    EPwm1Regs.TBCTL.bit.CLKDIV = TB_DIV1;

    // 动作限定：中间高电平
    EPwm1Regs.AQCTLA.bit.CAU = AQ_CLEAR;
    EPwm1Regs.AQCTLA.bit.CAD = AQ_SET;

    // 死区配置 (AH/AL)
    EPwm1Regs.DBCTL.bit.IN_MODE = DBA_ALL;
    EPwm1Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC; // 互补高有效
    EPwm1Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
    EPwm1Regs.DBFED = DEAD_TIME;
    EPwm1Regs.DBRED = DEAD_TIME;

    // 中断配置 (只在 EPWM1 开中断作为主控节拍)
    EPwm1Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;
    EPwm1Regs.ETSEL.bit.INTEN = 1;
    EPwm1Regs.ETPS.bit.INTPRD = ET_1ST;

    // --- EPWM2 ---
    EPwm2Regs.TBPRD = PWM_PRD;
    EPwm2Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;
    EPwm2Regs.AQCTLA.bit.CAU = AQ_CLEAR;
    EPwm2Regs.AQCTLA.bit.CAD = AQ_SET;
    EPwm2Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
    EPwm2Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
    EPwm2Regs.DBFED = DEAD_TIME;
    EPwm2Regs.DBRED = DEAD_TIME;

    // --- EPWM3 ---
    EPwm3Regs.TBPRD = PWM_PRD;
    EPwm3Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;
    EPwm3Regs.AQCTLA.bit.CAU = AQ_CLEAR;
    EPwm3Regs.AQCTLA.bit.CAD = AQ_SET;
    EPwm3Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
    EPwm3Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
    EPwm3Regs.DBFED = DEAD_TIME;
    EPwm3Regs.DBRED = DEAD_TIME;
}

void Update_Svpwm(float Ualpha, float Ubeta)
{
    float Va, Vb, Vc, X, Y, Z, t1, t2;
    Uint16 Sector = 0;

    // 1. 扇区判断
    Va = Ubeta;
    Vb = 0.866025f * Ualpha - 0.5f * Ubeta;
    Vc = -0.866025f * Ualpha - 0.5f * Ubeta;

    if (Va > 0) Sector += 1;
    if (Vb > 0) Sector += 2;
    if (Vc > 0) Sector += 4;

    // 2. 计算时间
    X = Ubeta;
    Y = 0.866025f * Ualpha + 0.5f * Ubeta;
    Z = -0.866025f * Ualpha + 0.5f * Ubeta;

    switch (Sector)
    {
        case 3: t1 = -Z; t2 = X; break;
        case 1: t1 = Z; t2 = Y; break;
        case 5: t1 = X; t2 = -Y; break;
        case 4: t1 = -X; t2 = Z; break;
        case 6: t1 = -Y; t2 = -Z; break;
        case 2: t1 = Y; t2 = -X; break;
        default: t1 = 0; t2 = 0; break;
    }

    // 饱和限制
    if((t1+t2) > 1.0f) {
        float temp = t1 + t2;
        t1 /= temp;
        t2 /= temp;
    }

    // 转换为 CMPA 值
    float T_on1 = (1.0f - t1 - t2) * 0.5f * PWM_PRD;
    float T_on2 = T_on1 + t1 * PWM_PRD;
    float T_on3 = T_on2 + t2 * PWM_PRD;

    // 3. 赋值给寄存器
    switch (Sector)
    {
        case 3: // I
            EPwm1Regs.CMPA.half.CMPA = (Uint16)T_on1;
            EPwm2Regs.CMPA.half.CMPA = (Uint16)T_on2;
            EPwm3Regs.CMPA.half.CMPA = (Uint16)T_on3;
            break;
        case 1: // II
            EPwm1Regs.CMPA.half.CMPA = (Uint16)T_on2;
            EPwm2Regs.CMPA.half.CMPA = (Uint16)T_on1;
            EPwm3Regs.CMPA.half.CMPA = (Uint16)T_on3;
            break;
        case 5: // III
            EPwm1Regs.CMPA.half.CMPA = (Uint16)T_on3;
            EPwm2Regs.CMPA.half.CMPA = (Uint16)T_on1;
            EPwm3Regs.CMPA.half.CMPA = (Uint16)T_on2;
            break;
        case 4: // IV
            EPwm1Regs.CMPA.half.CMPA = (Uint16)T_on3;
            EPwm2Regs.CMPA.half.CMPA = (Uint16)T_on2;
            EPwm3Regs.CMPA.half.CMPA = (Uint16)T_on1;
            break;
        case 6: // V
            EPwm1Regs.CMPA.half.CMPA = (Uint16)T_on2;
            EPwm2Regs.CMPA.half.CMPA = (Uint16)T_on3;
            EPwm3Regs.CMPA.half.CMPA = (Uint16)T_on1;
            break;
        case 2: // VI
            EPwm1Regs.CMPA.half.CMPA = (Uint16)T_on1;
            EPwm2Regs.CMPA.half.CMPA = (Uint16)T_on3;
            EPwm3Regs.CMPA.half.CMPA = (Uint16)T_on2;
            break;
    }
}


