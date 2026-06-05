/*
 * Svpwm.c
 *
 *  Created on: 2026年2月4日
 *      Author: L.YF
 */

#include "Svpwm.h"
#include "DSP2833x_EPwm_defines.h"
extern Uint16 PWM_PRD;

/*
 * 函数: Init_Svpwm_Gpio
 * 功能: 配置 GPIO0-GPIO11 为 EPWM1-6 功能
 * 对应:
 *  EPWM1 A/B -> U相 (GPIO0/1)  //此处注意电路板接线！！！
 *  EPWM2 A/B -> V相 (GPIO2/3)
 *  EPWM3 A/B -> W相 (GPIO4/5)
 *  EPWM4 A/B -> A相 (GPIO6/7)  [悬浮/第二组]
 *  EPWM5 A/B -> B相 (GPIO8/9)  [悬浮/第二组]
 *  EPWM6 A/B -> C相 (GPIO10/11)[悬浮/第三组]
 */


/*
void Init_Svpwm_Gpio(void)
{
    EALLOW;

    // --- 开启 GPIO 时钟 ---
    // (通常在 InitSysCtrl 中已开启，为保险再次确认)
    // SysCtrlRegs.PCLKCR3.bit.GPIOINENCLK = 1;

    // GPAMUX1 寄存器控制 GPIO0 - GPIO15
    // 配置 GPIO0 - GPIO11 为 EPWM 功能 (置 1)
    GpioCtrlRegs.GPAPUD.bit.GPIO0  = 0; // 内部上拉
    GpioCtrlRegs.GPAPUD.bit.GPIO1  = 0;
    GpioCtrlRegs.GPAPUD.bit.GPIO2  = 0;
    GpioCtrlRegs.GPAPUD.bit.GPIO3  = 0;
    GpioCtrlRegs.GPAPUD.bit.GPIO4  = 0;
    GpioCtrlRegs.GPAPUD.bit.GPIO5  = 0;
    GpioCtrlRegs.GPAPUD.bit.GPIO6  = 0;
    GpioCtrlRegs.GPAPUD.bit.GPIO7  = 0;
    GpioCtrlRegs.GPAPUD.bit.GPIO8  = 0;
    GpioCtrlRegs.GPAPUD.bit.GPIO9  = 0;
    GpioCtrlRegs.GPAPUD.bit.GPIO10 = 0;
    GpioCtrlRegs.GPAPUD.bit.GPIO11 = 0;
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
 */

/*
 * 函数: Init_Svpwm_Module
 * 功能: 初始化 EPWM1-6 模块
 * 配置: 中心对齐, 互补输出, 死区, 10kHz, 严格同步
 */
/*
void Init_Svpwm_Module(void)
{
    // 在配置前，SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC 应该在 main 中被置 0
    // 配置完成后再置 1，以确保所有 PWM 计数器同时开始计数
    EALLOW;
    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0; // 配置前锁定同步

    // ==========================================================
    // 1. EPWM1 (Master) - 转矩 U相
    // ==========================================================
    EPwm1Regs.TBPRD = PWM_PRD;                       // 周期设置
    EPwm1Regs.TBPHS.half.TBPHS = 0x0000;             // 相位寄存器清零
    EPwm1Regs.TBCTR = 0x0000;                        // 计数器清零

    EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;   // 增减计数 (中心对齐)
    EPwm1Regs.TBCTL.bit.PHSEN = TB_DISABLE;          // 主模块禁用相位加载
    EPwm1Regs.TBCTL.bit.PRDLD = TB_SHADOW;           // 映射模式
    EPwm1Regs.TBCTL.bit.SYNCOSEL = TB_CTR_ZERO;      // 计数值为0时输出同步脉冲 (给从模块)
    EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;         // 时钟不分频
    EPwm1Regs.TBCTL.bit.CLKDIV = TB_DIV1;

    // 动作限定 (AQ): 中间高电平
    EPwm1Regs.AQCTLA.bit.CAU = AQ_CLEAR;             // 向上计数到 CMPA -> 高
    EPwm1Regs.AQCTLA.bit.CAD = AQ_SET;               // 向下计数到 CMPA -> 低

    // 死区 (DB)
    EPwm1Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;   // 使能双边死区
    EPwm1Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;        // 高有效互补 (Active High Complementary)
    EPwm1Regs.DBCTL.bit.IN_MODE = DBA_ALL;           // 源自 A 通道
    EPwm1Regs.DBRED = DEAD_TIME;                     // 上升沿延时
    EPwm1Regs.DBFED = DEAD_TIME;                     // 下降沿延时

    EPwm1Regs.ETSEL.bit.INTEN = 0;                   // 禁止 EPWM 触发 CPU 中断！
    // 配置 EPWM1_SOCA 脉冲来触发 ADC
    EPwm1Regs.ETSEL.bit.SOCAEN = 1;                  // 使能 SOCA 脉冲输出
    EPwm1Regs.ETSEL.bit.SOCASEL = ET_CTR_PRD;        // 计数值等于 PWM_PRD 时触发 (在三角波顶点触发采样)
    EPwm1Regs.ETPS.bit.SOCAPRD = ET_1ST;             // 每次满足条件都产生一次 SOCA 脉冲

    // ==========================================================
    // 2. EPWM2 (Slave) - 转矩 V相
    // ==========================================================
    EPwm2Regs.TBPRD = PWM_PRD;
    EPwm2Regs.TBPHS.half.TBPHS = 0x0000;
    EPwm2Regs.TBCTR = 0x0000;

    EPwm2Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;
    EPwm2Regs.TBCTL.bit.PHSEN = TB_ENABLE;            // 从模块：使能相位加载 (实现同步)
    EPwm2Regs.TBCTL.bit.PRDLD = TB_SHADOW;
    EPwm2Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;        // 同步信号穿透
    EPwm2Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;
    EPwm2Regs.TBCTL.bit.CLKDIV = TB_DIV1;

    EPwm2Regs.AQCTLA.bit.CAU = AQ_CLEAR;
    EPwm2Regs.AQCTLA.bit.CAD = AQ_SET;

    EPwm2Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
    EPwm2Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
    EPwm2Regs.DBCTL.bit.IN_MODE = DBA_ALL;
    EPwm2Regs.DBRED = DEAD_TIME;
    EPwm2Regs.DBFED = DEAD_TIME;

    // ==========================================================
    // 3. EPWM3 (Slave) - 转矩 W相
    // ==========================================================
    EPwm3Regs.TBPRD = PWM_PRD;
    EPwm3Regs.TBPHS.half.TBPHS = 0x0000;
    EPwm3Regs.TBCTR = 0x0000;

    EPwm3Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;
    EPwm3Regs.TBCTL.bit.PHSEN = TB_ENABLE;           // Slave
    EPwm3Regs.TBCTL.bit.PRDLD = TB_SHADOW;
    EPwm3Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;
    EPwm3Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;
    EPwm3Regs.TBCTL.bit.CLKDIV = TB_DIV1;

    EPwm3Regs.AQCTLA.bit.CAU = AQ_CLEAR;
    EPwm3Regs.AQCTLA.bit.CAD = AQ_SET;

    EPwm3Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
    EPwm3Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
    EPwm3Regs.DBCTL.bit.IN_MODE = DBA_ALL;
    EPwm3Regs.DBRED = DEAD_TIME;
    EPwm3Regs.DBFED = DEAD_TIME;

    // ==========================================================
    // 4. EPWM4 (Slave) - 悬浮 A相 (AH/AL)
    // ==========================================================
    EPwm4Regs.TBPRD = PWM_PRD;
    EPwm4Regs.TBPHS.half.TBPHS = 0x0000;
    EPwm4Regs.TBCTR = 0x0000;

    EPwm4Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;
    EPwm4Regs.TBCTL.bit.PHSEN = TB_ENABLE;           // Slave
    EPwm4Regs.TBCTL.bit.PRDLD = TB_SHADOW;
    EPwm4Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;
    EPwm4Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;
    EPwm4Regs.TBCTL.bit.CLKDIV = TB_DIV1;

    EPwm4Regs.AQCTLA.bit.CAU = AQ_CLEAR;
    EPwm4Regs.AQCTLA.bit.CAD = AQ_SET;

    EPwm4Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
    EPwm4Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
    EPwm4Regs.DBCTL.bit.IN_MODE = DBA_ALL;
    EPwm4Regs.DBRED = DEAD_TIME;
    EPwm4Regs.DBFED = DEAD_TIME;

    // ==========================================================
    // 5. EPWM5 (Slave) - 悬浮 B相 (BH/BL)
    // ==========================================================
    EPwm5Regs.TBPRD = PWM_PRD;
    EPwm5Regs.TBPHS.half.TBPHS = 0x0000;
    EPwm5Regs.TBCTR = 0x0000;

    EPwm5Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;
    EPwm5Regs.TBCTL.bit.PHSEN = TB_ENABLE;           // Slave
    EPwm5Regs.TBCTL.bit.PRDLD = TB_SHADOW;
    EPwm5Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;
    EPwm5Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;
    EPwm5Regs.TBCTL.bit.CLKDIV = TB_DIV1;

    EPwm5Regs.AQCTLA.bit.CAU = AQ_CLEAR;
    EPwm5Regs.AQCTLA.bit.CAD = AQ_SET;

    EPwm5Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
    EPwm5Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
    EPwm5Regs.DBCTL.bit.IN_MODE = DBA_ALL;
    EPwm5Regs.DBRED = DEAD_TIME;
    EPwm5Regs.DBFED = DEAD_TIME;

    // ==========================================================
    // 6. EPWM6 (Slave) - 悬浮 C相 (CH/CL)
    // ==========================================================
    EPwm6Regs.TBPRD = PWM_PRD;
    EPwm6Regs.TBPHS.half.TBPHS = 0x0000;
    EPwm6Regs.TBCTR = 0x0000;

    EPwm6Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;
    EPwm6Regs.TBCTL.bit.PHSEN = TB_ENABLE;           // Slave
    EPwm6Regs.TBCTL.bit.PRDLD = TB_SHADOW;
    EPwm6Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;
    EPwm6Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;
    EPwm6Regs.TBCTL.bit.CLKDIV = TB_DIV1;

    EPwm6Regs.AQCTLA.bit.CAU = AQ_CLEAR;
    EPwm6Regs.AQCTLA.bit.CAD = AQ_SET;

    EPwm6Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
    EPwm6Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
    EPwm6Regs.DBCTL.bit.IN_MODE = DBA_ALL;
    EPwm6Regs.DBRED = DEAD_TIME;
    EPwm6Regs.DBFED = DEAD_TIME;

    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1; // 使能ePWM时基时钟同步（TBCLK），确保所有ePWM模块时钟同步
    EDIS;
}
*/

/*
 * 辅助函数: 计算比较值并填充 CMPA (内部使用)
 * 输入: Ualpha, Ubeta, 以及要操作的PWM寄存器指针
 */
void Calculate_And_Update_CMP(float Ualpha, float Ubeta,
                              volatile struct EPWM_REGS *pwmA,
                              volatile struct EPWM_REGS *pwmB,
                              volatile struct EPWM_REGS *pwmC)
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

    // 2. 计算中间变量
    //这个整个的SVPWM流程里面省去了TS，因为后面都是等比例乘以寄存器最大值
    X = Ubeta*0.0721687836487f;    // 0.0721687836487 = 根号3/Udc，后续要加电压采样。Udc的作用是将电压指令转化为占空比时间
    Y = (0.866025f * Ualpha + 0.5f * Ubeta)*0.0721687836487f;
    Z = (-0.866025f * Ualpha + 0.5f * Ubeta)*0.0721687836487f;

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

    // 过调制处理
    if((t1+t2) > 1.0f) {
        float temp = t1 + t2;
        t1 /= temp;
        t2 /= temp;
    }

    // 转换为 CMPA 值 (PWM_PRD 已在头文件定义)
    float T_on1 = (1.0f - t1 - t2) * 0.5f * PWM_PRD;
    float T_on2 = T_on1 + t1 * PWM_PRD;
    float T_on3 = T_on2 + t2 * PWM_PRD;

    // 3. 赋值给寄存器 (根据扇区分配)
    switch (Sector)
    {
    case 3: // I
        pwmA->CMPA.half.CMPA = (Uint16)T_on1;
        pwmB->CMPA.half.CMPA = (Uint16)T_on2;
        pwmC->CMPA.half.CMPA = (Uint16)T_on3;
        break;
    case 1: // II
        pwmA->CMPA.half.CMPA = (Uint16)T_on2;
        pwmB->CMPA.half.CMPA = (Uint16)T_on1;
        pwmC->CMPA.half.CMPA = (Uint16)T_on3;
        break;
    case 5: // III
        pwmA->CMPA.half.CMPA = (Uint16)T_on3;
        pwmB->CMPA.half.CMPA = (Uint16)T_on1;
        pwmC->CMPA.half.CMPA = (Uint16)T_on2;
        break;
    case 4: // IV
        pwmA->CMPA.half.CMPA = (Uint16)T_on3;
        pwmB->CMPA.half.CMPA = (Uint16)T_on2;
        pwmC->CMPA.half.CMPA = (Uint16)T_on1;
        break;
    case 6: // V
        pwmA->CMPA.half.CMPA = (Uint16)T_on2;
        pwmB->CMPA.half.CMPA = (Uint16)T_on3;
        pwmC->CMPA.half.CMPA = (Uint16)T_on1;
        break;
    case 2: // VI
        pwmA->CMPA.half.CMPA = (Uint16)T_on1;
        pwmB->CMPA.half.CMPA = (Uint16)T_on3;
        pwmC->CMPA.half.CMPA = (Uint16)T_on2;
        break;
    default: // 当输入为 Sector = 0 时，T_on1=T_on2=T_on3=50%，此时给谁赋值都一样，通常按默认顺序
        pwmA->CMPA.half.CMPA = (Uint16)T_on1;
        pwmB->CMPA.half.CMPA = (Uint16)T_on1; // T_on1 和 T_on2, T_on3 值相等
        pwmC->CMPA.half.CMPA = (Uint16)T_on1;
        break;
    }
}

/*
 * 函数: Update_Svpwm
 * 功能: 更新 转矩侧 PWM (EPWM1, EPWM2, EPWM3)
 */
void Update_Svpwm(float Ualpha, float Ubeta)
{
    // 调用通用计算函数，操作 PWM1, PWM2, PWM3
    Calculate_And_Update_CMP(Ualpha, Ubeta, &EPwm1Regs, &EPwm2Regs, &EPwm3Regs);
}

/*
 * 函数: Update_Suspension_PWM
 * 功能: 更新 悬浮侧 PWM (EPWM4, EPWM5, EPWM6)
 */
void Update_Suspension_PWM(float Ualpha, float Ubeta)
{
    // 调用通用计算函数，操作 PWM4, PWM5, PWM6
    Calculate_And_Update_CMP(Ualpha, Ubeta, &EPwm4Regs, &EPwm5Regs, &EPwm6Regs);
}
