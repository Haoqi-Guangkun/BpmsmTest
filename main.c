/*
 * main.c
 *
 *  Created on: 2026年2月4日
 *      Author: L.YF
 */

//test1: 2026年3月6日，FOC 开环测试，实现小电机转动
//test2: 2026年4月14日，电流环测试（直流跟踪测试），电流方向检验：反向，CURRENT2_SCALE = -3.3333333333f。
//test3: 2026年4月20日，霍尔传感器调节，角度方向调整；电涡流传感器调节，移动方向检验
//test4: 2026年4月21日，电流环调节（阶跃跟踪测试），电流环PI调参：Kp = 10，Ki = 0.3（用连续时间积分）。
//       转子受力方向验证。闭环调节，转子单侧靠壁抖动。
//test5: 2026年4月22日，闭环调节，转子双侧贴壁抖动。
//test6: 2026年4月23日，闭环调节，d-q轴同调，位移解算修正，未完全微分，实现初步悬浮！
//       位移环PD调参：D-> Kp = 30000/35000/40000，Kd = 100；Q-> Kp = 15000/20000，Kd = 100
//test7: 2026年4月24日，闭环调节，转子预定位，修正角度偏置。位移环Ki调参。
//test8: 2026年4月27日，转矩部分电流环调节。电流环PI调参：Kp = 5，Ki = 2500
//test9: 2026年4月28日，转矩部分转速环闭环调节。转速环PI调参：Kp = 0.01，Ki = 0
//test10: 2026年4月29日，转矩部分，降低转速采样和转速环计算频率，为1Khz
//test11: 2026年4月30日，重新调整悬浮系统位移环、电流环参数，转速可稳定运行在 3100 RPM，但伴随一些噪声


#include "DSP2833x_Device.h"    // DSP2833x芯片寄存器定义头文件
#include "DSP2833x_Examples.h"  // DSP2833x例程通用函数头文件
#include "Adc.h"
#include "control.h"
#include "wave.h"

// 将InitFlash函数段定位到RAM运行区（ramfuncs段），提升执行速度
#pragma CODE_SECTION(InitFlash, "ramfuncs");

// PWM 周期定义
Uint16 PWM_PRD = 7500;    // 10kHz @ 150MHz (Up-Down)
Uint16 DEAD_TIME = 150;   // 对应死区时间：150/150MHz = 1us

// ================= 全局变量定义 =================
volatile Uint16 EnSystem = 0;                        // 系统总使能 (手动置位)
volatile Uint16 EnDrive = 0;                         // 驱动使能 (手动置位)
volatile Uint16 EnSus = 0;                           // 悬浮使能 (手动置位)
//volatile Uint16 EnAlign = 0;                         // 预定位触发信号
volatile Uint16 AlignDone = 0;                       // 定位完成标志
volatile float ElecTheta_pu = 0.0f;                  // 全局电角度标幺值 (0.0~1.0)，代表修正后的电角度
volatile float Hall_Angle_Offset = 0.0f;             // 捕获到的霍尔零位偏差
volatile float Pos_D_Ref = 0.0f, Pos_Q_Ref = 0.0f;   // d、q轴目标位移 (0，即中心)
volatile float Speed_Ref_rpm = 300.0f;               // 目标给定转速 (RPM)

// ADC 零偏参数
float offsetA = 0, offsetB = 0, offsetC = 0;                             // 悬浮绕组偏置
float offsetU = 0, offsetV = 0, offsetW = 0;                             // 转矩绕组偏置
float Hall_OffsetA = 1.50f, Hall_OffsetB = 1.50f, Hall_OffsetC = 1.50f;  // 线性霍尔偏置
float Eddy_OffsetA = 1.50f, Eddy_OffsetB = 1.50f, Eddy_OffsetC = 1.50f;  // 涡流传感器中心点零偏 (转子机械居中时的电压)

float K1 = 0.998f, K2 = 0.001999f;                                       // 低通滤波系数，凑整 K1+K2=1
// 电流变量
float Ia=0, Ib=0, Ic=0;          // 悬浮三相电流
float Iu=0, Iv=0, Iw=0;          // 转矩三相电流
// 传感器变量
float ha=0, hb=0, hc=0;          // 霍尔传感器电压值
float Ea=0, Eb=0, Ec=0;          // 霍尔传感器电压值

// 数据结构体实例化
AdcData_t g_AdcData = {0};

WAVE testWave = WAVE_DEFAULTS;    // 测试波形实例化

// 全局区定义，这里在 RAM 中分配了固定的空间
PIDREG pid_id_sus   = PIDREG_ISUS_DEFAULTS;  // 悬浮d轴电流环
PIDREG pid_iq_sus   = PIDREG_ISUS_DEFAULTS;  // 悬浮q轴电流环
PIDREG pid_id  = PIDREG_ID_DEFAULTS;         // 转矩d轴电流环
PIDREG pid_iq  = PIDREG_IQ_DEFAULTS;         // 转矩q轴电流环
PIDREG pid_spd = PIDREG_SPD_DEFAULTS;        // 速度环
PARK   park_eddy = PARK_DEFAULTS;

// 数据记录
#define PLOT_SIZE 600            // 记录600个点，10kHz下对应60ms的波形
float buf_ref[PLOT_SIZE];
float buf_fdb[PLOT_SIZE];
Uint16 plot_idx = 0;             // 缓冲区索引
//Uint16 plot_flag = 0;            // 绘图触发标志

// ================= 中断服务函数声明 =================
//interrupt void adc_offset_isr(void);
interrupt void adc_isr(void);

void main(void)
{
    testWave.type = SIGNAL_STEP;      // 阶跃信号,测试电流环用
    testWave.value = 1.0f;          // 阶跃幅值
   // testWave.system_freq = 10000.0f;  // 中断频率：10kHz

    InitSysCtrl();         //初始化系统控制寄存器，配置150MHz系统时钟、75MHz ADC时钟

    MemCopy(&RamfuncsLoadStart, &RamfuncsLoadEnd, &RamfuncsRunStart);
    InitFlash();           //初始化Flash()

    // 1. 模块 GPIO 初始化
    InitEPwmGpio();        // 初始化EPwm模块管脚（此处采用 TI 官方标准的ePWM 引脚初始化框架）
//    Init_Svpwm_Gpio();     // 初始化 PWM1-6 引脚

    DINT;                  // 禁用CPU全局中断（Disable Interrupts），防止初始化过程中触发中断
    InitPieCtrl();         // 初始化PIE（外设中断扩展）控制器
    IER = 0x0000;          // 清空CPU中断使能寄存器，禁用所有CPU级中断
    IFR = 0x0000;          // 清空CPU中断标志寄存器，清除所有挂起的中断标志
    InitPieVectTable();    // 初始化中断向量表，将中断服务函数与向量表关联

    // 2. 模块功能初始化
    Init_Adc();                              // ADC 级联模式（SOCA触发配置）
//    Init_Svpwm_Module();
    InitEPwm();                              // PWM 模式、死区（SOCA生成配置）

    // 3. 零偏校准阶段
    // 此时全局中断 EINT 尚未开启，CPU 不会跳入中断服务程序

    // A. 强制封锁 PWM 输出，确保校准时没有电流流过绕组
    Stop_System();

    // B. 循环采样 20000 次 (约 2 秒 @ 10kHz)
    Uint32 i = 0;
    for(i = 0; i < 20000; i++)
    {
        // 等待 ADC 转换完成标志位 (SEQ1 转换结束)
        // “轮询”等待
        while (AdcRegs.ADCST.bit.INT_SEQ1 == 0) {
            // 等待硬件 SOCA 触发并完成采样
        }
        // 读取采样到的原始数据
        Read_Adc();
        // 舍弃前 5000 次，等待传感器运放电路达到电气稳定
        if (i >= 5000)
        {
            // 一阶低通滤波累积零偏值
            offsetU = K1 * offsetU + K2 * g_AdcData.Filtered_U;  //Phase U offset
            offsetV = K1 * offsetV + K2 * g_AdcData.Filtered_V;
            offsetW = K1 * offsetW + K2 * g_AdcData.Filtered_W;
            offsetA = K1 * offsetA + K2 * g_AdcData.Filtered_A;
            offsetB = K1 * offsetB + K2 * g_AdcData.Filtered_B;
            offsetC = K1 * offsetC + K2 * g_AdcData.Filtered_C;
        }

        // 手动复位排序器并清除中断标志，准备下一次 SOCA 触发
        AdcRegs.ADCTRL2.bit.RST_SEQ1 = 1;
        AdcRegs.ADCST.bit.INT_SEQ1_CLR = 1;
    }

    // C. 校准完成，解除 PWM 锁定
//    Resume_System();

    // 4. 映射中断
    EALLOW;                                  //允许修改受保护的寄存器
    PieVectTable.ADCINT = &adc_isr;          //ADC中断服务程序入口，先进入零偏校准中断
    EDIS;                                    //退出写保护

    // 5. 开启 Core INT1 和 PIE Group 1 (ADCINT 所在组)
    IER |= M_INT1;                           //开启 CPU 级 INT1
    PieCtrlRegs.PIEIER1.bit.INTx6 = 1;       //开启 PIE 级 Group 1 的第 6 个中断 (ADCINT)
    EINT;                                    //开启全局中断
    ERTM;                                    //开启实时中断扩展

    for(;;)
    {
        // 主循环，可通过 CCS Expression 窗口随时修改 EnSystem, EnSus, EnDrive 的值
    }
}

// ================= 中断：主控制中断 =================
interrupt void adc_isr(void)
{
    // 1. 读取数据与转换物理量
    Read_Adc();         // 读取最新的 ADC 采样值

    Convert_Adc_Data(); // 换算物理量（去除零偏）

    // 2. 实时计算公共反馈变量（角度、转速、电流）
    Calculate_Feedback();

    // 3. 根据手动标志位，调用各个控制模块
    if (EnSystem == 1)
    {
        // 悬浮模块先调用
        if (EnSus == 1)
        {
//            wave_calc(&testWave);
//            pid_iq_sus.Ref = testWave.out;  // 将阶跃波形给到 IQ 参考,或者0.01。不用要注释掉！
//            pid_id_sus.Ref = 0;  // 将阶跃波形给到 ID 参考
//            if(plot_idx < PLOT_SIZE)
//            {
//                buf_ref[plot_idx] = pid_iq_sus.Ref;   // pid_iq_sus.Ref;
//                buf_fdb[plot_idx] = park_eddy.Ds;
//                plot_idx++;
//            }
//            else plot_idx = 0;
            Control_Suspension();
        }
        else Stop_Suspension();
        // 驱动模块调用
//        if ((EnDrive == 1 || EnAlign == 1) && EnSus_Flag == 1)
        if (EnDrive == 1 && EnSus == 1)
        {
//            wave_calc(&testWave);
//            pid_id.Ref = testWave.out;  // 将阶跃波形给到 IQ 参考,或者0.01。不用要注释掉！
//            pid_iq.Ref = 0.1f;//将阶跃波形给到 ID 参考
//            if(plot_idx < PLOT_SIZE)
//            {
//                buf_ref[plot_idx] = pid_id.Ref;   // pid_iq_sus.Ref;
//                buf_fdb[plot_idx] = pid_spd.Fdb;  //pid_id.Fdb;
//                plot_idx++;
//            }
            Control_Torque();
        }
        else Stop_Torque();
    }
    else Stop_System(); // 系统未使能时，关闭所有 PWM
    // 4. 清除中断标志，准备下一次
    AdcRegs.ADCTRL2.bit.RST_SEQ1 = 1;
    AdcRegs.ADCST.bit.INT_SEQ1_CLR = 1;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

