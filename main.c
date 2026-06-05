/*
 * main.c
 *
 * Created on: 2026年2月4日
 * Author: L.YF
 */

// test1:  2026年3月6日，FOC 驱动矢量控制电流环小电流跑通
// test2:  2026年4月14日，修改电流传感器采样比例系数系数：修改 CURRENT2_SCALE =
// -3.3333333333f test3:
// 2026年4月20日，悬浮位置环与电流环联合调试，测试静态悬浮效果 test4:
// 2026年4月21日，调试转速外环 PI 参数：Kp = 10, Ki = 0.3，观察动态响应与超调
//         并在高转速下微调电流环解耦参数
// test5:  2026年4月22日，优化电涡流传感器差分采样与滤波算法
// test6:  2026年4月23日，测试悬浮 d-q
// 轴解耦与位置刚度。在未转动时进行静态悬浮稳定性测试
//         位置环 PD 参数配置：D轴 -> Kp = 30000/35000/40000, Kd = 100; Q轴 ->
//         Kp = 15000/20000, Kd = 100
// test7:  2026年4月24日，动态悬浮状态下加入位置环积分，测试位置环 Ki
// 参数抗扰动能力 test8:  2026年4月27日，调试悬浮电流内环 PI 参数：Kp = 5, Ki =
// 2500 test9:  2026年4月28日，调试转速环在极低增益下的稳定性：Kp = 0.01, Ki = 0
// test10: 2026年4月29日，将串口上位机数据发送频率修改分频，调整为 1kHz 刷新率
// test11: 2026年4月30日，联合调试悬浮与驱动旋转，成功实现最高稳定转速达 3100
// RPM，记录动态悬浮波动

#include "Adc.h"
#include "DSP2833x_Device.h"   // DSP2833x 芯片外设寄存器头文件
#include "DSP2833x_Examples.h" // DSP2833x 示例及通用函数头文件
#include "Sci.h"
#include "control.h"
#include "wave.h"

// 将 InitFlash 函数定位到 RAM 中运行（提高从 Flash 启动后的执行速度）
#pragma CODE_SECTION(InitFlash, "ramfuncs");

// PWM 参数配置
Uint16 PWM_PRD = 7500;  // 10kHz 载波频率 @ 150MHz 系统时钟 (递增/递减计数模式)
Uint16 DEAD_TIME = 150; // 死区时间配置: 150 / 150MHz = 1us

// ================= 系统控制状态变量 =================
volatile Uint16 EnSystem = 0; // 系统总使能标志 (1: 使能, 0: 关闭)
volatile Uint16 EnDrive = 0;  // 驱动/旋转使能标志
volatile Uint16 EnSus = 0;    //  悬浮使能标志

// volatile Uint16 EnAlign = 0; // 电机对齐/定位使能标志
volatile Uint16 AlignDone = 0;           // 对齐/定位完成标志
volatile float ElecTheta_pu = 0.0f;      // 标幺化电角度 (0.0~1.0)，对应 0~360度
volatile float Hall_Angle_Offset = 0.0f; // 霍尔传感器角度偏置量
volatile float Pos_D_Ref = 0.0f,
               Pos_Q_Ref = 0.0f;       // d轴与q轴的目标悬浮位置 (0表示中心位置)
volatile float Speed_Ref_rpm = 500.0f; // 电机目标转速 (RPM)

// ADC 偏置校准变量 (用于消除运放电路的硬件直流偏置)
float offsetA = 0, offsetB = 0, offsetC = 0; // 悬浮电流 A/B/C 相偏置
float offsetU = 0, offsetV = 0, offsetW = 0; // 驱动电流 U/V/W 相偏置
float Hall_OffsetA = 1.50f, Hall_OffsetB = 1.50f,
      Hall_OffsetC = 1.50f; // 霍尔传感器偏置
float Eddy_OffsetA = 1.50f, Eddy_OffsetB = 1.50f,
      Eddy_OffsetC = 1.50f; // 电涡流位移传感器偏置 (用于悬浮间隙检测)

float K1 = 0.998f,
      K2 =
          0.001999f; // 低通滤波器系数 (K1 + K2 = 1)，用于偏置电压的一阶滤波推导
// 实际物理采样值
float Ia = 0, Ib = 0, Ic = 0; // 悬浮绕组三相电流
float Iu = 0, Iv = 0, Iw = 0; // 驱动绕组三相电流
// 位移与角度原始信号
float ha = 0, hb = 0, hc = 0; // 霍尔传感器采集值
float Ea = 0, Eb = 0, Ec = 0; // 电涡流传感器采集值

// ADC 原始数据结构体
AdcData_t g_AdcData = {0};

WAVE testWave = WAVE_DEFAULTS; // 波形发生器结构体（用于阶跃或正弦测试）

// PID/PI 控制器结构体实例化（这些变量通常需要在 RAM 中快速运行）
volatile PIDREG gSuspCurrD_PID =
    PIDREG_ISUS_DEFAULTS; // 悬浮 d 轴电流 PID 控制器
volatile PIDREG gSuspCurrQ_PID =
    PIDREG_ISUS_DEFAULTS;                      // 悬浮 q 轴电流 PID 控制器
PIDREG gTorqCurrD_PI = PIDREG_CURRNT_DEFAULTS; // 驱动/转矩 d 轴电流 PI 控制器
PIDREG gTorqCurrQ_PI = PIDREG_CURRNT_DEFAULTS; // 驱动/转矩 q 轴电流 PI 控制器
PIDREG gSpd_PI = PIDREG_SPD_DEFAULTS;          // 转速外环 PI 控制器
PIDREG gSuspPosD_PID = PIDREG_dPOS_DEFAULTS;   // 悬浮 d 轴位置 PID 控制器
PIDREG gSuspPosQ_PID = PIDREG_qPOS_DEFAULTS;   // 悬浮 q 轴位置 PID 控制器
// 前馈补偿的初始化
UFC gUnbalanceComp = UNBALANCE_COMP_DEFAULTS;
// 外部引用的算法与状态变量
extern PARK park_eddy;
extern float Speed_Target_Ramp;
extern float Speed_Fdb_rpm;
extern float Pos_D_Filtered;
extern float Raw_Hall_Theta_rad; // 反正切计算角度
extern float ElecTheta_rad;      // 电角度

// 数据示波器/画图缓冲区配置
#define PLOT_SIZE 600 // 缓冲区大小 600 点（10kHz 采样率下可记录 60ms 数据）
float buf_ref[PLOT_SIZE];
float buf_fdb[PLOT_SIZE];
Uint16 plot_idx = 0; // 画图缓冲区索引
// Uint16 plot_flag = 0;
float x = 0.0f;
Uint16 j = 0;

// ================= 中断服务函数声明 =================
// interrupt void adc_offset_isr(void);
interrupt void adc_isr(void);

///================================ 主函数 =======================//
void main(void) {
  testWave.type = SIGNAL_STEP; // 设置测试波形为阶跃信号，用于系统阶跃响应测试
  testWave.value = 1.0f;       // 阶跃幅值
  // testWave.system_freq = 10000.0f;  // 中断频率 10kHz

  InitSysCtrl(); // 初始化系统时钟，配置核心频率为
                 // 150MHz，外设时钟（如高速外设时钟 HSPCLK）为 75MHz 以供给 ADC

  // 将 Flash 中的代码复制到 RAM 中运行
  MemCopy(&RamfuncsLoadStart, &RamfuncsLoadEnd, &RamfuncsRunStart);
  InitFlash(); // 初始化 Flash 运行环境（如等待周期设置）

  // 1. 初始化 GPIO 引脚配置
  InitEPwmGpio(); // 初始化 ePWM 的 GPIO 端口引脚（使用 TI 标准配置）
                  // 初始化完成后 ePWM 外设即可控制功率管
  //    Init_Svpwm_Gpio();     // 初始化 PWM1-6 硬件引脚

  DINT;          // 关闭 CPU 总中断，防止在初始化未完成时触发中断导致程序跑飞
  InitPieCtrl(); // 初始化 PIE（外设中断扩展）控制器
  IER = 0x0000;  // 清空 CPU 中断使能寄存器
  IFR = 0x0000;  // 清空 CPU 中断标志寄存器
  InitPieVectTable(); // 初始化 PIE 中断向量表，填充默认的硬错误处理入口

  // 2. 初始化核心硬件模块
  Init_Adc(); // 初始化 ADC 模块，配置为由 ePWM 的 SOCA（启动转换A）信号硬件触发
              //    Init_Svpwm_Module();
  InitEPwm(); // 初始化 EPWM 模块，设置时基、死区并生成 SOCA 触发信号
  SCI_Init(1152000); // 初始化 SCI 串口，波特率设为 1152000bps 用于高速数据传输

  // 3. 上电自动零点校准（ADC 硬件偏置校准）
  // 此时全局中断 EINT 尚未开启，CPU 将通过软件轮询方式完成校准过程

  // A. 首先强制停止系统，关闭所有 PWM 输出，确保此时逆变器无输出电流
  Stop_System();

  // B. 循环读取 20000 次 ADC（在 10kHz 下耗时约 2 秒）进行偏置均值计算
  Uint32 i = 0;
  for (i = 0; i < 20000; i++) {
    // 等待 ADC 的 SEQ1 状态机转换结束标志置位
    while (AdcRegs.ADCST.bit.INT_SEQ1 == 0) {
      // 硬件正等待来自 ePWM 的 SOCA 触发信号并自动采样转换
    }
    // 读取最新的各个通道转换结果
    Read_Adc();
    // 从第 5000 次循环开始累加，前 5000
    // 次用于跳过刚上电时模拟电路的暂态不稳定期
    if (i >= 5000) {
      // 使用一阶低通数字滤波器计算各通道的直流偏置电压值
      offsetU = K1 * offsetU + K2 * g_AdcData.Filtered_U; // Phase U offset
      offsetV = K1 * offsetV + K2 * g_AdcData.Filtered_V; // Phase V offset
      offsetW = K1 * offsetW + K2 * g_AdcData.Filtered_W; // Phase W offset
      offsetA = K1 * offsetA + K2 * g_AdcData.Filtered_A; // Phase A offset
      offsetB = K1 * offsetB + K2 * g_AdcData.Filtered_B; // Phase B offset
      offsetC = K1 * offsetC + K2 * g_AdcData.Filtered_C; // Phase C offset
    }

    // 复位 SEQ1 状态机并清除中断标志位，准备下一次由 ePWM 硬件触发的 SOCA
    AdcRegs.ADCTRL2.bit.RST_SEQ1 = 1;
    AdcRegs.ADCST.bit.INT_SEQ1_CLR = 1;
  }

  // C. 校准结束，可在必要时恢复系统 PWM 配置
  //    Resume_System();

  // 4. 注册中断服务子程序 (ISR)
  EALLOW; // 开启寄存器修改保护（允许修改受保护的配置寄存器）
  PieVectTable.ADCINT = &adc_isr; // 将 ADC 中断向量指向实际的闭环控制中断函数
  EDIS;                           // 关闭寄存器修改保护

  // 5. 开启核心及外设中断使能
  IER |= M_INT1; // 使能 CPU 第一组中断 (INT1)
  PieCtrlRegs.PIEIER1.bit.INTx6 =
      1; // 使能 PIE 模块中第一组的第 6 个通道 (即 ADCINT)
  EINT;  // 开启 CPU 全局中断
  ERTM;  // 开启实时调试中断支持

  while(1) {
// 检查中断是否送来了新采样
        if (gUnbalanceComp.NewData_Flag == 1)
        {
            // 进来后立刻把标志位清零，防止重复计算
            gUnbalanceComp.NewData_Flag = 0; 
            
            // 在这里安心、大胆地运行繁重的前馈算法
            // 即使它被位置环中断中途打断 3~4 次，也完全不会对中断的实时性造成任何威胁！
            Unbalance_Comp_Run(&gUnbalanceComp, 
                               gUnbalanceComp.x_raw_buf, 
                               gUnbalanceComp.y_raw_buf, 
                               gUnbalanceComp.theta_buf);

    // SCI串口数据发送
    x = SCI_GetOverflowCount(); // 检测串口接收/发送溢出计数
    SCI_ServiceTx(); // 执行串口发送服务函数，负责将缓冲区数据发出
  }
}
}

// ================= 中断服务程序（10kHz 核心闭环控制环） =================
interrupt void adc_isr(void) {
  // 1. 读取最新的采样数据
  Read_Adc(); // 获取 ADC 各通道寄存器的原始转换数值

  Convert_Adc_Data(); // 扣除校准所得偏置量，并将原始数值转换为实际物理量

  // 2. 根据电流与位移传感器信号更新计算反馈状态
  Calculate_Feedback();

  // 3. 上位机数据可视化（分频发送，降低串口带宽压力）
  j++;
  if (j >= 100) { // 100次中断执行一次（100Hz 刷新率），上传实时波形数据
    j = 0;
    float32 tx_data[3];
    tx_data[0] = park_eddy.Ds;      // 待观测变量1: 如电涡流转换后的D轴位移反馈
    tx_data[1] = park_eddy.Qs;      // 待观测变量2: 目标位移/转矩分量
    tx_data[2] = Speed_Target_Ramp; // 待观测变量3: 斜坡转速给定值
    SendFloatArray_JustFloat(tx_data,
                             3U); /* 通过串口发送浮点数数组给上位机波形软件 */
  }

  // 4. 系统多状态机逻辑控制
  if (EnSystem == 1) { // 检查系统总开关是否打开

    // --- A. 磁悬浮回路控制逻辑 ---
    if (EnSus == 1) {
      //            wave_calc(&testWave);
      //            gSuspCurrQ_PID.Ref = testWave.out;  //
      //            将测试波形输出作为Q轴悬浮电流的输入给定 IQ
      //            参考，比如给定0.01观察动态响应；  gSuspCurrD_PID.Ref = 0; //
      //            将D轴悬浮电流给定设为0参考。   if(plot_idx < PLOT_SIZE)
      //            {
      //                buf_ref[plot_idx] = gSuspCurrQ_PID.Ref;   //
      //                gSuspCurrQ_PID.Ref; buf_fdb[plot_idx] = park_eddy.Ds;
      //                plot_idx++;
      //            }
      //            else plot_idx = 0;
      Control_Suspension(); // 执行磁悬浮位置-电流双闭环控制算法
    } else
      Stop_Suspension(); // 关闭悬浮控制输出，清除悬浮积分器

    // --- B. 驱动/旋转回路控制逻辑 ---
    //        if ((EnDrive == 1 || EnAlign == 1) && EnSus_Flag == 1)
    if (EnDrive == 1 &&
        EnSus ==
            1) { // 只有在总使能、旋转使能、且悬浮成功稳定时，才可以启动旋转驱动
      //            wave_calc(&testWave);
      //            gTorqCurrD_PI.Ref = testWave.out;  //
      //            将测试波形输出作为转矩电流控制环输入 IQ
      //            参考，比如给定0.01观察动态响应；  gTorqCurrQ_PI.Ref =
      //            0.1f;//将转矩电流给定设为 ID 参考。 if(plot_idx < PLOT_SIZE)
      //            {
      //                buf_ref[plot_idx] = gTorqCurrD_PI.Ref;   //
      //                gSuspCurrQ_PID.Ref; buf_fdb[plot_idx] = gSpd_PI.Fdb;
      //                //gTorqCurrD_PI.Fdb; plot_idx++;
      //            }
      Control_Torque(); // 执行旋转电机 FOC 矢量控制算法（速度-电流双闭环）
    } else
      Stop_Torque(); // 关闭驱动控制输出，清除驱动积分器
  } else
    Stop_System(); // 系统未使能时强制关闭所有 PWM 信号

  // 5. 恢复中断控制状态，为下一次中断做准备
  AdcRegs.ADCTRL2.bit.RST_SEQ1 = 1;   // 复位 SEQ1 状态机
  AdcRegs.ADCST.bit.INT_SEQ1_CLR = 1; // 清除 ADC 自身的中断标志位
  PieCtrlRegs.PIEACK.all =
      PIEACK_GROUP1; // 向 PIE 模块发送 ACK 响应，允许接收第一组后续的中断请求
}

