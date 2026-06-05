/*
 * control.c
 *
 * Created on: 2026年3月16日
 * Author: L.YF
 */

#include "control.h"
#include "Svpwm.h"
#include <math.h>

// ===================== 滤波滑动窗口大小 =====================
#define FILTER_SIZE                                                            \
  10 // 窗口大小 10~20 效果最好；越大滤波效果越好，但是相位滞后越严重

// 内部动态变量
static Uint32 Align_Counter = 0; // 预定位时间计数器
// 角度相关变量
float Raw_Hall_Theta_rad = 0.0f; // 霍尔传感器计算得到的原始电角度（弧度值）
static float Raw_Hall_Theta_pu =
    0.0f;                   // 霍尔直接计算出的原始标幺化角度（0.0 ~ 1.0）
extern float ElecTheta_pu;  // 驱动环最终用于变换的标幺化角度 (0.0 ~ 1.0)
float ElecTheta_rad = 0.0f; // 驱动环实际电角度（弧度值）
static float ElecTheta_pre = 0.0f; // 上一次的角度值（用于差分计算转速）
static float temp_sus_theta_pu =
    0.0f; // 暂存变量，用于确定转矩角度与悬浮角度的解耦关系
static float ElecTheta_Sus_pu = 0.0f;  // 悬浮系统专用的标幺化角度（0.0 ~ 1.0）
static float ElecTheta_Sus_rad = 0.0f; // 悬浮系统专用的电角度（弧度值）
static float dTheta = 0.0f;            // 角度差值（用于计算瞬时速度）
// static Uint16 div_cnt = 0;          // 转速环分频计数器

// 角度滤波相关变量（通常用于对正余弦信号进行滤波）
static float h_alpha_filt = 0.0f;
static float h_beta_filt = 0.0f;

// 速度滤波专用缓冲区
static float speed_buffer[FILTER_SIZE] = {0.0f}; // 滑动平均值滤波器缓冲区
static Uint16 filter_idx = 0;                    // 环形缓冲区指针
static float speed_sum = 0.0f; // 实时累加和（避免每轮循环重新遍历，提高效率）
static Uint16 data_count = 0;  // 启动阶段已填充的数据个数

// 电机状态机控制模式
typedef enum {
  MODE_IDLE = 0,  // 空闲待机模式
  MODE_ALIGN = 1, // 强迫起动/转子预定位模式（用于无位置或需要对齐的情况）
  MODE_RUN = 2    // 闭环运行模式
} MOTOR_MODE;

volatile MOTOR_MODE MotorMode = MODE_RUN; // 默认工作模式设为：闭环运行模式

// 转速环控制变量
extern float Speed_Ref_rpm;        // 外部设定的目标理想转速 (RPM)
static float Speed_Raw_rpm = 0.0f; // 差分得到的瞬时转速（未滤波，噪声大）
float Speed_Fdb_rpm = 0.0f;        // 滤波后的实际反馈转速 (RPM)
// 转速斜坡（Ramp）发生器变量（防止给定阶跃导致电流过载）
float Speed_Target_Final = 1000.0f; // 最终期望达到的目标转速 (RPM)
float Speed_Target_Ramp = 0.0f; // 当前斜坡指令转速 (作为速度环当前的实际 Ref)
float Speed_Accel_Step =
    0.2f; // 每次中断的步进值 (RPM/每步), 对应加速度约 200 RPM/s
static Uint16 Speed_Loop_Cnt =
    0; // 速度外环分频计数器（通常速度环周期大于电流环）

// 悬浮位置控制变量
float Pos_D_Filtered = 0.0f,
      Pos_Q_Filtered = 0.0f; // 经低通滤波后的 d 轴和 q 轴实际悬浮位置反馈
static float Force_D_Ref = 0.0f,
             Force_Q_Ref = 0.0f; // 位置环 PID 算出的目标悬浮力给定值
static float i_Bd_ref = 0.0f,
             i_Bq_ref = 0.0f; // 换算得到的目标悬浮 d/q 轴控制电流

// 悬浮同频补偿控制标志
volatile Uint16 UFC_flag = 0; // 不平衡补偿标志
// 变换器结构体实例（双控制回路：环1通常用于驱动旋转，环2/eddy通常用于悬浮）
CLARK clark1 = CLARK_DEFAULTS;
PARK park1 = PARK_DEFAULTS;
IPARK ipark1 = IPARK_DEFAULTS;
CLARK clark2 = CLARK_DEFAULTS;
PARK park2 = PARK_DEFAULTS;
IPARK ipark2 = IPARK_DEFAULTS;
PARK park_eddy = PARK_DEFAULTS; // 用于电涡流传感器的坐标变换

// 外部 PID 控制器句柄
extern PIDREG gSuspCurrD_PID; // 悬浮电流内环 d 轴 PID
extern PIDREG gSuspCurrQ_PID; // 悬浮电流内环 q 轴 PID
extern PIDREG gTorqCurrD_PI;  // 驱动转矩电流内环 d 轴 PI
extern PIDREG gTorqCurrQ_PI;  // 驱动转矩电流内环 q 轴 PI
extern PIDREG gSpd_PI;        // 速度外环 PI
extern PIDREG gSuspPosD_PID;  // 悬浮位置外环 d 轴 PID
extern PIDREG gSuspPosQ_PID;  // 悬浮位置外环 q 轴 PID
// UFC
extern UFC gUnbalanceComp;
// 声明外部硬件采样与偏置变量（定义在 main.c 中）
extern float offsetA, offsetB, offsetC;
extern float offsetU, offsetV, offsetW;
extern float Hall_OffsetA, Hall_OffsetB, Hall_OffsetC;
extern float Eddy_OffsetA, Eddy_OffsetB, Eddy_OffsetC;
extern float Ia, Ib, Ic;           // 悬浮绕组实际三相电流
extern float Iu, Iv, Iw;           // 转矩驱动绕组实际三相电流
extern float ha, hb, hc;           // 霍尔传感器采样得到的原始电压值
extern float Ea, Eb, Ec;           // 电涡流位移传感器采样得到的原始电压值
extern float Pos_D_Ref, Pos_Q_Ref; // d轴与q轴的目标悬浮中心给定位置
extern float SENSOR_POS_SCAL;      // 位移传感器物理量转换比例系数
extern float Hall_Angle_Offset;    // 霍尔安装误差角度补偿量
extern Uint16 AlignDone, EnSus;

float cutoff_fre = 500.0f; // 默认截止频率设为 500Hz
float LPF_out[10] = {0};   // 多通道低通滤波器输出数组
float LPF_last[10] = {0};  // 多通道低通滤波器历史状态数组

/**
 * @brief 一阶低通数字滤波器 (LPF)，支持自定义截止频率
 * @note 适用于 10kHz 采样频率系统。
 * 推导基于连续域传递函数 G(s) = wc / (s + wc)，通过后向差分法离散化：
 * dy/dt = wc * (x - y)  =>  (y[k] - y[k-1])/Ts = wc * (x[k] - y[k])
 * 整理可得：y[k] = A * x[k] + (1 - A) * y[k-1]
 * 其中 A = (2*pi*fc*Ts) / (1 + 2*pi*fc*Ts)
 */
float one_order_LPF(float cutoff_fre, float input, Uint16 i) {
  const float Ts = 0.0001f;         // 采样周期 Ts = 0.0001s (10kHz)
  static float last_cutoff = -1.0f; // 缓存上一次的截止频率
  static float A = 0.0f;            // 滤波系数 A

  // 只有当截止频率发生改变时，才重新计算滤波系数
  // A，避免每轮都进行浮点乘除法，极大节省算力
  if (cutoff_fre != last_cutoff) {
    float alpha = 2.0f * PI * cutoff_fre * Ts;
    A = alpha / (1.0f + alpha);
    last_cutoff = cutoff_fre;
  }

  // 滤波器执行算法
  LPF_out[i] = A * input + (1.0f - A) * LPF_last[i];
  LPF_last[i] = LPF_out[i]; // 更新历史状态
  return LPF_out[i];
}

/**
 * @brief 转速反馈滑动窗口平均值滤波器
 * @param input 瞬时计算出的粗糙转速值
 * @return 滤波平滑后的转速值值
 */
float Speed_Filter(float input) {
  // 1. 减去即将被移出窗口的最老的数据
  speed_sum -= speed_buffer[filter_idx];
  // 2. 将最新数据存入当前缓冲区位置
  speed_buffer[filter_idx] = input;
  // 3. 实时累加最新输入的数据
  speed_sum += speed_buffer[filter_idx];

  // 4. 在系统刚启动、数据尚未填满窗口前，动态修正平均分母
  if (data_count < FILTER_SIZE) {
    data_count++;
  }

  // 5. 计算当前窗口内的算术平均转速
  float output = speed_sum / data_count;

  // 6. 维护环形队列的缓冲区索引
  filter_idx++;
  if (filter_idx >= FILTER_SIZE) {
    filter_idx = 0;
  }
  return output;
}

// ==================== 1. 读取ADC原始数据 ====================
void Read_Adc(void) {
  // 读取寄存器数值并将其转换为标称电压值
  // 转换公式: Voltage = (Digital_Value * 3.0V) / 4096.0
  // 注意: TMS320F28335 的 ADCRESULTx 寄存器是左对齐的，低12位有效数据需要右移 4
  // 位 (>> 4)
  const float k = 3.0f / 4096.0f; // 12位ADC数字量转电压系数

  g_AdcData.Filtered_U =
      (float)(AdcRegs.ADCRESULT0 >> 4) * k; // 驱动绕组 U 相电流采样电压
  g_AdcData.Filtered_V =
      (float)(AdcRegs.ADCRESULT1 >> 4) * k; // 驱动绕组 V 相电流采样电压
  g_AdcData.Filtered_W =
      (float)(AdcRegs.ADCRESULT2 >> 4) * k; // 驱动绕组 W 相电流采样电压
  g_AdcData.Filtered_A =
      (float)(AdcRegs.ADCRESULT3 >> 4) * k; // 悬浮绕组 A 相电流采样电压
  g_AdcData.Filtered_B =
      (float)(AdcRegs.ADCRESULT4 >> 4) * k; // 悬浮绕组 B 相电流采样电压
  g_AdcData.Filtered_C =
      (float)(AdcRegs.ADCRESULT5 >> 4) * k; // 悬浮绕组 C 相电流采样电压
  g_AdcData.ADC_VT = (float)(AdcRegs.ADCRESULT6 >> 4) * k; // 母线电压或总线通道
  g_AdcData.ADC_VS = (float)(AdcRegs.ADCRESULT7 >> 4) * k; // 其他辅助电压通道
  g_AdcData.Hall_A =
      (float)(AdcRegs.ADCRESULT8 >> 4) * k; // 霍尔位置传感器 A 通道
  g_AdcData.Hall_B =
      (float)(AdcRegs.ADCRESULT9 >> 4) * k; // 霍尔位置传感器 B 通道
  g_AdcData.Hall_C =
      (float)(AdcRegs.ADCRESULT10 >> 4) * k; // 霍尔位置传感器 C 通道
  g_AdcData.Eddy_C =
      (float)(AdcRegs.ADCRESULT11 >> 4) * k; // 电涡流位移传感器 C 通道
  g_AdcData.Eddy_B =
      (float)(AdcRegs.ADCRESULT12 >> 4) * k; // 电涡流位移传感器 B 通道
  g_AdcData.Eddy_A =
      (float)(AdcRegs.ADCRESULT13 >> 4) * k; // 电涡流位移传感器 A 通道
}

// ==================== 2. 物理量线性转换 ====================
void Convert_Adc_Data(void) {
  // 1. 换算转矩驱动绕组三相电流
  // CURRENT1_SCALE 是电压-电流转换系数，包含运放硬件放大倍数与采样电阻阻值
  Iu = (g_AdcData.Filtered_U - offsetU) * CURRENT1_SCALE;
  Iv = (g_AdcData.Filtered_V - offsetV) * CURRENT1_SCALE;
  Iw = (g_AdcData.Filtered_W - offsetW) * CURRENT1_SCALE;

  // 2. 换算悬浮绕组三相电流
  Ia = (g_AdcData.Filtered_A - offsetA) * CURRENT2_SCALE;
  Ib = (g_AdcData.Filtered_B - offsetB) * CURRENT2_SCALE;
  Ic = (g_AdcData.Filtered_C - offsetC) * CURRENT2_SCALE;

  // 3. 换算霍尔传感器电压值：扣除静态直流偏置，使正弦信号以 0 轴为中心对称
  ha = -(g_AdcData.Hall_A - Hall_OffsetA);
  hb = -(g_AdcData.Hall_B - Hall_OffsetB);
  hc = -(g_AdcData.Hall_C - Hall_OffsetC);

  // 4. 换算电涡流位移传感器电压值
  Ea = g_AdcData.Eddy_A - Eddy_OffsetA;
  Eb = g_AdcData.Eddy_B - Eddy_OffsetB;
  Ec = g_AdcData.Eddy_C - Eddy_OffsetC;
}

// ==================== 3. 闭环控制反馈量计算 ====================
void Calculate_Feedback(void) {
  // =================== 1. 角度计算与状态机 ===================
  // 将三相霍尔信号进行 Clark 变换（转换至两相静止坐标系 alpha-beta）
  // 此处采用等幅值 Clark 变换公式，通过移相处理使正余弦分量更平滑
  float h_alpha = 0.577350269f * (hb - hc); // 即 1/sqrt(3) * (hb - hc)
  float h_beta = 0.666666667f * (-ha + 0.5f * hb +
                                 0.5f * hc); // 即 2/3 * (-ha + 0.5*hb + 0.5*hc)

  // 之前的低通滤波代码（已注释，当前直接送入 atan2f
  // 以保证动态响应没有相位滞后） h_alpha_filt = 0.7f * h_alpha_filt + 0.3f *
  // h_alpha; h_beta_filt  = 0.7f * h_beta_filt  + 0.3f * h_beta;

  // 使用四象限反正切计算原始电角度，输出范围: [-PI, +PI]
  // 注：atan2f 相比 atan 的优势在于它能自动根据分母符号判断所处象限，分母为 0
  // 时不会发生除零错误
  Raw_Hall_Theta_rad = atan2f(h_beta, h_alpha);

  // 将角度转换为 0.0 ~ 1.0 的标幺化值 (pu)
  Raw_Hall_Theta_pu = Raw_Hall_Theta_rad * INV_PI2; // 角度除以 2*PI
  if (Raw_Hall_Theta_pu < 0.0f) {
    Raw_Hall_Theta_pu +=
        1.0f; // 将 [-0.5, +0.5] 的范围平移整合为闭环所需的 [0.0, 1.0) 递增区间
  }

  // 根据电机控制状态机执行不同的角度给定策略
  if (MotorMode == MODE_ALIGN) {
    // 预定位/对齐期间，人为固定驱动角度，使转子磁场强制定向锁死
    if (Align_Counter < 10000)
      ElecTheta_pu = 0.25f; // 强迫定向至 90 度（B相或Q轴轴线）
    else
      ElecTheta_pu = 0.0f; // 切换定向至 0 度（A相或D轴轴线）

    // 定位时悬浮角度依然使用位置传感器计算的真实角度，确保静态悬浮不失控
    temp_sus_theta_pu = Raw_Hall_Theta_pu;
  } else {
    // 正常闭环运行模式：最终旋转电角度 = 真实角度 - 霍尔安装偏置角度
    ElecTheta_pu = Raw_Hall_Theta_pu - Hall_Angle_Offset;

    // 对减去偏置后的角度进行 [0.0, 1.0) 轴线循环越界卷绕处理
    if (ElecTheta_pu < 0.0f)
      ElecTheta_pu += 1.0f;
    if (ElecTheta_pu >= 1.0f)
      ElecTheta_pu -= 1.0f;

    temp_sus_theta_pu = ElecTheta_pu;
  }

  // 换算得到旋转驱动环所需的弧度制电角度
  ElecTheta_rad = ElecTheta_pu * PI2;

  // 计算悬浮解耦所需的控制电角度
  // 针对无轴承电机，由于悬浮极对数（例如 P_sus = P_motor +-
  // 1）的特殊几何倍频关系，此处进行了 2 倍频处理
  ElecTheta_Sus_pu = temp_sus_theta_pu * 2.0f;
  if (ElecTheta_Sus_pu >= 1.0f)
    ElecTheta_Sus_pu -= 1.0f;
  if (ElecTheta_Sus_pu >= 1.0f)
    ElecTheta_Sus_pu -= 1.0f; // 周期越界限幅
  ElecTheta_Sus_rad = ElecTheta_Sus_pu * PI2;

  // =================== 2. 位移解析与坐标变换 ===================
  // 将三项电涡流传感器差分信号通过 Clark 变换，投射到两相静止坐标系下
  float E_alpha =
      0.577350269f * (Ea - Ec); // 等幅值 Clark 变换，分量 1/sqrt(3)*(Ea-Ec)
  float E_beta = 0.666666667f * (Eb - 0.5f * Ea - 0.5f * Ec);

  // 结合比例系数转换为实际的物理位移（如毫米或微米）
  park_eddy.Alpha = E_alpha * SENSOR_POS_SCALE;
  park_eddy.Beta = E_beta * SENSOR_POS_SCALE;

  // 利用转子当前电角度的余弦和正弦值，将位移信号通过 Park
  // 变换投射到转子旋转坐标系（dq 轴）上
  // 这一步是实现磁悬浮力与驱动转矩完全解耦的关键
  park_eddy.Cos = cosf(temp_sus_theta_pu * PI2);
  park_eddy.Sin = sinf(temp_sus_theta_pu * PI2);
  park_eddy.calc(&park_eddy); // 调用 TI 或自定义的 Park 变换算法结构体函数

  // 对得到的旋转坐标系下 D/Q
  // 轴位移反馈进行一阶低通滤波，滤除高频杂波噪声，供给位置环 PID
  Pos_D_Filtered = one_order_LPF(cutoff_fre, park_eddy.Ds, 0);
  Pos_Q_Filtered = one_order_LPF(cutoff_fre, park_eddy.Qs, 1);

  // ==================== 1. 电流 Clark 变换 ====================
  // 将三相静止坐标系下的电流转换到两相静止坐标系：Iabc -> Ialpha, Ibeta
  // 注：TI 算法库默认只需输入 A、B 两相电流，C 相电流根据 KCL (Ia+Ib+Ic=0)
  // 在内部自动计算

  // 1.1 悬浮绕组电流 Clark 变换
  clark2.As = Ia;
  clark2.Bs = Ib;
  clark2.calc(&clark2); // 计算得到 clark2.Alpha 和 clark2.Beta

  // 1.2 旋转驱动绕组电流 Clark 变换
  clark1.As = Iu;
  clark1.Bs = Iv;
  clark1.calc(&clark1); // 计算得到 clark1.Alpha 和 clark1.Beta

  // ==================== 2. 电流 Park 变换 ====================
  // 将两相静止坐标系下的电流转换到两相旋转坐标系：Ialpha, Ibeta -> Id, Iq
  // 这一步实现了交流信号向直流信号的等效映射，是矢量控制的关键

  // 2.1 旋转驱动绕组 Park 变换（使用驱动环基波电角度 ElecTheta_rad）
  park1.Alpha = clark1.Alpha;
  park1.Beta = clark1.Beta;
  park1.Cos = cosf(ElecTheta_rad);
  park1.Sin = sinf(ElecTheta_rad);
  park1.calc(&park1); // 计算得到驱动环实际的 park1.Ds (Id) 和 park1.Qs (Iq)

  // 2.2 磁悬浮绕组 Park 变换（使用悬浮环专用倍频电角度 ElecTheta_Sus_rad）
  park2.Alpha = clark2.Alpha;
  park2.Beta = clark2.Beta;
  park2.Cos = cosf(ElecTheta_Sus_rad);
  park2.Sin = sinf(ElecTheta_Sus_rad);
  park2.calc(&park2); // 计算得到悬浮环实际的 park2.Ds (Id) 和 park2.Qs (Iq)
}

// ==================== 3. 磁悬浮核心控制算法 ====================
void Control_Suspension(void) {
  // 每次中断中清除一次 TZ（Trip-Zone）单次触发故障标志，重新使能悬浮驱动桥臂的
  // PWM 输出
  EALLOW;
  EPwm4Regs.TZCLR.bit.OST = 1; // 清除 ePWM4 的 One-Shot 保护标志
  EPwm5Regs.TZCLR.bit.OST = 1; // 清除 ePWM5 的 One-Shot 保护标志
  EPwm6Regs.TZCLR.bit.OST = 1; // 清除 ePWM6 的 One-Shot 保护标志
  EDIS;

  // 1. 位置外环 PID 调节：输入目标位置 x*, y* 与位移反馈 x, y，输出目标悬浮力
  // Fx*, Fy*
  gSuspPosD_PID.Ref = Pos_D_Ref;
  gSuspPosD_PID.Fdb = park_eddy.Ds; // 采用电涡流变换后的 D 轴实际位移反馈值
  gSuspPosD_PID.calc(&gSuspPosD_PID);
  // Force_D_Ref = -gSuspPosD_PID.Out; // 得到 D 轴期望控制力 Fd*
  //注意引入负号构成负反馈

  gSuspPosQ_PID.Ref = Pos_Q_Ref;
  gSuspPosQ_PID.Fdb = park_eddy.Qs; // 采用电涡流变换后的 Q 轴实际位移反馈值
  gSuspPosQ_PID.calc(&gSuspPosQ_PID);
  // Force_Q_Ref = -gSuspPosQ_PID.Out; // 得到 Q 轴期望控制力 Fq*

  // 前馈补偿只赋值
  // 给后台源源不断地送去最新的传感器数据（单周期赋值，不耗时）
  gUnbalanceComp.x_raw_buf = park_eddy.Ds;
  gUnbalanceComp.y_raw_buf = park_eddy.Qs;
  gUnbalanceComp.theta_buf = ElecTheta_rad;

  // 触发标志位：告诉后台“新的一届中断采样到了，你可以开始算下一发了”
  gUnbalanceComp.NewData_Flag = 1;

  if (UFC_flag == 1) {
    // 直接拿取后台计算并锁存在这里的最新出力
    // 哪怕后台还没算完新的，这里也会拿到上一次算好的、稳定的旧出力，绝不卡中断
    Unbalance_Comp_Run(&gUnbalanceComp, park_eddy.Ds, park_eddy.Qs,
                       ElecTheta_rad);
    Force_D_Ref = -gSuspPosD_PID.Out -
                  gUnbalanceComp.Fx_out; // include UFC D 轴期望控制力
    Force_Q_Ref = -gSuspPosQ_PID.Out -
                  gUnbalanceComp.Fy_out; // include UFC Q 轴期望控制力
  } else {
    Force_D_Ref = -gSuspPosD_PID.Out; // 得到 D 轴期望控制力
    Force_Q_Ref = -gSuspPosQ_PID.Out; // 得到 Q 轴期望控制力
  }

  // 2. 悬浮力-电流非线性解耦矩阵算法
  // 针对无轴承电机，悬浮力的产生不仅取决于悬浮绕组电流，还受到旋转绕组电枢磁场（I1q）的强耦合调制。
  // 此处采用动态矩阵实时逆解耦，将期望力 Fd*, Fq* 实时换算为对应的悬浮电流指令
  // Id*, Iq*。
  float I1q = park1.Qs;       // 获取当前旋转驱动绕组的实际 q 轴转矩电流反馈
  float u3 = MOTOR_L1Q * I1q; // 计算中间动态交轴磁链：u(3) = L1q * I1q

  // 计算解耦变换矩阵的分母项: Denominator = Km * (Phi_f^2 + u(3)^2)
  float denom = MOTOR_KM * (MOTOR_PHI_F_SQ + u3 * u3);

  // 软边界保护：防止分母极小或为 0 导致 DSP
  // 抛出除零异常、数据溢出或导致复位挂起
  if (denom < 1e-6f && denom > -1e-6f)
    denom = 1e-6f;

  // 算力优化：将除法转化为倒数乘法（DSP
  // 执行浮点乘法仅需一个时钟周期，远快于除法指令）
  float inv_denom = 1.0f / denom;

  // 执行逆矩阵乘法，解耦得到悬浮绕组两相旋转坐标系的目标控制电流指令
  i_Bd_ref = (Force_D_Ref * MOTOR_PHI_F - Force_Q_Ref * u3) * inv_denom;
  i_Bq_ref = (Force_D_Ref * u3 + Force_Q_Ref * MOTOR_PHI_F) * inv_denom;

  // 3. 悬浮电流内环 PI 调节：输入 Id*, Iq* 和电流反馈，输出控制电压 Ud*, Uq*
  gSuspCurrD_PID.Ref = i_Bd_ref; // 注入 D 轴目标悬浮电流
  gSuspCurrD_PID.Fdb = park2.Ds; // D 轴悬浮实际电流反馈（来自 clark2 变换后）
  gSuspCurrD_PID.calc(&gSuspCurrD_PID);

  gSuspCurrQ_PID.Ref = i_Bq_ref; // 注入 Q 轴目标悬浮电流
  gSuspCurrQ_PID.Fdb = park2.Qs; // Q 轴悬浮实际电流反馈
  gSuspCurrQ_PID.calc(&gSuspCurrQ_PID);

  // 4. 悬浮电压反 Park（IPARK）变换：将旋转直流电压 Ud, Uq 变换为静止交流电压
  // Ualpha, Ubeta
  ipark2.Ds = gSuspCurrD_PID.Out;
  ipark2.Qs = gSuspCurrQ_PID.Out;
  ipark2.Cos = park2.Cos; // 使用悬浮专用的倍频旋转因子
  ipark2.Sin = park2.Sin;
  ipark2.calc(&ipark2);

  // 5. 将生成的两相静止电压送入悬浮专用 SVPWM 空间矢量输出模块
  Update_Suspension_PWM(ipark2.Alpha, ipark2.Beta);
}

// ==================== 4. 旋转驱动核心控制算法 ====================
void Control_Torque(void) {
  // 根据主状态机模式执行对应的控制逻辑
  switch (MotorMode) {

  // ------------------ 模式 0：空闲待机状态 ------------------
  case MODE_IDLE:
    Stop_Torque();         // 封锁旋转 PWM 模块并清除其内部所有的 PID 积分器
    Speed_Target_Ramp = 0; // 归零转速斜坡发生器的当前给定值
    Align_Counter = 0;     // 归零对齐定位计数器
    break;

  // ------------------ 模式 1：转子强迫预定位状态（对齐轴线）
  // ------------------
  case MODE_ALIGN:
    // 清除旋转桥臂的 TZ 封锁，恢复 PWM 输出能力
    EALLOW;
    EPwm1Regs.TZCLR.bit.OST = 1;
    EPwm2Regs.TZCLR.bit.OST = 1;
    EPwm3Regs.TZCLR.bit.OST = 1;
    EDIS;

    // 定位策略：人为给定一个固定的直流励磁电流（Id_ref = 0.1A，Iq_ref = 0A）
    // 产生一个空间固定方向的静止磁场，强迫转子磁极旋转并锁死在该轴线上，从而确定绝对零位
    gTorqCurrD_PI.Ref = 0.1f;
    gTorqCurrD_PI.Fdb = park1.Ds;
    gTorqCurrD_PI.calc(&gTorqCurrD_PI);

    gTorqCurrQ_PI.Ref = 0.0f;
    gTorqCurrQ_PI.Fdb = park1.Qs;
    gTorqCurrQ_PI.calc(&gTorqCurrQ_PI);

    // 对其执行反 Park 变换（此时角度在 Calculate_Feedback
    // 中根据定位阶段硬编码固定）
    ipark1.Ds = gTorqCurrD_PI.Out;
    ipark1.Qs = gTorqCurrQ_PI.Out;
    ipark1.Cos = park1.Cos;
    ipark1.Sin = park1.Sin;
    ipark1.calc(&ipark1);

    Update_Svpwm(ipark1.Alpha, ipark1.Beta);

    // 定位计时递增
    Align_Counter++;
    if (Align_Counter == 20000) // 在 10kHz 采样下计数达 20000
                                // 次（即持续直流励磁 2 秒）确保转子完全静止对齐
    {
      Hall_Angle_Offset =
          Raw_Hall_Theta_pu; // 将当前锁死轴线下的霍尔传感器读数记录为硬件安装偏置量
      AlignDone = 1;         // 标志位置 1：定位校准圆满成功
      MotorMode =
          MODE_IDLE; // 定位结束自动切换回 IDLE 待机状态，安全等待上位机运行指令
    }
    break;

  // ------------------ 模式 2：双闭环 FOC 矢量运行状态 ------------------
  case MODE_RUN:
    // 如果系统需要严苛的安全检查，可以在此处取消注释：未定位成功则拒绝闭环运行
    // if(AlignDone == 0) {
    //     MotorMode = MODE_IDLE;
    //     break;
    // }

    // 打开旋转回路的 PWM 驱动通道
    EALLOW;
    EPwm1Regs.TZCLR.bit.OST = 1;
    EPwm2Regs.TZCLR.bit.OST = 1;
    EPwm3Regs.TZCLR.bit.OST = 1;
    EDIS;

    // 1. 转速外环分频执行与斜坡发生器（电流内环 10kHz，转速外环分频 10 倍对应
    // 1kHz）
    if (++Speed_Loop_Cnt >= 10) {
      Speed_Loop_Cnt = 0;

      // === A. 瞬时转速微积分计算 ===
      dTheta = ElecTheta_pu - ElecTheta_pre; // 差分前后两帧标幺化电角度
      ElecTheta_pre = ElecTheta_pu;          // 滚动更新历史角度

      // 跨越 0 点和 1.0 边界处的电角度卷绕截断修正（防止极性翻转引发测速飞车）
      if (dTheta > 0.5f)
        dTheta -= 1.0f;
      if (dTheta < -0.5f)
        dTheta += 1.0f;

      // 计算瞬时转速 (RPM)。 公式: (电角度差值 * 速度环计算频率 * 60秒) /
      // 电机极对数 其中的 0.1f 系数是因为外环经过了 10 分频 (ISR_FREQ * 0.1 =
      // 1kHz)
      Speed_Raw_rpm = (dTheta * ISR_FREQ * 0.1f * 60.0f) / POLE_PAIRS;

      // === B. 转速反馈一阶低通滤波 ===
      // 用于滤除微分带来的高频数字噪声，提供平滑的速度反馈值
      Speed_Fdb_rpm = 0.95f * Speed_Fdb_rpm + 0.05f * Speed_Raw_rpm;
      // Speed_Fdb_rpm = Speed_Filter(Speed_Raw_rpm); // 也可选择滑动平均滤波器

      // === C. 转速斜坡限制器（Ramp Function） ===
      // 限制转速指令的变化速率，防止转速设定阶跃导致速度环瞬间饱和拉出最大电流
      float error = Speed_Ref_rpm - Speed_Target_Ramp;
      if (error > Speed_Accel_Step)
        Speed_Target_Ramp += Speed_Accel_Step; // 加速斜坡步进
      else if (error < -Speed_Accel_Step)
        Speed_Target_Ramp -= Speed_Accel_Step; // 减速斜坡步进
      else
        Speed_Target_Ramp = Speed_Ref_rpm; // 误差进入步进范围内，直接相等

      // === D. 转速环 PI 运算 ===
      gSpd_PI.Ref = Speed_Target_Ramp; // 设定当前斜坡速度给定值
      gSpd_PI.Fdb = Speed_Fdb_rpm;     // 注入平滑后的速度反馈
      gSpd_PI.calc(&gSpd_PI);          // 执行计算，输出为期望的目标转矩电流
    }

    // 2. 旋转电流内环调节（标准的 id=0 控制策略）
    gTorqCurrD_PI.Ref = 0.0f;     // D轴励磁电流给定：id* = 0
    gTorqCurrD_PI.Fdb = park1.Ds; // D轴驱动电流实际反馈
    gTorqCurrD_PI.calc(&gTorqCurrD_PI);

    gTorqCurrQ_PI.Ref =
        -gSpd_PI
             .Out; // Q轴转矩电流给定来自于速度环输出的取反（根据系统电机转向定义）
    gTorqCurrQ_PI.Fdb = park1.Qs; // Q轴驱动电流实际反馈
    gTorqCurrQ_PI.calc(&gTorqCurrQ_PI);

    // 3. 驱动回路反 Park（IPARK）坐标变换
    ipark1.Ds = gTorqCurrD_PI.Out;
    ipark1.Qs = gTorqCurrQ_PI.Out;
    ipark1.Cos = park1.Cos;
    ipark1.Sin = park1.Sin;
    ipark1.calc(&ipark1);

    // 4. 送入标准逆变桥的空间矢量 SVPWM 占空比更新函数
    Update_Svpwm(ipark1.Alpha, ipark1.Beta);
    break;

  default:
    MotorMode = MODE_IDLE;
    break;
  }
}

// ==================== 5. 紧急停机与死锁逻辑 ====================

/**
 * @brief 瞬时强制封锁磁悬浮绕组的 PWM 输出（硬保护机制）
 */
void Stop_Suspension(void) {
  EALLOW;
  // 硬件强制触发 Trip-Zone 控制器的 One-shot 软件强制事件（TZFRC.bit.OST）
  // 触发后，硬件会绕过软件，瞬间将 ePWM4/5/6 桥臂的所有 6
  // 路功率管拉到预设的安全阻抗或关断电平，防止过流烧毁桥臂
  EPwm4Regs.TZFRC.bit.OST = 1;
  EPwm5Regs.TZFRC.bit.OST = 1;
  EPwm6Regs.TZFRC.bit.OST = 1;
  EDIS;

  // 停机时必须同步强制清除位置环和悬浮电流内环的所有 PID 积分累加器 `Ui`
  // 这样做的核心目的是为了清除抗饱和风暴（Anti-windup），确保下一次使能启动时控制从
  // 0 开始干净平稳起悬，不会因为历史积分引发强烈的起悬电流冲击
  gSuspPosD_PID.Ui = 0;
  gSuspPosQ_PID.Ui = 0;
  gSuspCurrD_PID.Ui = 0;
  gSuspCurrQ_PID.Ui = 0;
}

/**
 * @brief 瞬时强制封锁旋转驱动绕组的 PWM 输出
 */
void Stop_Torque(void) {
  EALLOW;
  // 硬件强制触发驱动桥臂 ePWM1/2/3 的 One-shot 软件故障保护事件
  EPwm1Regs.TZFRC.bit.OST = 1;
  EPwm2Regs.TZFRC.bit.OST = 1;
  EPwm3Regs.TZFRC.bit.OST = 1;
  EDIS;

  // 清除旋转电流内环和速度外环的所有 PI
  // 积分器累加值，防止再次使能时发生剧烈飞车
  gTorqCurrD_PI.Ui = 0;
  gTorqCurrQ_PI.Ui = 0;
  gSpd_PI.Ui = 0;
}

/**
 * @brief 全局系统紧急避险停机
 */
void Stop_System(void) {
  Stop_Suspension(); // 封锁悬浮
  Stop_Torque();     // 封锁旋转
}

/**
 * @brief 全局故障清除恢复函数（通常在上电自检校准偏置完毕后调用）
 */
void Resume_System(void) {
  EALLOW;
  // 清除所有 ePWM 模块的 TZ
  // 封锁状态标志位，重新赋予软件闭环算法对功率管的控制权
  EPwm1Regs.TZCLR.bit.OST = 1;
  EPwm2Regs.TZCLR.bit.OST = 1;
  EPwm3Regs.TZCLR.bit.OST = 1;
  EPwm4Regs.TZCLR.bit.OST = 1;
  EPwm5Regs.TZCLR.bit.OST = 1;
  EPwm6Regs.TZCLR.bit.OST = 1;
  EDIS;
}
