/*
 * wave.h
 *
 *  Created on: 2026年4月13日
 *      Author: L.YF
 */

#ifndef WAVE_H_
#define WAVE_H_

#include "DSP2833x_Device.h"

// 信号类型宏定义
#define SIGNAL_ZERO   0x0
#define SIGNAL_STEP   0x1
#define SIGNAL_RAMP   0x2
#define SIGNAL_SQUARE 0x3
#define SIGNAL_SINE   0x4

typedef struct {
    Uint16 type;         // 信号类型
    float32 out;         // 输出值
    float32 value;       // 幅度/目标值
    float32 signal_freq; // 信号频率 (Hz)
    float32 system_freq; // 系统采样频率 (通常为 ISR 频率, 如 10000.0)
    float32 phase;       // 初始相位 (rad)
    float32 acc;         // 步进增量
    float32 theta;       // 当前角度累加器
    Uint32  count;       // 内部计数器
    Uint32  period;      // 周期计数
} WAVE;

// 默认初始化宏
#define WAVE_DEFAULTS { SIGNAL_ZERO, 0.0f, 0.0f, 0.0f, 10000.0f, 0.0f, 0.0f, 0.0f, 0, 0 }

// 函数声明
void wave_calc(WAVE *v);

#endif /* WAVE_H_ */
