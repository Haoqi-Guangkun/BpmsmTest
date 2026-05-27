/*
 * wave.c
 *
 *  Created on: 2026年4月13日
 *      Author: L.YF
 */

/*
 * wave.c
 * 信号发生器：用于电流环阶跃测试或位移环正弦扫频
 */
#include "wave.h"
#include <math.h>

#ifndef PI
#define PI 3.14159265358979f
#endif

#ifndef PI2
#define PI2 6.28318530717958f
#endif

void wave_calc(WAVE *v)
{
    v->count++;

    // 0. 零输出模式
    if(v->type == SIGNAL_ZERO)
    {
        v->out = 0.0f;
        v->count = 0;
        v->theta = 0.0f;
    }

    // 1. 阶跃信号模式 (用于电流环上升时间测试)
    else if(v->type == SIGNAL_STEP)
    {
        // 每 1000 个周期翻转一次，方便在示波器上观察
        if(v->count <= 20)
        {
            v->out = 0.0f;
        }
        else if(v->count <= 1000)
        {
            v->out = v->value;
        }
        else
        {
            v->count = 21;
        }
    }

    // 2. 斜坡信号模式 (用于平滑给定)
    else if(v->type == SIGNAL_RAMP)
    {
        if(v->out < v->value)
        {
            v->out += v->acc; // acc 为斜率
            if(v->out > v->value) v->out = v->value;
        }
        else if(v->out > v->value)
        {
            v->out -= v->acc;
            if(v->out < v->value) v->out = v->value;
        }
    }

    // 3. 方波信号模式
    else if(v->type == SIGNAL_SQUARE)
    {
        // 计算半周期对应的计数次数
        v->period = (Uint32)(0.5f * v->system_freq / v->signal_freq);

        if(v->count <= v->period)
        {
            v->out = v->value;
        }
        else if(v->count <= (2 * v->period))
        {
            v->out = -v->value; // 对称方波
        }
        else
        {
            v->count = 0;
        }
    }

    // 4. 正弦波模式 (用于悬浮抖动或动态响应测试)
    else if(v->type == SIGNAL_SINE)
    {
        // 第一次运行或参数改变时初始化增量
        v->acc = PI2 * v->signal_freq / v->system_freq;

        v->theta += v->acc;
        if(v->theta > PI2)
        {
            v->theta -= PI2;
        }

        // 使用 sinf 提高 F28335 执行效率
        v->out = v->value * sinf(v->theta + v->phase);
    }
}


