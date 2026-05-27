/*
 * park.h
 * 静止两相坐标系 -> 旋转坐标系
 *
 *  Created on: 2026年3月9日
 *      Author: L.YF
 */

#ifndef PARK_H_
#define PARK_H_
#include "DSP2833x_Device.h"

// 定义 PARK 结构体
typedef struct PARK_OBJ {
    float Alpha;    // 输入：静止坐标系 Alpha 轴分量
    float Beta;     // 输入：静止坐标系 Beta 轴分量
    float Cos;      // 输入：当前电角度的余弦值
    float Sin;      // 输入：当前电角度的正弦值
    float Ds;       // 输出：旋转坐标系 D 轴分量
    float Qs;       // 输出：旋转坐标系 Q 轴分量
    void (*calc)(void *); // 计算函数指针
} PARK;

// 定义句柄类型
typedef PARK * PARK_handle;

// 默认初始化宏定义
// 全部使用 0.0f 规范单精度浮点格式，修复强转指针警告
#define PARK_DEFAULTS {0.0f, 0.0f, \
                       0.0f, 0.0f, \
                       0.0f, 0.0f, \
                       (void (*)(void *))park_calc}

// 函数声明
void park_calc(PARK *p);

#endif /* PARK_H_ */
