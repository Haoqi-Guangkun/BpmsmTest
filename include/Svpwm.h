/*
 * Svpwm.h
 *
 *  Created on: 2026年2月4日
 *      Author: L.YF
 */

#ifndef INCLUDE_SVPWM_H_
#define INCLUDE_SVPWM_H_

#include "DSP2833x_Device.h"

// 初始化函数
//void Init_Svpwm_Gpio(void);
//void Init_Svpwm_Module(void);

// 核心计算函数 (转矩绕组 1-3)
void Update_Svpwm(float Ualpha, float Ubeta);
// 核心计算函数 (悬浮绕组 4-6) - 新增
void Update_Suspension_PWM(float Ualpha, float Ubeta);

#endif /* INCLUDE_SVPWM_H_ */
