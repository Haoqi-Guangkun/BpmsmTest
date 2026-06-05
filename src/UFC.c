
#include "control.h"
#include <math.h>


#define PI_TWO 6.283185307f

/**
 * @brief 不平衡前馈补偿运行主函数（在ADC中断中调用）
 */
void Unbalance_Comp_Run(UFC *p, float32 x_raw, float32 y_raw,
                        float32 theta_hall) {
  float32 sin_theta = sinf(theta_hall);
  float32 cos_theta = cosf(theta_hall);
  float32 x_hat, y_hat; // SPU处理结果

  // 1. 根据霍尔反馈的实际转速，实时更新采样点数
  if (p->omega_r > 1.0f) {
    p->spu_x.N_samples = (Uint32)(PI_TWO / (p->omega_r * p->Ts));
    p->spu_y.N_samples = p->spu_x.N_samples;
  }

  // 2. SPU 通道实时滑动积分
  p->spu_x.sum_sin += x_raw * sin_theta;
  p->spu_x.sum_cos += x_raw * cos_theta;
  p->spu_x.sample_cnt++;

  p->spu_y.sum_sin += y_raw * sin_theta;
  p->spu_y.sum_cos += y_raw * cos_theta;
  p->spu_y.sample_cnt++;

  // 满一个旋转周期，计算傅里叶系数
  if (p->spu_x.sample_cnt >= p->spu_x.N_samples && p->spu_x.N_samples > 0) {
    float32 coeff = 2.0f / (float32)p->spu_x.N_samples;

    p->spu_x.a = p->spu_x.sum_sin * coeff;
    p->spu_x.b = p->spu_x.sum_cos * coeff;

    p->spu_y.a = p->spu_y.sum_sin * coeff;
    p->spu_y.b = p->spu_y.sum_cos * coeff;

    // 清空累加器
    p->spu_x.sum_sin = 0.0f;
    p->spu_x.sum_cos = 0.0f;
    p->spu_x.sample_cnt = 0;
    p->spu_y.sum_sin = 0.0f;
    p->spu_y.sum_cos = 0.0f;
    p->spu_y.sample_cnt = 0;
  }

  // 3. 重构同频位移信号 $\hat{x}, \hat{y}$
  x_hat = p->spu_x.a * sin_theta + p->spu_x.b * cos_theta;
  y_hat = p->spu_y.a * sin_theta + p->spu_y.b * cos_theta;

  // 4. 标准 PARK 变换
  p->park.Alpha = x_hat;
  p->park.Beta = y_hat;
  p->park.Sin = sin_theta;
  p->park.Cos = cos_theta;
  p->park.calc(&(p->park));

  // 5. 执行前馈模块独立的 PID 运算 (目标值 Ref = 0)
  p->pid_u.Ref = 0.0f;
  p->pid_u.Fdb = p->park.Ds;
  p->pid_u.calc(&(p->pid_u));

  p->pid_v.Ref = 0.0f;
  p->pid_v.Fdb = p->park.Qs;
  p->pid_v.calc(&(p->pid_v));

  // 6. 标准 IPARK 变换
  p->ipark.Ds = p->pid_u.Out;
  p->ipark.Qs = p->pid_v.Out;
  p->ipark.Sin = sin_theta;
  p->ipark.Cos = cos_theta;
  p->ipark.calc(&(p->ipark));

  // 7. 输出最终的前馈力
  p->Fx_out = p->ipark.Alpha;
  p->Fy_out = p->ipark.Beta;
}
