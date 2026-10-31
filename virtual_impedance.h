#ifndef VIRTUAL_IMPEDANCE_H
#define VIRTUAL_IMPEDANCE_H

#ifdef __cplusplus
extern "C" {
#endif

/* 模型：Iref(s) / Error(s) = 1 / (Lv*s + Rv)。 */
#define VIRTUAL_IMPEDANCE_R_OHM    0.01f
#define VIRTUAL_IMPEDANCE_L_H      0.001f

typedef struct
{
    float i_ref;
} VirtualImpedance_t;

/* 清零状态；启动或重新开始控制时调用。 */
void VirtualImpedance_init(VirtualImpedance_t *ctrl);

/* error = 参考电压 - 实测电压（V），ts_s = 控制周期（s），返回电流参考（A）。
 * 与 MATLAB c2d(tf(1, [Lv Rv]), Ts, 'zoh') 的时序一致：
 * 当前误差作用于下一拍输出，零初值时首次输出为 0。
 * 每个虚拟阻抗环节使用独立的 ctrl，并在多次调用间保留其状态。
 */
float VirtualImpedance_run(VirtualImpedance_t *ctrl, float error, float ts_s);

#ifdef __cplusplus
}
#endif

#endif
