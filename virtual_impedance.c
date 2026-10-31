#include "virtual_impedance.h"

#include <math.h>

void VirtualImpedance_init(VirtualImpedance_t *ctrl)
{
    ctrl->i_ref = 0.0f;
}

float VirtualImpedance_run(VirtualImpedance_t *ctrl, float error, float ts_s)
{
    float a;
    float b;
    float reference;

    /* 零阶保持离散化：a = exp(-Rv*Ts/Lv)，b = (1-a)/Rv。 */
    a = expf(-VIRTUAL_IMPEDANCE_R_OHM * ts_s / VIRTUAL_IMPEDANCE_L_H);
    b = (1.0f - a) / VIRTUAL_IMPEDANCE_R_OHM;

    /* 先输出当前状态，再计算下一拍状态，与 MATLAB 离散模型顺序一致。 */
    reference = ctrl->i_ref;
    ctrl->i_ref = a * ctrl->i_ref + b * error;

    return reference;
}
