#include "qpr_rotor.h"

#include <math.h>

#ifndef QPR_EXP
#define QPR_EXP(x) expf(x)
#endif

#ifndef QPR_SIN
#define QPR_SIN(x) sinf(x)
#endif

#ifndef QPR_COS
#define QPR_COS(x) cosf(x)
#endif

static float QprRotor_clamp(float value, float lower, float upper)
{
    if (value > upper)
    {
        return upper;
    }
    if (value < lower)
    {
        return lower;
    }
    return value;
}

void QprRotor_init(QprRotor_t *ctrl,
                   float kp,
                   float kr,
                   float wc_rad_s,
                   float wo_rad_s,
                   float ts_s,
                   float lower,
                   float upper)
{
    float theta;

    if (ctrl == (QprRotor_t *)0)
    {
        return;
    }

    ctrl->kp = kp;
    ctrl->kr = kr;
    ctrl->wc_rad_s = wc_rad_s;
    ctrl->wo_rad_s = wo_rad_s;
    ctrl->ts_s = ts_s;
    ctrl->lower = lower;
    ctrl->upper = upper;

    theta = wo_rad_s * ts_s;
    ctrl->cos_theta = QPR_COS(theta);
    ctrl->sin_theta = QPR_SIN(theta);
    ctrl->decay = QPR_EXP(-2.0F * wc_rad_s * ts_s);
    ctrl->b1 = kr * 2.0F * wc_rad_s * ts_s;

    QprRotor_reset(ctrl);
}

void QprRotor_reset(QprRotor_t *ctrl)
{
    if (ctrl == (QprRotor_t *)0)
    {
        return;
    }

    ctrl->x1 = 0.0F;
    ctrl->x2 = 0.0F;
}

float QprRotor_run(QprRotor_t *ctrl, float error)
{
    float x1_next;
    float x2_next;
    float output;

    if (ctrl == (QprRotor_t *)0)
    {
        return 0.0F;
    }

    x1_next = (ctrl->decay * ctrl->cos_theta * ctrl->x1)
            - (ctrl->decay * ctrl->sin_theta * ctrl->x2)
            + (ctrl->b1 * error);

    x2_next = (ctrl->sin_theta * ctrl->x1)
            + (ctrl->cos_theta * ctrl->x2);

    output = (ctrl->kp * error) + x1_next;

    x2_next = QprRotor_clamp(x2_next, ctrl->lower, ctrl->upper);

    ctrl->x1 = x1_next;
    ctrl->x2 = x2_next;

    return output;
}
