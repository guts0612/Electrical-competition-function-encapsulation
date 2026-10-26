#ifndef QPR_ROTOR_H
#define QPR_ROTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#define QPR_KP          0.1f
#define QPR_KR          30.0f
#define QPR_WC_RAD_S    5.0f
#define QPR_TS_S        (1.0f / 20000.0f)
#define QPR_LOWER       (-5.0f)
#define QPR_UPPER       5.0f

typedef struct
{
    float kp;
    float kr;
    float wc_rad_s;
    float wo_rad_s;
    float ts_s;
    float lower;
    float upper;

    float cos_theta;
    float sin_theta;
    float decay;
    float b1;

    float x1;
    float x2;
} QprRotor_t;

void QprRotor_init(QprRotor_t *ctrl,
                   float kp,
                   float kr,
                   float wc_rad_s,
                   float wo_rad_s,
                   float ts_s,
                   float lower,
                   float upper);

void QprRotor_reset(QprRotor_t *ctrl);

float QprRotor_run(QprRotor_t *ctrl, float error);

#ifdef __cplusplus
}
#endif

#endif
