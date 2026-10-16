#ifndef HRTIM_SPWM_H
#define HRTIM_SPWM_H

#include "stm32g4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

#define HRTIM_SPWM_COMPARE_MARGIN_TICKS 6U

/* 本模块只使用 Master、Timer A/B 和 TA1/TA2/TB1/TB2。
 * CubeMX 仍负责外设时钟、GPIO复用、HAL_HRTIM_Init 和 DLL 校准。
 * 同一组定时器及其比较寄存器只允许由本模块管理。
 */
typedef struct
{
    uint32_t timer_clock_hz;      /* HRTIM 内核时钟，DIV1；当前工程为170 MHz */
    uint32_t carrier_peak_ticks;  /* A/B三角载波峰值；Master周期固定为其两倍 */
    float modulation_limit;      /* 0 < 限值 <= 1；默认沿用原工程0.95 */
    uint32_t deadtime_a_rising_ticks;
    uint32_t deadtime_a_falling_ticks;
    uint32_t deadtime_b_rising_ticks;
    uint32_t deadtime_b_falling_ticks;
} HRTIM_SPWM_Config;

typedef struct
{
    HRTIM_HandleTypeDef *hrtim;
    HRTIM_SPWM_Config config;
    float update_hz;             /* Master周期频率，也是完整A/B载波频率 */
    volatile float modulation;
    volatile uint32_t compare_a;
    volatile uint32_t compare_b;
    volatile uint8_t initialized;
    volatile uint8_t counters_started;
    volatile uint8_t outputs_enabled;
    volatile uint8_t ever_started; /* Stop后禁止以同一实例重新启计数 */
} HRTIM_SPWM_Context;

/* 默认参数复现现工程：170 MHz，A/B峰值4250，20 kHz载波，m限幅0.95。
 * 死区复现原值：A上升0/下降25，B上升0/下降0；按实际驱动硬件调整。
 */
void HRTIM_SPWM_DefaultConfig(HRTIM_SPWM_Config *config);

/* context必须为静态对象或用{0}初始化。
 * 在上电/完整HRTIM外设复位后的MX_HRTIM1_Init之后调用；要求M/A/B停止且CNT均为0。
 * 不会开启计数或栅极输出；不能用于任意停止位置的重新初始化。
 * 配置M/A/B时基、载波方向、谷底更新、互补输出、死区和Master周期ADC触发1。
 * HAL配置失败返回相应状态；initialized保持0，A/B输出保持禁用。
 */
HAL_StatusTypeDef HRTIM_SPWM_Init(HRTIM_SPWM_Context *context,
                                HRTIM_HandleTypeDef *hrtim,
                                const HRTIM_SPWM_Config *config);

/* 主循环初始化阶段调用：要求M/A/B仍为复位后的零计数、A/B初始向上方向。
 * 一次MCR写入统一启动M/A/B，对齐Master周期和A/B谷底；不打开栅极输出。
 * ADC/DMA应先准备完成；Start/Stop/Init不得与采样中断中的SetModulation并发。
 * 每个context只允许启动一次。日常暂停/恢复用Disable/EnableOutputs，计数持续运行。
 */
HAL_StatusTypeDef HRTIM_SPWM_StartCounters(HRTIM_SPWM_Context *context);

/* 控制中断中调用：dA=(1+m)/2，dB=(1-m)/2；6 tick边界保护。
 * 两路预装载一起暂禁、写入、恢复，在下一个共同谷底一起生效。
 * 无等待、无HAL锁；NaN/Inf返回HAL_ERROR，原比较值不变。
 */
HAL_StatusTypeDef HRTIM_SetModulation(HRTIM_SPWM_Context *context, float modulation);

/* 确认ADC和母线/保护状态正常后才启用；保持已有预装载在谷底更新，
 * 不在运行中强制SoftwareUpdate，以免把脉冲截在载波中途。
 */
HAL_StatusTypeDef HRTIM_SPWM_EnableOutputs(HRTIM_SPWM_Context *context);
void HRTIM_SPWM_DisableOutputs(HRTIM_SPWM_Context *context);
/* 先直接禁用A/B全部输出，再停止M/A/B计数；不操作其他定时器。
 * Stop后不可再Start，也不可重新Init同一context。若确需彻底重启：先停ADC/DMA，
 * 完整复位HRTIM外设并重新MX初始化，再使用重新清零的context。仅MX调用不等于外设复位。
 */
HAL_StatusTypeDef HRTIM_SPWM_Stop(HRTIM_SPWM_Context *context);

#ifdef __cplusplus
}
#endif
#endif
