#ifndef UFD_PWM_H
#define UFD_PWM_H

#include "stm32g4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef SPWM_OUTPUT_FREQUENCY_HZ
#define SPWM_OUTPUT_FREQUENCY_HZ    50.0f
#endif
#ifndef SPWM_MODULATION_LIMIT
#define SPWM_MODULATION_LIMIT      0.95f
#endif
#ifndef HRTIM_COMPARE_MARGIN_TICKS
#define HRTIM_COMPARE_MARGIN_TICKS  6U
#endif

#define UFD_PWM_OUTPUTS (HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2 | \
                         HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2)

/* 仅管理hhrtim1的MASTER、Timer A/B；CubeMX负责基础配置、DLL、GPIO和死区。
 * 配置沿用DC-AC-PAR：M/A/B均DIV1、REP=0；Master向上计数，PER_M=2*PER_A；
 * A/B周期相同、上下计数、开启预装载，在共同谷底更新；CMP1使用EQUAL模式，
 * x1在CMP1复位，x2由死区单元产生互补；输出高有效，禁用时为低电平。
 * ADC由MASTER_PERIOD每周期触发一次，控制中断每次执行一拍，不抽点。
 * 例如时钟170MHz、PER_M=8500、PER_A=PER_B=4250，完整载波和更新频率为20kHz。
 * 上电或完整HRTIM外设复位后，在MX_HRTIM1_Init()之后调用一次。
 * 调用时M/A/B尚未启动、计数均为0；本函数不重新复位运行过的计数器。
 * 四个指针都必须有效；变量由main.c定义，本模块只在Init中通过指针赋初值。
 * update_hz单位Hz，step_rad单位rad/控制拍，phase_rad单位rad。
 * 返回时M/A/B仍停止、四路输出关闭；主函数完成控制器和ADC/DMA准备后，
 * 一次调用HAL_HRTIM_WaveformCountStart_IT(&hhrtim1,
 *     HRTIM_TIMERID_MASTER | HRTIM_TIMERID_TIMER_A | HRTIM_TIMERID_TIMER_B)
 * 从0同步启动三个计数器，不要逐个启动。采样有效后再单独开启四路输出。
 * 日常停机用Slave_Stop()，保留计数同步；重新Init需先完整复位HRTIM。
 */
void HRTIM_SPWM_Init(float *spwm_update_hz,
                     float *phase_step_rad,
                     float *spwm_phase_rad,
                     volatile uint32_t *spwm_update_count);

/* 初始化后，每个控制拍调用一次。modulation为带正负号的瞬时调制度，
 * 开环时传入M*sin(theta)，闭环时传入控制器输出；限幅默认±0.95。
 * 唯一的控制中断负责调用，输入应为有限浮点数。
 */
void HRTIM_SetModulation(float modulation);

/* 立即关闭TA1/TA2/TB1/TB2四路栅极PWM，保持计数器和采样触发运行。
 * 函数名沿用参考工程，操作对象为本模块A/B；不管理从机状态或故障标志。
 * 需恢复时，由应用调用HAL_HRTIM_WaveformOutputStart(&hhrtim1, UFD_PWM_OUTPUTS)。
 */
void Slave_Stop(void);

#ifdef __cplusplus
}
#endif

#endif /* UFD_PWM_H */
