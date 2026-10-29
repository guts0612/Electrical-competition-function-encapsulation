#ifndef ADC_DIFF_H
#define ADC_DIFF_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ADC_DIFF_ZERO_CODE         2048

/* 初始化ADC1+ADC2、ADC3+ADC4两组同步差分采样；句柄由工程adc.h提供。
 * 在MX_DMA_Init()及四个MX_ADCx_Init()之后、外部定时器开始触发之前调用。
 * CubeMX需预先配置12位右对齐差分、双ADC同步、DMA循环及32位数据对齐。
 * 传入两个全局或static volatile uint32_t变量的地址，DMA各转运一个32位字。
 * 缓冲变量在DMA运行期间必须始终有效；失败沿用工程Error_Handler()处理。
 * ADC/DMA中断处理及HAL采样/错误回调仍由调用方提供。
 */
void AdcDiff_Init(volatile uint32_t *adc12_buffer,
                  volatile uint32_t *adc34_buffer);

/* 12位、右对齐：输出 = 原码 - 2048 - zero_offset。
 * zero_offset 为额外零偏，单位为ADC码，可正可负，不校正时填0。
 * 例如零输入时原码为2045，则填-3；零偏为0时输出范围为-2048~2047。
 */
int16_t AdcDiff_ConvertSingle(uint32_t raw, int16_t zero_offset);

/* 双ADC同步DMA数据：低16位为ADC1，高16位为ADC2。
 * 两路分别去掉2048中点并减去各自的额外零偏，通过指针输出。
 */
void AdcDiff_ConvertDual(uint32_t raw,
                        int16_t *adc1_value,
                        int16_t *adc2_value,
                        int16_t adc1_zero_offset,
                        int16_t adc2_zero_offset);

#ifdef __cplusplus
}
#endif

#endif
