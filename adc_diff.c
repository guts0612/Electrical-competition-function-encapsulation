#include "adc_diff.h"
#include "adc.h"

/*单adc转换和同步adc转换都放在中断中调用*/

/* 12位、右对齐：输出 = 原码 - 2048 - zero_offset。
 * zero_offset 为额外零偏，单位为ADC码，可正可负，不校正时填0。
 * 例如零输入时原码为2045，则填-3；零偏为0时输出范围为-2048~2047。
 */
int16_t AdcDiff_ConvertSingle(uint32_t raw, int16_t zero_offset)
{
    return (int16_t)((int32_t)(raw & 0x0FFFU) - ADC_DIFF_ZERO_CODE - zero_offset);
}
/* 双ADC同步DMA数据：低16位为ADC1，高16位为ADC2。
 * 两路分别去掉2048中点并减去各自的额外零偏，通过指针输出。
 */
void AdcDiff_ConvertDual(uint32_t raw,
                        int16_t *adc1_value,
                        int16_t *adc2_value,
                        int16_t adc1_zero_offset,
                        int16_t adc2_zero_offset)
{
    *adc1_value = AdcDiff_ConvertSingle(raw & 0xFFFFU, adc1_zero_offset);
    *adc2_value = AdcDiff_ConvertSingle(raw >> 16, adc2_zero_offset);
}

void AdcDiff_Init(volatile uint32_t *adc12_buffer,
                  volatile uint32_t *adc34_buffer)
{
    /* 在外部定时器启动触发之前，先完成四个ADC的差分校准。 */
    if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_DIFFERENTIAL_ENDED) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_ADCEx_Calibration_Start(&hadc2, ADC_DIFFERENTIAL_ENDED) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_ADCEx_Calibration_Start(&hadc3, ADC_DIFFERENTIAL_ENDED) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_ADCEx_Calibration_Start(&hadc4, ADC_DIFFERENTIAL_ENDED) != HAL_OK)
    {
        Error_Handler();
    }

    /* 直接传入调用方缓冲区的地址，DMA持续写入该变量。 */
    if (HAL_ADCEx_MultiModeStart_DMA(&hadc1, (uint32_t *)adc12_buffer, 1U) != HAL_OK)
    {
        Error_Handler();
    }
    /* 每组仅转运一个32位字，不使用半缓冲中断。 */
    __HAL_DMA_DISABLE_IT(hadc1.DMA_Handle, DMA_IT_HT);
    if (HAL_ADCEx_MultiModeStart_DMA(&hadc3, (uint32_t *)adc34_buffer, 1U) != HAL_OK)
    {
        Error_Handler();
    }
    __HAL_DMA_DISABLE_IT(hadc3.DMA_Handle, DMA_IT_HT);

    /* HAL已使能主、从ADC；从ADC不单独启动DMA，仅补充溢出监测。 */
    __HAL_ADC_CLEAR_FLAG(&hadc2, ADC_FLAG_OVR);
    __HAL_ADC_CLEAR_FLAG(&hadc4, ADC_FLAG_OVR);
    __HAL_ADC_ENABLE_IT(&hadc2, ADC_IT_OVR);
    __HAL_ADC_ENABLE_IT(&hadc4, ADC_IT_OVR);
}
