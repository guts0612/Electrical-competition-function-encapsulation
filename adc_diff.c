#include "adc_diff.h"
#include <float.h>
#include <string.h>

static uint8_t ADC_Diff_IsFinite(float value)
{
    return (uint8_t)((value >= -FLT_MAX) && (value <= FLT_MAX));
}

static uint8_t ADC_Diff_CheckADC(const ADC_HandleTypeDef *hadc)
{
    uint32_t channel = (hadc->Instance->SQR1 & ADC_SQR1_SQ1) >> ADC_SQR1_SQ1_Pos;
    /* 一次触发只转换一个12位右对齐差分通道，不接管注入组。 */
    if ((hadc->Init.Resolution != ADC_RESOLUTION_12B) ||
        (hadc->Init.DataAlign != ADC_DATAALIGN_RIGHT) ||
        (hadc->Init.NbrOfConversion != 1U) ||
        (hadc->Init.ContinuousConvMode != DISABLE) ||
        (hadc->Init.DiscontinuousConvMode != DISABLE) ||
        (hadc->Init.OversamplingMode != DISABLE) ||
        (hadc->Init.GainCompensation != 0U) ||
        ((hadc->Instance->SQR1 & ADC_SQR1_L) != 0U) ||
        ((hadc->Instance->CFGR & (ADC_CFGR_RES | ADC_CFGR_ALIGN | ADC_CFGR_CONT |
                                 ADC_CFGR_DISCEN | ADC_CFGR_JAUTO | ADC_CFGR_AUTDLY)) != 0U) ||
        ((hadc->Instance->CFGR2 & (ADC_CFGR2_ROVSE | ADC_CFGR2_GCOMP)) != 0U) ||
        ((hadc->Instance->JSQR & ADC_JSQR_JEXTEN) != 0U) ||
        ((hadc->Instance->OFR1 & ADC_OFR1_OFFSET1_EN) != 0U) ||
        ((hadc->Instance->OFR2 & ADC_OFR2_OFFSET2_EN) != 0U) ||
        ((hadc->Instance->OFR3 & ADC_OFR3_OFFSET3_EN) != 0U) ||
        ((hadc->Instance->OFR4 & ADC_OFR4_OFFSET4_EN) != 0U) ||
        (channel == 0U) || (channel > 15U) ||
        ((hadc->Instance->DIFSEL & (1UL << channel)) == 0U))
    {
        return 0U;
    }
    return 1U;
}

static uint8_t ADC_Diff_CheckConfig(const ADC_Diff_Config *config)
{
    ADC_Common_TypeDef *common;
    const DMA_HandleTypeDef *dma;
    uint32_t i;
    if ((config == NULL) || (config->master == NULL) || (config->slave == NULL)) return 0U;
    if (!(((config->master->Instance == ADC1) && (config->slave->Instance == ADC2)) ||
          ((config->master->Instance == ADC3) && (config->slave->Instance == ADC4)))) return 0U;
    if (!(config->reference_v > 0.0f) || !ADC_Diff_IsFinite(config->reference_v)) return 0U;
    for (i = 0U; i < 2U; i++)
    {
        const ADC_Diff_ChannelConfig *channel = &config->channel[i];
        if ((channel->zero_offset_code < -2048) || (channel->zero_offset_code > 2047) ||
            ((channel->polarity != 1) && (channel->polarity != -1)) ||
            !(channel->physical_gain > 0.0f) || !ADC_Diff_IsFinite(channel->physical_gain) ||
            !ADC_Diff_IsFinite(channel->physical_offset)) return 0U;
    }
    if (!ADC_Diff_CheckADC(config->master) || !ADC_Diff_CheckADC(config->slave)) return 0U;
    if (((config->master->Instance->CR | config->slave->Instance->CR) &
         (ADC_CR_ADEN | ADC_CR_ADSTART | ADC_CR_JADSTART | ADC_CR_ADCAL)) != 0U) return 0U;
    if ((config->master->Init.DMAContinuousRequests != ENABLE) ||
        (config->master->Init.ExternalTrigConvEdge == ADC_EXTERNALTRIGCONVEDGE_NONE) ||
        ((config->master->Instance->CFGR & (ADC_CFGR_EXTSEL | ADC_CFGR_EXTEN)) !=
         (config->master->Init.ExternalTrigConv | config->master->Init.ExternalTrigConvEdge))) return 0U;
    common = __LL_ADC_COMMON_INSTANCE(config->master->Instance);
    if ((common->CCR & (ADC_CCR_DUAL | ADC_CCR_MDMA | ADC_CCR_DMACFG)) !=
        (ADC_DUALMODE_REGSIMULT | ADC_DMAACCESSMODE_12_10_BITS | ADC_CCR_DMACFG)) return 0U;
    dma = config->master->DMA_Handle;
    if ((dma == NULL) || (dma->Instance == NULL) || (dma->Parent != config->master) ||
        (dma->State != HAL_DMA_STATE_READY) ||
        (dma->Init.Direction != DMA_PERIPH_TO_MEMORY) ||
        (dma->Init.PeriphInc != DMA_PINC_DISABLE) ||
        (dma->Init.PeriphDataAlignment != DMA_PDATAALIGN_WORD) ||
        (dma->Init.MemDataAlignment != DMA_MDATAALIGN_WORD) ||
        (dma->Init.Mode != DMA_CIRCULAR) ||
        (dma->Init.Request != ((config->master->Instance == ADC1) ? DMA_REQUEST_ADC1 : DMA_REQUEST_ADC3))) return 0U;
    return 1U;
}

HAL_StatusTypeDef ADC_Diff_Init(ADC_Diff_Context *context, const ADC_Diff_Config *config)
{
    ADC_Diff_Config saved_config;
    if ((context == NULL) || (config == NULL)) return HAL_ERROR;
    if ((context->initialized != 0U) && (context->state != ADC_DIFF_STOPPED)) return HAL_BUSY;
    saved_config = *config; /* 允许Stop后用&context->config重新初始化。 */
    if (!ADC_Diff_CheckConfig(&saved_config)) return HAL_ERROR;
    memset(context, 0, sizeof(*context));
    context->config = saved_config;
    context->state = ADC_DIFF_STOPPED;
    context->initialized = 1U;
    return HAL_OK;
}

/* 本地HAL双模Stop停两ADC并中止主DMA；从ADC没有独立DMA，不能给它调用Stop_DMA。
 * 再用普通Stop同步两个实际handle状态；双模Stop失败时仍逐项尝试清理。
 * 硬件拒绝停止时只能报告FAULT，不能把HAL错误伪装为“已停止”。
 */
static HAL_StatusTypeDef ADC_Diff_Cleanup(ADC_Diff_Context *context)
{
    HAL_StatusTypeDef status = HAL_ADCEx_MultiModeStop_DMA(context->config.master);
    HAL_StatusTypeDef master_status = HAL_ADC_Stop(context->config.master);
    HAL_StatusTypeDef slave_status = HAL_ADC_Stop(context->config.slave);
    HAL_StatusTypeDef dma_status = HAL_OK;
    if (context->config.master->DMA_Handle->State == HAL_DMA_STATE_BUSY)
        dma_status = HAL_DMA_Abort(context->config.master->DMA_Handle);
    __HAL_ADC_DISABLE_IT(context->config.master, ADC_IT_OVR);
    __HAL_ADC_DISABLE_IT(context->config.slave, ADC_IT_OVR);
    __HAL_DMA_DISABLE_IT(context->config.master->DMA_Handle, DMA_IT_HT);
    context->snapshot_valid = 0U;
    if ((status != HAL_OK) || (master_status != HAL_OK) ||
        (slave_status != HAL_OK) || (dma_status != HAL_OK) ||
        (((context->config.master->Instance->CR | context->config.slave->Instance->CR) &
          (ADC_CR_ADEN | ADC_CR_ADSTART | ADC_CR_JADSTART | ADC_CR_ADCAL)) != 0U))
    {
        context->state = ADC_DIFF_FAULT;
        context->cleanup_status = HAL_ERROR;
        return HAL_ERROR;
    }
    context->state = ADC_DIFF_STOPPED;
    context->cleanup_status = HAL_OK;
    return HAL_OK;
}

HAL_StatusTypeDef ADC_Diff_Start(ADC_Diff_Context *context)
{
    HAL_StatusTypeDef status;
    if ((context == NULL) || (context->initialized == 0U)) return HAL_ERROR;
    if ((context->state == ADC_DIFF_RUNNING) || (context->state == ADC_DIFF_STARTING)) return HAL_BUSY;
    if ((context->state != ADC_DIFF_STOPPED) || !ADC_Diff_CheckConfig(&context->config)) return HAL_ERROR;
    context->snapshot_valid = 0U;
    context->state = ADC_DIFF_STARTING;
    status = HAL_ADCEx_Calibration_Start(context->config.master, ADC_DIFFERENTIAL_ENDED);
    if (status == HAL_OK)
        status = HAL_ADCEx_Calibration_Start(context->config.slave, ADC_DIFFERENTIAL_ENDED);
    if (status == HAL_OK)
        status = HAL_ADCEx_MultiModeStart_DMA(context->config.master, (uint32_t *)&context->dma_word, 1U);
    context->last_status = status;
    if (status != HAL_OK)
    {
        context->error_count++;
        (void)ADC_Diff_Cleanup(context);
        return status;
    }
    __HAL_DMA_DISABLE_IT(context->config.master->DMA_Handle, DMA_IT_HT);
    __HAL_ADC_CLEAR_FLAG(context->config.slave, ADC_FLAG_OVR);
    __HAL_ADC_ENABLE_IT(context->config.slave, ADC_IT_OVR);
    context->cleanup_status = HAL_OK;
    context->state = ADC_DIFF_RUNNING;
    return HAL_OK;
}

HAL_StatusTypeDef ADC_Diff_Stop(ADC_Diff_Context *context)
{
    HAL_StatusTypeDef status;
    if ((context == NULL) || (context->initialized == 0U)) return HAL_ERROR;
    if (context->state == ADC_DIFF_STOPPED) return HAL_OK;
    status = ADC_Diff_Cleanup(context);
    context->last_status = status;
    if (status != HAL_OK) context->error_count++;
    return status;
}

uint8_t ADC_Diff_OnConversionComplete(ADC_Diff_Context *context, ADC_HandleTypeDef *hadc)
{
    ADC_Diff_Snapshot snapshot;
    uint32_t i, primask;
    if ((context == NULL) || (context->initialized == 0U) ||
        (context->state != ADC_DIFF_RUNNING) || (hadc != context->config.master)) return 0U;
    snapshot.packed_word = context->dma_word; /* DMA字只读一次，避免两个通道来自不同次传输。 */
    snapshot.sequence = context->sample_count + 1U;
    for (i = 0U; i < 2U; i++)
    {
        ADC_Diff_ChannelSample *sample = &snapshot.channel[i];
        const ADC_Diff_ChannelConfig *config = &context->config.channel[i];
        sample->raw = (uint16_t)((snapshot.packed_word >> (16U * i)) & 0xFFFU);
        sample->signed_code = (int16_t)((int32_t)sample->raw - 2048);
        sample->corrected_code = (int32_t)sample->signed_code - config->zero_offset_code;
        sample->adc_voltage_v = ((float)sample->corrected_code / 2048.0f) * context->config.reference_v;
        if (config->polarity < 0) sample->adc_voltage_v = -sample->adc_voltage_v;
        sample->value = sample->adc_voltage_v * config->physical_gain - config->physical_offset;
    }
    /* 更高优先级ADC错误可能在换算期间抢占，故发布前再次检查并原子提交。 */
    primask = __get_PRIMASK();
    __disable_irq();
    if (context->state != ADC_DIFF_RUNNING)
    {
        __set_PRIMASK(primask);
        return 0U;
    }
    context->snapshot_writing = 1U;
    __DMB();
    context->latest = snapshot;
    context->sample_count = snapshot.sequence;
    __DMB();
    context->snapshot_writing = 0U;
    context->snapshot_valid = 1U;
    __set_PRIMASK(primask);
    return 1U;
}

uint8_t ADC_Diff_OnError(ADC_Diff_Context *context, ADC_HandleTypeDef *hadc)
{
    if ((context == NULL) || (context->initialized == 0U) || (hadc == NULL) ||
        ((hadc != context->config.master) && (hadc != context->config.slave))) return 0U;
    context->error_count++;
    context->adc_error_code |= hadc->ErrorCode;
    context->last_status = HAL_ERROR;
    context->snapshot_valid = 0U;
    context->state = ADC_DIFF_FAULT;
    return 1U;
}

HAL_StatusTypeDef ADC_Diff_Read(const ADC_Diff_Context *context, ADC_Diff_Snapshot *snapshot)
{
    uint32_t primask;
    HAL_StatusTypeDef status = HAL_BUSY;
    if ((context == NULL) || (snapshot == NULL) || (context->initialized == 0U)) return HAL_ERROR;
    primask = __get_PRIMASK();
    __disable_irq();
    if ((context->state == ADC_DIFF_RUNNING) && (context->snapshot_valid != 0U) &&
        (context->snapshot_writing == 0U))
    {
        __DMB();
        *snapshot = context->latest;
        status = HAL_OK;
    }
    __set_PRIMASK(primask);
    return status;
}
