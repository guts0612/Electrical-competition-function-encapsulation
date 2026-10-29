#include "justfloat.h"
#include "arm_math.h"

#include <float.h>
#include <string.h>

typedef char JustFloatRequires32BitFloat[(sizeof(float) == 4U) ? 1 : -1];

static const uint8_t justfloat_frame_tail[] = {0x00U, 0x00U, 0x80U, 0x7FU};

static void JustFloat_accumulateVoltageRms(JustFloat_Context *context,
                                           float voltage_v,
                                           uint8_t closed_loop)
{
    float mean_square;
    float rms_v;

    if (closed_loop != context->rms_mode)
    {
        context->rms_square_sum = 0.0F;
        context->rms_count = 0U;
        context->rms_mode = closed_loop;
        context->output_voltage_rms_v = 0.0F;
        context->output_voltage_rms_valid = 0U;
    }

    context->rms_square_sum += voltage_v * voltage_v;
    context->rms_count++;
    if (context->rms_count < context->output_voltage_rms_window_samples)
    {
        return;
    }

    mean_square = context->rms_square_sum * context->rms_scale;
    (void)arm_sqrt_f32(mean_square, &rms_v);
    context->output_voltage_rms_v = rms_v;
    context->rms_square_sum = 0.0F;
    context->rms_count = 0U;
    context->output_voltage_rms_window_count++;
    context->output_voltage_rms_valid = 1U;
}

uint8_t JustFloat_Init(JustFloat_Context *context,
                       UART_HandleTypeDef *uart,
                       float sample_hz,
                       float fundamental_hz)
{
    float window;

    if (context == NULL)
    {
        return 0U;
    }

    memset(context, 0, sizeof(*context));
    if ((uart == NULL) ||
        !(sample_hz > 0.0F && sample_hz <= FLT_MAX) ||
        !(fundamental_hz > 0.0F && fundamental_hz <= FLT_MAX))
    {
        return 0U;
    }

    window = sample_hz / fundamental_hz;
    /* 避免浮点数转uint32_t时越界。 */
    if (!(window >= 1.0F && window < 4294967040.0F))
    {
        return 0U;
    }

    context->output_voltage_rms_window_samples = (uint32_t)(window + 0.5F);
    context->rms_scale = 1.0F / (float)context->output_voltage_rms_window_samples;
    context->uart = uart;
    context->initialized = 1U;
    return 1U;
}

void JustFloat_Sample(JustFloat_Context *context,
                      const JustFloat_SampleData *sample,
                      uint8_t closed_loop)
{
    if ((context == NULL) || (sample == NULL) || (context->initialized == 0U))
    {
        return;
    }

    /* 全部采样参与RMS计算，波形发送再按比例抽点。 */
    JustFloat_accumulateVoltageRms(context, sample->voltage_v, closed_loop);
    context->sample_div_count++;
    if (context->sample_div_count < JUSTFLOAT_SAMPLE_DIV)
    {
        return;
    }
    context->sample_div_count = 0U;

    if (context->frame_ready != 0U)
    {
        context->overwrite_count++;
    }
    context->channels[0] = sample->reference_v;
    context->channels[1] = sample->voltage_v;
    context->channels[2] = sample->current_reference_a;
    context->channels[3] = sample->current_a;
    context->channels[4] = sample->controller_limited_output_v;
    context->channels[5] = sample->auxiliary_current_a;
    context->channels[6] = sample->dc_bus_v;
    context->channels[7] = context->output_voltage_rms_v;
    context->frame_ready = 1U;
}

HAL_StatusTypeDef JustFloat_Send8(JustFloat_Context *context,
                                  float ch1,
                                  float ch2,
                                  float ch3,
                                  float ch4,
                                  float ch5,
                                  float ch6,
                                  float ch7,
                                  float ch8)
{
    HAL_StatusTypeDef status;
    float channels[JUSTFLOAT_CHANNEL_COUNT] = {ch1, ch2, ch3, ch4, ch5, ch6, ch7, ch8};

    if ((context == NULL) || (context->initialized == 0U))
    {
        return HAL_ERROR;
    }

    memcpy(context->frame, channels, sizeof(channels));
    memcpy(context->frame + sizeof(channels), justfloat_frame_tail, sizeof(justfloat_frame_tail));

    /* 主循环发送；等待期间保持调用前的中断状态。 */
    status = HAL_UART_Transmit(context->uart, context->frame,
                               (uint16_t)sizeof(context->frame), JUSTFLOAT_UART_TIMEOUT_MS);
    if (status == HAL_OK)
    {
        context->sent_count++;
    }
    else
    {
        context->tx_error_count++;
    }
    return status;
}

void JustFloat_Process(JustFloat_Context *context)
{
    float channels[JUSTFLOAT_CHANNEL_COUNT];
    uint32_t primask;
    uint32_t i;

    if ((context == NULL) || (context->initialized == 0U) || (context->frame_ready == 0U))
    {
        return;
    }

    /* 临界区只复制快照，发送前恢复原中断状态。 */
    primask = __get_PRIMASK();
    __disable_irq();
    for (i = 0U; i < JUSTFLOAT_CHANNEL_COUNT; ++i)
    {
        channels[i] = context->channels[i];
    }
    context->frame_ready = 0U;
    __set_PRIMASK(primask);

    (void)JustFloat_Send8(context, channels[0], channels[1], channels[2], channels[3],
                          channels[4], channels[5], channels[6], channels[7]);
}
