#ifndef JUSTFLOAT_H
#define JUSTFLOAT_H

#include "stm32g4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 在工程预处理宏中覆盖这些配置；justfloat.c 与调用方必须使用相同配置。 */
#ifndef JUSTFLOAT_SAMPLE_DIV
#define JUSTFLOAT_SAMPLE_DIV           10U
#endif
#ifndef JUSTFLOAT_UART_TIMEOUT_MS
#define JUSTFLOAT_UART_TIMEOUT_MS      5U
#endif

#if JUSTFLOAT_SAMPLE_DIV < 1U
#error "JUSTFLOAT_SAMPLE_DIV must be at least 1"
#endif

#define JUSTFLOAT_CHANNEL_COUNT        8U
#define JUSTFLOAT_FRAME_BYTES          36U

/* 传入同一控制拍的物理量；不依赖 QPR_Control 或 main.c 全局变量。 */
typedef struct
{
    float reference_v;
    float voltage_v;
    float current_reference_a;
    float current_a;
    float controller_limited_output_v; /* CH5：控制器限幅后输出，V，由调用方提供。 */
    float auxiliary_current_a;
    float dc_bus_v;
} JustFloat_SampleData;

/* 使用静态变量或 {0} 初始化，方便 Keil Watch 展开观测。
 * initialized、uart、窗口累加器及 frame 是内部状态，运行时不要修改。
 * 一个 context 由一个采样 ISR 写入、一个主循环读取，不支持并行发送者。
 */
typedef struct
{
    UART_HandleTypeDef *uart;
    volatile uint8_t initialized;
    volatile float channels[JUSTFLOAT_CHANNEL_COUNT];
    volatile uint8_t frame_ready;
    volatile uint32_t sample_div_count;
    volatile uint32_t sent_count;
    volatile uint32_t tx_error_count;
    volatile uint32_t overwrite_count;
    volatile float output_voltage_rms_v;
    volatile uint8_t output_voltage_rms_valid;
    volatile uint32_t output_voltage_rms_window_count;
    uint32_t output_voltage_rms_window_samples;
    float rms_square_sum;
    float rms_scale;
    uint32_t rms_count;
    uint8_t rms_mode;
    uint8_t frame[JUSTFLOAT_FRAME_BYTES];
} JustFloat_Context;

/* 在启动 ADC/HRTIM 采样前调用。不会初始化 UART/GPIO。
 * sample_hz/fundamental_hz 四舍五入为 RMS 窗口；20 kHz/50 Hz 对应400点。
 * 成功返回1；空串口句柄、非有限/非正频率或无效窗口返回0并禁用本实例。
 * 重初始化须先停止 Sample/Process；未初始化的零值实例安全不操作。
 */
uint8_t JustFloat_Init(JustFloat_Context *context, UART_HandleTypeDef *uart,
                       float sample_hz, float fundamental_hz);

/* ADC配齐且控制计算完成后，每拍调用一次；只累加/发布数据，不发送串口。
 * closed_loop 传入与本拍控制使用的同一0/1快照。
 * 波形 CH1~8：参考V、输出V、电流参考A、电感电流A、控制器限幅后输出V、
 * 辅助电流A、母线V、电压RMS。每10拍发布一帧。
 * CH1~7为抽取时刻的瞬时值；CH8为使用全部采样计算的电压总RMS。
 * 不提供均值输出模式；开闭环切换时重新开始RMS窗口。
 */
void JustFloat_Sample(JustFloat_Context *context,
                      const JustFloat_SampleData *sample, uint8_t closed_loop);

/* 主循环反复调用。PRIMASK保护一次八通道快照，随后恢复原中断状态，
 * 同步HAL发送36字节。默认5ms超时；失败计数后丢帧，不重试。
 * 发送期间的新采样留在邮箱，下一次Process取走；邮箱满时覆盖旧帧并计数。
 */
void JustFloat_Process(JustFloat_Context *context);

/* 可选：主循环直接同步发送八通道，不影响采样邮箱。
 * 同一串口不要混用其他发送者；不要在ISR中调用或与Process并发调用。
 * 返回HAL发送状态；空/未初始化实例返回HAL_ERROR，不计入发送次数。
 */
HAL_StatusTypeDef JustFloat_Send8(JustFloat_Context *context,
                                float ch1, float ch2, float ch3, float ch4,
                                float ch5, float ch6, float ch7, float ch8);

#ifdef __cplusplus
}
#endif
#endif
