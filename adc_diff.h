#ifndef ADC_DIFF_H
#define ADC_DIFF_H

#include "stm32g4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/* STM32G474：一个实例管理 ADC1+ADC2 或 ADC3+ADC4。
 * CubeMX 负责 GPIO、外部触发、规则双同步差分和循环 WORD DMA 配置。
 * 模块不定义 HAL 全局回调；请从现有回调转发，避免多个模块争用回调。
 */
typedef struct
{
    int16_t zero_offset_code;  /* 去掉2048中点后再减的零偏，单位为ADC码。 */
    int8_t polarity;           /* +1或-1，决定adc_voltage_v的测量极性。 */
    float physical_gain;      /* 正数，物理量/ADC端V；例如12.8 V/V。 */
    float physical_offset;    /* 物理量换算后再减的偏置；通常为0。 */
} ADC_Diff_ChannelConfig;

typedef struct
{
    ADC_HandleTypeDef *master;
    ADC_HandleTypeDef *slave;
    float reference_v;
    ADC_Diff_ChannelConfig channel[2]; /* [0]主ADC/CDR低16位，[1]从ADC/高16位。 */
} ADC_Diff_Config;

typedef struct
{
    uint16_t raw;
    int16_t signed_code;       /* (raw & 0xFFF)-2048。 */
    int32_t corrected_code;    /* signed_code-zero_offset_code。 */
    float adc_voltage_v;       /* polarity*(corrected_code/2048)*reference_v。 */
    float value;               /* adc_voltage_v*physical_gain-physical_offset。 */
} ADC_Diff_ChannelSample;

typedef struct
{
    uint32_t sequence;         /* 本实例成功发布次数；两实例之间不自动配帧。 */
    uint32_t packed_word;
    ADC_Diff_ChannelSample channel[2];
} ADC_Diff_Snapshot;

typedef enum
{
    ADC_DIFF_STOPPED = 0,
    ADC_DIFF_STARTING,
    ADC_DIFF_RUNNING,
    ADC_DIFF_FAULT
} ADC_Diff_State;

typedef struct
{
    ADC_Diff_Config config;    /* Init后不应直接修改，须Stop后重新Init。 */
    volatile uint32_t dma_word; /* 必须保持实例生命周期和32位对齐，不得放DMA不可达存储区。 */
    volatile ADC_Diff_Snapshot latest;
    volatile uint32_t sample_count;
    volatile uint32_t error_count;
    volatile uint32_t adc_error_code;
    volatile HAL_StatusTypeDef last_status;
    volatile HAL_StatusTypeDef cleanup_status;
    volatile ADC_Diff_State state;
    volatile uint8_t initialized;
    volatile uint8_t snapshot_valid;
    volatile uint8_t snapshot_writing;
} ADC_Diff_Context;

/* context首次使用前必须静态零初始化或={0}，且须独占该ADC对/DMA。
 * Init仅校验并绑定；ADC必须已由MX_ADCx_Init配置且尚未启动。
 * 已初始化实例只有STOPPED时可重新Init，否则返回HAL_BUSY；FAULT须先Stop成功。
 */
HAL_StatusTypeDef ADC_Diff_Init(ADC_Diff_Context *context, const ADC_Diff_Config *config);

/* 只在主循环/初始化阶段调用，调用时外部触发源必须处于停止状态。
 * Start：分别差分校准，再由主ADC启动单字双模式DMA；不额外启动从ADC。
 * 重复Start返回HAL_BUSY；HAL启动失败会回滚，回滚失败进入FAULT并禁止Start。
 * Stop前先停外部触发；Stop幂等，失败进入FAULT，可再次Stop重试清理。
 * Start/Stop不可与同一实例其它API并发，也不要在ISR中调用。
 */
HAL_StatusTypeDef ADC_Diff_Start(ADC_Diff_Context *context);
HAL_StatusTypeDef ADC_Diff_Stop(ADC_Diff_Context *context);

/* 从HAL_ADC_ConvCpltCallback转发：仅本实例主ADC且RUNNING时发布并返回1。
 * DMA循环单字邮箱不是队列，ISR延迟导致的覆盖不能靠该模块恢复。
 */
uint8_t ADC_Diff_OnConversionComplete(ADC_Diff_Context *context, ADC_HandleTypeDef *hadc);

/* 从HAL_ADC_ErrorCallback转发：匹配主/从ADC时记录错误并进入FAULT。
 * 此函数不阻塞停止硬件；上层须立即禁止功率输出，并在主循环停触发和Stop。
 */
uint8_t ADC_Diff_OnError(ADC_Diff_Context *context, ADC_HandleTypeDef *hadc);

/* 快照读取：有完整结果返回HAL_OK；首帧前/发布中返回HAL_BUSY。
 * 内部短暂屏蔽中断并恢复原PRIMASK，快照各字段来自同一次发布。
 * Stop或错误后返回HAL_BUSY，旧值可供调试但不会作为有效结果返回。
 */
HAL_StatusTypeDef ADC_Diff_Read(const ADC_Diff_Context *context, ADC_Diff_Snapshot *snapshot);

#ifdef __cplusplus
}
#endif
#endif
