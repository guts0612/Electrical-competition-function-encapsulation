#include "hrtim_spwm.h"

#include <float.h>
#include <string.h>

#define SPWM_TIMERS (HRTIM_TIMERID_MASTER | HRTIM_TIMERID_TIMER_A | HRTIM_TIMERID_TIMER_B)
#define SPWM_UPDATES (HRTIM_TIMERUPDATE_MASTER | HRTIM_TIMERUPDATE_A | HRTIM_TIMERUPDATE_B)
#define SPWM_AB_UPDATES (HRTIM_TIMERUPDATE_A | HRTIM_TIMERUPDATE_B)
#define SPWM_OUTPUTS (HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2 | HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2)
#define SPWM_TRY(call) do { status = (call); if (status != HAL_OK) return status; } while (0)

static uint8_t SPWM_IsReady(const HRTIM_SPWM_Context *context)
{
    return (uint8_t)((context != NULL) && (context->initialized != 0U) &&
                     (context->hrtim != NULL) && (context->hrtim->Instance != NULL));
}

void HRTIM_SPWM_DefaultConfig(HRTIM_SPWM_Config *config)
{
    if (config == NULL) return;
    config->timer_clock_hz = 170000000U;
    config->carrier_peak_ticks = 4250U;
    config->modulation_limit = 0.95f;
    config->deadtime_a_rising_ticks = 0U;
    config->deadtime_a_falling_ticks = 25U;
    config->deadtime_b_rising_ticks = 0U;
    config->deadtime_b_falling_ticks = 0U;
}

HAL_StatusTypeDef HRTIM_SPWM_Init(HRTIM_SPWM_Context *context,
                                HRTIM_HandleTypeDef *hrtim,
                                const HRTIM_SPWM_Config *config)
{
    HRTIM_SPWM_Config local;
    HRTIM_TimeBaseCfgTypeDef timebase = {0};
    HRTIM_TimerCfgTypeDef timer = {0};
    HRTIM_TimerCtlTypeDef control = {0};
    HRTIM_CompareCfgTypeDef compare = {0};
    HRTIM_DeadTimeCfgTypeDef deadtime = {0};
    HRTIM_OutputCfgTypeDef output = {0};
    HRTIM_ADCTriggerCfgTypeDef adc_trigger = {0};
    HAL_StatusTypeDef status;
    uint32_t index;

    if ((context == NULL) || (hrtim == NULL) || (hrtim->Instance == NULL) ||
        (config == NULL)) return HAL_ERROR;
    if (context->ever_started != 0U) return HAL_BUSY;
    /* 允许再次用context.config初始化，先复制，随后才清除上下文。 */
    local = *config;
    if ((local.timer_clock_hz == 0U) ||
        (local.carrier_peak_ticks <= 2U * HRTIM_SPWM_COMPARE_MARGIN_TICKS) ||
        (local.carrier_peak_ticks > 32767U) ||
        !(local.modulation_limit > 0.0f && local.modulation_limit <= 1.0f) ||
        (local.deadtime_a_rising_ticks > 511U) || (local.deadtime_a_falling_ticks > 511U) ||
        (local.deadtime_b_rising_ticks > 511U) || (local.deadtime_b_falling_ticks > 511U))
        return HAL_ERROR;
    if ((hrtim->State != HAL_HRTIM_STATE_READY) || (hrtim->Lock != HAL_UNLOCKED) ||
        ((hrtim->Instance->sMasterRegs.MCR & SPWM_TIMERS) != 0U) ||
        ((hrtim->Instance->sCommonRegs.OENR & SPWM_OUTPUTS) != 0U) ||
        (hrtim->Instance->sMasterRegs.MCNTR != 0U) ||
        (hrtim->Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].CNTxR != 0U) ||
        (hrtim->Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_B].CNTxR != 0U)) return HAL_BUSY;

    memset(context, 0, sizeof(*context));
    context->hrtim = hrtim;
    context->config = local;
    context->update_hz = (float)local.timer_clock_hz / (float)(2U * local.carrier_peak_ticks);
    hrtim->Instance->sCommonRegs.ODISR = SPWM_OUTPUTS;
    /* 本模块独占M/A/B更新；其余计时单元对应位保持不变。 */
    hrtim->Instance->sCommonRegs.CR1 &= ~SPWM_UPDATES;

    timebase.Period = 2U * local.carrier_peak_ticks;
    timebase.RepetitionCounter = 0U;
    timebase.PrescalerRatio = HRTIM_PRESCALERRATIO_DIV1;
    timebase.Mode = HRTIM_MODE_CONTINUOUS;
    SPWM_TRY(HAL_HRTIM_TimeBaseConfig(hrtim, HRTIM_TIMERINDEX_MASTER, &timebase));
    timer.HalfModeEnable = HRTIM_HALFMODE_DISABLED;
    timer.InterleavedMode = HRTIM_INTERLEAVED_MODE_DISABLED;
    timer.StartOnSync = HRTIM_SYNCSTART_DISABLED;
    timer.ResetOnSync = HRTIM_SYNCRESET_DISABLED;
    timer.DACSynchro = HRTIM_DACSYNC_NONE;
    timer.PreloadEnable = HRTIM_PRELOAD_ENABLED;
    timer.UpdateGating = HRTIM_UPDATEGATING_INDEPENDENT;
    timer.BurstMode = HRTIM_TIMERBURSTMODE_MAINTAINCLOCK;
    timer.RepetitionUpdate = HRTIM_UPDATEONREPETITION_ENABLED;
    timer.ReSyncUpdate = HRTIM_TIMERESYNC_UPDATE_UNCONDITIONAL;
    timer.DMASize = 1U;
    SPWM_TRY(HAL_HRTIM_WaveformTimerConfig(hrtim, HRTIM_TIMERINDEX_MASTER, &timer));

    timebase.Period = local.carrier_peak_ticks;
    control.UpDownMode = HRTIM_TIMERUPDOWNMODE_UPDOWN;
    control.GreaterCMP1 = HRTIM_TIMERGTCMP1_EQUAL;
    control.DualChannelDacEnable = HRTIM_TIMER_DCDE_DISABLED;
    timer.PushPull = HRTIM_TIMPUSHPULLMODE_DISABLED;
    timer.FaultEnable = HRTIM_TIMFAULTENABLE_NONE;
    timer.FaultLock = HRTIM_TIMFAULTLOCK_READWRITE;
    timer.DeadTimeInsertion = HRTIM_TIMDEADTIMEINSERTION_ENABLED;
    timer.DelayedProtectionMode = HRTIM_TIMER_A_B_C_DELAYEDPROTECTION_DISABLED;
    timer.UpdateTrigger = HRTIM_TIMUPDATETRIGGER_NONE;
    timer.ResetTrigger = HRTIM_TIMRESETTRIGGER_NONE;
    timer.ResetUpdate = HRTIM_TIMUPDATEONRESET_DISABLED;
    compare.CompareValue = local.carrier_peak_ticks / 2U + local.carrier_peak_ticks % 2U;
    deadtime.Prescaler = HRTIM_TIMDEADTIME_PRESCALERRATIO_DIV1;
    deadtime.RisingSign = HRTIM_TIMDEADTIME_RISINGSIGN_POSITIVE;
    deadtime.RisingLock = HRTIM_TIMDEADTIME_RISINGLOCK_WRITE;
    deadtime.RisingSignLock = HRTIM_TIMDEADTIME_RISINGSIGNLOCK_WRITE;
    deadtime.FallingSign = HRTIM_TIMDEADTIME_FALLINGSIGN_POSITIVE;
    deadtime.FallingLock = HRTIM_TIMDEADTIME_FALLINGLOCK_WRITE;
    deadtime.FallingSignLock = HRTIM_TIMDEADTIME_FALLINGSIGNLOCK_WRITE;
    output.Polarity = HRTIM_OUTPUTPOLARITY_HIGH;
    output.SetSource = HRTIM_OUTPUTSET_NONE;
    output.ResetSource = HRTIM_OUTPUTRESET_TIMCMP1;
    output.IdleMode = HRTIM_OUTPUTIDLEMODE_NONE;
    output.IdleLevel = HRTIM_OUTPUTIDLELEVEL_INACTIVE;
    output.FaultLevel = HRTIM_OUTPUTFAULTLEVEL_NONE;
    output.ChopperModeEnable = HRTIM_OUTPUTCHOPPERMODE_DISABLED;
    output.BurstModeEntryDelayed = HRTIM_OUTPUTBURSTMODEENTRY_REGULAR;
    for (index = HRTIM_TIMERINDEX_TIMER_A; index <= HRTIM_TIMERINDEX_TIMER_B; index++)
    {
        uint32_t primary = (index == HRTIM_TIMERINDEX_TIMER_A) ? HRTIM_OUTPUT_TA1 : HRTIM_OUTPUT_TB1;
        uint32_t complementary = (index == HRTIM_TIMERINDEX_TIMER_A) ? HRTIM_OUTPUT_TA2 : HRTIM_OUTPUT_TB2;
        deadtime.RisingValue = (index == HRTIM_TIMERINDEX_TIMER_A) ?
                               local.deadtime_a_rising_ticks : local.deadtime_b_rising_ticks;
        deadtime.FallingValue = (index == HRTIM_TIMERINDEX_TIMER_A) ?
                                local.deadtime_a_falling_ticks : local.deadtime_b_falling_ticks;
        SPWM_TRY(HAL_HRTIM_TimeBaseConfig(hrtim, index, &timebase));
        SPWM_TRY(HAL_HRTIM_WaveformTimerControl(hrtim, index, &control));
        SPWM_TRY(HAL_HRTIM_RollOverModeConfig(hrtim, index,
                 HRTIM_TIM_FEROM_VALLEY | HRTIM_TIM_BMROM_VALLEY | HRTIM_TIM_ADROM_VALLEY |
                 HRTIM_TIM_OUTROM_VALLEY | HRTIM_TIM_ROM_VALLEY));
        SPWM_TRY(HAL_HRTIM_WaveformTimerConfig(hrtim, index, &timer));
        SPWM_TRY(HAL_HRTIM_WaveformCompareConfig(hrtim, index, HRTIM_COMPAREUNIT_1, &compare));
        SPWM_TRY(HAL_HRTIM_DeadTimeConfig(hrtim, index, &deadtime));
        output.ResetSource = HRTIM_OUTPUTRESET_TIMCMP1;
        SPWM_TRY(HAL_HRTIM_WaveformOutputConfig(hrtim, index, primary, &output));
        output.ResetSource = HRTIM_OUTPUTRESET_NONE;
        SPWM_TRY(HAL_HRTIM_WaveformOutputConfig(hrtim, index, complementary, &output));
        /* UPDOWN+EQUAL+Reset CMP1：上数清零、下数置位；第二路由死区单元互补生成。 */
        SPWM_TRY(HAL_HRTIM_WaveformSetOutputLevel(hrtim, index, primary, HRTIM_OUTPUTLEVEL_ACTIVE));
    }
    adc_trigger.UpdateSource = HRTIM_ADCTRIGGERUPDATE_MASTER;
    adc_trigger.Trigger = HRTIM_ADCTRIGGEREVENT13_MASTER_PERIOD;
    SPWM_TRY(HAL_HRTIM_ADCTriggerConfig(hrtim, HRTIM_ADCTRIGGER_1, &adc_trigger));
    SPWM_TRY(HAL_HRTIM_ADCPostScalerConfig(hrtim, HRTIM_ADCTRIGGER_1, 0U));
    SPWM_TRY(HAL_HRTIM_SoftwareUpdate(hrtim, SPWM_UPDATES));
    context->compare_a = compare.CompareValue;
    context->compare_b = compare.CompareValue;
    context->initialized = 1U;
    return HAL_OK;
}

HAL_StatusTypeDef HRTIM_SetModulation(HRTIM_SPWM_Context *context, float modulation)
{
    uint32_t compare_a, compare_b, maximum, primask, previous_cr1;
    HRTIM_TypeDef *instance;
    if (!SPWM_IsReady(context) || !(modulation >= -FLT_MAX && modulation <= FLT_MAX))
        return HAL_ERROR;
    if (modulation > context->config.modulation_limit) modulation = context->config.modulation_limit;
    if (modulation < -context->config.modulation_limit) modulation = -context->config.modulation_limit;
    maximum = context->config.carrier_peak_ticks - HRTIM_SPWM_COMPARE_MARGIN_TICKS;
    compare_a = (uint32_t)(0.5f * (1.0f + modulation) * (float)context->config.carrier_peak_ticks + 0.5f);
    compare_b = (uint32_t)(0.5f * (1.0f - modulation) * (float)context->config.carrier_peak_ticks + 0.5f);
    if (compare_a < HRTIM_SPWM_COMPARE_MARGIN_TICKS) compare_a = HRTIM_SPWM_COMPARE_MARGIN_TICKS;
    if (compare_b < HRTIM_SPWM_COMPARE_MARGIN_TICKS) compare_b = HRTIM_SPWM_COMPARE_MARGIN_TICKS;
    if (compare_a > maximum) compare_a = maximum;
    if (compare_b > maximum) compare_b = maximum;

    instance = context->hrtim->Instance;
    primask = __get_PRIMASK();
    __disable_irq();
    previous_cr1 = instance->sCommonRegs.CR1;
    if ((previous_cr1 & SPWM_AB_UPDATES) != 0U)
    {
        __set_PRIMASK(primask);
        return HAL_BUSY;
    }
    /* HAL_HRTIM_UpdateDisable同样设置CR1相应位。一次同时禁止两路转移；
     * 即使谷底落在下面两次CMP写入之间，也不会出现一新一旧。
     * 不在此处SoftwareUpdate，保证运行中仍只在完整载波谷底换拍。
     */
    instance->sCommonRegs.CR1 = previous_cr1 | SPWM_AB_UPDATES;
    __HAL_HRTIM_SETCOMPARE(context->hrtim, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_1, compare_a);
    __HAL_HRTIM_SETCOMPARE(context->hrtim, HRTIM_TIMERINDEX_TIMER_B, HRTIM_COMPAREUNIT_1, compare_b);
    instance->sCommonRegs.CR1 = previous_cr1;
    context->compare_a = compare_a;
    context->compare_b = compare_b;
    context->modulation = modulation;
    __set_PRIMASK(primask);
    return HAL_OK;
}

HAL_StatusTypeDef HRTIM_SPWM_StartCounters(HRTIM_SPWM_Context *context)
{
    uint32_t primask;
    HRTIM_TypeDef *instance;
    if (!SPWM_IsReady(context)) return HAL_ERROR;
    instance = context->hrtim->Instance;
    if ((context->counters_started != 0U) || (context->ever_started != 0U)) return HAL_BUSY;
    if ((context->hrtim->State != HAL_HRTIM_STATE_READY) ||
        (context->hrtim->Lock != HAL_UNLOCKED) ||
        ((instance->sMasterRegs.MCR & SPWM_TIMERS) != 0U) ||
        ((instance->sCommonRegs.OENR & SPWM_OUTPUTS) != 0U) ||
        (instance->sMasterRegs.MCNTR != 0U) ||
        (instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].CNTxR != 0U) ||
        (instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_B].CNTxR != 0U)) return HAL_BUSY;
    instance->sCommonRegs.ODISR = SPWM_OUTPUTS;
    /* 仅接受完整外设复位后的首次启动。停表SoftwareReset无效；运行中再Reset
     * 可能让旧Master先溢出触发一帧ADC，因此此处既不运行复位，也不写CNT伪造方向。
     * 在零计数、复位初始方向的前提下，一次MCR写入同时使能M/A/B。
     */
    primask = __get_PRIMASK();
    __disable_irq();
    __HAL_HRTIM_MASTER_CLEAR_FLAG(context->hrtim, HRTIM_MASTER_FLAG_MREP);
    context->ever_started = 1U;
    instance->sMasterRegs.MCR |= SPWM_TIMERS;
    context->counters_started = 1U;
    __set_PRIMASK(primask);
    return HAL_OK;
}

HAL_StatusTypeDef HRTIM_SPWM_EnableOutputs(HRTIM_SPWM_Context *context)
{
    if (!SPWM_IsReady(context)) return HAL_ERROR;
    if ((context->counters_started == 0U) ||
        ((context->hrtim->Instance->sMasterRegs.MCR & SPWM_TIMERS) != SPWM_TIMERS)) return HAL_ERROR;
    context->hrtim->Instance->sCommonRegs.OENR = SPWM_OUTPUTS;
    context->outputs_enabled = 1U;
    return HAL_OK;
}

void HRTIM_SPWM_DisableOutputs(HRTIM_SPWM_Context *context)
{
    if (!SPWM_IsReady(context)) return;
    context->hrtim->Instance->sCommonRegs.ODISR = SPWM_OUTPUTS;
    context->outputs_enabled = 0U;
}

HAL_StatusTypeDef HRTIM_SPWM_Stop(HRTIM_SPWM_Context *context)
{
    HAL_StatusTypeDef status;
    if (!SPWM_IsReady(context)) return HAL_ERROR;
    HRTIM_SPWM_DisableOutputs(context);
    status = HAL_HRTIM_WaveformCountStop(context->hrtim, SPWM_TIMERS);
    if (status == HAL_OK) context->counters_started = 0U;
    return status;
}
