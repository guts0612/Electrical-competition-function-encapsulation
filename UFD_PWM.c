#include "UFD_PWM.h"
#include "hrtim.h"

/* Timer A/B 的周期峰值，由初始化读取；不等于完整三角载波的计数长度。 */
static uint32_t pwm_period;


// 注释的函数需要在adc初始化和闭环函数初始化后使用，保证定时器初始化得到更新周期，并传入闭环函数进行初始化，adc初始化后，再发波
// HAL_HRTIM_WaveformCountStart_IT(
//     &hhrtim1,
//     HRTIM_TIMERID_MASTER |
//     HRTIM_TIMERID_TIMER_A |
//     HRTIM_TIMERID_TIMER_B);


void HRTIM_SPWM_Init(float *spwm_update_hz,
                     float *phase_step_rad,
                     float *spwm_phase_rad,
                     volatile uint32_t *spwm_update_count)
{
    uint32_t master_period;
    uint32_t hrtim_clock_hz;

    pwm_period = __HAL_HRTIM_GETPERIOD(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A);
    master_period = __HAL_HRTIM_GETPERIOD(&hhrtim1, HRTIM_TIMERINDEX_MASTER);

    /* DIV1计数时钟：APB2不分频时等于PCLK2，否则为PCLK2的两倍。 */
    hrtim_clock_hz = HAL_RCC_GetPCLK2Freq();
    if ((RCC->CFGR & RCC_CFGR_PPRE2) != 0U) hrtim_clock_hz *= 2U;
    *spwm_update_hz = (float)hrtim_clock_hz / (float)master_period;
    *phase_step_rad = 2.0f * 3.14159265358979323846f * SPWM_OUTPUT_FREQUENCY_HZ / *spwm_update_hz;
    *spwm_phase_rad = 0.0f;
    *spwm_update_count = 0U;

    /* 同步启动准备：保持M/A/B停止且计数为0，两桥臂先装载相同的50%占空比。 */
    Slave_Stop();
    HRTIM_SetModulation(0.0f);
    if (HAL_HRTIM_SoftwareUpdate(&hhrtim1, HRTIM_TIMERUPDATE_MASTER |
                                HRTIM_TIMERUPDATE_A | HRTIM_TIMERUPDATE_B) != HAL_OK)
    {
        Error_Handler();
    }

    /* EQUAL + Reset CMP1：预置谷部x1为高电平，x2由死区单元生成互补。 */
    if (HAL_HRTIM_WaveformSetOutputLevel(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A,
                                       HRTIM_OUTPUT_TA1, HRTIM_OUTPUTLEVEL_ACTIVE) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_HRTIM_WaveformSetOutputLevel(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B,
                                       HRTIM_OUTPUT_TB1, HRTIM_OUTPUTLEVEL_ACTIVE) != HAL_OK)
    {
        Error_Handler();
    }
    __HAL_HRTIM_MASTER_CLEAR_FLAG(&hhrtim1, HRTIM_MASTER_FLAG_MREP);
}

void HRTIM_SetModulation(float modulation)
{
    uint32_t compare_a;
    uint32_t compare_b;
    uint32_t maximum = pwm_period - HRTIM_COMPARE_MARGIN_TICKS;
    uint32_t primask;

    if (modulation > SPWM_MODULATION_LIMIT) modulation = SPWM_MODULATION_LIMIT;
    if (modulation < -SPWM_MODULATION_LIMIT) modulation = -SPWM_MODULATION_LIMIT;

    /* 单极倍频：dA=(1+m)/2，dB=(1-m)/2；加0.5后取整，得到CMP1比较值。 */
    compare_a = (uint32_t)(0.5f * (1.0f + modulation) * (float)pwm_period + 0.5f);
    compare_b = (uint32_t)(0.5f * (1.0f - modulation) * (float)pwm_period + 0.5f);
    if (compare_a < HRTIM_COMPARE_MARGIN_TICKS) compare_a = HRTIM_COMPARE_MARGIN_TICKS;
    if (compare_b < HRTIM_COMPARE_MARGIN_TICKS) compare_b = HRTIM_COMPARE_MARGIN_TICKS;
    if (compare_a > maximum) compare_a = maximum;
    if (compare_b > maximum) compare_b = maximum;

    /* 暂停A/B预装载转移，避免谷底恰好落在两次写入之间，造成两桥臂相差一拍。
     * 写完共同恢复，比较值在下一共同谷底生效；正常调制不强制软件更新。
     */
    primask = __get_PRIMASK();
    __disable_irq();
    SET_BIT(hhrtim1.Instance->sCommonRegs.CR1, HRTIM_CR1_TAUDIS | HRTIM_CR1_TBUDIS);
    __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_1, compare_a);
    __HAL_HRTIM_SETCOMPARE(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_COMPAREUNIT_1, compare_b);
    CLEAR_BIT(hhrtim1.Instance->sCommonRegs.CR1, HRTIM_CR1_TAUDIS | HRTIM_CR1_TBUDIS);
    __set_PRIMASK(primask);
}

void Slave_Stop(void)
{
    /* m=0仍为两桥臂50%占空比；真正关断必须禁用TA1/TA2/TB1/TB2输出。 */
    hhrtim1.Instance->sCommonRegs.ODISR = UFD_PWM_OUTPUTS;
}
