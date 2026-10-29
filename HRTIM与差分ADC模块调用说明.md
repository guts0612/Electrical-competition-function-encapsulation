# 单机倍频 HRTIM 与差分 ADC 模块

将调制和采样独立成模块是合适的：控制器计算调制度，HRTIM 模块把调制度转换成 A/B 比较值，ADC 模块把采样码转换成物理量。更换 QPR 为其他控制器时，调制和采样模块仍可使用。

新文件均在项目根目录，与 `key_control.c/.h` 同目录：

| 文件 | 职责 |
|---|---|
| `Unipolar Frequency-Doubling PWM.c/.h` | 仅 MASTER、Timer A/B 的单极倍频调制、时基与同步配置、计数启动、四路输出使能/关闭 |
| `adc_diff.c/.h` | 一对双同步差分 ADC 的校验、校准、循环 DMA、回调处理、换算、快照与统计 |

这些模块尚未接入当前固件。本次提取不修改 `Core`、`Hardware`、Keil 项目或 `.ioc`；现有文件中的用户修改保留。它们面向 STM32G474 和当前 STM32G4 HAL，不是跨 MCU 的通用驱动。

## 1. CubeMX 与模块的分工

CubeMX 继续生成系统时钟、GPIO 复用、DMA/NVIC、HAL 句柄和 `MX_HRTIM1_Init()`、`MX_ADCx_Init()`。HRTIM 的基础初始化与 DLL 校准仍由 `MX_HRTIM1_Init()` 完成，新模块随后统一设置 MASTER/A/B 的运行时序。ADC 模块检查已生成的配置，再完成差分校准和采样启动。

模块不引用 QPR、正弦相位或 `main.c` 全局变量。控制器初始化、基波频率/相位、低母线输出禁止和故障动作属于应用层。单机应用只使用一个 `HRTIM_SetModulation()`；新模块没有 C/D 调制或从机控制代码。

添加两个新 C 文件到 Keil 工程，包含目录追加项目根目录 `..`。保持 `../Core/Inc` 排在根目录之前，避免根目录 QPR 候选头文件影响原工程。只添加所需模块，不要同时编译两份同名 QPR 实现。

## 2. HRTIM 的配置与同步边界

默认参数保留当前工程：

| 参数 | 默认值 |
|---|---|
| HRTIM 计数时钟 | 170 MHz，M/A/B 均 DIV1 |
| A/B 周期寄存器 | 4250，上下计数，共同谷底更新 |
| MASTER 周期寄存器 | 8500，向上计数，REP=0 |
| 完整载波和控制更新频率 | 20 kHz |
| 调制度限制 | ±0.95 |
| 比较值端点余量 | 6 tick |
| A 死区 | 上升 0、下降 25 |
| B 死区 | 上升 0、下降 0 |
| ADC 触发 | MASTER_PERIOD → HRTIM_TRG1，不分频 |

死区表保留的是当前 `Core/Src/hrtim.c` 的实际参数，不能当成其他功率板的默认安全值；更换硬件时应通过配置结构体重新设置。改变时钟时，也要同步修改 `timer_clock_hz`。

调制保持：

```c
dA = 0.5f * (1.0f + modulation);
dB = 0.5f * (1.0f - modulation);
```

`HRTIM_SetModulation()` 对调制度限幅、四舍五入比较值并保护端点。它在短临界区同时禁止 A/B 的预装载转移，写完两个比较值后共同恢复，使下一共同谷底使用同一拍数据。仅关闭 CPU 中断不能阻止硬件谷底，故还需要这一步更新控制。正常每拍不强制软件更新。

同步启动要求：**上电或完整 HRTIM 外设复位后，先执行 `MX_HRTIM1_Init()`，M/A/B 尚未运行且计数值均为零。** `HRTIM_SPWM_Init()` 配置相同载波条件，`StartCounters()` 一次 MCR 写入共同启动三个计数器。主定时器周期等于 A/B 三角波完整周期，因而 MASTER_PERIOD 与 A/B 谷底保持对齐。MASTER 只支持向上计数，A/B 使用上下计数，依据为 ST [RM0440 的 HRTIM 章节](https://www.st.com/resource/en/reference_manual/rm0440-stm32g4-series-advanced-armbased-32bit-mcus-stmicroelectronics.pdf)。

`Stop()` 后不允许直接再次 `StartCounters()` 或对同一已启动实例重新 `Init()`。日常暂停/恢复用 `DisableOutputs()` / `EnableOutputs()`，计数器和 ADC 触发保持运行。若确实需要重新开始计数，应先停 ADC、完整复位 HRTIM、重新运行 CubeMX 初始化并重新建立零值模块实例。这样避免沿用旧计数位置或未知计数方向；模块不依赖停表时无效的软件复位。

## 3. 差分 ADC 的配置与换算

一个 `ADC_Diff_Context` 管理 ADC1+ADC2 或 ADC3+ADC4。双模式通过主 ADC 的一个 DMA 通道，将 CDR 低/高 16 位写入同一个 32 位字；没有给 ADC2/4 单独开 DMA。

当前配置要求是 12 位、右对齐、每 ADC 一个规则差分通道、双规则同步、外部触发、循环 WORD DMA，无硬件偏移/过采样/增益补偿。两个主 ADC 使用相同 HRTIM_TRG1 下降沿；DMA IRQ 保持相同抢占优先级。模块会拒绝与换算前提不一致的配置。

| 通道 | CDR 来源 | `zero_offset_code` | `polarity` | `physical_gain` | `physical_offset` |
|---|---|---:|---:|---:|---:|
| 输出电压 | ADC12 低16位 | -3 | -1 | 12.8 | 0 |
| 主电感电流 | ADC12 高16位 | -8 | +1 | 2.400384 | 0 |
| 直流母线电压 | ADC34 低16位 | 16 | +1 | 24.18717 | 0 |
| 辅助电流 | ADC34 高16位 | -15 | +1 | 2.400384 | 0 |

参考电压沿用 3.0 V，完整换算为：

```c
signed_code    = (int32_t)(raw & 0x0FFFU) - 2048;
corrected_code = signed_code - zero_offset_code;
adc_voltage_v  = polarity * (corrected_code / 2048.0f) * reference_v;
value          = adc_voltage_v * physical_gain - physical_offset;
```

当前差分原始码是以 2048 为中点的无符号码，不能当成 12 位补码符号扩展。原 `main.c` 中母线零偏附近的旧注释与实际表达式不一致；本模块按**实际表达式**保留母线 16 个 ADC 码偏移，在除 2048 前扣除。

`Init()` 只检查和绑定；`Start()` 先对两个 ADC 分别差分校准，再调用主 ADC 的 `HAL_ADCEx_MultiModeStart_DMA()`。本地 HAL 会使能主从两个 ADC，不需要另行调用 `HAL_ADC_Start(slave)`。Start 的三个阶段任一失败会尝试回滚，清理失败进入 FAULT，不把失败伪装成已经停止。

## 4. 初始化调用

上下文必须是静态变量或先用 `{0}` 初始化。将下面声明放在应用文件中：

```c
#include "Unipolar Frequency-Doubling PWM.h"
#include "adc_diff.h"

static HRTIM_SPWM_Context pwm;
static ADC_Diff_Context adc12;
static ADC_Diff_Context adc34;
```

在各 `MX_*_Init()` 都完成后、HRTIM 尚未启动时调用：

```c
static HAL_StatusTypeDef SingleInverter_ModulesInit(void)
{
    HAL_StatusTypeDef status;
    HRTIM_SPWM_Config pwm_config;
    ADC_Diff_Config adc12_config = {
        &hadc1, &hadc2, 3.0f,
        {{-3, -1, 12.8f, 0.0f}, {-8, 1, 2.400384f, 0.0f}}
    };
    ADC_Diff_Config adc34_config = {
        &hadc3, &hadc4, 3.0f,
        {{16, 1, 24.18717f, 0.0f}, {-15, 1, 2.400384f, 0.0f}}
    };

    HRTIM_SPWM_DefaultConfig(&pwm_config);
    status = HRTIM_SPWM_Init(&pwm, &hhrtim1, &pwm_config);
    if (status != HAL_OK) return status;
    status = ADC_Diff_Init(&adc12, &adc12_config);
    if (status != HAL_OK) return status;
    status = ADC_Diff_Init(&adc34, &adc34_config);
    if (status != HAL_OK) return status;

    /* 在这里初始化控制器、正弦相位、JustFloat等应用状态。
     * 实际控制采样频率用pwm.update_hz，不能放到StartCounters之后才初始化。
     */

    status = ADC_Diff_Start(&adc12);
    if (status != HAL_OK) return status;
    status = ADC_Diff_Start(&adc34);
    if (status != HAL_OK)
    {
        (void)ADC_Diff_Stop(&adc12);
        return status;
    }
    status = HRTIM_SPWM_StartCounters(&pwm);
    if (status != HAL_OK)
    {
        (void)ADC_Diff_Stop(&adc34);
        (void)ADC_Diff_Stop(&adc12);
    }
    return status;
}
```

此时仅 MASTER/A/B 开始计数和触发 ADC，四路功率输出仍然关闭。清理失败可从相应实例的 `cleanup_status`、`state` 查看，初始化失败应进入应用故障处理，不继续开输出。

只需要电压、电流两路时，可只建立 ADC12 实例；当前工程还用 ADC34 采集母线，所以完整保留当前测量需要两实例。ADC34 在这里是测量通道，不代表启动 C/D 从机调制。

## 5. 中断转发与控制调用

模块不会自行定义 `HAL_ADC_ConvCpltCallback()`，应由应用的唯一 HAL 回调分发。下面展示当前四通道的配帧入口；`Control_OnFrame()` 是应用函数，在其中读取物理量并执行一次控制计算：

```c
static ADC_Diff_Snapshot frame12, frame34;
static uint8_t ready12, ready34;
static volatile uint8_t sampling_fault;

static void Control_OnFrame(const ADC_Diff_Snapshot *a12,
                            const ADC_Diff_Snapshot *a34);

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (sampling_fault != 0U) return;
    if (ADC_Diff_OnConversionComplete(&adc12, hadc) != 0U)
    {
        if (ready12 != 0U) goto sample_error;
        if (ADC_Diff_Read(&adc12, &frame12) != HAL_OK) goto sample_error;
        ready12 = 1U;
    }
    else if (ADC_Diff_OnConversionComplete(&adc34, hadc) != 0U)
    {
        if (ready34 != 0U) goto sample_error;
        if (ADC_Diff_Read(&adc34, &frame34) != HAL_OK) goto sample_error;
        ready34 = 1U;
    }
    else return;

    if ((ready12 == 0U) || (ready34 == 0U)) return;
    ready12 = ready34 = 0U;
    if (frame12.sequence != frame34.sequence) goto sample_error;
    Control_OnFrame(&frame12, &frame34);
    return;

sample_error:
    sampling_fault = 1U;
    HRTIM_SPWM_DisableOutputs(&pwm);
}

void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
    if ((ADC_Diff_OnError(&adc12, hadc) != 0U) ||
        (ADC_Diff_OnError(&adc34, hadc) != 0U))
    {
        sampling_fault = 1U;
        HRTIM_SPWM_DisableOutputs(&pwm);
    }
}
```

`Control_OnFrame()` 中，`a12->channel[0].value` 是输出 V，`[1].value` 是主电感 A；`a34->channel[0].value` 是母线 V，`[1].value` 是辅助电流 A。将这些值送给现有控制器，取得无量纲调制度 `m` 后调用：

```c
if (HRTIM_SetModulation(&pwm, m) != HAL_OK)
{
    HRTIM_SPWM_DisableOutputs(&pwm);
    /* 进入应用故障处理。 */
}
```

确认完整采样有效、母线满足原有最低值要求且无故障后，再调用 `HRTIM_SPWM_EnableOutputs(&pwm)`。该函数只开输出，**不会强制软件更新**；A/B 继续在共同谷底装载比较值，因此首个输出片段可能不是完整载波脉冲。若应用要求开启后的第一拍严格完整，需要另行设计与载波边界同步的栅极使能，不能把普通 OENR 写入当成边界同步启用。

普通低母线时，调用 `DisableOutputs()`、复位控制器并继续计数/采样，保留原工程的母线恢复后重新斜坡并开启输出行为。只有锁存故障或彻底停机时，才由主循环调用 HRTIM 的 `Stop()` 停触发，再停止两组 ADC；再次启动计数需要遵守前述完整外设复位要求。ISR 中不调用阻塞校准/停止 API。采样后调用 `JustFloat_Sample()`，主循环调用 `JustFloat_Process()`；不要在 ADC ISR 阻塞串口发送。

两组 ADC 内部各自硬件同步；同一 HRTIM 触发不等于四个 ADC 的采样孔径完全一致。上面的 ready/sequence 只对照已处理回调，单字循环 DMA 在长时间关中断时仍会覆盖，不能证明没有漏拍。以 20 kHz 为例，每拍只有 50 us，应保证两组回调和控制在下一拍前完成，并避免长临界区；若需要承受更长中断延迟，应改用有周期标记的缓冲方案。

## 6. 替换旧实现时

- 在 `main.c` 中删除原静态 `HRTIM_SetModulation()`、`HRTIM_SPWM_Init()`，再包含同名新接口，防止名称冲突和重复写寄存器。
- 删除旧 `ADC_Sampling_Init()`、原 DMA 字和原差分解码/换算分支，统一由新模块管理同一 ADC/DMA；不要让两套启动和回调同时运行。
- 单机应用移除旧 C/D 启动和从机调用，但保留所需 ADC34 母线测量；本次仅交付独立模块，没有替用户改造当前主从应用。
- 保留 CubeMX 的 DMA/ADC IRQ 到 HAL 处理入口。实际更改放在 USER CODE 区，并在重生成后复核配置。

## 7. 验证范围

`Tests/verify_hrtim_spwm_module.py` 验证调制公式、限幅、双比较值提交、MASTER/A/B 初始化与统一启动及生命周期约束；`Tests/verify_adc_diff_module.py` 验证全12位码范围、当前四路换算、失败回滚、回调与快照。`Tests/verify_module_target_build.py` 按当前 Keil 项目配置编译真实模块，结果在 `Tests/results`。

这些是源码对照、主机软件测试和 ARM 编译验证。尚未接入整个工程链接，也未完成示波器对 M/A/B 相位、上下管死区、首脉冲或 ADC 实际时序的验证。
