# Electrical-competition-function-encapsulation

面向电子设计竞赛、STM32 数字电源与逆变器项目的 C 语言功能模块集合。

将 QPR 控制、VOFA+ JustFloat 串口波形发送、按键、单机倍频 SPWM 和差分 ADC 采样封装为函数接口，方便在后续工程中按需接入。

当前外设模块面向 **STM32G474 + STM32G4 HAL**。仓库收录模块源码及调用说明；接入时需要由目标工程提供启动文件、HAL/CMSIS、外设初始化和应用控制逻辑。

## 模块一览

| 模块 | 文件 | 主要功能 |
| --- | --- | --- |
| QPR 控制 | [qpr_control.h](qpr_control.h) / [qpr_control.c](qpr_control.c) | 虚拟阻抗电压外环、QPR 电流内环、开环调制及独立电流环 |
| JustFloat 通信 | [justfloat.h](justfloat.h) / [justfloat.c](justfloat.c) | 八通道波形发送、抽点、最新帧邮箱、电压总 RMS 和发送统计 |
| 按键 | [key_control.h](key_control.h) / [key_control.c](key_control.c) | PE0 / PB9 / PE2 消抖、松手事件及原工程控制动作 |
| 单极倍频 SPWM | [Unipolar Frequency-Doubling PWM.h](<Unipolar Frequency-Doubling PWM.h>) / [Unipolar Frequency-Doubling PWM.c](<Unipolar Frequency-Doubling PWM.c>) | HRTIM Master、Timer A/B 同步启动、调制度更新与输出管理 |
| 差分 ADC | [adc_diff.h](adc_diff.h) / [adc_diff.c](adc_diff.c) | 双 ADC 同步差分采样、校准、DMA、物理量换算与快照读取 |

各模块按需选用。JustFloat 接收调用方传入的物理量，能够配合其他控制器使用；QPR 本身也不访问 ADC、PWM 或 UART。

## 环境与依赖

| 模块 | 依赖与移植位置 |
| --- | --- |
| `qpr_control` | 标准 C 头文件；控制参数集中在 `qpr_control.h` |
| `justfloat` | STM32G4 HAL UART、CMSIS 内核接口，以及提供 `arm_math.h` / `arm_sqrt_f32` 的 CMSIS-DSP 配置 |
| `key_control` | 目标工程的 `main.h`、HAL GPIO / Tick 和 CMSIS 内核接口；引脚读取集中在 `KeyControl_GetState()` |
| 单极倍频 SPWM | STM32G4 HAL HRTIM 及对应寄存器定义 |
| `adc_diff` | STM32G4 HAL ADC / DMA 及对应寄存器定义 |

CubeMX 继续负责时钟、GPIO、DMA、NVIC、HAL 句柄和 `MX_*_Init()` 等基础初始化。更换芯片或功率板时，需要核对外设能力、触发路径、引脚、死区和反馈标定。

## 接入步骤

1. 将需要的 `.c` 文件加入目标工程，将配套 `.h` 所在目录加入头文件搜索路径。同名旧模块应同步替换，避免新旧实现同时管理同一外设。
2. 完成 CubeMX 外设配置，保留 HAL、CMSIS 和所需 DSP 的包含目录与链接配置。
3. 将模块上下文声明为静态变量，或在首次使用前使用 `{0}` 初始化；保证上下文及绑定对象的生命周期覆盖整个运行过程。
4. 调用模块初始化接口，并检查返回值。控制采样频率应使用实际控制调用频率。
5. 在启动采样触发之前，完成 ADC/DMA、控制器和遥测状态准备；HRTIM 计数器启动后，等待采样有效且运行条件满足，再使能功率输出。
6. 将周期采样、控制计算、按键扫描和串口发送放到对应的调用位置。

### 调用位置

| 位置 | 典型调用 |
| --- | --- |
| 初始化阶段 | 各模块 `Init`，ADC 启动，HRTIM 计数器启动 |
| SysTick 每 1 ms，`HAL_IncTick()` 之后 | `KeyControl_Tick1ms()` |
| `HAL_ADC_ConvCpltCallback()` | 转发 `ADC_Diff_OnConversionComplete()`，读取有效快照 |
| 同一控制拍的数据配齐后 | QPR 计算 → `HRTIM_SetModulation()` → `JustFloat_Sample()` |
| 主循环 | 按键事件处理、`JustFloat_Process()`、故障恢复及硬件停止流程 |
| `HAL_ADC_ErrorCallback()` | 转发 `ADC_Diff_OnError()`，由应用立即禁止功率输出 |

应用统一实现 HAL 全局回调，再向相应模块转发。同一控制拍只执行一次控制与遥测采样；使用两对 ADC 时，由应用负责配帧。

## QPR 控制

主要接口：

| 函数 | 用途 |
| --- | --- |
| `QPR_ControlInit()` | 根据实际控制采样频率初始化系数，成功返回 `1` |
| `QPR_ControlReset()` | 保留系数，清除动态状态、参考斜坡、统计和运行故障 |
| `QPR_ControlStep()` | 虚拟阻抗电压外环与 QPR 电流内环，返回调制度 |
| `QPR_ControlOpenLoopStep()` | 带参考幅值斜坡的开环正弦调制 |
| `QPR_CurrentStep()` | 独立 QPR 电流环，接收电流参考及电压前馈 |

主机控制链路：

```text
参考电压 - 输出电压反馈
    → 慢速直流误差补偿 → 虚拟阻抗 → 电流参考
    → 减电感电流反馈 → QPR → 校正电压限幅
    → 加参考电压前馈 → 除以实测母线电压 → 调制度限幅
```

调用方负责生成单位正弦及其相位，并传入参考电压的**峰值**、标定后的电压/电流反馈和实际母线电压。各控制实例应使用独立的 `QPR_Control` 状态。

默认参数为 `50 Hz`、`Kp = 1.3`、`Kr = 10`、`ωc = 5 rad/s`，校正限幅 `±5 V`，调制度限幅 `±0.95`。这些参数来自原工程，应结合目标功率级、反馈比例和采样频率调整。

几个容易混淆的观测量：

| 字段 | 含义 |
| --- | --- |
| `qpr_unlimited_v` | 限幅前的控制器校正电压 |
| `qpr_correction_v` | 限幅后的控制器校正电压，当前 JustFloat CH5 的数据来源 |
| `bridge_reference_v` | 校正电压加参考电压前馈后的桥电压指令 |
| `modulation` | 最终限幅后的无量纲调制度 |

当前版本保留原主机控制模型，并修复独立电流环因浮点除乘残差导致谐振状态误冻结的问题。故障标志和调制度归零由算法提供，实际禁止 PWM 输出、保护与恢复由应用负责。

## JustFloat 串口波形

每帧包含 **8 个 32 位 `float` + 帧尾 `00 00 80 7F`**，共 **36 字节**，用于 VOFA+ JustFloat 波形显示。

| 通道 | `JustFloat_SampleData` 字段 | 含义 |
| --- | --- | --- |
| CH1 | `reference_v` | 参考电压，V |
| CH2 | `voltage_v` | 输出电压，V |
| CH3 | `current_reference_a` | 电流参考，A |
| CH4 | `current_a` | 电感电流，A |
| CH5 | `controller_limited_output_v` | 控制器限幅后输出，V |
| CH6 | `auxiliary_current_a` | 辅助电流，A |
| CH7 | `dc_bus_v` | 母线电压，V |
| CH8 | 模块计算 | 输出电压总 RMS，V |

CH5 由调用方提供，JustFloat 只发送该值。在本仓库 QPR 用法中，应传入 `qpr_correction_v`；更换控制器后，传入新控制器限幅后的输出即可。

推荐流程为 `JustFloat_Init()` → 每控制拍调用 `JustFloat_Sample()` → 主循环反复调用 `JustFloat_Process()`。

- 默认每 `10` 拍发布一帧；`20 kHz` 控制采样对应目标发布速率 `2000` 帧/秒，实际发送速率取决于串口和主循环负载。
- RMS 使用全部控制采样，窗口点数为 `round(采样频率 / 基波频率)`，例如 `20 kHz / 50 Hz = 400` 点。
- 当前提供波形模式；CH8 保留包含直流分量的总 RMS。开闭环切换时重新建立 RMS 窗口，完整窗口形成前可通过 `output_voltage_rms_valid` 判断有效性。
- 串口由调用方初始化并传入句柄；原工程参考配置为 UART4、`921600 baud`、`8N1`。
- 单槽邮箱保留最新帧，来不及发送时覆盖旧帧，并记录 `overwrite_count`。

需要直接发送自定义八通道数据时，也可以调用 `JustFloat_Send8()`，此入口直接发送传入值，不执行抽点或 RMS 计算。

**`JustFloat_Process()` 和 `JustFloat_Send8()` 使用阻塞式 HAL 发送，应在主循环调用。同一 UART 应统一管理发送，避免多个发送者并发使用。**

默认宏 `JUSTFLOAT_SAMPLE_DIV` 和 `JUSTFLOAT_UART_TIMEOUT_MS` 可以在头文件中修改，或在整个工程的预处理配置中覆盖；调用方与 `justfloat.c` 必须使用一致配置。仅在 `main()` 内定义宏不会改变模块源文件的编译配置。

## 按键

当前引脚均使用上拉输入，按下接地。按下和松开分别稳定 `20 ms` 后确认，**松手产生一次事件**，长按不重复。

| 按键 | 事件 | `KeyControl_Process()` 默认动作 |
| --- | --- | --- |
| PE0 | `KEYCONTROL_EVENT_PE0` | 切换主机开闭环；切至开环时调用绑定的从机关断函数 |
| PB9 | `KEYCONTROL_EVENT_PB9` | 在主机闭环时切换从机接入请求，并检查故障状态 |
| PE2 | `KEYCONTROL_EVENT_PE2` | 返回按键三事件，预留给应用自定义动作 |

通用事件用法：

```c
/* GPIO 初始化完成后调用一次。 */
KeyControl_Init();

/* 每 1 ms，在 HAL_IncTick() 之后调用。 */
KeyControl_Tick1ms();

/* 主循环中读取事件，由应用定义动作。 */
uint8_t event = KeyControl_GetEvent();
if (event == KEYCONTROL_EVENT_PE2)
{
    /* 按键三的处理逻辑。 */
}
```

以上片段分别放在对应调用位置。需要沿用原工程的主从控制行为时，通过 `KeyControl_Bindings` 绑定状态变量及停止函数，改用 `KeyControl_Process()`；同一套按键动作选择一个事件消费入口。

启动时已按住的按键需要先松开再重新按下。待处理事件按 PE0 → PB9 → PE2 的顺序返回，同一键尚未消费的重复事件会合并。

## 单极倍频 SPWM

`Unipolar Frequency-Doubling PWM.c/.h` 只管理 HRTIM **Master、Timer A/B 和 TA1/TA2/TB1/TB2**，提供一个调制度入口：

```c
HAL_StatusTypeDef HRTIM_SetModulation(HRTIM_SPWM_Context *context,
                                     float modulation);
```

调制关系为 `dA = (1 + m) / 2`、`dB = (1 - m) / 2`。两路比较值成对更新，在共同载波谷底生效。这里的 `m` 是当前控制拍带正负号的瞬时调制指令；如果开环输出正弦波，应在模块外计算 `m = M * sin(theta)`，再传入本接口。

| 对外函数 | 作用 |
| --- | --- |
| `HRTIM_SPWM_DefaultConfig` | 填入默认配置；不访问硬件。默认参数集中在头文件，可在调用后修改配置字段。 |
| `HRTIM_SPWM_Init` | 配置Master/A/B时基、互补输出、死区、预装载和ADC触发；保持计数停止、输出关闭。 |
| `HRTIM_SPWM_StartCounters` | 一次寄存器写入同步启动Master/A/B；开始产生ADC触发，仍保持输出关闭。 |
| `HRTIM_SetModulation` | 限幅m，计算两桥臂占空比，四舍五入并保护比较值端点，成对写入预装载。 |
| `HRTIM_SPWM_EnableOutputs` | 计数已运行时打开TA1/TA2/TB1/TB2输出。 |
| `HRTIM_SPWM_DisableOutputs` | 关闭四路输出，保持计数和ADC触发，便于暂停后恢复。 |
| `HRTIM_SPWM_Stop` | 先关闭输出，再停止Master/A/B；停止后不能直接重新启动。 |

初始化配置与启动分开，便于先准备ADC/DMA和控制器，再运行采样。计数启动与输出使能分开，便于先确认采样有效和运行条件，再打开功率输出。正常控制循环只需调用 `HRTIM_SetModulation()`。

实现中的 `SPWM_IsReady`、`SPWM_CanStart`、`SPWM_Clamp`、`SPWM_DutyToCompare` 和 `SPWM_ConfigureHardware` 都是内部辅助函数，应用无需调用；分别集中处理就绪检查、启动条件、限幅、占空比换算和底层配置。`SPWM_TRY` 是初始化阶段的HAL错误返回宏。

典型顺序：

```text
CubeMX 基础初始化
    → HRTIM_SPWM_DefaultConfig / HRTIM_SPWM_Init
    → 完成控制器、遥测及 ADC/DMA 准备
    → HRTIM_SPWM_StartCounters
    → 采样有效且运行条件满足
    → HRTIM_SPWM_EnableOutputs
```

- 默认 HRTIM 内核时钟 `170 MHz`、A/B 载波峰值 `4250 tick`，完整载波及控制更新频率为 `20 kHz`，调制度限制为 `±0.95`。
- 默认死区沿用原硬件：A 上升/下降为 `0/25 tick`，B 为 `0/0 tick`，迁移到其他功率板时必须据实际驱动调整。
- 同步启动要求完整 HRTIM 外设复位后的首次启动，M/A/B 停止且计数为零。每个上下文仅启动一次；调用 `Stop()` 后，重新运行需要完整外设复位、MX 初始化和重新清零的上下文。
- 日常暂停与恢复使用 `HRTIM_SPWM_DisableOutputs()` / `HRTIM_SPWM_EnableOutputs()`，保持计数器运行。
- `EnableOutputs()` 不保证首个输出片段恰好是完整载波脉冲；有此要求时，需要在应用中设计输出使能时序。

## 差分 ADC 采样

每个 `ADC_Diff_Context` 管理 **ADC1+ADC2** 或 **ADC3+ADC4** 一对 ADC。使用两对 ADC 时创建两个独立实例。

CubeMX 需要配置为：12 位右对齐、每个 ADC 一个规则差分通道、双规则同步、外部触发，以及单字循环 WORD DMA；当前换算模型要求关闭硬件偏移、过采样和增益补偿。

双模式 DMA 数据低 16 位属于主 ADC，高 16 位属于从 ADC。每路按以下顺序换算：

```text
raw             = 对应半字的低 12 位
signed_code     = raw - 2048
corrected_code  = signed_code - zero_offset_code
adc_voltage_v   = polarity × corrected_code / 2048 × reference_v
value           = adc_voltage_v × physical_gain - physical_offset
```

`zero_offset_code` 的单位是 ADC 码；`physical_offset` 的单位是换算后的物理量。原码采用中点偏置格式，应按 `raw - 2048` 解码。

- `ADC_Diff_Init()` 检查配置并绑定实例；`ADC_Diff_Start()` 执行差分校准并启动双模式 DMA。
- 在应用的 HAL 完成/错误回调中转发 `ADC_Diff_OnConversionComplete()` / `ADC_Diff_OnError()`。
- `ADC_Diff_Read()` 返回同一次发布的完整快照；首帧前、停止或故障后，快照不可作为有效采样使用。
- `Start()` / `Stop()` 放在初始化或主循环，调用前停止外部触发；错误回调负责记录故障，应用负责立即禁止功率输出并完成停止流程。
- 单字循环 DMA 保存最新结果；中断延迟可能造成覆盖。两实例之间不自动配帧，同一对 ADC 的硬件同步也不等同于四路 ADC 具有完全相同的采样时刻。

## 详细说明与验证范围

| 文档 | 内容 |
| --- | --- |
| [PR_QPR算法对比说明.md](PR_QPR算法对比说明.md) | 算法比较、保留原模型的原因及独立电流环修订 |
| [JustFloat与按键模块调用说明.md](JustFloat与按键模块调用说明.md) | 完整调用位置、参数配置、八通道示例和按键绑定 |
| [HRTIM与差分ADC模块调用说明.md](HRTIM与差分ADC模块调用说明.md) | 外设配置、初始化顺序、两对 ADC 配帧及停止流程 |

说明文档记录了模块在来源工程中的软件对照和目标编译结果，其中的 `Core/`、`MDK-ARM/`、`Tests/` 路径属于来源工程背景。当前仓库收录模块及说明文件，未包含这些目录、完整 CubeMX/Keil 工程或验证脚本。

接入目标工程后，需要完成整个工程的编译链接，并验证采样极性与比例、PWM 同步及死区、中断耗时、串口吞吐和控制效果。来源工程的软件验证记录不能替代目标硬件上的验证。
