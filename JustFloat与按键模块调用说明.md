JustFloat 与按键模块调用说明（仅波形版）

已将当前工程的串口遥测和按键功能提取成根目录的独立模块：

| 文件 | 功能 |
|---|---|
| justfloat.c / justfloat.h | 八通道波形打包、发送、邮箱、抽点、RMS 与统计 |
| key_control.c / key_control.h | PE0/PB9/PE2 去抖、松手事件；前两键控制主从状态，PE2 预留动作 |

当前 `Core/Src/main.c` 的波形 CH5 已改为 `qpr_control.qpr_correction_v`（限幅后 QPR 输出），独立模块尚未接入当前固件。JustFloat 仅保留本工程的波形发送模式，前两键行为沿用原工程，新增 PE2 作为按键三预留。下面给出调用位置和可直接使用的代码。模块不修改 ADC、PWM、QPR 或外设初始化；继续使用现有的 UART4、GPIO 配置。

**保持的串口行为**

- UART4 继续由 `MX_UART4_Init()` 配置：921600 baud、8 数据位、无校验、1 停止位。
- 每帧八个 32 位 float，随后是 `00 00 80 7F`，共 36 字节。
- 默认每 10 个控制采样发布一帧；20 kHz 控制率对应目标发布速率 2000 帧/s。实际成功发送数由主循环负载决定，可通过计数观察。
- ADC 回调仅准备数据，主循环调用 HAL 阻塞发送；超时 5 ms。失败计数后丢帧，不阻塞重试。
- 保留单槽邮箱、最新帧覆盖旧帧及覆盖计数。发送时允许 ADC 中断继续发布新帧；取八通道快照时保留调用前的 PRIMASK。
- 使用全部采样计算电压总 RMS；20 kHz / 50 Hz 为 400 点窗口。主机开闭环切换时丢弃未完成的旧 RMS 窗口，清零有效标记，完成新窗口后再有效。

| 通道 | 波形模式 |
|---|---|
| CH1 | 参考电压，V |
| CH2 | 输出电压，V |
| CH3 | 电流参考，A |
| CH4 | 电感电流，A |
| CH5 | 控制器限幅后输出，V |
| CH6 | 辅助电流，A |
| CH7 | 母线电压，V |
| CH8 | 输出电压总 RMS，V |

CH1~7 为抽取时刻的瞬时值，CH8 为最近完成窗口的总 RMS，包含直流分量。均值输出模式及其宏、累加器、计数器已删除；同时删除未用于波形发送的电压均值、去直流 AC_RMS 诊断量。计算总 RMS 必需的平方累加与窗口归一化仍保留。

CH5 通过通用字段 `controller_limited_output_v` 输入，JustFloat 只发送该值，不执行控制器限幅。当前工程应传入 `qpr_control.qpr_correction_v`，这是限幅后的 QPR 校正量；`bridge_reference_v` 已加上参考电压前馈，不作为 CH5。更换控制器时，改为传入新控制器的限幅后输出，模块无需引用 QPR 类型。

**按键行为与 PE2 预留**

- PE0、PB9、PE2 均上拉、低电平按下；GPIO 由 `MX_GPIO_Init()` 配置。当前 `gpio.c` 与 `.ioc` 已将 PE2 配为上拉输入，按键三接在 PE2 与 GND 之间。
- SysTick 每 1 ms 扫描，按下和松开分别稳定 20 ms，松手触发一次事件。
- 长按不重复；上电时已经按住的键，必须先松开再重新按下。
- 同时松开三键会保存三个事件，依次处理 PE0、PB9、PE2；同一键未消费前的重复事件会合并，保留原实现语义。
- PE0 切换主机开闭环；切到开环时调用原 `Slave_Stop(0U)`。
- PB9 只在主机闭环时切换从机接入请求。故障锁存时不重新接入；主机开环时消费并忽略 PB9，不延迟排队。
- PE2 为按键三：原始按下位图是 `KEYCONTROL_STATE_PE2`（`0x04`），消抖后的松手事件是 `KEYCONTROL_EVENT_PE2`（`3`）。`KeyControl_Process()` 只返回该事件，不修改控制状态；具体动作由调用方填写，`KeyControl_Bindings` 不需要增加字段。

**接入当前 Keil 工程**

1. 添加根目录 `justfloat.c`、`key_control.c` 到工程，包含目录增加项目根目录。相对于 `MDK-ARM`，根目录是 `..`，应追加到现有包含路径末尾，保持 `../Core/Inc` 在前，避免原工程意外选用根目录已有的同名 QPR 候选头文件。保留现有 HAL、CMSIS 和 DSP 包含目录；只添加这两个新 C 文件。

2. `main.c` 中将 `#include "key.h"` 改为以下两行：

```c
#include "key_control.h"
#include "justfloat.h"
```

在原主从状态变量声明之后增加以下全局变量与绑定。保留原来的 `Slave_Stop` 函数定义，前置声明仍使用 `static`：

```c
static JustFloat_Context justfloat;
static void Slave_Stop(uint32_t fault);

static const KeyControl_Bindings project_keys = {
    &voltage_closed_loop_enable,
    &slave_enable_request,
    &slave_fault_flags,
    Slave_Stop
};
```

`JustFloat_Context` 必须使用静态存储或先清零，不能传入未初始化的栈对象。

3. 在 `HRTIM_SPWM_Init()` 内，计算出 `spwm_update_hz` 后、启动 HRTIM 计数器前，替换原 RMS 窗口初始化代码：

```c
#if VOFA_OUTPUT_ENABLE
    if (JustFloat_Init(&justfloat, &huart4,
                       spwm_update_hz, SPWM_OUTPUT_FREQUENCY_HZ) == 0U)
    {
        Error_Handler();
    }
#endif
```

当前工程可直接放在原 `#if VOFA_OUTPUT_ENABLE` 的 RMS 窗口初始化位置。此时 UART4 已初始化，采样频率已知，HRTIM 仍未启动。**不要等 `HRTIM_SPWM_Init()` 返回后才初始化 JustFloat：该函数末尾已经启动采样，ADC 回调可能先执行。**

把 `main()` 中原 `Key_Init()` 替换为：

```c
    KeyControl_Init();
```

此调用继续位于 `MX_GPIO_Init()` 之后、进入主循环之前。模块初始化前的扫描会直接返回，不会提前读取未配置的 GPIO。

4. 在 `stm32g4xx_it.c` 中将头文件改为 `key_control.h`，并将 SysTick 内原 `Key_Loop()` 替换为：

```c
void SysTick_Handler(void)
{
    HAL_IncTick();
    KeyControl_Tick1ms();
}
```

实际编辑时保留 CubeMX 的 USER CODE 标记，把 `KeyControl_Tick1ms()` 放在原 `Key_Loop()` 的位置；不需要增加第二个中断处理函数。

5. 主循环用下面两个调用替换原 `Key()` 及 PE0/PB9 的动作判断、原 `VOFA_Process()`：

```c
    uint8_t key_event = KeyControl_Process(&project_keys);
    if (key_event == KEYCONTROL_EVENT_PE2)
    {
        /* 按键三 PE2：在此填写后续动作。 */
    }

#if VOFA_OUTPUT_ENABLE
    JustFloat_Process(&justfloat);
#endif
```

顺序保持“先按键、后发送”。`KeyControl_Process` 已包含原 PE0/PB9 主循环动作，不要再重复执行原按键切换分支。PE2 的应用动作填在上述预留位置即可，不要再次调用 `GetEvent()` 取同一个事件。

6. 在 `HAL_ADC_ConvCpltCallback()` 中，保留原来的 ADC 两组配对、主机控制和 `Slave_ControlUpdate()`。在它们完成之后，用下段替换原遥测尾部：

```c
#if VOFA_OUTPUT_ENABLE
    {
        JustFloat_SampleData sample;

        sample.reference_v = voltage_reference_v;
        sample.voltage_v = true_voltage;
        sample.current_reference_a = qpr_control.current_reference_a;
        sample.current_a = true_current;
        sample.controller_limited_output_v = qpr_control.qpr_correction_v;
        sample.auxiliary_current_a = true_aux_current;
        sample.dc_bus_v = dc_bus_voltage_v;

        JustFloat_Sample(&justfloat, &sample, closed_loop);
    }
#endif
```

这里的 `closed_loop` 沿用当前回调已经保存的本拍模式快照。必须在两组 ADC 配齐后每控制拍调用一次，不能在 ADC1、ADC3 各完成时都调用，也不能先抽点再调用，否则 RMS 窗口将改变。

`JustFloat_SampleData` 已删除仅供均值模式使用的 `resonant_v`、`qpr_unlimited_v`，原调用代码若给这两个字段赋值，应同步删除对应两行。`JustFloat_Context` 的均值和 AC_RMS 字段也已删除，窗口归一化系数现名为 `rms_scale`。

CH5 的原字段 `bridge_reference_v` 已更名为 `controller_limited_output_v`，已有调用应按上例同时修改字段名和数据来源。

接入后删除原 `VOFA_AccumulateVoltageRms`、`VOFA_AccumulateMean`、`VOFA_SendJustFloat`、`VOFA_Process` 四个静态函数，以及它们专用的通道、邮箱、累加器和发送计数全局变量。原功能已由 `JustFloat_Context` 接管，保留新旧两套并同时调用会重复计算或发送。

原 `VOFA_SAMPLE_DIV`、`VOFA_UART_TIMEOUT_MS` 分别由新头文件中的 `JUSTFLOAT_SAMPLE_DIV`、`JUSTFLOAT_UART_TIMEOUT_MS` 接管。删除 main.c 中这两个旧定义、`VOFA_MEAN_OUTPUT_ENABLE`、`VOFA_MEAN_SAMPLE_COUNT` 及原 `#if VOFA_SAMPLE_DIV < 1U` 检查，新头文件已完成抽点参数检查。新模块不再提供 `JUSTFLOAT_MEAN_OUTPUT_ENABLE`、`JUSTFLOAT_MEAN_SAMPLE_COUNT`，已设置的旧均值配置应删除。`VOFA_OUTPUT_ENABLE` 继续在 main.c 中控制整项功能。

**单独调用函数**

如果只需要直接发送八个浮点通道，在初始化后从主循环调用：

```c
HAL_StatusTypeDef status;

status = JustFloat_Send8(&justfloat,
                         ch1, ch2, ch3, ch4,
                         ch5, ch6, ch7, ch8);
```

该函数只发送传入的八个值、更新成功/失败计数，不进行抽点或 RMS 处理，也不消费采样邮箱。沿用当前项目的波形遥测流程，应使用前述 `Sample` + `Process`。同一串口的发送集中在主循环，不要从 ADC 中断调用 `Send8`，也不要让多个发送者并发操作同一实例。

如果只需要获取按键事件、自己处理动作：

```c
uint8_t event = KeyControl_GetEvent();
/* 0：无事件；1：PE0 松手；2：PB9 松手；3：PE2（按键三）松手。 */
```

`GetEvent` 和 `Process` 都会消费一个事件；同一套应用动作选择其中一种入口。

**配置和观察位置**

默认配置已经与当前工程一致：

```c
#define JUSTFLOAT_SAMPLE_DIV          10U
#define JUSTFLOAT_UART_TIMEOUT_MS      5U
```

可在 `justfloat.h` 修改默认值，或在整个 Keil 工程的预处理宏中覆盖。不能只在 main.c 的头文件之后局部定义这些宏，否则不会改变单独编译的 `justfloat.c`。

接入后的 Keil Watch 路径：

| 原观察量 | 模块观察位置 |
|---|---|
| vofa_ch1 ~ vofa_ch8 | justfloat.channels[0] ~ [7] |
| vofa_frame_ready | justfloat.frame_ready |
| adc_sample_div_count | justfloat.sample_div_count |
| vofa_sent_count / vofa_tx_error_count | justfloat.sent_count / tx_error_count |
| vofa_overwrite_count | justfloat.overwrite_count |
| output_voltage_rms_v | justfloat.output_voltage_rms_v |
| output_voltage_rms_valid / window_count / window_samples | justfloat 中的同名完整字段 |

主机模式、从机请求及故障标志继续使用原 `voltage_closed_loop_enable`、`slave_enable_request`、`slave_fault_flags`。关断动作通过原 `Slave_Stop` 执行，未把停止栅极简化成调制度清零。

**完成的验证**

- JustFloat：真实编译新模块，与自动抽取的现 main.c 波形遥测代码逐拍比较。7500 拍保留状态和发送字节一致；另检查帧尾、HAL 四种发送结果、覆盖计数、发送期间到达的新采样、PRIMASK 恢复、模式切换及含直流/谐波信号的 RMS。
- 按键：真实编译新旧模块，对照原 PE0/PB9 的 81 组控制输入状态及 12000 段随机输入；另验证 PE2 的短脉冲/抖动、长按、松手阈值、上电按住、tick 回绕、重新初始化、三键事件优先级与事件合并。确认 PE2 经 `Process` 返回时不切换主机状态、不接入或停止从机。前两键动作和关断副作用对照来自当前 main.c。
- ARM：按真实 `dcac.uvprojx` 的包含目录、宏及 C99 配置，用 ARM Compiler 5.06 update 7 编译 `main.c`、JustFloat 波形和按键模块。临时目标文件已清理。

验证脚本为 `Tests/verify_justfloat_module.py`、`Tests/verify_key_control_module.py`、`Tests/verify_module_target_build.py`；结果保存在 `Tests/results` 对应 JSON 文件。以上证明软件行为对照与模块目标编译通过，尚未进行接入后的整个工程链接、物理串口、按键电气或中断时序实测。
