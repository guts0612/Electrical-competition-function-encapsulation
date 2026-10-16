#ifndef DCAC_KEY_CONTROL_H
#define DCAC_KEY_CONTROL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 松手事件：每次 KeyControl_GetEvent / Process 最多取一个，优先级 PE0、PB9、PE2。 */
#define KEYCONTROL_EVENT_NONE 0U
#define KEYCONTROL_EVENT_PE0  1U
#define KEYCONTROL_EVENT_PB9  2U
#define KEYCONTROL_EVENT_PE2  3U /* 按键三：预留应用动作。 */

/* 原始按下位图（未消抖）；PE0 / PB9 / PE2 均上拉、按下接地。 */
#define KEYCONTROL_STATE_PE0  0x01U
#define KEYCONTROL_STATE_PB9  0x02U
#define KEYCONTROL_STATE_PE2  0x04U

/* 绑定 main.c 中的控制状态；必须全部填写且所指对象长期有效。
 * stop_slave 可传工程现有的 static Slave_Stop，故不需要移除 static。
 * stop_slave(0U) 必须保留工程原关断动作，包括清零从机接入请求。
 */
typedef struct
{
    volatile uint8_t *master_closed_loop;
    volatile uint8_t *slave_enable_request;
    volatile uint32_t *slave_fault_flags;
    void (*stop_slave)(uint32_t fault);
} KeyControl_Bindings;

/* 在 MX_GPIO_Init() 之后、进入主循环之前调用一次。
 * 由现有 MX_GPIO_Init 配置 PE0 / PB9 / PE2 的上拉输入，本模块不修改 GPIO。
 * 重复调用会清空待处理事件，重新建立当前按键状态。
 * 启动时已按住的按键，需要先松开再重新按下才会产生事件。
 */
void KeyControl_Init(void);

/* 在 SysTick 中 HAL_IncTick() 之后每 1 ms 调用一次，替代原 Key_Loop。
 * 初始化前调用直接返回；按下和松开各稳定 20 ms，松手产生一次事件。
 * 长按不重复；三键事件按位保存，同一键未消费前的多次事件会合并，
 * 与原工程 Key() 一致。HAL_GetTick 的 uint32_t 回绕可正常处理。
 */
void KeyControl_Tick1ms(void);

/* 主循环调用：仅取事件，不执行控制动作。初始化/取事件保留原 PRIMASK。 */
uint8_t KeyControl_GetEvent(void);

/* 即时读取物理 GPIO，返回 KEYCONTROL_STATE_* 按下位图（未消抖）。 */
uint8_t KeyControl_GetState(void);

/* 主循环调用：取一个事件并执行原工程主循环中的完整按键动作。
 * PE0：切换主机开/闭环；切为开环时 stop_slave(0U)。
 * PB9：仅主机闭环时切换从机接入请求；已请求则 stop_slave(0U)，
 *      未请求且故障为零才置请求为 1；主机开环时忽略该事件，不排队。
 * PE2：预留按键三，不修改控制状态；调用方根据返回的 EVENT_PE2 添加动作。
 * 返回本次已消费的事件（包括因开环/故障而没有动作的 PB9）。
 * bindings 或任一字段为空时返回 NONE，且不消费事件。
 * 不要在同一主循环再调用 GetEvent / 原 Key() 消费同一套按键动作。
 */
uint8_t KeyControl_Process(const KeyControl_Bindings *bindings);

#ifdef __cplusplus
}
#endif

#endif /* DCAC_KEY_CONTROL_H */
