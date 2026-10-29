#ifndef KEY_H
#define KEY_H

#include "stm32g4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

#define KEY_NONE           0U
#define KEY_PE0            1U
#define KEY_PB9            2U
#define KEY_PE2            3U
#define KEY_DEBOUNCE_MS    20U

/* GPIO由CubeMX配置为上拉输入，按键另一端接GND。
 * 主函数在MX_GPIO_Init()之后调用一次，随后允许SysTick扫描。
 */
void Key_Init(void);

/* 在SysTick中每1ms调用一次，不使用延时等待。
 * 三键独立消抖：按下、松开均连续确认20次，松手产生一次事件。
 * 长按不连发；上电时已按住的键，确认按下后松开也会产生事件。
 */
void Key_Loop(void);

/* 主循环读取并消费一个松手事件：0无事件，1为PE0，2为PB9，3为PE2。
 * 多键待处理时按1、2、3顺序返回；同一键未读取前的重复事件合并。
 */
uint8_t Key(void);

/* 可选：读取原始按下状态，不消抖，不消费事件。
 * 返回键码0~3；同时按下时按PE0、PB9、PE2顺序选择。
 */
uint8_t Key_GetState(void);

#ifdef __cplusplus
}
#endif

#endif
