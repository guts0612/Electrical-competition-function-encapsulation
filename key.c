#include "key.h"

typedef struct
{
    uint8_t pressed;             /* 消抖后状态：0松开，1按下。 */
    uint8_t count;               /* 与已确认状态不同的连续采样次数。 */
} Key_State;

static GPIO_TypeDef * const key_ports[3] = {GPIOE, GPIOB, GPIOE};
static const uint16_t key_pins[3] = {GPIO_PIN_0, GPIO_PIN_9, GPIO_PIN_2};
static Key_State key_states[3];
static volatile uint8_t key_events;
static volatile uint8_t key_ready;

void Key_Init(void)
{
    uint32_t primask = __get_PRIMASK();
    uint8_t i;

    __disable_irq();
    for (i = 0; i < 3; i++)
    {
        key_states[i].pressed = 0;
        key_states[i].count = 0;
    }
    key_events = 0;
    key_ready = 1;
    __set_PRIMASK(primask);
}

uint8_t Key(void)
{
    uint32_t primask = __get_PRIMASK();
    uint8_t event = KEY_NONE;
    uint8_t i;

    /* 读取并清除一个事件，避免被SysTick打断后误清新事件。 */
    __disable_irq();
    for (i = 0; i < 3; i++)
    {
        uint8_t mask = (uint8_t)(1U << i);

        if ((key_events & mask) != 0U)
        {
            key_events &= (uint8_t)~mask;
            event = (uint8_t)(i + 1U);
            break;
        }
    }
    __set_PRIMASK(primask);
    return event;
}

uint8_t Key_GetState(void)
{
    uint8_t i;

    for (i = 0; i < 3; i++)
    {
        if (HAL_GPIO_ReadPin(key_ports[i], key_pins[i]) == GPIO_PIN_RESET)
        {
            return (uint8_t)(i + 1U);
        }
    }
    return KEY_NONE;
}

void Key_Loop(void)
{
    uint8_t i;

    /* SysTick在GPIO初始化前就已运行，此时不读取引脚。 */
    if (key_ready == 0U)
    {
        return;
    }

    for (i = 0; i < 3; i++)
    {
        uint8_t pressed;

        pressed = (HAL_GPIO_ReadPin(key_ports[i], key_pins[i]) == GPIO_PIN_RESET);
        if (pressed == key_states[i].pressed)
        {
            key_states[i].count = 0;
        }
        else
        {
            key_states[i].count++;
            if (key_states[i].count >= KEY_DEBOUNCE_MS)
            {
                key_states[i].count = 0;
                key_states[i].pressed = pressed;
                if (pressed == 0U)
                {
                    /* 松开后发布事件；三键分别保留，不相互覆盖。 */
                    key_events |= (uint8_t)(1U << i);
                }
            }
        }
    }
}
