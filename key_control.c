#include "key_control.h"
#include "main.h"

#define KEYCONTROL_DEBOUNCE_MS 20U
#define KEYCONTROL_KEY_COUNT   3U

static volatile uint8_t keycontrol_pending;
static volatile uint8_t keycontrol_ready;
static uint8_t keycontrol_stable[KEYCONTROL_KEY_COUNT];
static uint8_t keycontrol_candidate[KEYCONTROL_KEY_COUNT];
static uint8_t keycontrol_confirmed[KEYCONTROL_KEY_COUNT];
static uint32_t keycontrol_change_tick[KEYCONTROL_KEY_COUNT];

void KeyControl_Init(void)
{
    uint32_t primask = __get_PRIMASK();
    uint8_t i, state;

    __disable_irq();
    state = KeyControl_GetState();
    for (i = 0U; i < KEYCONTROL_KEY_COUNT; ++i)
    {
        keycontrol_stable[i] = (state >> i) & 1U;
        keycontrol_candidate[i] = keycontrol_stable[i];
        keycontrol_confirmed[i] = 0U;
        keycontrol_change_tick[i] = HAL_GetTick();
    }
    keycontrol_pending = 0U;
    keycontrol_ready = 1U;
    __set_PRIMASK(primask);
}

uint8_t KeyControl_GetEvent(void)
{
    uint8_t event = KEYCONTROL_EVENT_NONE;
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    if ((keycontrol_pending & KEYCONTROL_STATE_PE0) != 0U)
    {
        event = KEYCONTROL_EVENT_PE0;
        keycontrol_pending &= (uint8_t)~KEYCONTROL_STATE_PE0;
    }
    else if ((keycontrol_pending & KEYCONTROL_STATE_PB9) != 0U)
    {
        event = KEYCONTROL_EVENT_PB9;
        keycontrol_pending &= (uint8_t)~KEYCONTROL_STATE_PB9;
    }
    else if ((keycontrol_pending & KEYCONTROL_STATE_PE2) != 0U)
    {
        event = KEYCONTROL_EVENT_PE2;
        keycontrol_pending &= (uint8_t)~KEYCONTROL_STATE_PE2;
    }
    __set_PRIMASK(primask);
    return event;
}

uint8_t KeyControl_GetState(void)
{
    uint8_t state = 0U;

    if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_0) == GPIO_PIN_RESET)
        state |= KEYCONTROL_STATE_PE0;
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) == GPIO_PIN_RESET)
        state |= KEYCONTROL_STATE_PB9;
    if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_2) == GPIO_PIN_RESET)
        state |= KEYCONTROL_STATE_PE2;
    return state;
}

void KeyControl_Tick1ms(void)
{
    uint8_t i, state, pressed;
    uint32_t now;

    if (keycontrol_ready == 0U) return;
    state = KeyControl_GetState();
    now = HAL_GetTick();
    for (i = 0U; i < KEYCONTROL_KEY_COUNT; ++i)
    {
        pressed = (state >> i) & 1U;
        if (pressed != keycontrol_candidate[i])
        {
            keycontrol_candidate[i] = pressed;
            keycontrol_change_tick[i] = now;
        }
        if ((pressed != keycontrol_stable[i]) &&
            ((uint32_t)(now - keycontrol_change_tick[i]) >= KEYCONTROL_DEBOUNCE_MS))
        {
            keycontrol_stable[i] = pressed;
            if (pressed != 0U) keycontrol_confirmed[i] = 1U;
            else if (keycontrol_confirmed[i] != 0U)
            {
                keycontrol_confirmed[i] = 0U;
                keycontrol_pending |= (uint8_t)(1U << i);
            }
        }
    }
}

uint8_t KeyControl_Process(const KeyControl_Bindings *bindings)
{
    uint8_t event;

    if ((bindings == 0) || (bindings->master_closed_loop == 0) ||
        (bindings->slave_enable_request == 0) || (bindings->slave_fault_flags == 0) ||
        (bindings->stop_slave == 0))
        return KEYCONTROL_EVENT_NONE;

    event = KeyControl_GetEvent();
    if (event == KEYCONTROL_EVENT_PE0)
    {
        *bindings->master_closed_loop = (*bindings->master_closed_loop == 0U) ? 1U : 0U;
        if (*bindings->master_closed_loop == 0U) bindings->stop_slave(0U);
    }
    else if (event == KEYCONTROL_EVENT_PB9)
    {
        if (*bindings->master_closed_loop != 0U)
        {
            if (*bindings->slave_enable_request != 0U) bindings->stop_slave(0U);
            else if (*bindings->slave_fault_flags == 0U) *bindings->slave_enable_request = 1U;
        }
    }
    /* PE2（按键三）保留给调用方处理，此处只返回事件。 */
    return event;
}
