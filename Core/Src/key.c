#include "key.h"
#include "main.h"

#define KEY_SCAN_PERIOD_MS 20U
#define KEY_LONG_TIME_MS   1000U

typedef struct {
    GPIO_TypeDef *port;
    uint16_t pin;
} Key_Hardware;

static const Key_Hardware key_hardware[KEY_ID_COUNT] = {
    [KEY_ID_UP] = {UP_GPIO_Port, UP_Pin},
    [KEY_ID_DOWN] = {DOWN_GPIO_Port, DOWN_Pin},
    [KEY_ID_OK] = {OK_GPIO_Port, OK_Pin},
    [KEY_ID_LEFT] = {LEFT_GPIO_Port, LEFT_Pin},
    [KEY_ID_RIGHT] = {RIGHT_GPIO_Port, RIGHT_Pin},
    [KEY_ID_NS] = {NS_GPIO_Port, NS_Pin},
    [KEY_ID_SF] = {SF_GPIO_Port, SF_Pin},
};

static volatile uint8_t key_events[KEY_ID_COUNT];
static volatile bool key_pressed[KEY_ID_COUNT];
static uint16_t key_hold_time[KEY_ID_COUNT];
static bool key_long_reported[KEY_ID_COUNT];
static volatile bool key_initialized;

static bool Key_ReadPressed(Key_Id key)
{
    return HAL_GPIO_ReadPin(key_hardware[key].port, key_hardware[key].pin) == GPIO_PIN_RESET;
}

void Key_Init(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    key_initialized = false;
    for (uint8_t i = 0; i < KEY_ID_COUNT; i++) {
        key_events[i] = 0U;
        key_pressed[i] = Key_ReadPressed((Key_Id)i);
        key_hold_time[i] = 0U;
        key_long_reported[i] = false;
    }
    key_initialized = true;

    if (primask == 0U) {
        __enable_irq();
    }
}

void Key_Tick(void)
{
    static uint8_t scan_count;

    if (!key_initialized) {
        return;
    }

    scan_count++;
    if (scan_count < KEY_SCAN_PERIOD_MS) {
        return;
    }
    scan_count = 0U;

    for (uint8_t i = 0; i < KEY_ID_COUNT; i++) {
        bool pressed = Key_ReadPressed((Key_Id)i);

        if (pressed != key_pressed[i]) {
            bool was_long = key_long_reported[i];
            key_pressed[i] = pressed;
            key_hold_time[i] = 0U;
            key_long_reported[i] = false;

            if (pressed) {
                key_events[i] |= KEY_EVENT_DOWN;
            } else {
                key_events[i] |= KEY_EVENT_UP;
                if (!was_long) {
                    key_events[i] |= KEY_EVENT_SHORT;
                }
            }
            continue;
        }

        if (!pressed || key_long_reported[i]) {
            continue;
        }

        if (key_hold_time[i] < KEY_LONG_TIME_MS) {
            key_hold_time[i] += KEY_SCAN_PERIOD_MS;
        }
        if (key_hold_time[i] >= KEY_LONG_TIME_MS) {
            key_long_reported[i] = true;
            key_events[i] |= KEY_EVENT_LONG;
            key_events[i] &= (uint8_t)~KEY_EVENT_SHORT;
        }
    }
}

bool Key_Check(Key_Id key, Key_Event event)
{
    bool occurred = false;
    uint32_t primask;

    if (key >= KEY_ID_COUNT) {
        return false;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    if ((key_events[key] & (uint8_t)event) != 0U) {
        key_events[key] &= (uint8_t)~event;
        occurred = true;
    }
    if (primask == 0U) {
        __enable_irq();
    }

    return occurred;
}

bool Key_IsPressed(Key_Id key)
{
    return key < KEY_ID_COUNT && key_pressed[key];
}

bool Key_AnyPressed(void)
{
    for (uint8_t i = 0; i < KEY_ID_COUNT; i++) {
        if (key_pressed[i]) {
            return true;
        }
    }
    return false;
}

void Key_ClearAllEvents(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    for (uint8_t i = 0; i < KEY_ID_COUNT; i++) {
        key_events[i] = 0U;
    }
    if (primask == 0U) {
        __enable_irq();
    }
}
