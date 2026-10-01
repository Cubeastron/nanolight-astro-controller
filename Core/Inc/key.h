#ifndef __KEY_H__
#define __KEY_H__

/**
 * @file key.h
 * @brief 基于 1 ms SysTick 采样的非阻塞按键事件接口。
 *
 * 7 个按键均为低电平有效。状态机每 20 ms 采样一次，提供按下沿、释放沿、
 * 短按、稳定按住状态和 1 秒长按事件。
 */

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    KEY_ID_UP = 0,
    KEY_ID_DOWN,
    KEY_ID_OK,
    KEY_ID_LEFT,
    KEY_ID_RIGHT,
    KEY_ID_NS,
    KEY_ID_SF,
    KEY_ID_COUNT
} Key_Id;

typedef enum {
    KEY_EVENT_DOWN = 0x01U,
    KEY_EVENT_UP = 0x02U,
    KEY_EVENT_SHORT = 0x04U,
    KEY_EVENT_LONG = 0x08U
} Key_Event;

void Key_Init(void);
/* 由 SysTick_Handler() 每 1 ms 调用；这里只采样和记录事件，不执行前台业务。 */
void Key_Tick(void);
bool Key_Check(Key_Id key, Key_Event event);
bool Key_IsPressed(Key_Id key);
bool Key_AnyPressed(void);
void Key_ClearAllEvents(void);

#ifdef __cplusplus
}
#endif

#endif /* __KEY_H__ */
