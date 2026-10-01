/**
 * @file focuser_motor.h
 * @brief Step/dir motor control shared by UI, shutter and USB protocol.
 *
 * TIM3 generates the step cadence. Periods are expressed in microseconds;
 * position is updated only on rising STEP edges.
 */
#ifndef __FOCUSER_MOTOR_H
#define __FOCUSER_MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* Initialization and finite USB moves. */
void Motor_Init(void);
bool Motor_Step(bool direction);
void Motor_MoveSteps(uint32_t steps, bool direction);
void Motor_Stop(void);
uint32_t Motor_GetPosition(void);
void Motor_SetPosition(uint32_t pos);

/* Timer period control (microseconds). */
void Motor_SetPeriod(uint32_t period_us);

/* 方向接口使用逻辑方向，物理 DIR 电平由反转设置统一换算。 */
void Motor_SetDir(bool dir);
bool Motor_GetDir(void);
void Motor_ToggleDir(void);
void Motor_SetReverse(bool reversed);
bool Motor_GetReverse(void);
void Motor_ToggleStep(void);

/* Foreground mode handlers and the TIM3 callback hook. */
void Set_Period(void);
void MotorStep_Callback(void);
void Shutter_EQ_Run(void);
void Shutter_Gimbal_Run(void);
void Focuser_Run(void);



extern TIM_HandleTypeDef htim3;

#ifdef __cplusplus
}
#endif

#endif /* __FOCUSER_MOTOR_H */
