/**
 * @file focuser_motor.c
 * @brief 步进电机控制及赤道仪、转台、对焦模式动作处理。
 *
 * TIM3 中断调用 HAL_TIM_PeriodElapsedCallback()。USB 移动使用
 * target_steps 计数；跟踪和转台模式使用同一个定时器连续或限角输出 STEP。
 */
#include "focuser_motor.h"
#include "focuser_protocol.h"
#include "focuser_ui.h"
#include "app_config.h"
#include "key.h"
#include "tim.h"
#include "usbd_cdc_if.h"

/* USB 移动由 TIM3 中断递减 target_steps。 */
static volatile uint32_t target_steps = 0;
static volatile bool motor_dir = false;
static volatile bool motor_active = false;
static volatile uint32_t current_period = 1000;
static volatile bool logical_direction = true;

static void Motor_WriteDirectionPin(bool direction) {
    bool physical_direction = direction ^ (Motor_reverse != 0U);
    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin,
                      physical_direction ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void MenuCameraLines_Write(GPIO_PinState focus, GPIO_PinState shutter) {
    if (SerialBulb_IsActive()) {
        return;
    }

    HAL_GPIO_WritePin(FOCUS_GPIO_Port, FOCUS_Pin, focus);
    HAL_GPIO_WritePin(SHUTTER_GPIO_Port, SHUTTER_Pin, shutter);
}

void Motor_Init(void) {
    /* Don't reset motor_position here - it's set by App_Init() from EEPROM */
    target_steps = 0;
    motor_dir = false;
    motor_active = false;
    logical_direction = true;
    
    /* Enable motor driver (EN = LOW to enable for TMC2208) */
    HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_RESET);
    Motor_WriteDirectionPin(logical_direction);
    HAL_GPIO_WritePin(STEP_GPIO_Port, STEP_Pin, GPIO_PIN_RESET);
    
    HAL_TIM_Base_Start_IT(&htim3);
}

bool Motor_Step(bool direction) {
    if (logical_direction != direction) {
        Motor_SetDir(direction);
        /* Small delay for direction signal to stabilize */
        for (volatile int i = 0; i < 5; i++);
    }

    bool prev_step_high = (HAL_GPIO_ReadPin(STEP_GPIO_Port, STEP_Pin) == GPIO_PIN_SET);
    Motor_ToggleStep();
    bool curr_step_high = (HAL_GPIO_ReadPin(STEP_GPIO_Port, STEP_Pin) == GPIO_PIN_SET);
    bool step_edge = (!prev_step_high && curr_step_high);  /* Rising edge = one motor step */
    
    /* Update position once per effective motor step. */
    if (step_edge) {
        if (direction) {
            if (Motor_position < RFCP_MAX_POSITION) {
                Motor_position++;
                EEPROM_Write(focuser_position_eeprom, Motor_position);
                focuser_position_dirty = true;
            }
        } else {
            if (Motor_position > 0) {
                Motor_position--;
                EEPROM_Write(focuser_position_eeprom, Motor_position);
                focuser_position_dirty = true;
            }
        }
    }

    return step_edge;
}

void Motor_MoveSteps(uint32_t steps, bool direction) {
    if (steps == 0) return;
    
    motor_dir = direction;
    target_steps = steps;
    motor_active = true;
    is_moving = true;
    
    /* USB 移动使用当前选择的对焦速度档位。 */
    uint32_t period = (uint32_t)FC_speedtab[speedtabi_2];
    Motor_SetPeriod(period);
}

void Motor_Stop(void) {
    motor_active = false;
    target_steps = 0;
    is_moving = false;
}

uint32_t Motor_GetPosition(void) {
    return Motor_position;
}

void Motor_SetPosition(uint32_t pos) {
    if (pos <= RFCP_MAX_POSITION) {
        Motor_position  = pos;
        //focuser_position = pos;
    }
}

void Motor_SetPeriod(uint32_t period_us) {
    current_period = period_us;
    __HAL_TIM_SET_AUTORELOAD(&htim3, period_us - 1);
}

void Motor_SetDir(bool dir) {
    logical_direction = dir;
    Motor_WriteDirectionPin(logical_direction);
}

bool Motor_GetDir(void) {
    return logical_direction;
}

void Motor_ToggleDir(void) {
    Motor_SetDir(!logical_direction);
}

void Motor_SetReverse(bool reversed) {
    Motor_reverse = reversed ? 1U : 0U;
    /* 反转设置只改变物理电平，当前逻辑方向保持不变。 */
    Motor_WriteDirectionPin(logical_direction);
}

bool Motor_GetReverse(void) {
    return Motor_reverse != 0U;
}

void Motor_ToggleStep(void) {
    HAL_GPIO_WritePin(STEP_GPIO_Port, STEP_Pin, 
        HAL_GPIO_ReadPin(STEP_GPIO_Port, STEP_Pin) == GPIO_PIN_SET ? GPIO_PIN_RESET : GPIO_PIN_SET);
}



/* ========== Set Period Function ========== */
void Set_Period(void) {
    if (!UI_ButtonsEnabled()) {
        return;
    }

    /* EQ Mode */
    if (func_index == 4 || func_index == 5) {
        Motor_SetPeriod(EQ_speedtab[speedtabi]);
        if (Key_Check(KEY_ID_NS, KEY_EVENT_DOWN)) {
            Motor_ToggleDir();
            UI_Refresh();
        }
        if (Key_Check(KEY_ID_SF, KEY_EVENT_DOWN)) {
            speedtabi++;
            if (speedtabi > 3) speedtabi = 0;
            Motor_SetPeriod(EQ_speedtab[speedtabi]);
            UI_Refresh();
        }
    }
    
    /* Gimbal Mode */
    if (func_index == 6||func_index == 7||func_index == 8) {
        if (Key_Check(KEY_ID_NS, KEY_EVENT_DOWN)) {
            Motor_ToggleDir();
            UI_Refresh();
        }
    }

}

/* ========== Motor Step Callback ========== */
void MotorStep_Callback(void) {

    /* EQ/Gimbal modes - continuous rotation */
    if (func_index == 4 || func_index == 5 ||
        (func_index >= 18 && func_index <= 22) || func_index == 24) {
        Motor_ToggleStep();
    }

    /* Gimbal shutter rotation with angle limit */
    if (func_index == 23 && !if_pulseend) {
        if (pulseCount < angle_pulsen) {
            Motor_ToggleStep();
            pulseCount++;
        } else {
            if_pulseend = true;
            angle_p = 100;
            pulseCount = 0;
        }
    }

    /* USB 移动占用定时器时，禁止手动对焦逻辑抢占 STEP 输出。 */
    if ((func_index == 12||func_index == 13)&& outputEnabled && !motor_active) {
        Motor_Step(Motor_GetDir());
    }
}

/* ========== Shutter EQ Run ========== */
void Shutter_EQ_Run(void) {
    
    if (func_index == 24) {
        if (SerialBulb_IsActive()) {
            return;
        }

        Motor_SetPeriod(EQ_speedtab[speedtabi]);
        uint32_t startTime = HAL_GetTick();
        ifex = true;

        MenuCameraLines_Write(GPIO_PIN_SET, GPIO_PIN_SET);

        Shutter_n = Shutter_num;
        UI_Refresh();
        
        while (Shutter_n > 0) {
            UI_ServiceDisplayPower();
            if ((UI_ButtonsEnabled() && Key_Check(KEY_ID_OK, KEY_EVENT_SHORT)) || Shutter_n == 0) {
                break;
            }
            
            if ((HAL_GetTick() - startTime > 1000) && ifex) {
                if (Ex_time > 0) Ex_time--;
                startTime = HAL_GetTick();
                UI_Refresh();
            }
            
            if ((HAL_GetTick() - startTime > 1000) && !ifex) {
                if (Interval_time > 0) Interval_time--;
                startTime = HAL_GetTick();
                UI_Refresh();
            }
            
            if (Ex_time == 0 && Shutter_n > 0) {
                MenuCameraLines_Write(GPIO_PIN_RESET, GPIO_PIN_RESET);
                ifex = false;
                Ex_time = EEPROM_Read(Ex_time_eeprom);
                if (Ex_time == 0xFFFF) Ex_time = 4;
                Shutter_n--;
                UI_Refresh();
            }
            
            if (Interval_time == 0 && Shutter_n > 0) {

                MenuCameraLines_Write(GPIO_PIN_SET, GPIO_PIN_SET);

                ifex = true;
                Interval_time = EEPROM_Read(Interval_time_eeprom);
                if (Interval_time == 0xFFFF) Interval_time = 2;
                UI_Refresh();
            }
        }
        
        Interval_time = EEPROM_Read(Interval_time_eeprom);
        if (Interval_time == 0xFFFF) Interval_time = 2;
        Ex_time = EEPROM_Read(Ex_time_eeprom);
        if (Ex_time == 0xFFFF) Ex_time = 4;
        MenuCameraLines_Write(GPIO_PIN_RESET, GPIO_PIN_RESET);
        ifex = false;
        
        func_index = 18;

        UI_Refresh();
    }
}

/* ========== Shutter Gimbal Run ========== */
void Shutter_Gimbal_Run(void) {
    
    if (func_index == 23) {
        if (SerialBulb_IsActive()) {
            return;
        }

        pulseCount = 0;
        angle_pulsen = (unsigned long)(Rota_angle * (float)Dece_ratio * (float)Motor_total_step / 360.0f * 32.0f);
        if (angle_pulsen == 0) {
            /* Invalid rotation setup (e.g. Rota_angle == 0), abort safely. */
            func_index = 6;
            current_operation_index = table[func_index].current_operation;
            UI_Refresh();
            return;
        }
        period_Gimbal = (unsigned long)((float)Interval_time * 1000.0f / (float)angle_pulsen * 1000.0f / 2.0f);
        if (period_Gimbal <= MAX_us) period_Gimbal = MAX_us;
        Motor_SetPeriod(period_Gimbal);
        
        if_pulseend = true;  /* Start with exposure first */
        angle_p = 0;
        
        uint32_t currentTime = HAL_GetTick();
        uint32_t startTime_forUI = currentTime;
        uint16_t Ex_time_bp = Ex_time * 1000;
        Shutter_n = EEPROM_Read(Shutter_num_eeprom);
        UI_Refresh();
        
        while (Shutter_n > 0) {
            UI_ServiceDisplayPower();
            if (UI_ButtonsEnabled() && Key_Check(KEY_ID_OK, KEY_EVENT_SHORT)) {
                break;
            }

            /* Update UI countdown during exposure */
            if ((HAL_GetTick() - startTime_forUI > 1000) && ifex)
            {
                if (Ex_time > 0) Ex_time--;
                startTime_forUI = HAL_GetTick();
                UI_Refresh();
            }

            /* Exposure phase */
            if (if_pulseend) {
                // UI_Refresh();
                if (!SerialBulb_IsActive()) {
                    HAL_GPIO_WritePin(FOCUS_GPIO_Port, FOCUS_Pin, GPIO_PIN_SET);
                }
                uint32_t elapsedTime = HAL_GetTick() - currentTime;
                
                if (elapsedTime >= focustime) {
                    MenuCameraLines_Write(GPIO_PIN_SET, GPIO_PIN_SET);
                    ifex = true;
                }
                
                if (elapsedTime >= Ex_time_bp + focustime) {

                    MenuCameraLines_Write(GPIO_PIN_RESET, GPIO_PIN_RESET);

                    ifex = false;
                    currentTime = HAL_GetTick();
                    startTime_forUI = currentTime;
                    Ex_time = EEPROM_Read(Ex_time_eeprom);
                    Shutter_n--;
                    UI_Refresh();
                    HAL_Delay(20);//稳定时间
                    if(Shutter_n>0)
                    {
                        if_pulseend = false;
                    }

                }
            }
            

            /* Motor rotation phase */
            if (!if_pulseend) {
                currentTime = HAL_GetTick();
                startTime_forUI = currentTime;
                angle_p = (float)pulseCount / (float)angle_pulsen * 100.0f;
                UI_Refresh();
            }
        }
        
        /* Cleanup and exit */

        if_pulseend = true;
        pulseCount = angle_pulsen;
        Motor_SetPeriod(EQ_speedtab[speedtabi]);
        Ex_time = EEPROM_Read(Ex_time_eeprom);
        MenuCameraLines_Write(GPIO_PIN_RESET, GPIO_PIN_RESET);
        func_index = 6;
        current_operation_index = table[func_index].current_operation;
        UI_Refresh();
    }
}

/* ========== Focuser Run ========== */
void Focuser_Run(void) {
    /* 手动对焦页面：按键按住时移动电机并刷新 OLED 光标。 */
    static int8_t active_direction = 0;
    static uint32_t cursor_tick = 0U;
    bool left_pressed;
    bool right_pressed;
    int8_t requested_direction = 0;
    uint32_t now;

    if (func_index != 12 || !UI_ButtonsEnabled()) {
        if (outputEnabled) {
            outputEnabled = false;
            active_direction = 0;
            cursor_p = 128U;
            UI_Refresh();
        }
        return;
    }

    Motor_SetPeriod(FC_speedtab[speedtabi_2]);
    if (Key_Check(KEY_ID_SF, KEY_EVENT_DOWN)) {
        speedtabi_2++;
        if (speedtabi_2 > 3) speedtabi_2 = 0;
        Motor_SetPeriod(FC_speedtab[speedtabi_2]);
        UI_Refresh();
    }

    left_pressed = Key_IsPressed(KEY_ID_LEFT);
    right_pressed = Key_IsPressed(KEY_ID_RIGHT);
    /* 手动点动方向与按键名称反向，保持原有双键时 LEFT 优先。 */
    if (left_pressed) {
        requested_direction = 1;
    } else if (right_pressed) {
        requested_direction = -1;
    }

    now = HAL_GetTick();
    if (requested_direction != 0) {
        if (!outputEnabled || requested_direction != active_direction) {
            active_direction = requested_direction;
            Motor_SetDir(requested_direction > 0);
            outputEnabled = true;
            cursor_p = requested_direction < 0 ? 32U : 96U;
            cursor_tick = now;
            UI_Refresh();
        } else if ((now - cursor_tick) > 50U) {
            if (requested_direction < 0) {
                cursor_p -= 8U;
                if (cursor_p == 0U) cursor_p = 32U;
            } else {
                cursor_p += 8U;
                if (cursor_p >= 128U) cursor_p = 96U;
            }
            cursor_tick = now;
            UI_Refresh();
        }
    } else if (outputEnabled) {
        outputEnabled = false;
        active_direction = 0;
        cursor_p = 128U;
        UI_Refresh();
    }

}

/* ========== Timer Interrupt Callback ========== */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM3) {
        /* USB focuser protocol motor control */
        if (motor_active && target_steps > 0) {
            if (Motor_Step(motor_dir)) {
                target_steps--;

                if (target_steps == 0) {
                    motor_active = false;
                    is_moving = false;
                }
            }
        }

        /* Motor step callback for UI modes */
        MotorStep_Callback();
    }
}
