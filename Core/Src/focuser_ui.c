/**
 * @file focuser_ui.c
 * @brief OLED 页面、非阻塞按键导航、显示休眠和延迟保存参数。
 */
#include "focuser_ui.h"
#include "focuser_motor.h"
#include "oled.h"
#include "font.h"
#include "app_config.h"
#include "key.h"
#include <stdio.h>
#include <string.h>

uchar func_index = 0;
static uchar last_index = 100;
void (*current_operation_index)(void);
bool dirty_flag = 0;
static bool display_on = true;
static bool input_locked = false;
static bool save_message_active = false;
static uint32_t save_message_until = 0U;
static uint32_t parameter_adjust_tick = 0U;
static bool parameter_adjust_active = false;

/* 页面编号就是 table[] 下标；表内跳转值保留旧代码使用的从 1 开始格式。 */
typedef enum {
    PAGE_A1 = 0,
    PAGE_B1,
    PAGE_C1,
    PAGE_D1,
    PAGE_A21,
    PAGE_A22,
    PAGE_B21,
    PAGE_B22,
    PAGE_B23,
    PAGE_B24,
    PAGE_B25,
    PAGE_B26,
    PAGE_C21,
    PAGE_C22,
    PAGE_D21,
    PAGE_D22,
    PAGE_D23,
    PAGE_D24,
    PAGE_A31,
    PAGE_A32,
    PAGE_A33,
    PAGE_A34,
    PAGE_A35,
    PAGE_B31,
    PAGE_A41,
    /* 追加返回页，避免改变曝光和转台运行页的既有编号。 */
    PAGE_D25
} UI_PageIndex;

key_table table[32] = {
    { PAGE_A1 + 1, PAGE_D1 + 1, PAGE_B1 + 1, PAGE_A21 + 1, fun_a1, 100},
    { PAGE_B1 + 1, PAGE_A1 + 1, PAGE_C1 + 1, PAGE_B21 + 1, fun_b1, 100},
    { PAGE_C1 + 1, PAGE_B1 + 1, PAGE_D1 + 1, PAGE_C21 + 1, fun_c1, 100},
    { PAGE_D1 + 1, PAGE_C1 + 1, PAGE_A1 + 1, PAGE_D21 + 1, fun_d1, 100},
    
    { PAGE_A21 + 1, PAGE_A22 + 1, PAGE_A22 + 1, PAGE_A31 + 1, fun_a21, 100},
    { PAGE_A22 + 1, PAGE_A21 + 1, PAGE_A21 + 1, PAGE_A1 + 1, fun_a22, 100},
    
    { PAGE_B21 + 1, PAGE_B26 + 1, PAGE_B22 + 1, PAGE_B21 + 1, fun_b21, Rota_angle_eeprom},
    { PAGE_B22 + 1, PAGE_B21 + 1, PAGE_B23 + 1, PAGE_B22 + 1, fun_b22, Interval_time_eeprom},
    { PAGE_B23 + 1, PAGE_B22 + 1, PAGE_B24 + 1, PAGE_B23 + 1, fun_b23, Ex_time_eeprom},
    { PAGE_B24 + 1, PAGE_B23 + 1, PAGE_B25 + 1, PAGE_B24 + 1, fun_b24, Shutter_num_eeprom},
    { PAGE_B25 + 1, PAGE_B24 + 1, PAGE_B26 + 1, PAGE_B31 + 1, fun_b25, 100},
    { PAGE_B26 + 1, PAGE_B25 + 1, PAGE_B21 + 1, PAGE_B1 + 1, fun_b26, 100},
    
    { PAGE_C21 + 1, PAGE_C22 + 1, PAGE_C22 + 1, PAGE_C21 + 1, fun_c21, focuser_position_eeprom},
    { PAGE_C22 + 1, PAGE_C21 + 1, PAGE_C21 + 1, PAGE_C1 + 1, fun_c22, 100},
    
    { PAGE_D21 + 1, PAGE_D25 + 1, PAGE_D22 + 1, PAGE_D21 + 1, fun_d21, Motor_total_step_eeprom},
    { PAGE_D22 + 1, PAGE_D21 + 1, PAGE_D23 + 1, PAGE_D22 + 1, fun_d22, Dece_ratio_eeprom},
    { PAGE_D23 + 1, PAGE_D22 + 1, PAGE_D24 + 1, PAGE_D23 + 1, fun_d23, MAX_us_eeprom},
    { PAGE_D24 + 1, PAGE_D23 + 1, PAGE_D25 + 1, PAGE_D24 + 1, fun_d24, Motor_reverse_eeprom},
    
    { PAGE_A31 + 1, PAGE_A35 + 1, PAGE_A32 + 1, PAGE_A31 + 1, fun_a31, Ex_time_eeprom},
    { PAGE_A32 + 1, PAGE_A31 + 1, PAGE_A33 + 1, PAGE_A32 + 1, fun_a32, Shutter_num_eeprom},
    { PAGE_A33 + 1, PAGE_A32 + 1, PAGE_A34 + 1, PAGE_A33 + 1, fun_a33, Interval_time_eeprom},
    { PAGE_A34 + 1, PAGE_A33 + 1, PAGE_A35 + 1, PAGE_A41 + 1, fun_a34, 100},
    { PAGE_A35 + 1, PAGE_A34 + 1, PAGE_A31 + 1, PAGE_A21 + 1, fun_a35, 100},
    
    { PAGE_B31 + 1, PAGE_B31 + 1, PAGE_B31 + 1, PAGE_B21 + 1, fun_b31, 100},

    { PAGE_A41 + 1, PAGE_A41 + 1, PAGE_A41 + 1, PAGE_A31 + 1, fun_a41, 100},

    { PAGE_D25 + 1, PAGE_D24 + 1, PAGE_D21 + 1, PAGE_D1 + 1, fun_d25, 100},

};

static uint8_t Get_Dir_State(void) {
    return Motor_GetDir() ? 1U : 0U;
}

static void UI_ShowSaveSuccess(void) {
    OLED_NewFrame();
    OLED_PrintString(34, 24, "保存成功", &font15x15, OLED_COLOR_NORMAL);
    OLED_ShowFrame();
    save_message_active = true;
    save_message_until = HAL_GetTick() + 500U;
}

static void UI_SaveCurrentParameter(void) {
    if (!dirty_flag && !focuser_position_dirty) {
        return;
    }

    /* 同步当前位置，确保手动对焦页面的变化也会写入 Flash。 */
    EEPROM_Write(focuser_position_eeprom, (uint16_t)Motor_GetPosition());
    EEPROM_Save();
    dirty_flag = 0;
    UI_ShowSaveSuccess();
}

static void UI_AdjustCurrentParameter(bool increase) {
    switch (table[func_index].eeprom_num) {
        case Ex_time_eeprom:
            if (increase) Ex_time++;
            else if (Ex_time > 0U) Ex_time--;
            current_operation_data = Ex_time;
            break;
        case Interval_time_eeprom:
            if (increase) Interval_time++;
            else if (Interval_time > 0U) Interval_time--;
            current_operation_data = Interval_time;
            break;
        case Shutter_num_eeprom:
            if (increase) Shutter_num++;
            else if (Shutter_num > 0U) Shutter_num--;
            current_operation_data = Shutter_num;
            break;
        case Rota_angle_eeprom:
            if (increase) Rota_angle++;
            else if (Rota_angle > 0U) Rota_angle--;
            current_operation_data = Rota_angle;
            break;
        case Motor_total_step_eeprom:
            if (increase) Motor_total_step++;
            else Motor_total_step--;
            if (Motor_total_step == 0U) Motor_total_step = 1U;
            current_operation_data = Motor_total_step;
            break;
        case Dece_ratio_eeprom:
            if (increase) Dece_ratio++;
            else Dece_ratio--;
            if (Dece_ratio == 0U) Dece_ratio = 1U;
            current_operation_data = Dece_ratio;
            break;
        case MAX_us_eeprom:
            if (increase) MAX_us++;
            else MAX_us--;
            if (MAX_us <= 5U) MAX_us = 5U;
            current_operation_data = MAX_us;
            break;
        default:
            return;
    }

    EEPROM_Write(table[func_index].eeprom_num, current_operation_data);
    dirty_flag = true;
}

static void UI_ApplyPageMotorState(void) {
    if (func_index == PAGE_A1) {
        Motor_SetPeriod((uint32_t)EQ_speedtab[speedtabi]);
        return;
    }

    if (func_index == PAGE_C1) {
        Motor_SetPeriod((uint32_t)FC_speedtab[speedtabi_2]);
    }
}

void UI_Init(void) {
    func_index = PAGE_A1;
    last_index = 100;
    current_operation_index = table[func_index].current_operation;
    display_on = true;
    input_locked = false;
    save_message_active = false;
    parameter_adjust_active = false;
    Key_Init();
    OLED_Init();
}

void UI_Refresh(void) {
    if (!display_on || save_message_active) {
        return;
    }
    OLED_NewFrame();
    if (current_operation_index != NULL) {
        current_operation_index();
    }
    OLED_ShowFrame();
}

bool UI_ServiceDisplayPower(void) {
    if (!display_on) {
        for (uint8_t i = 0; i < KEY_ID_COUNT; i++) {
            if (Key_Check((Key_Id)i, KEY_EVENT_DOWN)) {
                display_on = true;
                input_locked = true;
                Key_ClearAllEvents();
                OLED_DisPlay_On();
                if (func_index != last_index) {
                    UI_ApplyPageMotorState();
                    current_operation_index = table[func_index].current_operation;
                    last_index = func_index;
                }
                UI_Refresh();
                return false;
            }
        }
        return false;
    }

    if (Key_Check(KEY_ID_OK, KEY_EVENT_LONG)) {
        display_on = false;
        input_locked = false;
        parameter_adjust_active = false;
        Key_ClearAllEvents();
        OLED_DisPlay_Off();
        return false;
    }

    if (input_locked) {
        Key_ClearAllEvents();
        if (!Key_AnyPressed()) {
            input_locked = false;
        }
        return false;
    }

    return true;
}

bool UI_IsDisplayOn(void) {
    return display_on;
}

bool UI_ButtonsEnabled(void) {
    return display_on && !input_locked && !save_message_active;
}

void UI_Turn(void) {
    uint32_t now;
    bool left_pressed;
    bool right_pressed;

    if (!UI_ServiceDisplayPower()) {
        parameter_adjust_active = false;
        return;
    }

    if (save_message_active) {
        if ((int32_t)(HAL_GetTick() - save_message_until) < 0) {
            Key_ClearAllEvents();
            return;
        }
        Key_ClearAllEvents();
        save_message_active = false;
        UI_Refresh();
    }

    if (Key_Check(KEY_ID_UP, KEY_EVENT_DOWN)) {
        func_index = table[func_index].up - 1;
    }
    if (Key_Check(KEY_ID_DOWN, KEY_EVENT_DOWN)) {
        func_index = table[func_index].down - 1;
    }
    if (Key_Check(KEY_ID_OK, KEY_EVENT_SHORT)) {
        if (table[func_index].eeprom_num != 100) {
            UI_SaveCurrentParameter();
        } else {
            func_index = table[func_index].enter - 1;
        }
    }
    
    if (func_index != last_index) {
        UI_ApplyPageMotorState();
        current_operation_index = table[func_index].current_operation;
        last_index = func_index;

        /* 丢弃旧页面事件，并等待所有按键释放后再接收新页面输入。 */
        Key_ClearAllEvents();
        input_locked = true;
        parameter_adjust_active = false;

        UI_Refresh();
        return;
    }

    if (func_index == PAGE_D24) {
        bool toggle_reverse = Key_Check(KEY_ID_LEFT, KEY_EVENT_DOWN);
        toggle_reverse = Key_Check(KEY_ID_RIGHT, KEY_EVENT_DOWN) || toggle_reverse;
        parameter_adjust_active = false;

        if (toggle_reverse) {
            Motor_SetReverse(!Motor_GetReverse());
            current_operation_data = Motor_GetReverse() ? 1U : 0U;
            EEPROM_Write(Motor_reverse_eeprom, current_operation_data);
            dirty_flag = true;
            UI_Refresh();
        }
        return;
    }
    
    /* LEFT/RIGHT 只在映射到持久化参数的页面修改数值。 */
    /* Skip in Focuser mode (func_index 12-14) - handled by Focuser_Run */
    left_pressed = Key_IsPressed(KEY_ID_LEFT);
    right_pressed = Key_IsPressed(KEY_ID_RIGHT);
    if ((left_pressed || right_pressed) &&
        table[func_index].eeprom_num != 100 &&
        func_index != PAGE_C21) {
        now = HAL_GetTick();
        if (!parameter_adjust_active || (now - parameter_adjust_tick) >= 20U) {
            parameter_adjust_active = true;
            parameter_adjust_tick = now;

            /* 保持原参数编辑逻辑：同时按住时先处理 RIGHT，再处理 LEFT。 */
            if (right_pressed) {
                UI_AdjustCurrentParameter(true);
            }
            if (left_pressed) {
                UI_AdjustCurrentParameter(false);
            }
            if (func_index >= PAGE_D21 && func_index <= PAGE_D23) {
                App_UpdateSpeedTables();
            }
            UI_Refresh();
        }
    } else {
        parameter_adjust_active = false;
    }
}

/* ========== First Layer Menu Functions ========== */
void fun_a1(void) {
    OLED_PrintString(cursor_row, oneline - 12, ">", &font15x15, OLED_COLOR_NORMAL);
    OLED_PrintString(selec, oneline - 12, "赤道仪模式", &font15x15, OLED_COLOR_NORMAL);
    OLED_PrintString(nonselec, twoline - 12, "延时模式", &font15x15, OLED_COLOR_NORMAL);
    OLED_PrintString(nonselec, thrline - 12, "电调模式", &font15x15, OLED_COLOR_NORMAL);
    OLED_PrintString(nonselec, fline - 12, "电机设置", &font15x15, OLED_COLOR_NORMAL);
}

void fun_b1(void) {
    OLED_PrintString(cursor_row, twoline - 12, ">", &font15x15, OLED_COLOR_NORMAL);
    OLED_PrintString(nonselec, oneline - 12, "赤道仪模式", &font15x15, OLED_COLOR_NORMAL);
    OLED_PrintString(selec, twoline - 12, "延时模式", &font15x15, OLED_COLOR_NORMAL);
    OLED_PrintString(nonselec, thrline - 12, "电调模式", &font15x15, OLED_COLOR_NORMAL);
    OLED_PrintString(nonselec, fline - 12, "电机设置", &font15x15, OLED_COLOR_NORMAL);
}

void fun_c1(void) {
    OLED_PrintString(cursor_row, thrline - 12, ">", &font15x15, OLED_COLOR_NORMAL);
    OLED_PrintString(nonselec, oneline - 12, "赤道仪模式", &font15x15, OLED_COLOR_NORMAL);
    OLED_PrintString(nonselec, twoline - 12, "延时模式", &font15x15, OLED_COLOR_NORMAL);
    OLED_PrintString(selec, thrline - 12, "电调模式", &font15x15, OLED_COLOR_NORMAL);
    OLED_PrintString(nonselec, fline - 12, "电机设置", &font15x15, OLED_COLOR_NORMAL);
}

void fun_d1(void) {
    OLED_PrintString(cursor_row, fline - 12, ">", &font15x15, OLED_COLOR_NORMAL);
    OLED_PrintString(nonselec, oneline - 12, "赤道仪模式", &font15x15, OLED_COLOR_NORMAL);
    OLED_PrintString(nonselec, twoline - 12, "延时模式", &font15x15, OLED_COLOR_NORMAL);
    OLED_PrintString(nonselec, thrline - 12, "电调模式", &font15x15, OLED_COLOR_NORMAL);
    OLED_PrintString(selec, fline - 12, "电机设置", &font15x15, OLED_COLOR_NORMAL);
}

/* ========== Second Layer Menu Functions ========== */
void fun_a21(void) {
    OLED_PrintString(0, twoline - 12, ">", &font15x15, OLED_COLOR_NORMAL);
    OLED_PrintString(0, oneline - 12, "方向", &font15x15, OLED_COLOR_NORMAL);
    OLED_PrintString(54, oneline - 12, "速度", &font15x15, OLED_COLOR_NORMAL);
    OLED_PrintString(selec, twoline - 12, "快门设置", &font15x15, OLED_COLOR_NORMAL);
    OLED_PrintString(nonselec, thrline - 12, "返回", &font15x15, OLED_COLOR_NORMAL);

    OLED_PrintString(34, oneline - 12, N_S[Get_Dir_State()], &font15x15, OLED_COLOR_NORMAL);
    OLED_PrintString(86, oneline - 12, EQ_speedtab_UI[speedtabi], &font15x15, OLED_COLOR_NORMAL);
}

void fun_a22(void) {
    OLED_PrintString(0, thrline - 12, ">", &font15x15, OLED_COLOR_NORMAL);
    OLED_PrintString(0, oneline - 12, "方向", &font15x15, OLED_COLOR_NORMAL);
    OLED_PrintString(54, oneline - 12, "速度", &font15x15, OLED_COLOR_NORMAL);
    OLED_PrintString(nonselec, twoline - 12, "快门设置", &font15x15, OLED_COLOR_NORMAL);
    OLED_PrintString(selec, thrline - 12, "返回", &font15x15, OLED_COLOR_NORMAL);

    OLED_PrintString(34, oneline - 12, N_S[Get_Dir_State()], &font15x15, OLED_COLOR_NORMAL);
    OLED_PrintString(86, oneline - 12, EQ_speedtab_UI[speedtabi], &font15x15, OLED_COLOR_NORMAL);
}

void fun_b21(void) {
    char buf[16];
    OLED_PrintString(0, twoline - 12, ">", &font15x15, OLED_COLOR_NORMAL);
    OLED_PrintString(0, oneline - 12, "方向", &font15x15, OLED_COLOR_NORMAL);
    OLED_PrintString(selec, twoline - 12, "旋转角度", &font15x15, OLED_COLOR_NORMAL);
    OLED_PrintString(nonselec, thrline - 12, "间隔时间", &font15x15, OLED_COLOR_NORMAL);
    OLED_PrintString(nonselec, fline - 12, "曝光时间", &font15x15, OLED_COLOR_NORMAL);

    OLED_PrintString(34, oneline - 12,
                     Get_Dir_State() ? "逆" : "顺",
                     &font15x15, OLED_COLOR_NORMAL);
    snprintf(buf, sizeof(buf), "%d", Rota_angle);
    OLED_PrintString(80, twoline - 12, buf, &font15x15, OLED_COLOR_NORMAL);
    snprintf(buf, sizeof(buf), "%d", Interval_time);
    OLED_PrintString(80, thrline - 12, buf, &font15x15, OLED_COLOR_NORMAL);
    snprintf(buf, sizeof(buf), "%d", Ex_time);
    OLED_PrintString(80, fline - 12, buf, &font15x15, OLED_COLOR_NORMAL);
}

void fun_b22(void) {
    char buf[16];
    OLED_PrintString(0, thrline - 12, ">", &font15x15, OLED_COLOR_NORMAL);
    OLED_PrintString(0, oneline - 12, "方向", &font15x15, OLED_COLOR_NORMAL);
    OLED_PrintString(nonselec, twoline - 12, "旋转角度", &font15x15, OLED_COLOR_NORMAL);
    OLED_PrintString(selec, thrline - 12, "间隔时间", &font15x15, OLED_COLOR_NORMAL);
    OLED_PrintString(nonselec, fline - 12, "曝光时间", &font15x15, OLED_COLOR_NORMAL);

    OLED_PrintString(34, oneline - 12,
                     Get_Dir_State() ? "逆" : "顺",
                     &font15x15, OLED_COLOR_NORMAL);
    snprintf(buf, sizeof(buf), "%d", Rota_angle);
    OLED_PrintString(80, twoline - 12, buf, &font15x15, OLED_COLOR_NORMAL);
    snprintf(buf, sizeof(buf), "%d", Interval_time);
    OLED_PrintString(80, thrline - 12, buf, &font15x15, OLED_COLOR_NORMAL);
    snprintf(buf, sizeof(buf), "%d", Ex_time);
    OLED_PrintString(80, fline - 12, buf, &font15x15, OLED_COLOR_NORMAL);
}

void fun_b23(void) {
    char buf[16];
    OLED_PrintString(0, fline - 12, ">", &font15x15, OLED_COLOR_NORMAL);
    OLED_PrintString(0, oneline - 12, "方向", &font15x15, OLED_COLOR_NORMAL);
    OLED_PrintString(nonselec, twoline - 12, "旋转角度", &font15x15, OLED_COLOR_NORMAL);
    OLED_PrintString(nonselec, thrline - 12, "间隔时间", &font15x15, OLED_COLOR_NORMAL);
    OLED_PrintString(selec, fline - 12, "曝光时间", &font15x15, OLED_COLOR_NORMAL);

    OLED_PrintString(34, oneline - 12,
                     Get_Dir_State() ? "逆" : "顺",
                     &font15x15, OLED_COLOR_NORMAL);
    snprintf(buf, sizeof(buf), "%d", Rota_angle);
    OLED_PrintString(80, twoline - 12, buf, &font15x15, OLED_COLOR_NORMAL);
    snprintf(buf, sizeof(buf), "%d", Interval_time);
    OLED_PrintString(80, thrline - 12, buf, &font15x15, OLED_COLOR_NORMAL);
    snprintf(buf, sizeof(buf), "%d", Ex_time);
    OLED_PrintString(80, fline - 12, buf, &font15x15, OLED_COLOR_NORMAL);
}

void fun_b24(void) {
    char buf[16];
    OLED_PrintString(0, oneline - 12, ">", &font15x15, OLED_COLOR_NORMAL);
    OLED_PrintString(selec, oneline - 12, "快门数", &font15x15, OLED_COLOR_NORMAL);
    OLED_PrintString(nonselec, twoline - 12, "启动", &font15x15, OLED_COLOR_NORMAL);
    OLED_PrintString(nonselec, thrline - 12, "返回", &font15x15, OLED_COLOR_NORMAL);


    snprintf(buf, sizeof(buf), "%d", Shutter_num);
    OLED_PrintString(80, oneline - 12, buf, &font15x15, OLED_COLOR_NORMAL);

}

void fun_b25(void) {
    char buf[16];
    OLED_PrintString(0, twoline - 12, ">", &font15x15, OLED_COLOR_NORMAL);
    OLED_PrintString(nonselec, oneline - 12, "快门数", &font15x15, OLED_COLOR_NORMAL);
    OLED_PrintString(selec, twoline - 12, "启动", &font15x15, OLED_COLOR_NORMAL);
    OLED_PrintString(nonselec, thrline - 12, "返回", &font15x15, OLED_COLOR_NORMAL);


    snprintf(buf, sizeof(buf), "%d", Shutter_num);
    OLED_PrintString(80, oneline - 12, buf, &font15x15, OLED_COLOR_NORMAL);
}

void fun_b26(void) {
    char buf[16];
    OLED_PrintString(0, thrline - 12, ">", &font15x15, OLED_COLOR_NORMAL);
    OLED_PrintString(nonselec, oneline - 12, "快门数", &font15x15, OLED_COLOR_NORMAL);
    OLED_PrintString(nonselec, twoline - 12, "启动", &font15x15, OLED_COLOR_NORMAL);
    OLED_PrintString(selec, thrline - 12, "返回", &font15x15, OLED_COLOR_NORMAL);


    snprintf(buf, sizeof(buf), "%d", Shutter_num);
    OLED_PrintString(80, oneline - 12, buf, &font15x15, OLED_COLOR_NORMAL);
}

void fun_c21(void) {
    char buf[16];
    OLED_PrintString(0, twoline - 12, ">", &font15x15, OLED_COLOR_NORMAL);
    OLED_PrintString(46, oneline - 12, FC_speedtab_UI[speedtabi_2], &font15x15, OLED_COLOR_NORMAL);
    OLED_PrintString(selec, twoline - 12, "电机位置", &font15x15, OLED_COLOR_NORMAL);
    OLED_PrintString(nonselec, thrline - 12, "返回", &font15x15, OLED_COLOR_NORMAL);

    snprintf(buf, sizeof(buf), "%lu", Motor_GetPosition());
    OLED_PrintString(80, twoline - 12, buf, &font15x15, OLED_COLOR_NORMAL);
    OLED_PrintString(cursor_p, oneline - 12, Z_F[Get_Dir_State()], &font15x15, OLED_COLOR_NORMAL);
}

void fun_c22(void) {
    char buf[16];
    OLED_PrintString(0, thrline - 12, ">", &font15x15, OLED_COLOR_NORMAL);
    OLED_PrintString(46, oneline - 12, FC_speedtab_UI[speedtabi_2], &font15x15, OLED_COLOR_NORMAL);
    OLED_PrintString(nonselec, twoline - 12, "电机位置", &font15x15, OLED_COLOR_NORMAL);
    OLED_PrintString(selec, thrline - 12, "返回", &font15x15, OLED_COLOR_NORMAL);
    snprintf(buf, sizeof(buf), "%lu", Motor_GetPosition());
    OLED_PrintString(80, twoline - 12, buf, &font15x15, OLED_COLOR_NORMAL);
    OLED_PrintString(cursor_p, oneline - 12, Z_F[Get_Dir_State()], &font15x15, OLED_COLOR_NORMAL);
}


void fun_d21(void) {
    char buf[16];
    OLED_PrintString(0, oneline - 12, ">", &font15x15, OLED_COLOR_NORMAL);
    OLED_PrintString(selec, oneline - 12, "总步数", &font15x15, OLED_COLOR_NORMAL);
    OLED_PrintString(nonselec, twoline - 12, "减速比", &font15x15, OLED_COLOR_NORMAL);
    OLED_PrintString(nonselec, thrline - 12, "最短脉冲", &font15x15, OLED_COLOR_NORMAL);
    OLED_PrintString(nonselec, fline - 12, "电机反转", &font15x15, OLED_COLOR_NORMAL);

    snprintf(buf, sizeof(buf), "%d", Motor_total_step);
    OLED_PrintString(80, oneline - 12, buf, &font15x15, OLED_COLOR_NORMAL);
    snprintf(buf, sizeof(buf), "%d", Dece_ratio);
    OLED_PrintString(80, twoline - 12, buf, &font15x15, OLED_COLOR_NORMAL);
    snprintf(buf, sizeof(buf), "%d", MAX_us);
    OLED_PrintString(80, thrline - 12, buf, &font15x15, OLED_COLOR_NORMAL);
    OLED_PrintString(80, fline - 12, Motor_GetReverse() ? "ON" : "OFF", &font15x15, OLED_COLOR_NORMAL);
}

void fun_d22(void) {
    char buf[16];
    OLED_PrintString(0, twoline - 12, ">", &font15x15, OLED_COLOR_NORMAL);
    OLED_PrintString(nonselec, oneline - 12, "总步数", &font15x15, OLED_COLOR_NORMAL);
    OLED_PrintString(selec, twoline - 12, "减速比", &font15x15, OLED_COLOR_NORMAL);
    OLED_PrintString(nonselec, thrline - 12, "最短脉冲", &font15x15, OLED_COLOR_NORMAL);
    OLED_PrintString(nonselec, fline - 12, "电机反转", &font15x15, OLED_COLOR_NORMAL);

    snprintf(buf, sizeof(buf), "%d", Motor_total_step);
    OLED_PrintString(80, oneline - 12, buf, &font15x15, OLED_COLOR_NORMAL);
    snprintf(buf, sizeof(buf), "%d", Dece_ratio);
    OLED_PrintString(80, twoline - 12, buf, &font15x15, OLED_COLOR_NORMAL);
    snprintf(buf, sizeof(buf), "%d", MAX_us);
    OLED_PrintString(80, thrline - 12, buf, &font15x15, OLED_COLOR_NORMAL);
    OLED_PrintString(80, fline - 12, Motor_GetReverse() ? "ON" : "OFF", &font15x15, OLED_COLOR_NORMAL);
}

void fun_d23(void) {
    char buf[16];
    OLED_PrintString(0, thrline - 12, ">", &font15x15, OLED_COLOR_NORMAL);
    OLED_PrintString(nonselec, oneline - 12, "总步数", &font15x15, OLED_COLOR_NORMAL);
    OLED_PrintString(nonselec, twoline - 12, "减速比", &font15x15, OLED_COLOR_NORMAL);
    OLED_PrintString(selec, thrline - 12, "最短脉冲", &font15x15, OLED_COLOR_NORMAL);
    OLED_PrintString(nonselec, fline - 12, "电机反转", &font15x15, OLED_COLOR_NORMAL);

    snprintf(buf, sizeof(buf), "%d", Motor_total_step);
    OLED_PrintString(80, oneline - 12, buf, &font15x15 , OLED_COLOR_NORMAL);
    snprintf(buf, sizeof(buf), "%d", Dece_ratio);
    OLED_PrintString(80, twoline - 12, buf, &font15x15, OLED_COLOR_NORMAL);
    snprintf(buf, sizeof(buf), "%d", MAX_us);
    OLED_PrintString(80, thrline - 12, buf, &font15x15, OLED_COLOR_NORMAL);
    OLED_PrintString(80, fline - 12, Motor_GetReverse() ? "ON" : "OFF", &font15x15, OLED_COLOR_NORMAL);
}

void fun_d24(void) {
    char buf[16];

    OLED_PrintString(0, fline - 12, ">", &font15x15, OLED_COLOR_NORMAL);
    OLED_PrintString(nonselec, oneline - 12, "总步数", &font15x15, OLED_COLOR_NORMAL);
    OLED_PrintString(nonselec, twoline - 12, "减速比", &font15x15, OLED_COLOR_NORMAL);
    OLED_PrintString(nonselec, thrline - 12, "最短脉冲", &font15x15, OLED_COLOR_NORMAL);
    OLED_PrintString(selec, fline - 12, "电机反转", &font15x15, OLED_COLOR_NORMAL);

    snprintf(buf, sizeof(buf), "%d", Motor_total_step);
    OLED_PrintString(80, oneline - 12, buf, &font15x15, OLED_COLOR_NORMAL);
    snprintf(buf, sizeof(buf), "%d", Dece_ratio);
    OLED_PrintString(80, twoline - 12, buf, &font15x15, OLED_COLOR_NORMAL);
    snprintf(buf, sizeof(buf), "%d", MAX_us);
    OLED_PrintString(80, thrline - 12, buf, &font15x15, OLED_COLOR_NORMAL);
    OLED_PrintString(80, fline - 12, Motor_GetReverse() ? "ON" : "OFF", &font15x15, OLED_COLOR_NORMAL);
}

void fun_d25(void) {
    OLED_PrintString(0, oneline - 12, ">", &font15x15, OLED_COLOR_NORMAL);
    OLED_PrintString(selec, oneline - 12, "返回", &font15x15, OLED_COLOR_NORMAL);
}

/* ========== Third Layer Menu Functions ========== */
void fun_a31(void) {
    char buf[16];
    OLED_PrintString(0, oneline - 12, ">", &font15x15, OLED_COLOR_NORMAL);
    OLED_PrintString(selec, oneline - 12, "曝光时间", &font15x15, OLED_COLOR_NORMAL);
    OLED_PrintString(nonselec, twoline - 12, "快门数", &font15x15, OLED_COLOR_NORMAL);
    OLED_PrintString(nonselec, thrline - 12, "间隔时间", &font15x15, OLED_COLOR_NORMAL);
    OLED_PrintString(nonselec, fline - 12, "启动", &font15x15, OLED_COLOR_NORMAL);

    snprintf(buf, sizeof(buf), "%d", Ex_time);
    OLED_PrintString(80, oneline - 12, buf, &font15x15, OLED_COLOR_NORMAL);
    snprintf(buf, sizeof(buf), "%d", Shutter_num);
    OLED_PrintString(80, twoline - 12, buf, &font15x15, OLED_COLOR_NORMAL);
    snprintf(buf, sizeof(buf), "%d", Interval_time);
    OLED_PrintString(80, thrline - 12, buf, &font15x15, OLED_COLOR_NORMAL);
}

void fun_a32(void) {
    char buf[16];
    OLED_PrintString(0, twoline - 12, ">", &font15x15, OLED_COLOR_NORMAL);
    OLED_PrintString(nonselec, oneline - 12, "曝光时间", &font15x15, OLED_COLOR_NORMAL);
    OLED_PrintString(selec, twoline - 12, "快门数", &font15x15, OLED_COLOR_NORMAL);
    OLED_PrintString(nonselec, thrline - 12, "间隔时间", &font15x15, OLED_COLOR_NORMAL);
    OLED_PrintString(nonselec, fline - 12, "启动", &font15x15, OLED_COLOR_NORMAL);

    snprintf(buf, sizeof(buf), "%d", Ex_time);
    OLED_PrintString(80, oneline - 12, buf, &font15x15, OLED_COLOR_NORMAL);
    snprintf(buf, sizeof(buf), "%d", Shutter_num);
    OLED_PrintString(80, twoline - 12, buf, &font15x15, OLED_COLOR_NORMAL);
    snprintf(buf, sizeof(buf), "%d", Interval_time);
    OLED_PrintString(80, thrline - 12, buf, &font15x15, OLED_COLOR_NORMAL);
}

void fun_a33(void) {
    char buf[16];
    OLED_PrintString(0, thrline - 12, ">", &font15x15, OLED_COLOR_NORMAL);
    OLED_PrintString(nonselec, oneline - 12, "曝光时间", &font15x15, OLED_COLOR_NORMAL);
    OLED_PrintString(nonselec, twoline - 12, "快门数", &font15x15, OLED_COLOR_NORMAL);
    OLED_PrintString(selec, thrline - 12, "间隔时间", &font15x15, OLED_COLOR_NORMAL);
    OLED_PrintString(nonselec, fline - 12, "启动", &font15x15, OLED_COLOR_NORMAL);

    snprintf(buf, sizeof(buf), "%d", Ex_time);
    OLED_PrintString(80, oneline - 12, buf, &font15x15, OLED_COLOR_NORMAL);
    snprintf(buf, sizeof(buf), "%d", Shutter_num);
    OLED_PrintString(80, twoline - 12, buf, &font15x15, OLED_COLOR_NORMAL);
    snprintf(buf, sizeof(buf), "%d", Interval_time);
    OLED_PrintString(80, thrline - 12, buf, &font15x15, OLED_COLOR_NORMAL);
}

void fun_a34(void) {
    char buf[16];
    OLED_PrintString(0, fline - 12, ">", &font15x15, OLED_COLOR_NORMAL);
    OLED_PrintString(nonselec, oneline - 12, "曝光时间", &font15x15, OLED_COLOR_NORMAL);
    OLED_PrintString(nonselec, twoline - 12, "快门数", &font15x15, OLED_COLOR_NORMAL);
    OLED_PrintString(nonselec, thrline - 12, "间隔时间", &font15x15, OLED_COLOR_NORMAL);
    OLED_PrintString(selec, fline - 12, "启动", &font15x15, OLED_COLOR_NORMAL);

    snprintf(buf, sizeof(buf), "%d", Ex_time);
    OLED_PrintString(80, oneline - 12, buf, &font15x15, OLED_COLOR_NORMAL);
    snprintf(buf, sizeof(buf), "%d", Shutter_num);
    OLED_PrintString(80, twoline - 12, buf, &font15x15, OLED_COLOR_NORMAL);
    snprintf(buf, sizeof(buf), "%d", Interval_time);
    OLED_PrintString(80, thrline - 12, buf, &font15x15, OLED_COLOR_NORMAL);
}

void fun_a35(void) {
    OLED_PrintString(0, oneline - 12, ">", &font15x15, OLED_COLOR_NORMAL);
    OLED_PrintString(selec, oneline - 12, "返回", &font15x15, OLED_COLOR_NORMAL);

}

void fun_b31(void) {
    char buf[16];
    OLED_PrintString(0, oneline - 12, "方向", &font15x15, OLED_COLOR_NORMAL);
    OLED_PrintString(0, twoline - 12, "旋转...", &font15x15, OLED_COLOR_NORMAL);
    OLED_PrintString(0, thrline - 12, "曝光剩余", &font15x15, OLED_COLOR_NORMAL);
    OLED_PrintString(0, fline - 12, "张数剩余", &font15x15, OLED_COLOR_NORMAL);

    OLED_PrintString(40, oneline - 12,
                     Get_Dir_State() ? "逆" : "顺",
                     &font15x15, OLED_COLOR_NORMAL);
    snprintf(buf, sizeof(buf), "%d", (uint16_t)angle_p);
    OLED_PrintString(64, twoline - 12, buf, &font15x15, OLED_COLOR_NORMAL);
    OLED_PrintString(64 + 24, twoline - 12, "%", &font15x15, OLED_COLOR_NORMAL);
    snprintf(buf, sizeof(buf), "%d", Ex_time);
    OLED_PrintString(64, thrline - 12, buf, &font15x15, OLED_COLOR_NORMAL);
    snprintf(buf, sizeof(buf), "%d", Shutter_n);
    OLED_PrintString(64, fline - 12, buf, &font15x15, OLED_COLOR_NORMAL);
}


/* ========== Fourth Layer Menu Functions ========== */
void fun_a41(void) {
    char buf[16];
    OLED_PrintString(0, oneline - 12, "方向", &font15x15, OLED_COLOR_NORMAL);
    OLED_PrintString(54, oneline - 12, "速度", &font15x15, OLED_COLOR_NORMAL);
    OLED_PrintString(0, twoline - 12, "曝光剩余", &font15x15, OLED_COLOR_NORMAL);
    OLED_PrintString(0, thrline - 12, "间隔剩余", &font15x15, OLED_COLOR_NORMAL);
    OLED_PrintString(0, fline - 12, "张数剩余", &font15x15, OLED_COLOR_NORMAL);

    OLED_PrintString(34, oneline - 12, N_S[Get_Dir_State()], &font15x15, OLED_COLOR_NORMAL);
    OLED_PrintString(86, oneline - 12, EQ_speedtab_UI[speedtabi], &font15x15, OLED_COLOR_NORMAL);
    snprintf(buf, sizeof(buf), "%d", Ex_time);
    OLED_PrintString(64, twoline - 12, buf, &font15x15, OLED_COLOR_NORMAL);
    snprintf(buf, sizeof(buf), "%d", Interval_time);
    OLED_PrintString(64, thrline - 12, buf, &font15x15, OLED_COLOR_NORMAL);
    snprintf(buf, sizeof(buf), "%d", Shutter_n);
    OLED_PrintString(64, fline - 12, buf, &font15x15, OLED_COLOR_NORMAL);
}
