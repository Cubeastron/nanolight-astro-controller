/**
 * @file focuser_ui.h
 * @brief 事件驱动的 OLED 菜单、参数编辑和显示电源控制接口。
 *
 * UI_Turn() 消费消抖后的按键事件、更新菜单页并通过 EEPROM_Save() 保存参数。
 * OK 按住 1 秒关闭 OLED；下一次按键下降沿只负责唤醒，不执行该按键的原操作。
 */
#ifndef __FOCUSER_UI_H
#define __FOCUSER_UI_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

#define oneline 14
#define twoline 30
#define thrline 46
#define fline   62
#define cursor_row 0
#define nonselec 18
#define selec 15

typedef unsigned char uchar;

typedef struct {
    uchar current;
    uchar up;
    uchar down;
    uchar enter;
    void (*current_operation)(void);
    int eeprom_num;
} key_table;

extern key_table table[32];
extern uchar func_index;
extern void (*current_operation_index)(void);


void UI_Init(void);
void UI_Refresh(void);
void UI_Turn(void);
/* 处理 OLED 休眠和唤醒事件；返回 false 时禁止执行普通按键操作。 */
bool UI_ServiceDisplayPower(void);
bool UI_IsDisplayOn(void);
bool UI_ButtonsEnabled(void);

/* 一级菜单绘制函数 */
void fun_a1(void);
void fun_b1(void);
void fun_c1(void);
void fun_d1(void);

/* 二级菜单绘制函数 */
void fun_a21(void);
void fun_a22(void);
void fun_b21(void);
void fun_b22(void);
void fun_b23(void);
void fun_b24(void);
void fun_b25(void);
void fun_b26(void);
void fun_c21(void);
void fun_c22(void);
void fun_c23(void);
void fun_d21(void);
void fun_d22(void);
void fun_d23(void);
void fun_d24(void);
void fun_d25(void);

/* 三级菜单绘制函数 */
void fun_a31(void);
void fun_a32(void);
void fun_a33(void);
void fun_a34(void);
void fun_a35(void);
void fun_b31(void);


/* 四级菜单绘制函数 */
void fun_a41(void);


#ifdef __cplusplus
}
#endif

#endif /* __FOCUSER_UI_H */
