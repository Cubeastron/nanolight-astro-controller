/**
 * @file app_config.h
 * @brief Runtime parameters and Flash-backed configuration for NanoLight.
 *
 * EEPROM_* functions operate on a RAM cache and persist the complete record
 * to the final Flash page when the UI explicitly requests a save.
 */
#ifndef __APP_CONFIG_H
#define __APP_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* STM32F103C8T6: 64 KiB Flash; reserve the final 1 KiB page for settings. */
#define FLASH_EEPROM_BASE_ADDR    0x0800FC00  /* Page 63 start address */
#define FLASH_EEPROM_PAGE_SIZE    1024        /* 1KB per page */
#define FLASH_EEPROM_MAGIC        0x4E4C      /* "NL" - NanoLight magic number */
#define FLASH_EEPROM_VERSION      0x0002      /* Flash 记录结构版本 */

/* Keep this on-flash layout stable when changing FLASH_EEPROM_VERSION. */
typedef struct __attribute__((packed)){
    uint16_t magic;           /* Magic number for validation */
    uint16_t version;         /* Data structure version */
    uint16_t checksum;        /* Checksum for data integrity */
    /* Configuration parameters */
    uint16_t Ex_time;         /* Exposure time (seconds) */
    uint16_t Interval_time;   /* Interval time (seconds) */
    uint16_t Shutter_num;     /* Shutter count */
    uint16_t Rota_angle;      /* Rotation angle */
    uint16_t Motor_total_step;/* Motor total steps */
    uint16_t Dece_ratio;      /* Gear reduction ratio */
    uint16_t MAX_us;          /* Minimum pulse width */
    uint16_t focuser_position;/* Focuser current position */
    uint16_t Motor_reverse;   /* 电机方向极性：0=正常，1=反转 */
} EEPROM_Data_t;

/* Legacy byte offsets used by the menu table and older firmware. */
#define Ex_time_eeprom           0
#define Interval_time_eeprom     2
#define Shutter_num_eeprom       4
#define Rota_angle_eeprom        6
#define Motor_total_step_eeprom  8
#define Dece_ratio_eeprom        10
#define MAX_us_eeprom            12
#define focuser_position_eeprom  14
#define Motor_reverse_eeprom     16

/* ========== Global Variables ========== */
extern volatile uint16_t Ex_time;
extern volatile uint16_t Interval_time;
extern volatile uint16_t Shutter_num;
extern volatile uint16_t Shutter_n;
extern volatile uint16_t Rota_angle;
extern volatile uint16_t Motor_total_step;
extern volatile uint16_t Dece_ratio;
extern volatile uint16_t MAX_us;
extern volatile uint16_t Motor_position;
extern volatile uint16_t Motor_reverse;
extern volatile bool focuser_position_dirty;

/* ========== EQ Mode Variables ========== */
extern char *EQ_speedtab_UI[4];
extern long EQ_speedtab[4];
extern uint8_t speedtabi;
extern char *N_S[2];
extern float starspeed;

/* ========== Shutter Variables ========== */
extern bool ifex;

/* ========== Gimbal Variables ========== */
#define focustime 0
extern bool if_pulseend;
extern long pulseCount;
extern unsigned long angle_pulsen;
extern unsigned long period_Gimbal;
extern float angle_p;

/* ========== Focuser Variables ========== */
extern bool outputEnabled;
extern uint8_t cursor_p;
extern uint8_t speedtabi_2;
extern uint8_t extime_fc;
extern char *FC_speedtab_UI[4];
extern long FC_speedtab[4];
extern char *Z_F[2];

/* ========== UI Variables ========== */
extern uint16_t current_operation_data;

/* ========== Function Prototypes ========== */
void App_Init(void);
void App_UpdateSpeedTables(void);

/* EEPROM Functions */
void EEPROM_Write(uint16_t addr, uint16_t data);
uint16_t EEPROM_Read(uint16_t addr);
//void EEPROM_Write32(uint16_t addr, uint32_t data);
//uint32_t EEPROM_Read32(uint16_t addr);

/* Flash EEPROM Persistence Functions */
bool EEPROM_Save(void);           /* Save all data to Flash */
bool EEPROM_Load(void);           /* Load all data from Flash */
void EEPROM_Format(void);         /* Format Flash storage with defaults */

/* Debug Functions */
uint16_t EEPROM_GetFlashMagic(void);
uint16_t EEPROM_GetFlashExTime(void);
uint16_t EEPROM_GetFlashChecksum(void);

#ifdef __cplusplus
}
#endif

#endif /* __APP_CONFIG_H */
