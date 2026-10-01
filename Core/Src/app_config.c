/**
 * @file app_config.c
 * @brief 默认参数、速度表计算和 Flash 持久化实现。
 */
#include "app_config.h"
#include "focuser_motor.h"
#include "stm32f1xx_hal.h"
#include <string.h>
#include <math.h>

/* ========== Global Variables ========== */
volatile uint16_t Ex_time = 4;
volatile uint16_t Interval_time = 2;
volatile uint16_t Shutter_num = 3;
volatile uint16_t Shutter_n = 3;
volatile uint16_t Rota_angle = 3;
volatile uint16_t Motor_total_step = 48;
volatile uint16_t Dece_ratio = 972;
volatile uint16_t MAX_us = 100;
volatile uint16_t Motor_position = 30000;
volatile uint16_t Motor_reverse = 0;
volatile bool focuser_position_dirty = false;

/* ========== EQ Mode Variables ========== */
char *EQ_speedtab_UI[4] = {" *", " Moon", "100*", " MAX"};
long EQ_speedtab[4] = {0, 0, 0, 0};
uint8_t speedtabi = 0;
char *N_S[2] = {"南", "北"};
float starspeed = 0;

/* ========== Shutter Variables ========== */
bool ifex = false;

/* ========== Gimbal Variables ========== */
bool if_pulseend = true;
long pulseCount = 0;
unsigned long angle_pulsen = 0;
unsigned long period_Gimbal = 0;
float angle_p = 0;

/* ========== Focuser Variables ========== */
bool outputEnabled = false;
uint8_t cursor_p = 128;
uint8_t speedtabi_2 = 0;
uint8_t extime_fc = 0;
char *FC_speedtab_UI[4] = {"1/100", "1/50", "1/10", "MAX"};
long FC_speedtab[4] = {0, 0, 0, 0};
char *Z_F[2] = {"<", ">"};

/* ========== UI Variables ========== */
uint16_t current_operation_data = 0;

/* ========== EEPROM RAM Cache ========== */
static EEPROM_Data_t eeprom_cache;
static bool eeprom_needs_upgrade = false;

/* 版本 1 的布局仅用于升级旧数据，新字段必须继续追加在当前记录末尾。 */
typedef struct __attribute__((packed)) {
    uint16_t magic;
    uint16_t version;
    uint16_t checksum;
    uint16_t Ex_time;
    uint16_t Interval_time;
    uint16_t Shutter_num;
    uint16_t Rota_angle;
    uint16_t Motor_total_step;
    uint16_t Dece_ratio;
    uint16_t MAX_us;
    uint16_t focuser_position;
} EEPROM_DataV1_t;

/* ========== Private Function Prototypes ========== */
static uint16_t EEPROM_CalculateChecksum(const EEPROM_Data_t *data);
static uint16_t EEPROM_CalculateChecksumBytes(const void *data, size_t size);
static void EEPROM_SetDefaults(EEPROM_Data_t *data);


/* ========== Checksum Calculation ========== */
static uint16_t EEPROM_CalculateChecksum(const EEPROM_Data_t *data) {
    return EEPROM_CalculateChecksumBytes(data, sizeof(EEPROM_Data_t));
}

static uint16_t EEPROM_CalculateChecksumBytes(const void *data, size_t size) {
    uint16_t sum = 0;
    const uint8_t *ptr = (const uint8_t *)data;
    
    /* 不参与校验 magic、version 和 checksum，只累加配置字段。 */
    for (size_t i = 6; i < size; i++) {
        sum += ptr[i];
    }
    return sum;
}



/* ========== Load Data from Flash ========== */
bool EEPROM_Load(void) {
    EEPROM_Data_t *flash_data = (EEPROM_Data_t *)FLASH_EEPROM_BASE_ADDR;
    EEPROM_DataV1_t *flash_data_v1 = (EEPROM_DataV1_t *)FLASH_EEPROM_BASE_ADDR;

    eeprom_needs_upgrade = false;
    
    /* Check magic number */
    if (flash_data->magic != FLASH_EEPROM_MAGIC) {
        return false;
    }

    if (flash_data->version == FLASH_EEPROM_VERSION) {
        if (EEPROM_CalculateChecksum(flash_data) != flash_data->checksum) {
            return false;
        }

        memcpy(&eeprom_cache, flash_data, sizeof(EEPROM_Data_t));
        return true;
    }

    if (flash_data_v1->version != 0x0001U ||
        EEPROM_CalculateChecksumBytes(flash_data_v1, sizeof(EEPROM_DataV1_t)) != flash_data_v1->checksum) {
        return false;
    }

    /* 保留版本 1 的全部参数，新加入的电机反转使用默认关闭。 */
    eeprom_cache.magic = FLASH_EEPROM_MAGIC;
    eeprom_cache.version = FLASH_EEPROM_VERSION;
    eeprom_cache.Ex_time = flash_data_v1->Ex_time;
    eeprom_cache.Interval_time = flash_data_v1->Interval_time;
    eeprom_cache.Shutter_num = flash_data_v1->Shutter_num;
    eeprom_cache.Rota_angle = flash_data_v1->Rota_angle;
    eeprom_cache.Motor_total_step = flash_data_v1->Motor_total_step;
    eeprom_cache.Dece_ratio = flash_data_v1->Dece_ratio;
    eeprom_cache.MAX_us = flash_data_v1->MAX_us;
    eeprom_cache.focuser_position = flash_data_v1->focuser_position;
    eeprom_cache.Motor_reverse = 0U;
    eeprom_cache.checksum = EEPROM_CalculateChecksum(&eeprom_cache);
    eeprom_needs_upgrade = true;
    return true;
}

/* ========== Save Data to Flash ========== */
bool EEPROM_Save(void) {
    FLASH_EraseInitTypeDef erase_init;
    uint32_t page_error;
    uint16_t *src_ptr;
    HAL_StatusTypeDef status;
    
    /* Update cache from global variables first */
    eeprom_cache.magic = FLASH_EEPROM_MAGIC;
    eeprom_cache.version = FLASH_EEPROM_VERSION;
    eeprom_cache.Ex_time = Ex_time;
    eeprom_cache.Interval_time = Interval_time;
    eeprom_cache.Shutter_num = Shutter_num;
    eeprom_cache.Rota_angle = Rota_angle;
    eeprom_cache.Motor_total_step = Motor_total_step;
    eeprom_cache.Dece_ratio = Dece_ratio;
    eeprom_cache.MAX_us = MAX_us;
    eeprom_cache.focuser_position = Motor_GetPosition();
    eeprom_cache.Motor_reverse = Motor_reverse;
    eeprom_cache.checksum = EEPROM_CalculateChecksum(&eeprom_cache);
    
    /* Flash 擦除/写入期间禁止 TIM3 中断，避免半写入状态被打断。 */
    __disable_irq();
    
    /* Unlock Flash */
    HAL_FLASH_Unlock();
    
    /* Clear pending flags */
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR);
    
    /* Erase page */
    erase_init.TypeErase = FLASH_TYPEERASE_PAGES;
    erase_init.PageAddress = FLASH_EEPROM_BASE_ADDR;
    erase_init.NbPages = 1;
    
    status = HAL_FLASHEx_Erase(&erase_init, &page_error);
    if (status != HAL_OK) {
        HAL_FLASH_Lock();
        __enable_irq();
        return false;
    }
    
    /* Write data (half-word at a time for STM32F1) */
    src_ptr = (uint16_t *)&eeprom_cache;
    for (size_t i = 0; i < sizeof(EEPROM_Data_t) / 2; i++) {
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, 
                                   FLASH_EEPROM_BASE_ADDR + (i * 2), 
                                   src_ptr[i]);
        if (status != HAL_OK) {
            HAL_FLASH_Lock();
            __enable_irq();
            return false;
        }
    }
    
    /* Lock Flash */
    HAL_FLASH_Lock();
    
    /* Re-enable interrupts */
    __enable_irq();
    focuser_position_dirty = false;
    return true;
}

/* ========== Check if Stored Data is Valid ========== */
bool EEPROM_IsValid(void) {
    EEPROM_Data_t *flash_data = (EEPROM_Data_t *)FLASH_EEPROM_BASE_ADDR;
    
    if (flash_data->magic != FLASH_EEPROM_MAGIC) return false;
    if (flash_data->version != FLASH_EEPROM_VERSION) return false;
    if (EEPROM_CalculateChecksum(flash_data) != flash_data->checksum) return false;
    
    return true;
}

/* ========== Set Default Values ========== */
static void EEPROM_SetDefaults(EEPROM_Data_t *data) {
    memset(data, 0xFF, sizeof(EEPROM_Data_t));
    data->magic = FLASH_EEPROM_MAGIC;
    data->version = FLASH_EEPROM_VERSION;
    data->Ex_time = Ex_time;
    data->Interval_time = Interval_time;
    data->Shutter_num = Shutter_num;
    data->Rota_angle = Rota_angle;
    data->Motor_total_step = Motor_total_step;
    data->Dece_ratio = Dece_ratio;
    data->MAX_us = MAX_us;
    data->focuser_position = Motor_GetPosition();
    data->Motor_reverse = Motor_reverse;
    data->checksum = EEPROM_CalculateChecksum(data);
}

/* ========== Format Flash Storage ========== */
void EEPROM_Format(void) {
    EEPROM_SetDefaults(&eeprom_cache);

    EEPROM_Save();
}

/* ========== EEPROM Read/Write Functions ========== */
uint16_t EEPROM_Read(uint16_t addr) {
    switch (addr) {
        case Ex_time_eeprom: return eeprom_cache.Ex_time;
        case Interval_time_eeprom: return eeprom_cache.Interval_time;
        case Shutter_num_eeprom: return eeprom_cache.Shutter_num;
        case Rota_angle_eeprom: return eeprom_cache.Rota_angle;
        case Motor_total_step_eeprom: return eeprom_cache.Motor_total_step;
        case Dece_ratio_eeprom: return eeprom_cache.Dece_ratio;
        case MAX_us_eeprom: return eeprom_cache.MAX_us;
        case focuser_position_eeprom: return eeprom_cache.focuser_position;
        case Motor_reverse_eeprom: return eeprom_cache.Motor_reverse;
        default: return 0xFFFF;
    }
}

void EEPROM_Write(uint16_t addr, uint16_t data) {
    switch (addr) {
        case Ex_time_eeprom: eeprom_cache.Ex_time = data; break;
        case Interval_time_eeprom: eeprom_cache.Interval_time = data; break;
        case Shutter_num_eeprom: eeprom_cache.Shutter_num = data; break;
        case Rota_angle_eeprom: eeprom_cache.Rota_angle = data; break;
        case Motor_total_step_eeprom: eeprom_cache.Motor_total_step = data; break;
        case Dece_ratio_eeprom: eeprom_cache.Dece_ratio = data; break;
        case MAX_us_eeprom: eeprom_cache.MAX_us = data; break;
        case focuser_position_eeprom: eeprom_cache.focuser_position = data; break;
        case Motor_reverse_eeprom: eeprom_cache.Motor_reverse = data; break;
        default: break;
    }
}

/* ========== Debug: Get Flash content for diagnosis ========== */
uint16_t EEPROM_GetFlashMagic(void) {
    return *(uint16_t *)FLASH_EEPROM_BASE_ADDR;
}

uint16_t EEPROM_GetFlashExTime(void) {
    return *(uint16_t *)(FLASH_EEPROM_BASE_ADDR + 6);  /* Offset to Ex_time field */
}

uint16_t EEPROM_GetFlashChecksum(void) {
    return *(uint16_t *)(FLASH_EEPROM_BASE_ADDR + 4);  /* Offset to checksum field */
}

/* ========== Check if EEPROM area is erased (all 0xFF) ========== */
static bool EEPROM_IsErased(void) {
    uint16_t *flash_ptr = (uint16_t *)FLASH_EEPROM_BASE_ADDR;
    for (size_t i = 0; i < sizeof(EEPROM_Data_t) / 2; i++) {
        if (flash_ptr[i] != 0xFFFF) {
            return false;
        }
    }
    return true;
}

/* 加载持久化参数，必要时使用默认值，然后重新计算速度表。 */
void App_Init(void) {

    //EEPROM_Format();

    /* Initialize EEPROM cache with defaults first */
    
    /* Check if EEPROM area is erased (first boot after flash) */
    if (EEPROM_IsErased()) {

        EEPROM_Save();
    } else if (EEPROM_Load()) {
        /* Valid data found - load saved values */
        Ex_time = eeprom_cache.Ex_time;
        Interval_time = eeprom_cache.Interval_time;
        Shutter_num = eeprom_cache.Shutter_num;
        Rota_angle = eeprom_cache.Rota_angle;
        Motor_total_step = eeprom_cache.Motor_total_step;
        Dece_ratio = eeprom_cache.Dece_ratio;
        MAX_us = eeprom_cache.MAX_us;
        Motor_SetPosition(eeprom_cache.focuser_position);
        Motor_reverse = eeprom_cache.Motor_reverse != 0U ? 1U : 0U;

        if (eeprom_needs_upgrade) {
            EEPROM_Save();
        }
    } else {
        /* Data exists but is invalid - use defaults and save */
        // eeprom_dirty = true;
        EEPROM_Save();
    }
    
    /* Initialize runtime variables */
    Shutter_n = Shutter_num;
    focuser_position_dirty = false;
    
    /* Update speed tables */
    App_UpdateSpeedTables();
}

void App_UpdateSpeedTables(void) {
    /* 恒星时跟踪周期：86164 秒恒星日，按减速比、步数和 32 微步折算。 */
    starspeed = 86164.0f * 1000.0f / (float)Dece_ratio / (float)Motor_total_step / 32.0f * 1000.0f;
    
    if (floor(starspeed / 100.0f) <= MAX_us) {
        EQ_speedtab[0] = (long)floor(starspeed);
        EQ_speedtab[1] = (long)floor(starspeed * 1.03788f);
        EQ_speedtab[2] = MAX_us;
        EQ_speedtab[3] = MAX_us;
    } else {
        EQ_speedtab[0] = (long)floor(starspeed);
        EQ_speedtab[1] = (long)floor(starspeed * 1.03788f);
        EQ_speedtab[2] = (long)floor(starspeed / 100.0f);
        EQ_speedtab[3] = MAX_us;
    }
    
    /* Focuser speed table */
    FC_speedtab[0] = 100 * MAX_us;
    FC_speedtab[1] = 50 * MAX_us;
    FC_speedtab[2] = 10 * MAX_us;
    FC_speedtab[3] = MAX_us;
}
