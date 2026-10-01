/**
 * @file focuser_protocol.c
 * @brief USB 对焦协议的流式解析、错帧恢复和发送队列。
 */
#include "focuser_protocol.h"
#include "focuser_motor.h"
#include "app_config.h"
#include "focuser_ui.h"
#include "usbd_cdc_if.h"
#include <string.h>
#include <stdio.h>
#include <stdbool.h>

volatile uint32_t focuser_position = 0;
volatile bool is_moving = false;

static uint8_t rx_buffer[64];
static volatile uint16_t rx_index = 0;
/* 仅作为解析状态提示；真正的取帧仍按长度和校验和进行。 */
static volatile bool cmd_received = false;
static bool move_response_pending = false;
static uint8_t move_response_cmd = RFCP_CMD_MOVE_OUT;
#define RFCP_TX_QUEUE_SIZE 4
static uint8_t tx_queue[RFCP_TX_QUEUE_SIZE][RFCP_RESPONSE_LEN];
static uint8_t tx_head = 0;
static uint8_t tx_tail = 0;
static uint8_t tx_count = 0;

static void RFCP_ConsumeRx(uint16_t count) {
    if (count >= rx_index) {
        rx_index = 0;
        return;
    }
    memmove(rx_buffer, &rx_buffer[count], rx_index - count);
    rx_index -= count;
}

static bool RFCP_ParseDataField(const uint8_t *dataField, uint32_t *outData) {
    uint32_t value = 0;
    for (uint8_t i = 0; i < 6; i++) {
        if (dataField[i] < '0' || dataField[i] > '9') {
            return false;
        }
        value = value * 10u + (uint32_t)(dataField[i] - '0');
    }
    *outData = value;
    return true;
}

static bool RFCP_IsAllX(const uint8_t *dataField) {
    for (uint8_t i = 0; i < 6; i++) {
        if (dataField[i] != 'X') {
            return false;
        }
    }
    return true;
}

static bool RFCP_EnqueueResponse(const uint8_t *response) {
    if (tx_count >= RFCP_TX_QUEUE_SIZE) {
        return false;
    }
    memcpy(tx_queue[tx_tail], response, RFCP_RESPONSE_LEN);
    tx_tail = (uint8_t)((tx_tail + 1u) % RFCP_TX_QUEUE_SIZE);
    tx_count++;
    return true;
}

static void RFCP_FlushTxQueue(void) {
    while (tx_count > 0) {
        if (CDC_Transmit_FS(tx_queue[tx_head], RFCP_RESPONSE_LEN) != USBD_OK) {
            break;
        }
        tx_head = (uint8_t)((tx_head + 1u) % RFCP_TX_QUEUE_SIZE);
        tx_count--;
    }
}

static bool RFCP_TryPopFrame(uint8_t *frame) {
    bool got_frame = false;

    __disable_irq();
    while (rx_index > 0 && rx_buffer[0] != 'F') {
        RFCP_ConsumeRx(1);
    }

    if (rx_index >= RFCP_CMD_LEN) {
        uint8_t checksum = RFCP_CalculateChecksum(rx_buffer, 8);
        if (checksum == rx_buffer[8]) {
            memcpy(frame, rx_buffer, RFCP_CMD_LEN);
            RFCP_ConsumeRx(RFCP_CMD_LEN);
            got_frame = true;
        } else {
            RFCP_ConsumeRx(1);
        }
    }
    cmd_received = (rx_index >= RFCP_CMD_LEN);
    __enable_irq();

    return got_frame;
}

void RFCP_Init(void) {
    /* Don't reset focuser_position here - it's set by App_Init() from EEPROM */
    is_moving = false;
    rx_index = 0;
    cmd_received = false;
    move_response_pending = false;
    tx_head = 0;
    tx_tail = 0;
    tx_count = 0;
    memset(rx_buffer, 0, sizeof(rx_buffer));
}

uint8_t RFCP_CalculateChecksum(uint8_t* data, uint8_t len) {
    uint8_t sum = 0;
    for (uint8_t i = 0; i < len; i++) {
        sum += data[i];
    }
    return sum;
}

void RFCP_SendResponse(uint8_t cmdType, uint32_t data) {
    uint8_t response[RFCP_RESPONSE_LEN];
    char data_str[7];
    
    response[0] = 'F';
    response[1] = cmdType;
    
    snprintf(data_str, 7, "%06lu", data);
    memcpy(&response[2], data_str, 6);
    
    response[8] = RFCP_CalculateChecksum(response, 8);

    if (RFCP_EnqueueResponse(response)) {
        RFCP_FlushTxQueue();
    }
}

void RFCP_ReceiveData(uint8_t* data, uint32_t len) {
    for (uint32_t i = 0; i < len; i++) {
        /* Resync when buffer is full to avoid RX deadlock. */
        if (rx_index >= sizeof(rx_buffer)) {
            rx_index = 0;
        }

        /* Ignore noise until frame start. */
        if (rx_index == 0 && data[i] != 'F') {
            continue;
        }

        rx_buffer[rx_index++] = data[i];
        if (rx_index >= RFCP_CMD_LEN) {
            cmd_received = true;
        }
    }
}

static void RFCP_HandleFrame(const uint8_t *frame) {
    switch (frame[1]) {
        case RFCP_CMD_VERSION:
            if (RFCP_IsAllX(&frame[2])) {
                /* RoboFocus/SharpCap stop command: FVXXXXXX */
                Motor_Stop();
                move_response_pending = false;
            }
            RFCP_SendResponse('V', 1);
            break;
            
        case RFCP_CMD_GET_POSITION:
            /* Return current motor position */
            RFCP_SendResponse('D', Motor_GetPosition());
            break;
            
        case RFCP_CMD_MOVE_OUT: {
            uint32_t data = 0;
            if (!RFCP_ParseDataField(&frame[2], &data)) {
                break;
            }
            if (data > 0) {
                Motor_MoveSteps(data, true);
                move_response_pending = true;
                move_response_cmd = 'O';
            } else {
                RFCP_SendResponse('O', Motor_GetPosition());
            }
            break;
        }
        case RFCP_CMD_MOVE_IN: {
            uint32_t data = 0;
            if (!RFCP_ParseDataField(&frame[2], &data)) {
                break;
            }
            if (data > 0) {
                Motor_MoveSteps(data, false);
                move_response_pending = true;
                move_response_cmd = 'I';
            } else {
                RFCP_SendResponse('I', Motor_GetPosition());
            }
            break;
        }
        case RFCP_CMD_STOP:
            Motor_Stop();
            move_response_pending = false;
            RFCP_SendResponse('V', 1);
            break;
            
        default:
            break;
    }
}

void RFCP_Process(void) {
    RFCP_FlushTxQueue();

    while (1) {
        uint8_t frame[RFCP_CMD_LEN];
        if (!RFCP_TryPopFrame(frame)) {
            break;
        }
        RFCP_HandleFrame(frame);
        RFCP_FlushTxQueue();
    }

    /* 移动完成后再异步返回当前位置，避免阻塞主循环。 */
    if (move_response_pending && !is_moving) {
        RFCP_SendResponse(move_response_cmd, Motor_GetPosition());
        move_response_pending = false;
    }

    RFCP_FlushTxQueue();
}
