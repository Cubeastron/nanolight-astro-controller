/**
 * @file focuser_protocol.h
 * @brief Nine-byte RoboFocus-compatible USB CDC protocol.
 *
 * A frame is `F`, a command byte, six ASCII data bytes and an 8-bit additive
 * checksum over the first eight bytes. RX bytes arrive from the USB callback;
 * RFCP_Process() executes complete frames from the main loop.
 */
#ifndef __FOCUSER_PROTOCOL_H
#define __FOCUSER_PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

#define RFCP_CMD_LEN 9
#define RFCP_RESPONSE_LEN 9

#define RFCP_MAX_POSITION 65535U

typedef enum {
    RFCP_CMD_VERSION = 'V',
    RFCP_CMD_GET_POSITION = 'G',
    RFCP_CMD_MOVE_OUT = 'O',
    RFCP_CMD_MOVE_IN = 'I',
    RFCP_CMD_STOP = 'S'
} RFCP_CommandType;

void RFCP_Init(void);
/** Drain queued USB TX data, parse complete frames and finish pending moves. */
void RFCP_Process(void);
void RFCP_ReceiveData(uint8_t* data, uint32_t len);
uint8_t RFCP_CalculateChecksum(uint8_t* data, uint8_t len);
void RFCP_SendResponse(uint8_t cmdType, uint32_t data);

extern volatile bool is_moving;

#ifdef __cplusplus
}
#endif

#endif
