/*
 * LIN.h
 *
 *  Created on: 7 may. 2025
 *      Author: Gustavo Sanchez
 */

#ifndef LIN_H_
#define LIN_H_

#include <stdint.h>

#define LIN_MAX_DATA_LENGTH 8

typedef enum
{
	LIN_TX_SENT_CORRECTLY = 0,
	LIN_TX_SENT_FAILURE,
}LIN_tx_status_t;

typedef enum
{
	LIN_RX_RECEIVED_DATA = 0,
	LIN_RX_RECEIVED_DATA_FAIL,
}LIN_rx_status_t;

typedef enum
{
	LIN_CHK_CORRECT,
	LIN_CHK_WRONG
}LIN_checksum_status_t;

typedef struct {
    uint8_t identifier;                  // 6 bits ID + 2 bits parity → lo tratamos como un byte
    uint8_t data_length;                 // Cantidad de bytes válidos en data[]
    uint8_t data[LIN_MAX_DATA_LENGTH];   // Hasta 8 bytes de datos
    uint8_t checksum;                    // Checksum del frame
} LIN_Frame_t;

void LIN_init();

LIN_tx_status_t LIN_tx(LIN_Frame_t frame);

LIN_rx_status_t LIN_rx();

static void LIN_parse(LIN_Frame_t frame);

static uint8_t LIN_check(LIN_Frame_t frame);

#endif /* LIN_H_ */
