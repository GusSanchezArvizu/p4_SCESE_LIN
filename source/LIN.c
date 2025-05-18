/*
 * LIN.c
 *
 *  Created on: 7 may. 2025
 *      Author: Gustavo Sanchez
 */
#include "LIN.h"
#include "fsl_usart.h"
#include "RW612_COMMON.h"
#include "led_rgb.h"
#include "app.h"

usart_handle_t g_usartHandle;
/***
 * TX
***/
usart_transfer_t sendXfer;
volatile bool txFinished;
/***
 *RX
***/
usart_transfer_t receiveXfer;
volatile bool rxFinished;
uint8_t receiveData[1] = {0};

/* USART user callback */
void USART_UserCallback(USART_Type *base, usart_handle_t *handle, status_t status, void *userData)
{
    userData = userData;

    if (kStatus_USART_TxIdle == status)
    {
        txFinished = true;
    }

    if (kStatus_USART_RxIdle == status)
    {

        rxFinished = true;
    }
}
void LIN_init()
{
    usart_config_t config;
	/*
	 * config.baudRate_Bps = 115200U;
	 * config.parityMode = kUSART_ParityDisabled;
	 * config.stopBitCount = kUSART_OneStopBit;
	 * config.loopback = false;
	 * config.enableTx = false;
	 * config.enableRx = false;
	 */
	USART_GetDefaultConfig(&config);
	config.baudRate_Bps = 19200;
	config.enableTx     = true;
	config.enableRx     = true;
	config.linMode		= (1U);
	config.linTxBreak	= (1U);
	config.linAutobaud	= (0U);

	USART_Init(DEMO_USART, &config, DEMO_USART_CLK_FREQ);
	USART_TransferCreateHandle(DEMO_USART, &g_usartHandle, USART_UserCallback, NULL);
}

/*!
 * @fn LIN_tx_status_t LIN_tx(LIN_Frame_t)
 * @brief Sends frame via USART initialized
 *
 * @param frame
 * @return LIN_tx_status_t
 */
LIN_tx_status_t LIN_tx(LIN_Frame_t frame)
{
    uint8_t sendData[LIN_MAX_DATA_LENGTH + 1];  // Datos + Checksum

    for (uint8_t i = 0; i < frame.data_length; i++) {
        sendData[i] = frame.data[i];
    }
    sendData[frame.data_length] = LIN_check(frame);
    sendXfer.data = sendData;
    sendXfer.dataSize = frame.data_length + 1;
    txFinished = false;

    if (USART_TransferSendNonBlocking(USART0, &g_usartHandle, &sendXfer) != kStatus_Success)
        return LIN_TX_SENT_FAILURE;

    while (!txFinished) {
        __NOP();
    }

    return LIN_TX_SENT_CORRECTLY;
}
/*!
 * @fn void LIN_SendHeader(uint8_t)
 * @brief Sends header before data
 *
 * @param id
 */
void LIN_SendHeader(uint8_t id) {
    // Paso 1: Deshabilita transmisor
    //USART0->CTL |= (1 << 6); // TXDIS
    //while (!(USART0->STAT & (1 << 6))); // Espera TXDISSTAT

    // Paso 2: Enviar break
    USART0->CTL |= (1 << 1); // TXBRKEN
    USART0->FIFOWR = 0xFF;
    while (!(USART0->STAT & (1 << 3))); // Espera TXIDLE
    USART0->CTL &= ~(1 << 1); // Fin break

    // Paso 3: Rehabilita transmisor
    USART0->CTL &= ~(1 << 6); // TXDIS = 0
    while (USART0->STAT & (1 << 6)); // Espera TXDISSTAT = 0

    // Paso 4: Enviar SYNC (0x55)
    USART0->FIFOWR = 0x55;
    while (!(USART0->STAT & (1 << 3))); // Espera TXIDLE

    // Paso 5: Enviar ID (con bits de paridad incluidos)
    USART0->FIFOWR = id;
    while (!(USART0->STAT & (1 << 3))); // Espera TXIDLE
}

/*!
 * @fn uint8_t LIN_CalculateID(uint8_t)
 * @brief Makes the operation for id and parity bits
 *
 * @param id
 * @return
 */
uint8_t LIN_CalculateID(uint8_t id) {
    id &= 0x3F;  // Asegura que solo los bits ID0–ID5 estén activos

    uint8_t id0 = (id >> 0) & 0x01;
    uint8_t id1 = (id >> 1) & 0x01;
    uint8_t id2 = (id >> 2) & 0x01;
    uint8_t id3 = (id >> 3) & 0x01;
    uint8_t id4 = (id >> 4) & 0x01;
    uint8_t id5 = (id >> 5) & 0x01;

    uint8_t p0 = id0 ^ id1 ^ id2 ^ id4;
    uint8_t p1 = ~(id1 ^ id3 ^ id4 ^ id5) & 0x01;

    return id | (p0 << 6) | (p1 << 7);
}


/*!
 * @fn uint8_t LIN_check(LIN_Frame_t)
 * @brief Obtain the checksum of the sent frame
 *
 * @param frame
 * @return
 */
uint8_t LIN_check(LIN_Frame_t frame)
{
    uint16_t sum = frame.identifier;  // Inicia con el ID incluido (mejorado)

    for (uint8_t i = 0; i < frame.data_length; i++) {
        sum += frame.data[i];
        if (sum > 255)
            sum -= 255;  // Implementa suma modular 255 (carry-around)
    }

    return (uint8_t)(~sum);  // Complemento a uno
}

void LIN_HandlerHeader(uint8_t receivedId) {

	switch(receivedId)
	{
	case 1:
		rgb_led_color_WHITE();
		break;
	case 2:
	    LIN_Frame_t slave2;
	    slave2.identifier = 0x40;               // Ejemplo de ID
	    slave2.data_length = 2;
	    slave2.data[0] = 0x22;                  // Datos de ejemplo
	    slave2.data[1] = 0x11;
	    LIN_tx(slave2);
		break;
	case 3:
	    LIN_Frame_t slave3;
	    slave3.identifier = 0x41;               // Ejemplo de ID
	    slave3.data_length = 2;
	    slave3.data[0] = 0x33;                  // Datos de ejemplo
	    slave3.data[1] = 0xAC;
	    LIN_tx(slave3);
		break;
	case 4:
		rgb_led_color_MAGENTA();
		break;
	case 5:
		rgb_led_color_YELLOW();
		break;
	default:
		break;
	}
}
