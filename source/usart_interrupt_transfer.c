/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include "LIN.h"
#include "board.h"
#include "app.h"
#include "fsl_usart.h"
#include "led_rgb.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define TEST 1
/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/* USART user callback */
void USART_UserCallback(USART_Type *base, usart_handle_t *handle, status_t status, void *userData);

/*******************************************************************************
 * Variables
 ******************************************************************************/
usart_handle_t g_usartHandle;

uint8_t g_tipString[] =
    "Usart interrupt example\r\nBoard receives 8 characters then sends them out\r\nNow please input:\r\n";

uint8_t g_txBuffer[ECHO_BUFFER_LENGTH] = {0};
uint8_t g_rxBuffer[ECHO_BUFFER_LENGTH] = {0};
volatile bool rxBufferEmpty            = true;
volatile bool txBufferFull             = false;
volatile bool txOnGoing                = false;
volatile bool rxOnGoing                = false;
volatile bool txFinished;
volatile bool rxFinished;
uint8_t receivedID;

volatile bool lin_headerReceived = false;

/*******************************************************************************
 * Code
 ******************************************************************************/
/* USART user callback */
void USART_UserCallback(USART_Type *base, usart_handle_t *handle, status_t status, void *userData)
{
    userData = userData;

    if (kStatus_USART_TxIdle == status)
    {
        txFinished = true;
        txBufferFull = false;
        txOnGoing    = false;
    }

    if (kStatus_USART_RxIdle == status)
    {
        rxFinished = true;
        rxBufferEmpty = false;
        rxOnGoing     = false;
    }
}

/*!
 * @brief Main function
 */
int main(void)
{
    usart_config_t config;
    usart_transfer_t xfer;
    usart_transfer_t sendXfer;
    usart_transfer_t receiveXfer;

    BOARD_InitHardware();
    rgb_led_init();

    /*
     * config.baudRate_Bps = 115200U;
     * config.parityMode = kUSART_ParityDisabled;
     * config.stopBitCount = kUSART_OneStopBit;
     * config.loopback = false;
     * config.enableTx = false;
     * config.enableRx = false;
     */
    USART_GetDefaultConfig(&config);
    config.baudRate_Bps = BOARD_DEBUG_UART_BAUDRATE;
    config.enableTx     = true;
    config.enableRx     = true;
    config.linMode		= (1U);
    config.linTxBreak	= (1U);
    config.linAutobaud	= (0U);

    USART_Init(DEMO_USART, &config, DEMO_USART_CLK_FREQ);
    USART_TransferCreateHandle(DEMO_USART, &g_usartHandle, USART_UserCallback, NULL);

    receiveXfer.data     = g_rxBuffer;
    receiveXfer.dataSize = sizeof(g_rxBuffer);

    uint8_t payload[] = { 0xA1, 0xB2, 0xC3 };
	uint32_t deltaRxBrk;

    LIN_Frame_t myFrame;
    myFrame.identifier = 0x12;
    myFrame.data_length = 3U;
    memcpy(myFrame.data, payload, myFrame.data_length);

    while (1)
    {
    	deltaRxBrk = (USART0->STAT & USART_STAT_DELTARXBRK_MASK) >> USART_STAT_DELTARXBRK_SHIFT;
    	if (deltaRxBrk) {
    		lin_headerReceived = true;
    		rgb_led_turn_RED(LOGIC_LED_TOOGLE);
    		USART_ReadBlocking(USART1, &receivedID, 1);

    	} else {
    		lin_headerReceived = false;
    	    rxFinished = false;

//    	    // Wait receive finished.
//    	    while (!rxFinished)
//    	    {
//    	    }
    	}

#if (TEST)
        // Prepare to send.
    	for (uint8_t i = 1; i <= 5; i++) {
    		txFinished = false;
			uint8_t full_id = LIN_CalculateID(i);
			LIN_SendHeader(full_id);
			USART_WriteBlocking(USART0, &full_id, 1);
			for(int i=0; i< 10000000; i++); //Delay
    	    }
#endif
//        /* If RX is idle and g_rxBuffer is empty, start to read data to g_rxBuffer. */
//        if ((!rxOnGoing) && rxBufferEmpty)
//        {
//            rxOnGoing = true;
//            USART_TransferReceiveNonBlocking(DEMO_USART, &g_uartHandle, &receiveXfer, NULL);
//        }
//
//        /* If TX is idle and g_txBuffer is full, start to send data. */
//        if ((!txOnGoing) && txBufferFull)
//        {
//            txOnGoing = true;
//            USART_TransferSendNonBlocking(DEMO_USART, &g_uartHandle, &sendXfer);
//        }
//
//        /* If g_txBuffer is empty and g_rxBuffer is full, copy g_rxBuffer to g_txBuffer. */
//        if ((!rxBufferEmpty) && (!txBufferFull))
//        {
//            memcpy(g_txBuffer, g_rxBuffer, ECHO_BUFFER_LENGTH);
//            rxBufferEmpty = true;
//            txBufferFull  = true;
//        }
    }
}
