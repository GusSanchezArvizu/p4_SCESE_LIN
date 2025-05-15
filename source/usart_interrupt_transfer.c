/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include "LIN.h"
#include "board.h"
#include "fsl_usart.h"
#include "led_rgb.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define MASTER 0
/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
volatile bool lin_headerReceived = false;

/*******************************************************************************
 * Code
 ******************************************************************************/

/*!
 * @brief Main function
 */
int main(void)
{
    uint8_t i = 1;
    BOARD_InitHardware();
    rgb_led_init();
    LIN_init();

    uint8_t payload[] = { 0xA1, 0xB2, 0xC3 };
	uint32_t deltaRxBrk;

    while (1)
    {
    	deltaRxBrk = (USART0->STAT & USART_STAT_DELTARXBRK_MASK) >> USART_STAT_DELTARXBRK_SHIFT;
    	if (deltaRxBrk) {
    		lin_headerReceived = true;
    		//rgb_led_turn_RED(LOGIC_LED_TOOGLE);
    		uint8_t id = USART_ReadByte(USART0);
    		uint8_t id6bit = id & 0x3F;
    		LIN_HandlerHeader(id6bit);
    		/* Clean Sync Break Interruption Flag */
    		USART_STAT_DELTARXBRK(0);
    	}
    	else
    	{

    	}

#if (MASTER)
    	uint8_t full_id = LIN_CalculateID(i);
		LIN_SendHeader(full_id);
		for(int i=0; i< 100000000; i++); //Delay
		(i == 5)?i=1:i++;

#endif

    }
}
