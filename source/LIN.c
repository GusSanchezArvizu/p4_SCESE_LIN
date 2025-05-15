/*
 * LIN.c
 *
 *  Created on: 7 may. 2025
 *      Author: Gustavo Sanchez
 */
#include "LIN.h"
#include "fsl_usart.h"
#include "RW612_COMMON.h"

/*!
 * @fn LIN_tx_status_t LIN_tx(LIN_Frame_t)
 * @brief Sends frame via USART initialized
 *
 * @param frame
 * @return LIN_tx_status_t
 */
LIN_tx_status_t LIN_tx(LIN_Frame_t frame)
{

    if (!(frame.data_length == 0 || frame.data_length > 8))
    {
        // 1. Calcular ID con paridad
        uint8_t lin_id = LIN_CalculateID(frame.identifier);

        // 2. Enviar encabezado: break + sync + ID
        //LIN_SendHeader(lin_id);

        // 3. Enviar datos
        for (uint8_t i = 0; i < frame.data_length; i++) {
            USART0->FIFOWR = frame.data[i];
            while (!(USART0->STAT & (1 << 3))); // Esperar TXIDLE
        }

        // 4. Calcular y enviar checksum
        uint8_t checksum = LIN_check(frame);
        USART0->FIFOWR = checksum;
        while (!(USART0->STAT & (1 << 3))); // Esperar TXIDLE
    }

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
