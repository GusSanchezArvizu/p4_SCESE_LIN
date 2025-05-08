/*
 * led_rgb.c
 *
 *  Created on: 21 mar. 2025
 *      Author: Gustavo Sanchez
 */
#include "led_rgb.h"
#include "fsl_gpio.h"
#include "fsl_io_mux.h"
#include "pin_mux.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define APP_RGB_PORT BOARD_LED_BLUE_GPIO_PORT | BOARD_LED_RED_GPIO_PORT | BOARD_LED_GREEN_GPIO_PORT
#define APP_RGB_PIN BOARD_LED_BLUE_GPIO_PIN | BOARD_LED_RED_GPIO_PIN | BOARD_LED_GREEN_GPIO_PIN
/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
uint32_t port_state = 0;
/*******************************************************************************
 * Code
 ******************************************************************************/
void init_rgb_led()
{
    gpio_pin_config_t leds_config = {
        .pinDirection = kGPIO_DigitalOutput,
        .outputLogic = 0U
    };

	/* Init output BLUE LED GPIO. */
	GPIO_PortInit(GPIO, 0U);

	GPIO_PinInit(GPIO, BOARD_LED_BLUE_GPIO_PORT, BOARD_LED_BLUE_GPIO_PIN, &leds_config);
    /* Initialize GPIO0 functionality on pin GPIO_0 (pin D12) */
    IO_MUX_SetPinMux(IO_MUX_GPIO0);
	/* Init output RED LED GPIO. */
	GPIO_PinInit(GPIO, BOARD_LED_RED_GPIO_PORT, BOARD_LED_RED_GPIO_PIN, &leds_config);
    /* Initialize GPIO0 functionality on pin GPIO_1 (pin D12) */
	IO_MUX_SetPinMux(IO_MUX_GPIO1);
	/* Init output GREEN LED GPIO. */
	GPIO_PinInit(GPIO, BOARD_LED_GREEN_GPIO_PORT, BOARD_LED_GREEN_GPIO_PIN, &leds_config);
    /* Initialize GPIO0 functionality on pin GPIO_0 (pin D12) */
	IO_MUX_SetPinMux(IO_MUX_GPIO12);
}

void rgb_led_turn_all_off()
{
	LED_RED_OFF();
	LED_BLUE_OFF();
	LED_GREEN_OFF();
}

void rgb_led_color_WHITE()
{
	LED_RED_ON();
	LED_BLUE_ON();
	LED_GREEN_ON();
}

void rgb_led_color_YELLOW()
{
	LED_RED_ON();
	LED_BLUE_OFF();
	LED_GREEN_ON();
}

void rgb_led_color_MAGENTA()
{
	LED_RED_ON();
	LED_BLUE_ON();
	LED_GREEN_OFF();
}

void rgb_led_color_CYAN()
{
	LED_RED_OFF();
	LED_BLUE_ON();
	LED_GREEN_ON();
}
