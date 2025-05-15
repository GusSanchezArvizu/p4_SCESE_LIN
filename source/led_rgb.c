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
void rgb_led_init()
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
    GPIO_PinWrite(GPIO, BOARD_LED_BLUE_GPIO_PORT, BOARD_LED_BLUE_GPIO_PIN, LOGIC_LED_OFF);
	/* Init output RED LED GPIO. */
	GPIO_PinInit(GPIO, BOARD_LED_RED_GPIO_PORT, BOARD_LED_RED_GPIO_PIN, &leds_config);
    /* Initialize GPIO0 functionality on pin GPIO_1 (pin D12) */
	IO_MUX_SetPinMux(IO_MUX_GPIO1);
	GPIO_PinWrite(GPIO, BOARD_LED_RED_GPIO_PORT, BOARD_LED_RED_GPIO_PIN, LOGIC_LED_OFF);
	/* Init output GREEN LED GPIO. */
	GPIO_PinInit(GPIO, BOARD_LED_GREEN_GPIO_PORT, BOARD_LED_GREEN_GPIO_PIN, &leds_config);
    /* Initialize GPIO0 functionality on pin GPIO_12 (pin D12) */
	IO_MUX_SetPinMux(IO_MUX_GPIO12);
	GPIO_PinWrite(GPIO, BOARD_LED_GREEN_GPIO_PORT, BOARD_LED_GREEN_GPIO_PIN, LOGIC_LED_OFF);
}

void rgb_led_turn_all_off()
{
	GPIO_PinWrite(GPIO, BOARD_LED_BLUE_GPIO_PORT, BOARD_LED_BLUE_GPIO_PIN, LOGIC_LED_OFF);
	GPIO_PinWrite(GPIO, BOARD_LED_RED_GPIO_PORT, BOARD_LED_RED_GPIO_PIN, LOGIC_LED_OFF);
	GPIO_PinWrite(GPIO, BOARD_LED_GREEN_GPIO_PORT, BOARD_LED_GREEN_GPIO_PIN, LOGIC_LED_OFF);
}

void rgb_led_turn_RED(int state)
{
	if(LOGIC_LED_ON == state){
		GPIO_PinWrite(GPIO, BOARD_LED_RED_GPIO_PORT, BOARD_LED_RED_GPIO_PIN, LOGIC_LED_ON);
	}
	else if (LOGIC_LED_TOOGLE == state)
	{
		GPIO_PortToggle(GPIO, BOARD_LED_RED_GPIO_PORT, BOARD_LED_RED_GPIO_MASK);
	}
	else
	{
		GPIO_PinWrite(GPIO, BOARD_LED_RED_GPIO_PORT, BOARD_LED_RED_GPIO_PIN, LOGIC_LED_OFF);
	}
}

void rgb_led_turn_BLUE(int state)
{
	if(LOGIC_LED_ON == state){
		GPIO_PinWrite(GPIO, BOARD_LED_BLUE_GPIO_PORT, BOARD_LED_BLUE_GPIO_PIN, LOGIC_LED_ON);
	}
	else{
		GPIO_PinWrite(GPIO, BOARD_LED_BLUE_GPIO_PORT, BOARD_LED_BLUE_GPIO_PIN, LOGIC_LED_OFF);
	}
}

void rgb_led_turn_GREEN(int state)
{
	if(LOGIC_LED_ON == state){
		GPIO_PinWrite(GPIO, BOARD_LED_GREEN_GPIO_PORT, BOARD_LED_GREEN_GPIO_PIN, LOGIC_LED_ON);
	}
	else{
		GPIO_PinWrite(GPIO, BOARD_LED_GREEN_GPIO_PORT, BOARD_LED_GREEN_GPIO_PIN, LOGIC_LED_OFF);
	}
}

void rgb_led_color_WHITE()
{
	rgb_led_turn_all_off();
	rgb_led_turn_BLUE(LOGIC_LED_ON);
	rgb_led_turn_RED(LOGIC_LED_ON);
	rgb_led_turn_GREEN(LOGIC_LED_ON);
}

void rgb_led_color_YELLOW()
{
	rgb_led_turn_all_off();
	rgb_led_turn_BLUE(LOGIC_LED_ON);
	rgb_led_turn_RED(LOGIC_LED_OFF);
	rgb_led_turn_GREEN(LOGIC_LED_ON);
}

void rgb_led_color_MAGENTA()
{
	rgb_led_turn_all_off();
	rgb_led_turn_BLUE(LOGIC_LED_ON);
	rgb_led_turn_RED(LOGIC_LED_ON);
	rgb_led_turn_GREEN(LOGIC_LED_OFF);
}

void rgb_led_color_CYAN()
{
	rgb_led_turn_all_off();
	rgb_led_turn_BLUE(LOGIC_LED_OFF);
	rgb_led_turn_RED(LOGIC_LED_ON);
	rgb_led_turn_GREEN(LOGIC_LED_ON);
}
