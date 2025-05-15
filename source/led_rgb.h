/*
 * led_rgb.h
 *
 *  Created on: 21 mar. 2025
 *      Author: Gustavo Sanchez
 */
#include "fsl_common.h"
#include "fsl_gpio.h"

#ifndef LED_RGB_H_
#define LED_RGB_H_

/* Board led color mapping */
#define LOGIC_LED_ON  0U
#define LOGIC_LED_OFF 1U
#define LOGIC_LED_TOOGLE 2U
/* BLUE */
#ifndef BOARD_LED_BLUE_GPIO
#define BOARD_LED_BLUE_GPIO GPIO
#endif
#define BOARD_LED_BLUE_GPIO_PORT 0U
#ifndef BOARD_LED_BLUE_GPIO_PIN
#define BOARD_LED_BLUE_GPIO_PIN 0U
#endif
/* RED */
#ifndef BOARD_LED_RED_GPIO
#define BOARD_LED_RED_GPIO GPIO
#endif
#define BOARD_LED_RED_GPIO_PORT 0U
#define BOARD_LED_RED_GPIO_MASK 1 << 1U
#ifndef BOARD_LED_RED_GPIO_PIN
#define BOARD_LED_RED_GPIO_PIN 1U
#endif
/* GREEN */
#ifndef BOARD_LED_GREEN_GPIO
#define BOARD_LED_GREEN_GPIO GPIO
#endif
#define BOARD_LED_GREEN_GPIO_PORT 0U
#ifndef BOARD_LED_GREEN_GPIO_PIN
#define BOARD_LED_GREEN_GPIO_PIN 12U
#endif

void rgb_led_init();
/*
 * BLUE
 */
#define LED_RGB_BLUE_INIT(output)                                                            \
    GPIO_PinInit(BOARD_LED_BLUE_GPIO, BOARD_LED_BLUE_GPIO_PORT, BOARD_LED_BLUE_GPIO_PIN, \
                 &(gpio_pin_config_t){kGPIO_DigitalOutput, (output)}) /*!< Enable target LED_BLUE */
/*
 * RED
 */
#define LED_RGB_RED_INIT(output)                                                            \
    GPIO_PinInit(BOARD_LED_RED_GPIO, BOARD_LED_RED_GPIO_PORT, BOARD_LED_RED_GPIO_PIN, \
                 &(gpio_pin_config_t){kGPIO_DigitalOutput, (output)}) /*!< Enable target LED_BLUE */
/*
 * GREEN
 */
#define LED_RGB_GREEN_INIT(output)                                                            \
    GPIO_PinInit(BOARD_LED_GREEN_GPIO, BOARD_LED_GREEN_GPIO_PORT, BOARD_LED_GREEN_GPIO_PIN, \
                 &(gpio_pin_config_t){kGPIO_DigitalOutput, (output)}) /*!< Enable target LED_GREEN */


void rgb_led_turn_RED(int state);
void rgb_led_turn_BLUE(int state);
void rgb_led_turn_GREEN(int state);
void rgb_led_turn_all_off();
void rgb_led_color_WHITE();
void rgb_led_color_YELLOW();
void rgb_led_color_MAGENTA();
void rgb_led_color_CYAN();
#endif /* LED_RGB_H_ */
