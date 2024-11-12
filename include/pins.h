/**
 * @file pins.h
 * @brief Pin definitions for the STM32F103C8T6
 * @version 0.1
 * @date 2024-11-05
 *
 * @details This file contains the definitions for the GPIO ports and pins used in the project.
 *
 * @note Ensure that the correct GPIO ports and pins are defined for your specific hardware configuration.
 *
 * @copyright Copyright (c) 2024
 */

#pragma once

/**
 * @brief GPIO port for the LEDs.
 */
#define LED_PORT GPIOA
/**
 * @brief GPIO port for the buzzer.
 */
#define BUZZER_PORT GPIOA

/**
 * @brief GPIO port for the button.
 */
#define BUTTON_PORT GPIOA

/**
 * @brief GPIO port for the ADC.
 */
#define ADC_PORT GPIOA

/**
 * @brief GPIO pin for the yellow LED.
 */
#define YELLOW_LED_PIN GPIO12
/**
 * @brief GPIO pin for the red LED.
 */
#define RED_LED_PIN GPIO9
/**
 * @brief GPIO pin for the green LED.
 */
#define GREEN_LED_PIN GPIO8
/**
 * @brief GPIO pin for the buzzer.
 */
#define BUZZER_PIN GPIO3
/**
 * @brief GPIO pin for the button.
 */
#define BUTTON_PIN GPIO0
/**
 * @brief GPIO pin for the TX LED.
 */
#define LED_TX GPIO10
/**
 * @brief GPIO pin for the vibration sensor ADC.
 */
#define ADC_PIN_vib GPIO1
/**
 * @brief GPIO pin for the humidity sensor ADC.
 */
#define ADC_PIN_hum GPIO2

/**
 * @brief Configures the GPIO ports and pins.
 */
void gpio_setup(void);