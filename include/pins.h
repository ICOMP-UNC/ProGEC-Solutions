/**
 * @file pins.h
 * @author 
 * @brief Pin definitions for the STM32F103C8T6
 * @version 0.1
 * @date 2024-11-05
 * 
 * @copyright Copyright (c) 2024
 * 
 */


/**
 * PORT definitions
 */
#define LED_PORT GPIOA
#define BUZZER_PORT GPIOC
#define BUTTON_PORT GPIOA
#define ADC_PORT GPIOA

/**
 * PIN definitions 
 */
#define YELLOW_LED_PIN GPIO12
#define RED_LED_PIN GPIO9
#define GREEN_LED_PIN GPIO8
#define BUZZER_PIN GPIO13
#define BUTTON_PIN GPIO0
#define ADC_PIN_vib GPIO1
#define ADC_PIN_hum GPIO2

/**
 * ADC channel definitions
 */
#define ADC_CHANNEL_vib ADC_CHANNEL1
#define ADC_CHANNEL_hum ADC_CHANNEL2




