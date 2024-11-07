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

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/timer.h>

/* Standard library includes */
#include <stdint.h>
#include <stddef.h>

/**
 * PORT definitions
 */
#define LED_PORT GPIOC
#define BUZZER_PORT GPIOC
#define BUTTON_PORT GPIOA
#define ADC_PORT GPIOA

/**
 * PIN definitions 
 */
#define RED_LED_PIN GPIO13
#define GREEN_LED_PIN GPIO9
#define BUZZER_PIN GPIO11
#define BUTTON_PIN GPIO0
#define ADC_PIN_vib GPIO1
#define ADC_PIN_hum GPIO2

/**
 * ADC channel definitions
 */
#define ADC_CHANNEL_vib ADC_CHANNEL1
#define ADC_CHANNEL_hum ADC_CHANNEL2


/* Function Prototypes */
void system_clock_setup(void);
void gpio_setup(void);
void adc_setup(void);
void timer2_setup(void);

