#pragma once
#include <libopencm3/stm32/adc.h>

/**
 * Rename the ADC channel for humidity
 */
#define ADC_CHANNEL_hum ADC_CHANNEL1
/**
 * Rename the ADC channel for vibration
 */
#define ADC_CHANNEL_vib ADC_CHANNEL2
/**
 * Adc buffer size for buffer space allocation (adc->mem)
 */
#define ADC_BUFFER_SIZE 32

/** 
 * Variables
 */
extern uint16_t adc_buffer[ADC_BUFFER_SIZE]; 

/**
 * Function prototypes ------------------------------------------------------
 */
/**
 * @brief Configures the ADC.
 */
void adc_setup(void);
/**
 * @brief DMA configurations.
 */
void dma_setup(void);
