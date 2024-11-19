#pragma once
#include <libopencm3/stm32/usart.h>

/**
 * Buffer size for the UART communication.
 */
#define UART_BUFFER_SIZE 2

/**
 * @brief Configure UART communication.
 */
void configure_UART(void);

/**
 * @brief Sends the data through UART.
 * @param vib_freq
 * @param env_hum
 */
void send_uart_data(uint16_t vib_freq, uint16_t env_hum);