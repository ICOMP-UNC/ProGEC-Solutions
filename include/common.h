#include <libopencm3/stm32/rcc.h>
#pragma once

#define TRUE  1
#define FALSE 0

#define ON  1
#define OFF 0

#define _MAX_VIB_N 200
/**
 *
 */
#define BYTE_MASK 0xFF

/**
 * Threshold vibration frequency HIGH
 */
#define THRESHOLD_VIB_FREQ_H 15
/**
 * Threshold vibration frequency MIDDLE
 */
#define THRESHOLD_VIB_FREQ_M 10
/**
 * Threshold vibration frequency LOW
 */
#define THRESHOLD_VIB_FREQ_L 0
/**
 * Threshold huimidity HIGH
 */
#define THRESHOLD_HUM_L 90
/**
 * Threshold huimidity MIDDLE
 */
#define THRESHOLD_HUM_M 50
/**
 * Threshold huimidity LOW
 */
#define THRESHOLD_HUM_H 10
/**
 * Threshold vibration frequency
 */
#define THRESHOLD_FREQ 60
/**
 * Buzzer frequency
 */
#define BUZZER_FREQ 1000
/**
 * System clock time int.
 */
#define SYSTICK_INTERVAL_MS 10
/**
 * Cases for the analyze process.
 */
#define UART_BUFFER_SIZE 2
/**
 * Buffer size for the UART communication.
 */
typedef enum
{
    ANALYZING,
    ANALYZED,
    CAN_ANALYZE,
} analyze_flag_t;

/*
Variables ----------------------------------------------------------------
*/

/**
 * Used to index the vibration vector.
 */
extern int index_hist_vib;
/**
 * Global variable for the vibration frequency.
 */
extern uint16_t vib_freq;
/**
 * Vector to store the past vibrations.
 */
extern uint16_t historic_vib[_MAX_VIB_N];
/**
 * Environment vibration value.
 */
extern uint16_t env_vib;
/**
 * Environment humidity value.
 */
extern uint16_t env_hum;
/**
 * @brief Average vibration value.
 *
 * Environment vibration value.
 */

extern uint16_t prom_vib;
/**
 * Buffer to store the data to be sent through UART.
 */
extern uint8_t usart3_tx_buffer[UART_BUFFER_SIZE];
/**
 * Buffer to store the data to be sent through UART.
 */
extern uint16_t uart_tail;
/**
 * Buffer to store the data to be sent through UART.
 */
extern uint16_t uart_head;
/**
 * Variable to modify the buzzer mode. If it is ON, the buzzer will sound.
 * This is managed by environment.
 */
extern int buzzer_mode;
/**
 * Variable to modify the alarm activation. It is managed by the user.
 * Activated by external interrupt.
 */
extern int alarm_activation_mode;
/**
 * Variable to manage the analyze process.
 */
extern analyze_flag_t analyze_proc_flag;

/*
Function prototypes ------------------------------------------------------
*/

/**
 * @brief Analyze the environment info and
 * update the LEDs and alarm system.
 * Alarm and LED system mechanism:
 * -> Either vibrations frequency or humidity are above the threshold : RED LED
 * -> Both vibrations frequency and humidity are above the threshold : RED LED + ALARM
 * -> One of both is below the threshold : YELLOW LED
 * -> Both are in the middle : YELLOW LED
 * -> Both vibrations frequency and humidity are below the threshold : GREEN LED
 */
void analyze_and_update_system(void);
/**
 * @brief Converts the ADC values to environment values.
 * @param vib_to_convert
 * @param hum_to_convert
 */
void convert_adc_to_env(uint16_t, uint16_t);
/**
 * @brief Updates the vibration frequency.
 */
void update_vib_frequency(void);
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
/**
 * @brief Sets the system clock to 72 MHz.
 */
void system_clock_setup(void);
/**
 * @brief Configures the systick timer.
 */
void configure_systick(void);
