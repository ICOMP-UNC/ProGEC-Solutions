/**
 * @file common.c
 * @author 
 * @brief Implementation of the common functions
 * @version 0.1
 * @date 2024-11-09
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/usart.h>


#include "common.h"
#include "pwm.h"
#include "adc.h"
#include "pins.h"


uint8_t index_hist_vib = 0; 
uint16_t vib_freq = 0;  // frecuencia de los sismos
uint16_t historic_vib[_MAX_VIB_N];  // vibraciones pasadas de los sismos
uint16_t env_vib;
uint16_t env_hum;
uint8_t usart1_tx_buffer[4];
int buzzer_mode = OFF; // estado del buzzer ON/OFF
int alarm_activation_mode = OFF; // modo de activacion de la alarma
analyze_flag_t analyze_proc_flag = CAN_ANALYZE;

uint16_t adc_buffer[ADC_BUFFER_SIZE];
/**
 * @brief analyzes the environment info and updates the LEDs and alarm system.
 * 
 */
void analyze_and_update_system(void) // esto es asincrono a la interrupcion
{
        if (env_hum > THRESHOLD_HUM_H || env_vib > THRESHOLD_VIB_FREQ_H) {
        gpio_clear(LED_PORT, YELLOW_LED_PIN);
        gpio_clear(LED_PORT, GREEN_LED_PIN);
        gpio_set(LED_PORT, RED_LED_PIN);
         buzzer_mode = OFF;
        if (env_hum > THRESHOLD_HUM_H && env_vib > THRESHOLD_VIB_FREQ_H) {
             buzzer_mode = ON; // alarma y led rojo
        }
    } else if (env_hum <= THRESHOLD_HUM_L && env_vib <= THRESHOLD_VIB_FREQ_L) {
        gpio_clear(LED_PORT, RED_LED_PIN);
        gpio_clear(LED_PORT, YELLOW_LED_PIN);
        gpio_set(LED_PORT, GREEN_LED_PIN); // led verde encendido
         buzzer_mode = OFF;
    } else if((env_hum > THRESHOLD_HUM_L && env_hum < THRESHOLD_HUM_M) || (env_vib > THRESHOLD_VIB_FREQ_L && env_vib < THRESHOLD_VIB_FREQ_M)){ // cualquier estado amarillo
        gpio_clear(LED_PORT, RED_LED_PIN);
        gpio_clear(LED_PORT, GREEN_LED_PIN);
         buzzer_mode = OFF;
        gpio_set(LED_PORT, YELLOW_LED_PIN);
    }
}

/**
 * @brief conversion of the ADC values to environment values.
 * 
 */
void convert_adc_to_env(uint16_t vib_to_convert, uint16_t hum_to_convert)
{
  env_hum  = (hum_to_convert * 3.3 / 4096.0) * 100; 
  env_vib = (vib_to_convert * 3.3 / 4096.0) * 100; 
} 

/**
 * @brief Manages the PWM control of the buzzer.
 * 
 * @param delay 
 * @param initial_buzzer_mode 
 */
void control_pwm(uint32_t delay, int initial_buzzer_mode) {
  for (uint32_t duty_cycle = 0; duty_cycle <= 1000; duty_cycle += 50) {
    timer_set_oc_value(TIM2, TIM_OC4, duty_cycle);
      for (volatile uint32_t i = 0; i < delay; i++) {
        if (buzzer_mode != initial_buzzer_mode && alarm_activation_mode == OFF) {
          return; 
        }
      }
  }
  for (uint32_t duty_cycle = 1000; duty_cycle > 0; duty_cycle -= 50) {
    timer_set_oc_value(TIM2, TIM_OC4, duty_cycle);
      for (volatile uint32_t i = 0; i < delay; i++) {
        if (buzzer_mode != initial_buzzer_mode && alarm_activation_mode == OFF) {
          return; 
        }
      }
  }
}

/**
 * @brief Updates the vibration frequency. 
 * 
 */
void update_vib_frequency(void)
{
  historic_vib[index_hist_vib] = env_vib;
  index_hist_vib = (index_hist_vib + 1) % _MAX_VIB_N; // circular
  if (index_hist_vib == 0) {
    vib_freq = 0;
    for (int i = 0; i < _MAX_VIB_N; i++) {
      vib_freq += historic_vib[i];
    }
    vib_freq /= _MAX_VIB_N;   // promedio de las vibraciones
  }
}

/**
 * @brief Sends the environment info through UART.
 * 
 */
void send_uart_data(uint16_t vib, uint16_t hum){
    usart1_tx_buffer[0] = (vib >> 8) & BYTE_MASK;   // Parte alta de vib_freq
    usart1_tx_buffer[1] = vib & BYTE_MASK;          // Parte baja de vib_freq
    usart1_tx_buffer[2] = (hum >> 8) & BYTE_MASK;    // Parte alta de env_hum
    usart1_tx_buffer[3] = hum & BYTE_MASK;           // Parte baja de env_hum

  for (int i = 0; i < 4; i++) {                        // Enviamos los 4 bytes uno por uno
    usart_wait_send_ready(USART1);                     // Funcion que el buffer este vacio
    usart_send_blocking(USART1, usart1_tx_buffer[i]);  // Enviamos el byte en bloque
  }
}
