#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/adc.h>

#include "util.h"
#include "../include/pins.h"

#define TRUE 1
#define FALSE 0

#define ON 1
#define OFF 0

uint8_t index_hist_vib; // para indexar el vector de vibraciones
uint16_t vib_freq;  // frecuencia de los sismos 
uint16_t historic_vib[_MAX_VIB_N];  // vibraciones pasadas de los sismos
uint16_t env_vib;
uint16_t env_hum;

analyze_flag_t analyze_proc_flag = CAN_ANALYZE; 

int buzzer_mode; // estado del buzzer ON/OFF

void system_clock_setup(void);
void gpio_setup(void);
void adc_setup(void);
void configure_systick(void);

void analyze_and_update_system(void);
void update_env_state(uint16_t adc_vib, uint16_t adc_hum);
void update_vib_frequency(void);

void control_leds_based_on_hum(uint16_t hum);
uint16_t read_adc(uint32_t channel);

//uint8_t usart1_rx_buffer[128]; // Define the buffer with an appropriate size


/**
 * @brief Main function.
 * Initializes system clock, GPIO, ADC, Timer, and DMA for periodic ADC conversion and LED control.
 */
int main(void)
{
  // execute_setup(&setup);
    system_clock_setup();
    gpio_setup();
    adc_setup();
    configure_systick();
    //configure_dma();

    while (TRUE)
    {
      if(buzzer_mode == 1){
        gpio_clear(BUZZER_PORT, BUZZER_PIN);
      }
      else{
        gpio_set(BUZZER_PORT, BUZZER_PIN);
      }
    }
    return 0;
}


/**
 * @brief Configures the system clock to 72 MHz using an 8 MHz external crystal.
 */
void system_clock_setup(void)
{
    rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]);
}


void analyze_and_update_system(void) // esto es asincrono a la interrupcion
{
   if (env_vib > THRESHOLD_VIB_FREQ_H || env_hum > THRESHOLD_HUM_H) {
    gpio_set(LED_PORT, YELLOW_LED_PIN); 
    gpio_set(LED_PORT, GREEN_LED_PIN);
    gpio_clear(LED_PORT, RED_LED_PIN); 
    //buzzer_mode = OFF;
      if(vib_freq > THRESHOLD_VIB_FREQ_H && env_hum > THRESHOLD_HUM_H){
       // buzzer_mode = ON; // alarma y led rojo

  } else if(env_vib <= THRESHOLD_VIB_FREQ_L || env_hum <= THRESHOLD_HUM_L) {
    gpio_set(LED_PORT, RED_LED_PIN); 
    gpio_set(LED_PORT, YELLOW_LED_PIN); 
    gpio_clear(LED_PORT, GREEN_LED_PIN); //led verde encendido
    //buzzer_mode = OFF; 
  }//estado normal
    else{  //cualquier estado amarillo 
    gpio_set(LED_PORT, RED_LED_PIN); 
    gpio_set(LED_PORT, GREEN_LED_PIN);
   // buzzer_mode = OFF; 
    gpio_clear(LED_PORT, YELLOW_LED_PIN); 
  }
}}
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
void update_env_state(uint16_t adc_vib, uint16_t adc_hum)
{
  env_hum  = (adc_hum * 3.3 / 4096.0) * 100; // Convert ADC value to humidity
  env_vib = (adc_vib * 3.3 / 4096.0) * 100; // Convert ADC value to vibration


  //Aca deberiamos procesarlos un poco mas y ver que rango nos tira el adc
} 
void sys_tick_handler(void)
{ 
  //Convertir datos y guardarlos
  update_env_state(read_adc(ADC_CHANNEL_hum), read_adc(ADC_CHANNEL_vib));
  if(env_hum > 100){
    gpio_clear(LED_PORT, YELLOW_LED_PIN);
  }
  update_vib_frequency();
  analyze_and_update_system();
}
/**
 * @brief 
 * void timer0_isr()
{
  timer_clear_flag(TIM2, TIM_SR_UIF);
  

}
 * 
 */

void exti0_isr(void)
{
  exti_reset_request(EXTI0);
  buzzer_mode = !buzzer_mode;
}

/**
 * @brief Configures GPIO pins for the three LEDs.
 */
void gpio_setup(void)
{
    /* Enable GPIO clocks */
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOA);
    gpio_set_mode(LED_PORT, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GREEN_LED_PIN);
    gpio_set_mode(LED_PORT, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, RED_LED_PIN);
    gpio_set_mode(LED_PORT, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, YELLOW_LED_PIN);

    //config buzzer
    rcc_periph_clock_enable(RCC_GPIOC);
    gpio_set_mode(BUZZER_PORT, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, BUZZER_PIN);

    //config switch manual
    rcc_periph_clock_enable(RCC_GPIOA);
    gpio_set_mode(BUTTON_PORT, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, BUTTON_PIN);
    gpio_set(BUTTON_PORT, BUTTON_PIN);
    nvic_enable_irq(NVIC_EXTI0_IRQ);

    exti_select_source(EXTI0, BUZZER_PORT);        /* Select PA0 as the source for EXTI0 */
    exti_set_trigger(EXTI0, EXTI_TRIGGER_FALLING); /* Trigger on falling edge */
    exti_enable_request(EXTI0);                    /* Enable EXTI0 interrupt request */

}

/**
 * @brief Configures the systick timer.
 */
void configure_systick(void)
{
    systick_set_reload(rcc_ahb_frequency / 1000 * SYSTICK_INTERVAL_MS - 1); /* Set reload for SYSTICK_INTERVAL_MS */
    systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
    systick_counter_enable();
    systick_interrupt_enable();
}

/**
 * @brief Configures ADC1 with DMA for humidity sensor readings.
 */
void adc_setup(void)
{
    rcc_periph_clock_enable(RCC_ADC1);
    adc_power_off(ADC1);
    adc_disable_scan_mode(ADC1);
    adc_disable_external_trigger_regular(ADC1);
    adc_set_sample_time(ADC1, ADC_PIN_hum, ADC_SMPR_SMP_55DOT5CYC); /* Set sample time */
    adc_set_sample_time(ADC1, ADC_PIN_vib, ADC_SMPR_SMP_55DOT5CYC); /* Set sample time */
    adc_power_on(ADC1);
    adc_reset_calibration(ADC1);
    adc_calibrate(ADC1);

}

uint16_t read_adc(uint32_t channel)
{
  uint8_t channels[1] = { channel };
  adc_set_regular_sequence(ADC1, 1, channels);
  adc_start_conversion_direct(ADC1);
  while (!adc_eoc(ADC1));
  return adc_read_regular(ADC1);
}



/*
void control_leds_based_on_hum(uint16_t hum)
{
    uint16_t humidity = (hum * 3.3 / 4096.0) * 100; // Convert ADC value to humidity

    if (humidity < THRESHOLD_HUM_M)
    {
        gpio_clear(RED_LED_PORT, RED_LED_PIN);       
    }
    else
    {
        gpio_set(RED_LED_PORT, RED_LED_PIN);         
    }
}
*/
