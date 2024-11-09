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
uint16_t adc_buffer[ADC_BUFFER_SIZE]; // buffer para guardar los valores del ADC
uint8_t usart1_tx_buffer[4]; // Yo pense que solo ibamos a transmitir, cambio el rx? 

analyze_flag_t analyze_proc_flag = CAN_ANALYZE; 

int buzzer_mode; // estado del buzzer ON/OFF

void system_clock_setup(void);
void gpio_setup(void);
void adc_setup(void);
void configure_systick(void);
void dma_setup(void);
void analyze_and_update_system(void);
void update_env_state(uint16_t adc_vib, uint16_t adc_hum);
void update_vib_frequency(void);

void control_leds_based_on_hum(uint16_t hum);
uint16_t read_adc(uint32_t channel);

void configure_UART(void);
void configure_PWM(void);
void send_uart_data(uint16_t vib_freq, uint16_t env_hum);
void control_pwm(uint32_t delay, int initial_buzzer_mode);

/**
 * @brief Main function.
 * Initializes system clock, GPIO, ADC, Timer, and DMA for periodic ADC conversion and LED control.
 */
int main(void)
{
    system_clock_setup();
    gpio_setup();
    dma_setup();
    adc_setup();
    configure_systick();
    configure_UART();
    configure_PWM();
    buzzer_mode = OFF;  // inicializamos el buzzer en OFF 

    while (TRUE)
    {
      if (buzzer_mode == ON) {
        control_pwm(DELAY_PWM, buzzer_mode);
      } else{
          timer_set_oc_value(TIM2, TIM_OC4, 0);
      }
      if(analyze_proc_flag == CAN_ANALYZE){
        analyze_and_update_system();
        analyze_proc_flag = ANALYZED;
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
void dma_setup(void) {
   rcc_periph_clock_enable(RCC_DMA1);

    dma_channel_reset(DMA1, DMA_CHANNEL1);
    dma_set_peripheral_address(DMA1, DMA_CHANNEL1, (uint32_t)&ADC_DR(ADC1));
    dma_set_memory_address(DMA1, DMA_CHANNEL1, (uint32_t)adc_buffer);
    dma_set_number_of_data(DMA1, DMA_CHANNEL1, ADC_BUFFER_SIZE);
    dma_set_read_from_peripheral(DMA1, DMA_CHANNEL1);
    dma_enable_memory_increment_mode(DMA1, DMA_CHANNEL1);
    dma_set_peripheral_size(DMA1, DMA_CHANNEL1, DMA_CCR_PSIZE_16BIT);
    dma_set_memory_size(DMA1, DMA_CHANNEL1, DMA_CCR_MSIZE_16BIT);
    dma_enable_circular_mode(DMA1, DMA_CHANNEL1);
    dma_enable_transfer_complete_interrupt(DMA1, DMA_CHANNEL1);
    dma_enable_channel(DMA1, DMA_CHANNEL1);

    nvic_enable_irq(NVIC_DMA1_CHANNEL1_IRQ);
}
void analyze_and_update_system(void) // esto es asincrono a la interrupcion
{
 
        if (env_hum > THRESHOLD_HUM_H /*|| env_vib > THRESHOLD_HUM_H*/) {
        gpio_clear(LED_PORT, YELLOW_LED_PIN);
        gpio_clear(LED_PORT, GREEN_LED_PIN);
        gpio_set(LED_PORT, RED_LED_PIN);
         buzzer_mode = OFF;
        if (env_hum > THRESHOLD_HUM_H /*&& env_vib > THRESHOLD_HUM_H*/) {
             buzzer_mode = ON; // alarma y led rojo
        }
    } else if (env_hum <= THRESHOLD_HUM_L /*|| env_vib <= THRESHOLD_HUM_L*/) {
        gpio_clear(LED_PORT, RED_LED_PIN);
        gpio_clear(LED_PORT, YELLOW_LED_PIN);
        gpio_set(LED_PORT, GREEN_LED_PIN); // led verde encendido
         buzzer_mode = OFF;
    } else { // cualquier estado amarillo
        gpio_clear(LED_PORT, RED_LED_PIN);
        gpio_clear(LED_PORT, GREEN_LED_PIN);
         buzzer_mode = OFF;
        gpio_set(LED_PORT, YELLOW_LED_PIN);
    }
   
    
   
}
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
} 
void sys_tick_handler(void) {
    uint16_t sum_hum = 0;
    uint16_t sum_vib = 0;

    // Convertir datos y guardarlos
    for (int i = 0; i < ADC_BUFFER_SIZE; i++) {
        sum_vib += adc_buffer[i];
        // sum_hum += adc_buffer[i+1];
    }

    sum_hum = sum_hum / ADC_BUFFER_SIZE;
    sum_vib = sum_vib / ADC_BUFFER_SIZE;

    update_env_state(sum_hum, sum_vib);

    if (analyze_proc_flag == ANALYZED) {
        analyze_proc_flag = CAN_ANALYZE;
    }

    // update_vib_frequency();
    // analyze_and_update_system();
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
    rcc_periph_clock_enable(RCC_GPIOC);
    gpio_set_mode(LED_PORT, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GREEN_LED_PIN);
    gpio_set_mode(LED_PORT, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, RED_LED_PIN);
    gpio_set_mode(LED_PORT, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, YELLOW_LED_PIN);
    gpio_set_mode(LED_PORT, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL,LED_TX);
    gpio_set_mode(BUZZER_PORT, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, BUZZER_PIN);
    
    //config switch manual
    gpio_set_mode(BUTTON_PORT, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, BUTTON_PIN);
    gpio_set(BUTTON_PORT, BUTTON_PIN);
    nvic_enable_irq(NVIC_EXTI0_IRQ);

    exti_select_source(EXTI0, BUZZER_PORT);        // Select PA0 as the source for EXTI0 
    exti_set_trigger(EXTI0, EXTI_TRIGGER_FALLING); // Trigger on falling edge 
    exti_enable_request(EXTI0);                    // Enable EXTI0 interrupt request      
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
void adc_setup(void) {
    rcc_periph_clock_enable(RCC_ADC1);
    rcc_periph_clock_enable(RCC_GPIOA);

    // Configure ADC pin as analog input
    gpio_set_mode(ADC_PORT, GPIO_MODE_INPUT, GPIO_CNF_INPUT_ANALOG, ADC_PIN_vib);

    // Power off ADC before configuration
    ADC_CR2(ADC1) &= ~ADC_CR2_ADON;

    // Set ADC to continuous conversion mode
    ADC_CR2(ADC1) |= ADC_CR2_CONT;

    // Set sample time for the ADC channel
    ADC_SMPR2(ADC1) |= (0x7 << (3 * ADC_CHANNEL_vib)); // 239.5 cycles

    // Set the regular sequence to read from the vibration channel
    ADC_SQR3(ADC1) = ADC_CHANNEL_vib;

    // Enable DMA for ADC
    ADC_CR2(ADC1) |= ADC_CR2_DMA;

    // Power on ADC
    ADC_CR2(ADC1) |= ADC_CR2_ADON;

   
    // Calibrate ADC
    ADC_CR2(ADC1) |= ADC_CR2_RSTCAL;
    while (ADC_CR2(ADC1) & ADC_CR2_RSTCAL);
    ADC_CR2(ADC1) |= ADC_CR2_CAL;
    while (ADC_CR2(ADC1) & ADC_CR2_CAL);

    // Start ADC conversion
    ADC_CR2(ADC1) |= ADC_CR2_ADON;
    ADC_CR2(ADC1) |= ADC_CR2_SWSTART;
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
=======
  rcc_periph_clock_enable(RCC_USART1);
  usart_set_baudrate(USART1, 9600);
  usart_set_databits(USART1, 8);                     
  usart_set_stopbits(USART1, USART_STOPBITS_1);
  usart_set_parity(USART1, USART_PARITY_NONE);
  usart_set_mode(USART1, USART_MODE_TX);
  usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
  usart_enable(USART1);
  nvic_enable_irq(NVIC_USART1_IRQ);
}
*/
void configure_PWM(void)
{
    rcc_periph_clock_enable(RCC_TIM2);

    // Se configura el modo de funcionamiento del Timer2
    timer_set_mode(TIM2, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);  

    timer_set_prescaler(TIM2, 71); // 72MHz / 72 = 1MHz (1us por tick)
    timer_set_period(TIM2, 1000);  // 1kHz PWM frecuencia

    // Configura el canal 4 (PA3) en modo PWM1
    timer_set_oc_mode(TIM2, TIM_OC4, TIM_OCM_PWM1);
    timer_enable_oc_output(TIM2, TIM_OC4);

    // Inicializa el duty cycle al 0%
    timer_set_oc_value(TIM2, TIM_OC4, 0);

    // Activa el contador del Timer2
    timer_enable_counter(TIM2);
}

void control_pwm(uint32_t delay, int initial_buzzer_mode) {
  for (uint32_t duty_cycle = 0; duty_cycle <= 1000; duty_cycle += 50) {
    timer_set_oc_value(TIM2, TIM_OC4, duty_cycle);
      for (volatile uint32_t i = 0; i < delay; i++) {
        if (buzzer_mode != initial_buzzer_mode) {
          return; // Salir de la función si buzzer_mode cambia
        }
      }
  }
  for (uint32_t duty_cycle = 1000; duty_cycle > 0; duty_cycle -= 50) {
    timer_set_oc_value(TIM2, TIM_OC4, duty_cycle);
      for (volatile uint32_t i = 0; i < delay; i++) {
        if (buzzer_mode != initial_buzzer_mode) {
          return; // Salir de la función si buzzer_mode cambia
        }
      }
  }
}

void configure_UART()
{
  rcc_periph_clock_enable(RCC_USART1);
  usart_set_baudrate(USART1, 9600);
  usart_set_databits(USART1, 8);                     //enviariamos 8 bits por data
  usart_set_stopbits(USART1, USART_STOPBITS_1);
  usart_set_parity(USART1, USART_PARITY_NONE);
  usart_set_mode(USART1, USART_MODE_TX);
  usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
  usart_enable(USART1);
  nvic_enable_irq(NVIC_USART1_IRQ);
}

void send_uart_data(uint16_t vib, uint16_t hum){
    //como se pueden enviar datos cada 1 byte, y nuestros datos son de 2 bytes
    usart1_tx_buffer[0] = (vib >> 8) & BYTE_MASK;   // Parte alta de vib_freq
    usart1_tx_buffer[1] = vib & BYTE_MASK;          // Parte baja de vib_freq
    usart1_tx_buffer[2] = (hum >> 8) & BYTE_MASK;    // Parte alta de env_hum
    usart1_tx_buffer[3] = hum & BYTE_MASK;           // Parte baja de env_hum

  //capaz 0xff podemos definirlo como una constante
  for (int i = 0; i < 4; i++) {                        // Enviamos los 4 bytes uno por uno
    usart_wait_send_ready(USART1);                     // Funcion que el buffer este vacio
    usart_send_blocking(USART1, usart1_tx_buffer[i]);  // Enviamos el byte en bloque
  }
  gpio_set(LED_PORT, LED_TX);
}

/*
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
void dma1_channel1_isr(void)
{
    if (dma_get_interrupt_flag(DMA1, DMA_CHANNEL1, DMA_TCIF)) {
        dma_clear_interrupt_flags(DMA1, DMA_CHANNEL1, DMA_TCIF);
        // Toggle LED to indicate DMA transfer complete
    }
}