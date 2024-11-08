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

uint16_t adc_buffer[16];
uint8_t usart1_tx_buffer[4]; // Yo pense que solo ibamos a transmitir, cambio el rx? 


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

void configure_uart(void);
void send_uart_data(uint16_t vib_freq, uint16_t env_hum);
//uint8_t usart1_rx_buffer[128]; // Define the buffer with an appropriate size

void timer2_setup(void);

/**
 * @brief Main function.
 * Initializes system clock, GPIO, ADC, Timer, and DMA for periodic ADC conversion and LED control.
 */
int main(void)
{
    system_clock_setup();
    gpio_setup();
    setup_dma(); // Ajusta los parámetros según tu configuración
    setup_adc();
    configure_systick();
    configure_UART();
    timer2_setup();

    buzzer_mode = OFF;  // inicializamos el buzzer en OFF 
    
    
    while (TRUE)
    {
      /*
       send_uart_data(vib_freq, env_hum);
      if(buzzer_mode == ON){
        gpio_clear(BUZZER_PORT, BUZZER_PIN);
      }
      else{
        gpio_set(BUZZER_PORT, BUZZER_PIN);
      }

      */
     
      if(adc_buffer[8] > 0){
        gpio_set(LED_PORT, YELLOW_LED_PIN);
        gpio_set(LED_PORT, GREEN_LED_PIN);
      }
      else{
        gpio_clear(LED_PORT, YELLOW_LED_PIN);
        gpio_set(LED_PORT, RED_LED_PIN);
      }
    }
    return 0;
}

void setup_adc(void) {
    // Habilitar el reloj para el ADC
    rcc_periph_clock_enable(RCC_ADC1);

    // Configurar el ADC en modo de escaneo con DMA
    adc_disable_scan_mode(ADC1);                     // Modo escaneo deshabilitado si solo tienes un canal
    adc_enable_dma(ADC1);                            // Habilitar DMA
    adc_set_single_conversion_mode(ADC1);            // Modo de conversión única
    adc_set_sample_time_on_all_channels(ADC1, ADC_SMPR_SMP_239DOT5CYC); // Configuración de tiempo de muestreo

    // Calibrar el ADC
    adc_power_on(ADC1);
    adc_reset_calibration(ADC1);
    adc_calibrate(ADC1);

    // Configurar el primer canal a muestrear
    uint8_t channels[] = {ADC_CHANNEL_hum}; // Cambia por el canal que necesitas
    adc_set_regular_sequence(ADC1, 1, channels);

    // Iniciar la conversión
    adc_start_conversion_direct(ADC1);
}

void setup_dma(void) {
    // Habilitar el reloj para el DMA
    rcc_periph_clock_enable(RCC_DMA1);

    // Resetear el canal del DMA para asegurarse de que está en un estado conocido
    dma_channel_reset(DMA1, DMA_CHANNEL1);

    // Configurar el canal del DMA
    dma_set_peripheral_address(DMA1, DMA_CHANNEL1, (uint32_t)&ADC1_DR); // Dirección del registro de datos del ADC
    dma_set_memory_address(DMA1, DMA_CHANNEL1, (uint32_t)adc_buffer);    // Dirección del buffer de memoria
    dma_set_number_of_data(DMA1, DMA_CHANNEL1, 16);                      // Tamaño del buffer (16 muestras)

    // Configuración de transferencia
    dma_set_read_from_peripheral(DMA1, DMA_CHANNEL1);   // Leer desde el periférico (ADC)
    dma_enable_memory_increment_mode(DMA1, DMA_CHANNEL1); // Incremento de dirección de memoria
    dma_set_peripheral_size(DMA1, DMA_CHANNEL1, DMA_CCR_PSIZE_16BIT); // Tamaño de dato en periférico
    dma_set_memory_size(DMA1, DMA_CHANNEL1, DMA_CCR_MSIZE_16BIT);      // Tamaño de dato en memoria
    dma_set_priority(DMA1, DMA_CHANNEL1, DMA_CCR_PL_HIGH);             // Alta prioridad

    // Habilitar el DMA
    dma_enable_channel(DMA1, DMA_CHANNEL1);
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
    gpio_set_mode(LED_PORT, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL,LED_TX);
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
void timer2_setup(void)
{
  timer_set_mode(TIM2, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);

  /* Enable Timer 2 clock */
  rcc_periph_clock_enable(RCC_TIM2);

  /* Enable TIM2 interrupt. */
  nvic_enable_irq(NVIC_TIM2_IRQ);

  /* Reset TIM2 peripheral to defaults. */
  rcc_periph_reset_pulse(RST_TIM2);

  /* Timer configuration */
  timer_set_prescaler(TIM2, 7200 - 1); // Prescaler for 10 kHz timer clock
  timer_set_period(TIM2, 300000 - 1);  // Period for 30 seconds (10 kHz * 30 s)

  /* Enable the timer interrupt for update events */
  timer_enable_irq(TIM2, TIM_DIER_UIE); // Enable interrupt on update event

  /* Start Timer 2 */
  timer_enable_counter(TIM2);
}


void tim2_isr(void)
{
  if (timer_get_flag(TIM2, TIM_SR_UIF)) {
    timer_clear_flag(TIM2, TIM_SR_UIF);
    //enviar datos por uart
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

void send_uart_data(uint16_t vib_freq, uint16_t env_hum){
    //como se pueden enviar datos cada 1 byte, y nuestros datos son de 2 bytes
    usart1_tx_buffer[0] = (vib_freq >> 8) & 0xFF;   // Parte alta de vib_freq
    usart1_tx_buffer[1] = vib_freq & 0xFF;          // Parte baja de vib_freq
    usart1_tx_buffer[2] = (env_hum >> 8) & 0xFF;    // Parte alta de env_hum
    usart1_tx_buffer[3] = env_hum & 0xFF;           // Parte baja de env_hum

  //capaz 0xff podemos definirlo como una constante
  for (int i = 0; i < 4; i++) {                        // Enviamos los 4 bytes uno por uno
    usart_wait_send_ready(USART1);                     // Funcion que el buffer este vacio
    usart_send_blocking(USART1, usart1_tx_buffer[i]);  // Enviamos el byte en bloque
  }
  gpio_toggle(LED_PORT, LED_TX);                       // Para verificar si se en
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
