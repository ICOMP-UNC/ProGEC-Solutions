#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/adc.h>

#include "../include/util.h"
#include "../include/pins.h"

#define TRUE 1
#define FALSE 0

#define ON 1
#define OFF 0

void control_leds_based_on_hum(uint16_t hum);
uint16_t read_adc(uint32_t channel);

//uint8_t usart1_rx_buffer[128]; // Define the buffer with an appropriate size
/*{
 int main() {
  execute_setup(&setup);

  while(TRUE) 
  {
    if(analyze_proc_flag == CAN_ANALYZE)  // se ejecuta cada 30 segundos (timer0)
    {
      analyze_proc_flag = ANALYZING; 
      analyze_and_update_system();
      analyze_proc_flag = ANALYZED;
    }
  }
  return 0;
}*/

/**
 * @brief Main function.
 * Initializes system clock, GPIO, ADC, Timer, and DMA for periodic ADC conversion and LED control.
 */
int main(void)
{
    system_clock_setup(); /* Set up system clock */
    gpio_setup();         /* Configure GPIO pins */
    adc_setup();          /* Configure ADC1 */
    configure_systick();  /* Configure SysTick */

    while (TRUE)
    {
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

/*
void analyze_and_update_system() // esto es asincrono a la interrupcion
{
   if (vib_freq > THRESHOLD_VIB_FREQ_H || env_hum > THRESHOLD_HUM_H) {
    gpio_clear(LED_PORT, YELLOW_LED_PIN); 
    gpio_clear(LED_PORT, GREEN_LED_PIN);
    gpio_set(LED_PORT, RED_LED_PIN); 
    buzzer_mode = OFF;
      if(vib_freq > THRESHOLD_VIB_FREQ_H && env_hum > THRESHOLD_HUM_H)
        buzzer_mode = ON; // alarma y led rojo

  } else if(vib_freq <= THRESHOLD_VIB_FREQ_L || env_hum <= THRESHOLD_HUM_L) {
    gpio_clear(LED_PORT, RED_LED_PIN); 
    gpio_clear(LED_PORT, YELLOW_LED_PIN); 
    gpio_set(LED_PORT, GREEN_LED_PIN); //led verde encendido
    buzzer_mode = OFF; 
  }//estado normal
    else{  //cualquier estado amarillo 
    gpio_clear(LED_PORT, RED_LED_PIN); 
    gpio_clear(LED_PORT, GREEN_LED_PIN);
    buzzer_mode = OFF; 
    gpio_set(LED_PORT, YELLOW_LED_PIN); 
  }
}
void update_vib_frequency()
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
  env_hum = adc_hum;
  env_vib = adc_vib;
  //Aca deberiamos procesarlos un poco mas y ver que rango nos tira el adc
} 
void sys_tick_handler()
{ 
  //Convertir datos y guardarlos
  update_env_state(read_adc(ADC_CHANNEL_hum), read_adc(ADC_CHANNEL_vib));
  update_vib_frequency();
}

void timer0_isr()
{
  timer_clear_flag(TIM2, TIM_SR_UIF);
  if(analyze_proc_flag == ANALYZED);
    analyze_proc_flag = CAN_ANALYZE;

}
void exti0_isr()
{
  exti_reset_request(EXTI0);
  buzzer_mode = !buzzer_mode;
}
void configure_GPIO()
{
  rcc_periph_clock_enable(RCC_GPIOC);
  gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO8);
}

void configure_SYSTICK()
{
  systick_set_reload(STK_RVR_RELOAD);
  systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
  systick_counter_enable();
}
/**
 * @brief Configures Timer 2 to trigger ADC conversion every 60 seconds.
 */

/**
 * @brief Configures GPIO pins for the three LEDs.
 */
void gpio_setup(void)
{
    /* Enable GPIO clocks */
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOC);

    /* Configure PA8 (Green LED) as output */
    gpio_set_mode(GREEN_LED_PORT, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GREEN_LED_PIN);

    /* Configure PC13 (Red LED) as output */
    gpio_set_mode(RED_LED_PORT, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, RED_LED_PIN);
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
 * @brief SysTick interrupt handler.
 */
void sys_tick_handler(void)
{   
    control_leds_based_on_hum(read_adc(ADC_CHANNEL_hum));
}

/**
 * @brief Configures ADC1 with DMA for humidity sensor readings.
 */
void adc_setup(void)
{
    /* Enable ADC1 clock */
    rcc_periph_clock_enable(RCC_ADC1);

    /* Configure ADC1 */
    adc_power_off(ADC1);
    adc_disable_scan_mode(ADC1);
    //adc_set_continuous_conversion_mode(ADC1);
    adc_disable_external_trigger_regular(ADC1);
    //adc_set_right_aligned(ADC1);
    adc_set_sample_time(ADC1, ADC_PIN_hum, ADC_SMPR_SMP_55DOT5CYC); /* Set sample time */

    /* Calibrate ADC1 */
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



void control_leds_based_on_hum(uint16_t hum)
{
    uint16_t humidity = (hum * 3.3 / 4096.0) * 100; // Convert ADC value to humidity

    /* Control LEDs based on humidity range */
    if (humidity < THRESHOLD_HUM_M)
    {
        gpio_clear(RED_LED_PORT, RED_LED_PIN);       /* Red LED off */
    }
    else
    {
        gpio_set(RED_LED_PORT, RED_LED_PIN);         /* Red LED on */
    }
}
/*
void configure_TIMER()
{
  rcc_periph_clock_enable(RCC_TIM2);
  timer_set_mode(TIM2, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
  timer_set_prescaler(TIM2, 0x0000);
  timer_set_period(TIM2, 0x0000);
  timer_enable_irq(TIM2, TIM_DIER_UIE);
  nvic_enable_irq(NVIC_TIM2_IRQ);
  timer_enable_counter(TIM2);
}

void configure_DMA() 
{
  rcc_periph_clock_enable(RCC_DMA1);
  dma_set_peripheral_address(DMA1, DMA_CHANNEL1, (uint32_t)&USART1_DR);
  dma_set_memory_address(DMA1, DMA_CHANNEL1, (uint32_t)&usart1_rx_buffer);
  dma_set_number_of_data(DMA1, DMA_CHANNEL1, sizeof(usart1_rx_buffer));
  dma_set_memory_size(DMA1, DMA_CHANNEL1, DMA_CCR_MSIZE_8BIT);
  dma_set_peripheral_size(DMA1, DMA_CHANNEL1, DMA_CCR_PSIZE_8BIT);
  dma_enable_memory_increment_mode(DMA1, DMA_CHANNEL1);
  dma_disable_peripheral_increment_mode(DMA1, DMA_CHANNEL1);
  dma_enable_circular_mode(DMA1, DMA_CHANNEL1);
  dma_set_read_from_peripheral(DMA1, DMA_CHANNEL1);
  dma_enable_transfer_complete_interrupt(DMA1, DMA_CHANNEL1);
  dma_enable_channel(DMA1, DMA_CHANNEL1);
  nvic_enable_irq(NVIC_DMA1_CHANNEL1_IRQ);
}

void configure_UART()
{
  rcc_periph_clock_enable(RCC_USART1);
  usart_set_baudrate(USART1, 9600);
  usart_set_databits(USART1, 8);
  usart_set_stopbits(USART1, USART_STOPBITS_1);
  usart_set_parity(USART1, USART_PARITY_NONE);
  usart_set_mode(USART1, USART_MODE_TX_RX);
  usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
  usart_enable(USART1);
  usart_enable_rx_interrupt(USART1);
  nvic_enable_irq(NVIC_USART1_IRQ);
}

void configure_EXTI()
{
  rcc_periph_clock_enable(RCC_GPIOA);
  gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, GPIO0);
  gpio_set(GPIOA, GPIO0);
  rcc_periph_clock_enable(RCC_AFIO);
  exti_select_source(EXTI0, GPIOA);
  exti_set_trigger(EXTI0, EXTI_TRIGGER_RISING);
  exti_enable_request(EXTI0);
  nvic_enable_irq(NVIC_EXTI0_IRQ);
}
*/