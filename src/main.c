#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/usart.h>

#include "adc.h"
#include "common.h"
#include "pins.h"
#include "pwm.h"
#include "uart.h"

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
    buzzer_mode = OFF;

    while (TRUE)
    {
        /* Active monitor */
        if (buzzer_mode == ON || alarm_activation_mode == ON)
        {
            control_pwm(DELAY_PWM, buzzer_mode);
        }
        else
        {
            timer_set_oc_value(TIM2, TIM_OC4, 0);
        }
        if (analyze_proc_flag == CAN_ANALYZE)
        {
            analyze_proc_flag = ANALYZING;
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

/**
 * @brief DMA configurations.
 * Confgured for move ADC data to memory in adc_buffer.
 * Circular mode is set.
 */
void dma_setup(void)
{
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
/**
 * @brief Systick handler.
 * Every SYSTICK_INTERVAL_MS, the system will analyze the environment.
 * This is used to avoid inconsistencies in the environment analysis
 */
void sys_tick_handler(void)
{
    if (analyze_proc_flag == ANALYZED)
    {
        update_vib_frequency();
        index_hist_vib++;
    }
    if (index_hist_vib >= _MAX_VIB_N)
    {
        index_hist_vib = 0;
        analyze_proc_flag = CAN_ANALYZE;
    }
}
/**
 * @brief External interrupt handler.
 * Changes the alarm activation.
 */
void exti0_isr(void)
{
    exti_reset_request(EXTI0);
    alarm_activation_mode = !alarm_activation_mode;
}
/**
 * @brief Configures GPIO.
 */
void gpio_setup(void)
{
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_GPIOC);
    gpio_set_mode(LED_PORT, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GREEN_LED_PIN);
    gpio_set_mode(LED_PORT, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, RED_LED_PIN);
    gpio_set_mode(LED_PORT, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, YELLOW_LED_PIN);
    gpio_set_mode(LED_PORT, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, LED_TX);
    gpio_set_mode(BUZZER_PORT, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, BUZZER_PIN);
    gpio_set_mode(UART_PORT, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART3_TX);
    gpio_set_mode(UART_PORT, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO_USART3_RX);
    gpio_set_mode(BUTTON_PORT, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, BUTTON_PIN);
    gpio_set(BUTTON_PORT, BUTTON_PIN);
    nvic_enable_irq(NVIC_EXTI0_IRQ);
    exti_select_source(EXTI0, BUZZER_PORT);
    exti_set_trigger(EXTI0, EXTI_TRIGGER_FALLING);
    exti_enable_request(EXTI0);
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
 * @brief Configure the ADC.
 * Sets an structure for intercalate the ADC channels.
 * Sets the ADC in continuous mode and DMA mode.
 */
void adc_setup(void)
{
    rcc_periph_clock_enable(RCC_ADC1);
    rcc_periph_clock_enable(RCC_GPIOA);
    gpio_set_mode(ADC_PORT, GPIO_MODE_INPUT, GPIO_CNF_INPUT_ANALOG, ADC_PIN_vib);
    gpio_set_mode(ADC_PORT, GPIO_MODE_INPUT, GPIO_CNF_INPUT_ANALOG, ADC_PIN_hum);
    ADC_CR2(ADC1) &= ~ADC_CR2_ADON;                         // ADC off para configurar
    ADC_CR2(ADC1) |= ADC_CR2_CONT;                          // modo de conversion continua
    ADC_SMPR2(ADC1) |= (0x7 << (3 * 1)) | (0x7 << (3 * 2)); // muestreo para los canales (tiempos)
    ADC_CR1(ADC1) |= ADC_CR1_SCAN;                          // activar el modo scan
    ADC_SQR3(ADC1) =
        (ADC_CHANNEL_hum << (5 * 0)) | (ADC_CHANNEL_vib << (5 * 1)); // hum en primer lugar y vib en el segundo
    ADC_SQR1(ADC1) = (1 << 20);                                      // L[3:0] = 1 para 2 conversiones
    ADC_CR2(ADC1) |= ADC_CR2_DMA;                                    // activar el dma
    ADC_CR2(ADC1) |= ADC_CR2_ADON;                                   // ADC on
    ADC_CR2(ADC1) |= ADC_CR2_RSTCAL;                                 // calibracion
    while (ADC_CR2(ADC1) & ADC_CR2_RSTCAL);
    ADC_CR2(ADC1) |= ADC_CR2_CAL;
    while (ADC_CR2(ADC1) & ADC_CR2_CAL);
    ADC_CR2(ADC1) |= ADC_CR2_ADON; // ADC on para empezar a convertir
    ADC_CR2(ADC1) |= ADC_CR2_SWSTART;
}
/**
 * @brief Configure PWM.
 *
 */
void configure_PWM(void)
{
    rcc_periph_clock_enable(RCC_TIM2);
    timer_set_mode(TIM2, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
    timer_set_prescaler(TIM2, 71);
    timer_set_period(TIM2, 1000);
    timer_set_oc_mode(TIM2, TIM_OC4, TIM_OCM_PWM1);
    timer_enable_oc_output(TIM2, TIM_OC4);
    timer_set_oc_value(TIM2, TIM_OC4, 0);
    timer_enable_counter(TIM2);
}
/**
 * @brief Configure UART communication.
 *
 */
void configure_UART()
{
    rcc_periph_clock_enable(RCC_USART3);
    usart_set_baudrate(USART3, 9600);
    usart_set_databits(USART3, 8);
    usart_set_stopbits(USART3, USART_STOPBITS_1);
    usart_set_mode(USART3, USART_MODE_TX);
    usart_set_parity(USART3, USART_PARITY_NONE);
    usart_set_flow_control(USART3, USART_FLOWCONTROL_NONE);
    usart_enable(USART3);
}
/**
 * @brief ISR DMA.
 * Calculates the average of the ADC values and converts them to environment values.
 */
void dma1_channel1_isr(void)
{
    if (dma_get_interrupt_flag(DMA1, DMA_CHANNEL1, DMA_TCIF))
    {
        dma_clear_interrupt_flags(DMA1, DMA_CHANNEL1, DMA_TCIF);
        uint16_t sum_vib = 0;
        uint16_t sum_hum = 0;
        for (int i = 0; i < ADC_BUFFER_SIZE; i += 2)
        {
            sum_vib += adc_buffer[i];     // Valores de vibraciones
            sum_hum += adc_buffer[i + 1]; // Valores de humedad
        }
        sum_vib /= (ADC_BUFFER_SIZE / 2);
        sum_hum /= (ADC_BUFFER_SIZE / 2);
        convert_adc_to_env(sum_vib, sum_hum);
    }
}
