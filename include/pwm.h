#pragma once

/**
 * pwm delay
 */
#define DELAY_PWM 100000

/*
Functions prototypes ------------------------------------------------------
*/
/**
 * @brief Configures the PWM.
 * @param void
 */
void configure_PWM(void);
/**
 * @brief Manages the PWM control of the buzzer.
 *
 * @param delay
 * @param initial_buzzer_mode
 */
void control_pwm(uint32_t delay, int initial_buzzer_mode);