#define _MAX_VIB_N 50 // cantidad maxima de vibraciones a guardar
#define BYTE_MASK 0xFF

#define THRESHOLD_VIB_FREQ_H 80
#define THRESHOLD_VIB_FREQ_M 50
#define THRESHOLD_VIB_FREQ_L 0

#define THRESHOLD_HUM_H 80
#define THRESHOLD_HUM_M 50
#define THRESHOLD_HUM_L 0

#define BUZZER_FREQ 1000

/* Define SysTick interval */
#define SYSTICK_INTERVAL_MS 100

typedef enum {
  ANALYZING,
  ANALYZED,
  CAN_ANALYZE,
} analyze_flag_t;



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


