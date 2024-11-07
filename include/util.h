#define _MAX_VIB_N 50 // cantidad maxima de vibraciones a guardar

#define THRESHOLD_VIB_FREQ_H 1000
#define THRESHOLD_VIB_FREQ_M 500
#define THRESHOLD_VIB_FREQ_L 0

#define THRESHOLD_HUM_H 1000
#define THRESHOLD_HUM_M 50
#define THRESHOLD_HUM_L 0



/* Define SysTick interval */
#define SYSTICK_INTERVAL_MS 100


typedef struct {
  void (*system_clock_setup)(void);
  void (*gpio_setup)(void);
  void (*adc_setup)(void);
  void (*timer2_setup)(void);
} setup_functions;


setup = {
.system_clock_setup = system_clock_setup,
.gpio_setup = gpio_setup,
.adc_setup = adc_setup,
.timer2_setup = timer2_setup

};    // Esto no se si esta bien implementarlo en un header
      // pero deja limpio el main
    


typedef enum {
  ANALYZING,
  ANALYZED,
  CAN_ANALYZE,
} analyze_flag_t;


void system_clock_setup(void);
void gpio_setup(void);
void adc_setup(void);
void timer2_setup(void);


uint8_t index_hist_vib; // para indexar el vector de vibraciones
uint16_t vib_freq;  // frecuencia de los sismos 
uint16_t historic_vib[_MAX_VIB_N];  // vibraciones pasadas de los sismos
uint16_t env_vib;
uint16_t env_hum;

analyze_flag_t analyze_proc_flag = CAN_ANALYZE; 

int buzzer_mode; // estado del buzzer ON/OFF

void update_env_state(uint16_t adc_vib, uint16_t adc_hum);
void update_vib_frequency();
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
void analyze_and_update_system();


