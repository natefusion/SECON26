#include "esp_err.h"
#include <stdint.h>

typedef struct {
  int pin; // GPIO pin motor will run on
  char *task_name;
} motor_config;

// motor interface used by user
typedef void *motor_handler;

// creates motor and starts motor thread
motor_handler *init_motor(motor_config *);

// Sets motor speed as long as speed is valid
esp_err_t set_motor_speed(motor_handler *, uint32_t);

// Stops motor thread and frees shared memory
// THIS THROWS AN ERROR DO NOT WORRY EVERYTHING IS FINE!!!!
void destroy_motor(motor_handler *);

// returns the maximum duty cyle in ticks
uint32_t get_max_duty_cycle();

// returns the current duty cycle in ticks
uint32_t get_current_duty_cycle(motor_handler *);

// Option to set motor speed via duty cycle percentage rather than ticks
// must be between 0.0 and 1.0
esp_err_t set_motor_speed_pcnt(motor_handler *motor_arg, float duty_cycle_pcnt);

// Option to get duty cycle as a percentage rather than in ticks
float get_duty_cycle_pcnt(motor_handler *motor_arg);
