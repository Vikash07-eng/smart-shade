#pragma once
#include <stdbool.h>
#include <stdint.h>

/* Step/dir stepper driver with trapezoidal acceleration.
 * Pulses are generated from a 1 MHz hardware timer interrupt (gptimer),
 * so WiFi/HTTP activity cannot make the motor stutter. */

void stepper_init(int pin_step, int pin_dir, int pin_en, bool invert_dir);

/* speed in steps/s, accel in steps/s^2 (takes effect on the next step) */
void stepper_set_speed(uint32_t speed, uint32_t accel);

void stepper_move_to(int32_t target);   /* absolute, may reverse safely */
void stepper_move_rel(int32_t delta);   /* relative to current target (or position when idle) */
void stepper_stop(void);                /* decelerate to a stop */

bool stepper_running(void);
int32_t stepper_pos(void);
int32_t stepper_target(void);
void stepper_set_pos(int32_t pos);      /* only while idle */

/* Called from the driver's housekeeping task once the motor has been idle
 * and the driver has been disabled (a good moment to save the position). */
void stepper_set_idle_cb(void (*cb)(void));
