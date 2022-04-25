//#include <avr/io.h>
//#include <avr/interrupt.h>
//#include <math.h>

#ifndef A4988_H
#define A4988_H

#define DIR PB4
#define STEP PB3
#define SLEEP PB2 // active low
#define RESET PB1 // active low
#define MS3 PB0

#define ENABLE PD5 // active low
#define MS1 PD6
#define MS2 PD7

#define SW_NEG PD2
#define SW_POS PD4


#ifdef __cplusplus
	extern "C" {
#endif

typedef enum motor_state{
	STOPPED = 0,
	MOVING = 1
} motor_state_t;

typedef enum switch_state{
	FREE = 0,
    FAULT = 1,
	LIMIT_NEG = 2,
	LIMIT_POS = 3
} switch_state_t;

typedef enum run_mode{
    NORMAL = 0,
    HOME_RUN_POS = 1,
    HOME_RUN_NEG = 2,
    RETURN_FROM_NEG = 3,
	RETURN_FROM_POS = 4
} run_mode_t;

typedef enum motor_direction{
	CW = 0, 
	CCW = 1
} motor_direction_t;

typedef enum microstep{
	FULL = 1,
	HALF = 2,
	QUARTER = 4,
	EIGHTH = 8,
	SIXTEENTH = 16
} microstep_t;

extern volatile motor_state_t STATE;
extern volatile microstep_t MICROSTEPS;


/* Internal functions */
/**
 * Setup 16bit Timer1 of the AVR controller for CTC interrupts.
 */
void initialize_timer1();

/**
 * Let Timer1 run to start the preconfigured motor movement.
 */
void run();

/**
 * Stop Timer1 to stop the motor movement. This will instantly stop the
 * motor, so don't call this function during normal operation. This
 * should only be used internally! Use soft_stop() instead!
 */
void halt();

/**
 * Calculate the number of steps for acceleration and deceleration and
 * prepare the movement.
 */
void calculate_steps(uint32_t steps);


void set_microstepping(microstep_t stepping);


void update_state();



/* Interface functions */
/**
 * Move the motor to a new position relative to its current one. The
 * distance to move is passed in units of full steps. The Motor will
 * execute the movement with the precision of its microstepping mode.
 */
void move_relative(float distance);

/*
void home_run_pos();

void home_run_neg();
*/

/**
 * Stop the current motor movement without exceeding the configured 
 * deceleration. 
 */
void soft_stop();

void set_max_speed(uint16_t max_speed);

void set_acceleration(uint16_t acceleration);

void set_deceleration(uint16_t deceleration);

void set_position(float cnt);

uint16_t get_speed_limit();

uint16_t get_acceleration();

uint16_t get_deceleration();

/**
 * Returns a float of the current motor position.
 */
float get_position();

motor_state_t get_motor_state();

switch_state_t get_switch_state();

#ifdef __cplusplus
	}
#endif

#endif
