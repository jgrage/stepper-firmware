// SPDX-License-Identifier: MIT
/*
 * Allegro A4988 stepper driver functions
 * Copyright (c) 2022, Jonas Grage <grage@physik.tu-berlin.de>
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <math.h>

#include "A4988.h"

/*
#define A4988_PORT PORTB
#define A4988_DDR DDRB

#define DIR PB0
#define STEP PB1
#define MS1 PB2
#define MS2 PB4
#define MS3 PB3
*/

volatile microstep_t MICROSTEPS = HALF;
volatile motor_state_t STATE;
volatile switch_state_t SW_STATE;
volatile run_mode_t RUN;
volatile motor_direction_t DIRECTION;

volatile float speedprint;
volatile float speed;

volatile uint16_t speed_limit = 200;	// speed limit in full steps per s
volatile uint16_t acc = 100;        	// acceleration in full steps per second per step (NOT a time derivative!)
volatile uint16_t dec = 100;			// deceleration in full steps per second per step (NOT a time derivative!)

volatile uint32_t step;
volatile uint32_t total_steps;
volatile uint32_t steps_to_move;
volatile uint32_t steps_to_accelerate;
volatile uint32_t steps_to_decelerate;

volatile int32_t MICROSTEPS_CNT = 0;	//integer value of current position in 1/16 steps (minimum microstepping)
extern volatile uint8_t UPDATE_FLAG;


/*
 * Setup Timer1 for Compare Match ISR
 */
void initialize_timer1(){
  TCCR1A = 0x00;
  TCCR1B = 0x00;
  TCNT1 = 0x00;
  OCR1A = 780;				// preload compare unit for first prescaler start
  TCCR1B |= _BV(WGM12);		// CTC mode
  TIMSK1 |= _BV(OCIE1A);	// enable compare match interrupt
}

/* 
 * Enable the prescaler to let Timer1 run. This will start to call the
 * TIMER1_COMPA ISR in accelerating/decelerating time intervals. And the
 * motor will start to move.
 */
void run(){
    TCCR1B |= (_BV(CS12) | _BV(CS10));
}

/* 
 * Disable the prescaler to halt the timer and set the timer1 (OCR1A) to 10 Hz.
 * This is a preparation for the next run, so that the run() function can start the pulse generation
 * by simply enabling the prescaler again. The 10 Hz initial timer value will make sure the first step
 * takes place 0.1s after calling the run() function.
 */
void halt(){
    OCR1A = 780;
    TCCR1B &= ~( _BV(CS12) | _BV(CS11) | _BV(CS10) );
}

/*
 * Calculate the number of steps for acceleration and decceleration ramps. steps in units of current microstepping
 */
void calculate_steps(uint32_t steps){   
    // calculate acceleration step candidates
    uint32_t acc_steps = (uint32_t)(1.0*speed_limit * speed_limit * MICROSTEPS/ (2.0*acc));
    
    // calculate deceleration step candidates
    uint32_t dec_steps = (uint32_t)(1.0*speed_limit * speed_limit * MICROSTEPS/ (2.0*dec));
    
    // remaining steps will be moving with constant top speed
    if((acc_steps + dec_steps) <= steps){
        steps_to_accelerate = acc_steps;
        steps_to_decelerate = dec_steps;
        steps_to_move = steps - (acc_steps + dec_steps);
    }
    
    // if no steps would remain, calculate new ramp with lower top speed
    else{
        steps_to_move = 0;
        steps_to_accelerate = (uint32_t)(1.0*steps / (1.0 + (1.0*acc/dec)));
        steps_to_decelerate = steps - steps_to_accelerate;
    }
    
    total_steps = steps;
    step = 0;
    speed = 0.0;
}

/* 
 * Generate accelerating pulses with Timer1.
 * Executing the ISR on a Controller with F_CPU of 16MHz will result in a 100µs pulse
 * during acceleration and deceleration. This is mainly caused by the calculation of the
 * new timer value. For the constant speed phase the pulse duration is much shorter (~4-5µs)
 * but well within specs of the A4988 controller (pulse duration > 1µs).
 */
ISR(TIMER1_COMPA_vect){
    // generate rising edge for the pulse on the step pin
    PORTB |= _BV(STEP);
    
    if(step < steps_to_accelerate){
        // acceleration phase
        speed = sqrt(2.0*(step+1)*acc/1.0*MICROSTEPS);/// MICROSTEPS);
        OCR1A = (uint16_t)(F_CPU/(1024*speed) - 1);
    }
    
    else if((step >= steps_to_accelerate) && (step <= (steps_to_accelerate + steps_to_move))){
        // moving with constant top speed
        // helper variable for top speed printing
        speedprint = speed;
    }
    
    else if((step > (steps_to_accelerate + steps_to_move)) && (step < total_steps -1)){
        // deceleration phase. true until second to last step.
        speed = sqrt(2.0*(total_steps - (step+1))*dec/1.0*MICROSTEPS);/// MICROSTEPS);
        OCR1A = (uint16_t)(F_CPU/(1024*speed) - 1);
    }
    
    else{
		// last step. Halt Timer1 and update motor state
        halt();
        STATE = STOPPED;
    }
    
    step++;
    
    uint8_t increment = 16/MICROSTEPS;
    MICROSTEPS_CNT = (DIRECTION == CW) ? MICROSTEPS_CNT + increment : MICROSTEPS_CNT - increment;
    
    // generate falling edge for the pulse on the step pin
    PORTB &= ~_BV(STEP);
}

/*
 * Method for gently stopping the current movement. Useful to call before
 * moving to another position while motor is still busy.
 */
void soft_stop(){
	if(STATE == MOVING){
		steps_to_decelerate = (uint16_t)(speed * speed / (2.0*dec));
		steps_to_move = 0;
		steps_to_accelerate = 0;
		total_steps = steps_to_decelerate;
		step = 0;
	}
}

void set_microstepping(microstep_t stepping){
	switch(stepping){
		case FULL:
			PORTB &= ~_BV(MS3);
			PORTD &= ~(_BV(MS1) | _BV(MS2));
            break;
			
		case HALF:
            PORTB &= ~_BV(MS3);
			PORTD &= ~_BV(MS2);
			PORTD |= _BV(MS1);
			break;
			
		case QUARTER:
			PORTB &= ~_BV(MS3);
			PORTD &= ~_BV(MS1);
			PORTD |= _BV(MS2);
			break;
			
		case EIGHTH:
			PORTB &= ~_BV(MS3);
			PORTD |= (_BV(MS1) | _BV(MS2));
			break;
			
		case SIXTEENTH:
            PORTB |= _BV(MS3);
			PORTD |= (_BV(MS1) | _BV(MS2));
			break;
			
		default: return;
	}
	MICROSTEPS = stepping;
}

ISR(PCINT2_vect){
    UPDATE_FLAG = 1;
}

/** This function will be called each time a limit switch changes its state */
void update_state(){
	/* ToDo: read switches. The input turns low when activated */
	uint8_t neg = (PIND & (1 << SW_NEG));
	uint8_t pos = (PIND & (1 << SW_POS));
	/* ----- */
    
    
    /* high limit switch activated */
	if(neg && !pos){
        halt();
        STATE = STOPPED;
        SW_STATE = LIMIT_POS;
        PORTB |= ( 1 << PB5 );
        
        /* slowly move out of the switch during homerun 
        if (RUN == HOME_RUN_POS){
            RUN = RETURN_FROM_POS;
            move_relative(-1000.0); //TODO: adjust speed and distance
        }*/
    }
    
    /* low limit switch activated */
    else if(!neg && pos){
        halt();
        STATE = STOPPED;
        SW_STATE = LIMIT_NEG;
        PORTB |= ( 1 << PB5 );
        
        /* slowly move out of the switch during homerun 
        if (RUN == HOME_RUN_NEG){
            RUN = RETURN_FROM_NEG;
            move_relative(1000.0); //TODO: adjust speed and distance
        }*/
    }
    
    /* both limit switches activated */
    else if(!neg && !pos){
        halt();
        STATE = STOPPED;
        SW_STATE = FAULT;
        PORTB |= ( 1 << PB5 );
    }

    else{
        SW_STATE = FREE;
        PORTB &= ~( 1 << PB5 );
        
        /* home run complete after limit switch is released 
        if (RUN == RETURN_FROM_POS || RUN == RETURN_FROM_NEG){
            halt();
            STATE = STOPPED;
            RUN = NORMAL; //return to default limit switch operation
        }*/
    }
}


/*
 * Return the current position in full steps
 */
float get_position(){
	return (float)(MICROSTEPS_CNT/16.0);
}


void move_relative(float distance){
	uint32_t dist;
	
	if(distance >= 0.0){
        if(SW_STATE == FAULT || SW_STATE == LIMIT_POS || STATE == MOVING) return;
        dist = (uint32_t)(distance * MICROSTEPS);
        PORTB |= _BV(DIR);
        DIRECTION = CW;
    }
    else{
        if(SW_STATE == FAULT || SW_STATE == LIMIT_NEG || STATE == MOVING) return;
        dist = (uint32_t)(-1.0*distance * MICROSTEPS);
        PORTB &= ~_BV(DIR);
        DIRECTION = CCW;
    }
	
    calculate_steps(dist);
    run();
    STATE = MOVING;
}

void set_max_speed(uint16_t max_speed){
	speed_limit = max_speed;
}

void set_acceleration(uint16_t acceleration){
	acc = acceleration;
}

void set_deceleration(uint16_t deceleration){
	dec = deceleration;
}

void set_position(float cnt){
	MICROSTEPS_CNT = (int32_t)(16.0*cnt);
}

uint16_t get_speed_limit(){
	return speed_limit;
}

uint16_t get_acceleration(){
	return acc;
}

uint16_t get_deceleration(){
	return dec;
}

motor_state_t get_motor_state(){
	return STATE;
}

switch_state_t get_switch_state(){
	return SW_STATE;
}
