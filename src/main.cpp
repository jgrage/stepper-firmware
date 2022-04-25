#include <avr/wdt.h>
#include <util/delay.h>
#include <Arduino.h>
#include <scpiparser.h>
#include "A4988.h"
#include "scpi_functions.h"


struct scpi_parser_context ctx;
char response_buffer[BUF_LEN];
uint8_t response_len;

volatile uint8_t UPDATE_FLAG;

void setup() {
  
  // initialize A4988 pins as an outputs
  DDRB |= (_BV(DIR) | _BV(STEP) | _BV(SLEEP) | _BV(RESET) | _BV(MS3) | _BV(PB5));
  DDRD |= (_BV(ENABLE) | _BV(MS1) | _BV(MS2));
  
  // initialize switches as tri state inputs
  DDRD &= ~(_BV(SW_NEG) | _BV(SW_POS));
  PORTD &= ~(_BV(SW_NEG) | _BV(SW_POS));     // Tri-State
  
  // driver outputs low while initializing
  PORTB &= ~( _BV(SLEEP) | _BV(RESET) );
  PORTD |= _BV(ENABLE);
  
  PCICR |= _BV(PCIE2);
  PCMSK2 = 0x00;
  PCMSK2 |= (_BV(PCINT18) | _BV(PCINT20));
  
  
  Serial.begin(9600);
  initialize_timer1();
  
  sei();	// enable interrupts
  
  

  struct scpi_command* motor;
  struct scpi_command* limit;
  struct scpi_command* move;
  struct scpi_command* home;
  
  scpi_init(&ctx);
  
  scpi_register_command(ctx.command_tree, SCPI_CL_SAMELEVEL, "*IDN?", 5, "*IDN?", 5, identify);
  
  motor = scpi_register_command(ctx.command_tree, SCPI_CL_CHILD, "MOTOR", 5, "MOT", 3, NULL);
  limit = scpi_register_command(motor, SCPI_CL_CHILD, "LIMIT", 5, "LIM", 3, NULL);
  move = scpi_register_command(motor, SCPI_CL_CHILD, "MOVE", 4, "MOV", 3, NULL);
  home = scpi_register_command(motor, SCPI_CL_CHILD, "HOME", 4, "HOM", 3, NULL);
  
  
  scpi_register_command(move, SCPI_CL_CHILD, "ABSOLUTE", 8, "ABS", 3, scpi_move_absolute);
  scpi_register_command(move, SCPI_CL_CHILD, "RELATIVE", 8, "REL", 3, scpi_move_relative);
  
  scpi_register_command(motor, SCPI_CL_CHILD, "STOP", 4, "STP", 3, scpi_soft_stop);
  scpi_register_command(motor, SCPI_CL_CHILD, "STATE?", 6, "ST?", 3, scpi_get_state);
  
  
  scpi_register_command(motor, SCPI_CL_CHILD, "POSITION", 8, "POS", 3, scpi_set_position);
  scpi_register_command(motor, SCPI_CL_CHILD, "POSITION?", 9, "POS?", 4, scpi_get_position);
  
  scpi_register_command(motor, SCPI_CL_CHILD, "ACCELERATION", 12, "ACC", 3, scpi_set_acceleration);
  scpi_register_command(motor, SCPI_CL_CHILD, "ACCELERATION?", 13, "ACC?", 4, scpi_get_acceleration);
  
  scpi_register_command(motor, SCPI_CL_CHILD, "DECELERATION", 12, "DEC", 3, scpi_set_deceleration);
  scpi_register_command(motor, SCPI_CL_CHILD, "DECELERATION?", 13, "DEC?", 4, scpi_get_deceleration);
  
  scpi_register_command(motor, SCPI_CL_CHILD, "SPEED", 5, "SP", 2, scpi_set_max_speed);
  scpi_register_command(motor, SCPI_CL_CHILD, "SPEED?", 6, "SP?", 3, scpi_get_speed_limit);
  
  scpi_register_command(limit, SCPI_CL_CHILD, "POSITIVE", 8, "POS", 3, scpi_set_softlimit_pos);
  scpi_register_command(limit, SCPI_CL_CHILD, "POSITIVE?", 9, "POS?", 4, scpi_get_softlimit_pos);
  
  scpi_register_command(limit, SCPI_CL_CHILD, "NEGATIVE", 8, "NEG", 3, scpi_set_softlimit_neg);
  scpi_register_command(limit, SCPI_CL_CHILD, "NEGATIVE?", 9, "NEG?", 4, scpi_get_softlimit_neg);
  
  scpi_register_command(home, SCPI_CL_CHILD, "POSITIVE", 8, "POS", 3, scpi_home_pos);
  scpi_register_command(home, SCPI_CL_CHILD, "NEGATIVE", 8, "NEG", 3, scpi_home_neg);
}


void loop(){
	set_microstepping(QUARTER);
    
    // driver outputs high after startup
    PORTB |= (_BV(SLEEP) | _BV(RESET));
    PORTD &= ~_BV(ENABLE);
    
    update_state();
    
    char line_buffer[128];
	unsigned char read_length;

	while(1){   
        if (Serial.available()){
            read_length = Serial.readBytesUntil('\n', line_buffer, 128);
            
            if(read_length > 0)
            {
                scpi_execute_command(&ctx, line_buffer, read_length);
            }
		
            if (response_len > 0) // hier wäre response_len klüger
            {
                Serial.write(response_buffer, response_len);
                response_len = 0;
            }
        }
        
        if(UPDATE_FLAG == 1){
            _delay_ms(50);
            update_state();
            UPDATE_FLAG = 0;
        }
        
        // debug printing
        if (response_len > 0) // hier wäre response_len klüger
            {
                Serial.write(response_buffer, response_len);
                response_len = 0;
            }
		
	}
}
