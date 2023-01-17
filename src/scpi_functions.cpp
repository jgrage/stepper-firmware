// SPDX-License-Identifier: MIT
/*
 * SCPI functions for stepper motor controller
 * Copyright (c) 2022, Jonas Grage <grage@physik.tu-berlin.de>
 */
 
#include <scpiparser.h>
#include "scpi_functions.h"
#include "A4988.h"

float softlimit_pos = INT_MAX;
float softlimit_neg = INT_MIN;


/**
 * Respond to *IDN?
 */
scpi_error_t identify(struct scpi_parser_context* context, struct scpi_token* command){
  response_len = snprintf(response_buffer, BUF_LEN, "BLiX Stepper Motor Controller rev. 1.0\n");
  scpi_free_tokens(command);
  return SCPI_SUCCESS;
}

/**
 * 
 */
scpi_error_t scpi_get_position(struct scpi_parser_context* context, struct scpi_token* command){
  response_len = snprintf(response_buffer, BUF_LEN, "%.2f\n", (double)(get_position()));
  scpi_free_tokens(command);
  return SCPI_SUCCESS;
}

/**
 * 
 */
scpi_error_t scpi_get_acceleration(struct scpi_parser_context* context, struct scpi_token* command){
  response_len = snprintf(response_buffer, BUF_LEN, "%d\n", get_acceleration());
  scpi_free_tokens(command);
  return SCPI_SUCCESS;
}

/**
 * 
 */
scpi_error_t scpi_get_deceleration(struct scpi_parser_context* context, struct scpi_token* command){
  response_len = snprintf(response_buffer, BUF_LEN, "%d\n", get_deceleration());
  scpi_free_tokens(command);
  return SCPI_SUCCESS;
}

/**
 * 
 */
scpi_error_t scpi_get_softlimit_neg(struct scpi_parser_context* context, struct scpi_token* command){
  response_len = snprintf(response_buffer, BUF_LEN, "%.2f\n", (double)(softlimit_neg));
  scpi_free_tokens(command);
  return SCPI_SUCCESS;
}

/**
 * 
 */
scpi_error_t scpi_get_softlimit_pos(struct scpi_parser_context* context, struct scpi_token* command){
  response_len = snprintf(response_buffer, BUF_LEN, "%.2f\n", (double)(softlimit_pos));
  scpi_free_tokens(command);
  return SCPI_SUCCESS;
}

/**
 * 
 */
scpi_error_t scpi_soft_stop(struct scpi_parser_context* context, struct scpi_token* command){
  soft_stop();
  scpi_free_tokens(command);
  return SCPI_SUCCESS;
}

/**
 * 
 */
scpi_error_t scpi_get_speed_limit(struct scpi_parser_context* context, struct scpi_token* command){
  response_len = snprintf(response_buffer, BUF_LEN, "%d\n", get_speed_limit());
  scpi_free_tokens(command);
  return SCPI_SUCCESS;
}

/**
 * 
 */
scpi_error_t scpi_move_relative(struct scpi_parser_context* context, struct scpi_token* command){
  struct scpi_token* args;
  struct scpi_numeric output_numeric;
  args = command;

  while(args != NULL && args->type == 0){
    args = args->next;
  }

  float output_value;
  output_numeric = scpi_parse_numeric(args->value, args->length, 0, 0, 0);
  
  if(get_motor_state() != STOPPED){
	scpi_error error;
	error.id = -300;
	error.description = "Command error: Motor busy";
	error.length = 25;
	response_len = snprintf(response_buffer, BUF_LEN, "%d->%s\n", error.id, error.description);
	scpi_queue_error(&ctx, error);
	scpi_free_tokens(command);
	return SCPI_SUCCESS;
  }
  
  if(output_numeric.length == 0){
    output_value = output_numeric.value;
    
    if(output_value + get_position() < softlimit_neg){
		scpi_error error;
		error.id = -301;
		error.description = "Command error: Position below negative softlimit";
		error.length = 48;
		scpi_queue_error(&ctx, error);
		scpi_free_tokens(command);
		return SCPI_SUCCESS;
	}
	
	if(output_value + get_position() > softlimit_pos){
		scpi_error error;
		error.id = -302;
		error.description = "Command error: Position above positive softlimit";
		error.length = 48;
		scpi_queue_error(&ctx, error);
		scpi_free_tokens(command);
		return SCPI_SUCCESS;
	}
	
	else{
		move_relative(output_value);
		scpi_free_tokens(command);
		return SCPI_SUCCESS;
	}

  }

  else{
    scpi_error error;
    error.id = -200;
    error.description = "Command error: Invalid unit";
    error.length = 27;
    scpi_queue_error(&ctx, error);
    scpi_free_tokens(command);
    return SCPI_SUCCESS;
  }
}

/**
 * 
 */
scpi_error_t scpi_move_absolute(struct scpi_parser_context* context, struct scpi_token* command){
  struct scpi_token* args;
  struct scpi_numeric output_numeric;
  args = command;

  while(args != NULL && args->type == 0){
    args = args->next;
  }

  float output_value;
  output_numeric = scpi_parse_numeric(args->value, args->length, 0, 0, 0);
  
  if(get_motor_state() == MOVING){
	soft_stop();
  }
  
  if(output_numeric.length == 0){
    output_value = output_numeric.value;
    
    if(output_value < softlimit_neg){
		scpi_error error;
		error.id = -301;
		error.description = "Command error: Position below negative softlimit";
		error.length = 48;
		scpi_queue_error(&ctx, error);
		scpi_free_tokens(command);
		return SCPI_SUCCESS;
	}
	
	if(output_value > softlimit_pos){
		scpi_error error;
		error.id = -302;
		error.description = "Command error: Position above positive softlimit";
		error.length = 48;
		scpi_queue_error(&ctx, error);
		scpi_free_tokens(command);
		return SCPI_SUCCESS;
	}
	
	else{
		move_relative(output_value - get_position());
		scpi_free_tokens(command);
		return SCPI_SUCCESS;
	}

  }

  else{
    scpi_error error;
    error.id = -200;
    error.description = "Command error: Invalid unit";
    error.length = 27;
    scpi_queue_error(&ctx, error);
    scpi_free_tokens(command);
    return SCPI_SUCCESS;
  }
}

/**
 * 
 */
scpi_error_t scpi_set_acceleration(struct scpi_parser_context* context, struct scpi_token* command){
  struct scpi_token* args;
  struct scpi_numeric output_numeric;
  args = command;

  while(args != NULL && args->type == 0){
    args = args->next;
  }

  float output_value;
  output_numeric = scpi_parse_numeric(args->value, args->length, 100, 10, 400);
  
  if(output_numeric.length == 0){
    output_value = output_numeric.value;

  }

  else{
    scpi_error error;
    error.id = -200;
    error.description = "Command error: Invalid unit";
    error.length = 27;
    scpi_queue_error(&ctx, error);
    scpi_free_tokens(command);
    return SCPI_SUCCESS;
  }

  set_acceleration((uint16_t)(output_value));
  scpi_free_tokens(command);
  return SCPI_SUCCESS;
}

/**
 * 
 */
scpi_error_t scpi_set_deceleration(struct scpi_parser_context* context, struct scpi_token* command){
  struct scpi_token* args;
  struct scpi_numeric output_numeric;
  args = command;

  while(args != NULL && args->type == 0){
    args = args->next;
  }

  float output_value;
  output_numeric = scpi_parse_numeric(args->value, args->length, 100, 10, 400);
  
  if(output_numeric.length == 0){
    output_value = output_numeric.value;
  }

  else{
    scpi_error error;
    error.id = -200;
    error.description = "Command error: Invalid unit";
    error.length = 27;
    scpi_queue_error(&ctx, error);
    scpi_free_tokens(command);
    return SCPI_SUCCESS;
  }

  set_deceleration((uint16_t)(output_value));
  scpi_free_tokens(command);
  return SCPI_SUCCESS;
}

/**
 * 
 */
scpi_error_t scpi_set_max_speed(struct scpi_parser_context* context, struct scpi_token* command){
  struct scpi_token* args;
  struct scpi_numeric output_numeric;
  args = command;

  while(args != NULL && args->type == 0){
    args = args->next;
  }

  float output_value;
  output_numeric = scpi_parse_numeric(args->value, args->length, 200, 10, 800);
  
  if(output_numeric.length == 0){
    output_value = output_numeric.value;
  }

  else{
    scpi_error error;
    error.id = -200;
    error.description = "Command error: Invalid unit";
    error.length = 27;
    scpi_queue_error(&ctx, error);
    scpi_free_tokens(command);
    return SCPI_SUCCESS;
  }

  set_max_speed((uint16_t)(output_value));
  scpi_free_tokens(command);
  return SCPI_SUCCESS;
}

/**
 * 
 */
scpi_error_t scpi_set_softlimit_pos(struct scpi_parser_context* context, struct scpi_token* command){
  struct scpi_token* args;
  struct scpi_numeric output_numeric;
  args = command;

  while(args != NULL && args->type == 0){
    args = args->next;
  }

  float output_value;
  output_numeric = scpi_parse_numeric(args->value, args->length, softlimit_pos, 0, 5);
  
  if(output_numeric.length == 0){
    output_value = output_numeric.value;
  }

  else{
    scpi_error error;
    error.id = -200;
    error.description = "Command error: Invalid unit";
    error.length = 27;
    scpi_queue_error(&ctx, error);
    scpi_free_tokens(command);
    return SCPI_SUCCESS;
  }

  softlimit_pos = output_value;
  scpi_free_tokens(command);
  return SCPI_SUCCESS;
}

/**
 * 
 */
scpi_error_t scpi_set_softlimit_neg(struct scpi_parser_context* context, struct scpi_token* command){
  struct scpi_token* args;
  struct scpi_numeric output_numeric;
  args = command;

  while(args != NULL && args->type == 0){
    args = args->next;
  }

  float output_value;
  output_numeric = scpi_parse_numeric(args->value, args->length, softlimit_neg, 0, 5);
  
  if(output_numeric.length == 0){
    output_value = output_numeric.value;
  }

  else{
    scpi_error error;
    error.id = -200;
    error.description = "Command error: Invalid unit";
    error.length = 27;
    scpi_queue_error(&ctx, error);
    scpi_free_tokens(command);
    return SCPI_SUCCESS;
  }

  softlimit_neg = output_value;
  scpi_free_tokens(command);
  return SCPI_SUCCESS;
}


/**
 * 
 */
scpi_error_t scpi_set_position(struct scpi_parser_context* context, struct scpi_token* command){
  struct scpi_token* args;
  struct scpi_numeric output_numeric;
  args = command;

  while(args != NULL && args->type == 0){
    args = args->next;
  }

  float output_value;
  output_numeric = scpi_parse_numeric(args->value, args->length, get_position(), 0, 0);
  
  if(output_numeric.length == 0){
    output_value = output_numeric.value;
  }

  else{
    scpi_error error;
    error.id = -200;
    error.description = "Command error: Invalid unit";
    error.length = 27;
    scpi_queue_error(&ctx, error);
    scpi_free_tokens(command);
    return SCPI_SUCCESS;
  }

  set_position(output_value);
  scpi_free_tokens(command);
  return SCPI_SUCCESS;
}


scpi_error_t scpi_home_pos(struct scpi_parser_context* context, struct scpi_token* command){
  //home_run_pos();
  move_relative(40000);
  scpi_free_tokens(command);
  return SCPI_SUCCESS;
}


scpi_error_t scpi_home_neg(struct scpi_parser_context* context, struct scpi_token* command){
  //home_run_neg();
  move_relative(-40000);
  scpi_free_tokens(command);
  return SCPI_SUCCESS;
}


scpi_error_t scpi_get_state(struct scpi_parser_context* context, struct scpi_token* command){
    if(get_motor_state() == MOVING){
        response_len = snprintf(response_buffer, BUF_LEN, "MOVING\n");
    }
    
    else{
        switch_state_t tmp = get_switch_state();
        
        if(tmp == FREE){
            response_len = snprintf(response_buffer, BUF_LEN, "STOPPED\n");
        }
        
        else if(tmp == LIMIT_POS){
            response_len = snprintf(response_buffer, BUF_LEN, "LIM+\n");
        }
        
        else if(tmp == LIMIT_NEG){
            response_len = snprintf(response_buffer, BUF_LEN, "LIM-\n");
        }
        
        else{
            response_len = snprintf(response_buffer, BUF_LEN, "FAULT\n");
        }
    }
  
    scpi_free_tokens(command);
    return SCPI_SUCCESS;
}

/**
 * 
 */
/*
scpi_error_t scpi_set_microstepping(struct scpi_parser_context* context, struct scpi_token* command){
  struct scpi_token* args;
  struct scpi_numeric output_numeric;
  args = command;

  while(args != NULL && args->type == 0){
    args = args->next;
  }

  float output_value;
  output_numeric = scpi_parse_numeric(args->value, args->length, 0, 0, 5);
  
  if(output_numeric.length == 0){
    output_value = output_numeric.value;
  }

  else{
    scpi_error error;
    error.id = -200;
    error.description = "Command error;Invalid unit";
    error.length = 26;
    respond(output_numeric.length);

    scpi_queue_error(&ctx, error);
    scpi_free_tokens(command);
    return SCPI_SUCCESS;
  }

  set_microstepping(output_value);
  scpi_free_tokens(command);
  return SCPI_SUCCESS;
}
*/
