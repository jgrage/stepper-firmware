/* SPDX-License-Identifier: MIT */
/*
 * SCPI functions for stepper motor controller
 * Copyright (c) 2022, Jonas Grage <grage@physik.tu-berlin.de>
 */
 
#include <scpiparser.h>

#ifndef SCPI_FUNCTIONS_H
#define SCPI_FUNCTIONS_H

#define BUF_LEN 128
#define INT_MIN -2147483648
#define INT_MAX 2147483647


extern struct scpi_parser_context ctx;
extern char response_buffer[BUF_LEN];
extern uint8_t response_len;


/**
 * Respond to *IDN?
 */
scpi_error_t identify(struct scpi_parser_context* context, struct scpi_token* command);

/**
 * 
 */
scpi_error_t scpi_get_position(struct scpi_parser_context* context, struct scpi_token* command);

/**
 * 
 */
scpi_error_t scpi_get_acceleration(struct scpi_parser_context* context, struct scpi_token* command);

/**
 * 
 */
scpi_error_t scpi_get_deceleration(struct scpi_parser_context* context, struct scpi_token* command);

/**
 * 
 */
scpi_error_t scpi_get_softlimit_neg(struct scpi_parser_context* context, struct scpi_token* command);

/**
 * 
 */
scpi_error_t scpi_get_softlimit_pos(struct scpi_parser_context* context, struct scpi_token* command);

/**
 * 
 */
scpi_error_t scpi_soft_stop(struct scpi_parser_context* context, struct scpi_token* command);

/**
 * 
 */
scpi_error_t scpi_get_speed_limit(struct scpi_parser_context* context, struct scpi_token* command);

/**
 * 
 */
scpi_error_t scpi_move_relative(struct scpi_parser_context* context, struct scpi_token* command);

/**
 * 
 */
scpi_error_t scpi_move_absolute(struct scpi_parser_context* context, struct scpi_token* command);

/**
 * 
 */
scpi_error_t scpi_set_acceleration(struct scpi_parser_context* context, struct scpi_token* command);

/**
 * 
 */
scpi_error_t scpi_set_deceleration(struct scpi_parser_context* context, struct scpi_token* command);

/**
 * 
 */
scpi_error_t scpi_set_max_speed(struct scpi_parser_context* context, struct scpi_token* command);

/**
 * 
 */
scpi_error_t scpi_set_softlimit_pos(struct scpi_parser_context* context, struct scpi_token* command);

/**
 * 
 */
scpi_error_t scpi_set_softlimit_neg(struct scpi_parser_context* context, struct scpi_token* command);

/**
 * 
 */
scpi_error_t scpi_set_position(struct scpi_parser_context* context, struct scpi_token* command);




scpi_error_t scpi_get_state(struct scpi_parser_context* context, struct scpi_token* command);

scpi_error_t scpi_home_pos(struct scpi_parser_context* context, struct scpi_token* command);

scpi_error_t scpi_home_neg(struct scpi_parser_context* context, struct scpi_token* command);






#endif
