/***********************************************************************************************************************
 * Main_Controller
 * motor.h
 *
 * wilson
 * 10/7/22
 * 10:28 PM
 *
 * Description:
 *
 **********************************************************************************************************************/

//
// Created by wilson on 10/7/22.
//

#ifndef MAIN_CONTROLLER_MOTOR_H
#define MAIN_CONTROLLER_MOTOR_H

#include "stm32f4xx.h"
#include <cstdint>


void i2c_motor_init();
void i2c_motor_send_command( uint8_t flag, uint8_t data_byte_1, uint8_t data_byte_2 );
void i2c_motor_set_pwm_frequency();
float i2c_motor_get_speed( unsigned char motor_id );
short i2c_motor_get_direction(unsigned char motor_id);
void i2c_motor_set_direction( uint8_t motor_directions );
float i2c_motor_set_speed( unsigned char motor_id, short new_speed );
float i2c_motor_nudge_speed_up( unsigned char motor_id, short amount );
float i2c_motor_nudge_speed_down( unsigned char motor_id, short amount );
float i2c_motor_stop( unsigned char motor_id );

#endif //MAIN_CONTROLLER_MOTOR_H
