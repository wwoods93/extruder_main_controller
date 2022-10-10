/***********************************************************************************************************************
 * Main_Controller
 * driver_i2c_motor.cpp
 *
 * wilson
 * 10/8/22
 * 3:20 AM
 *
 * Description:
 *
 **********************************************************************************************************************/

#include <cstdint>
#include "stm32f4xx.h"
#include "mcu_clock_timers.h"
#include "../hardware_abstraction_layer/hal_i2c.h"
#include "driver_dc_motor_controller.h"

/***************************************************** hal objects ****************************************************/
I2C_HandleTypeDef hi2c2;

namespace hal
{
    i2c i2c_2(&hi2c2);
}
/******************************************************* public *******************************************************/
dc_motor_controller::dc_motor_controller() = default;

void dc_motor_controller::initialize_controller(uint8_t controller_address, uint8_t controller_frequency, bool init_motor_1, bool init_motor_2)
{
    i2c_address = controller_address;
    pwm_frequency = controller_frequency;
    if (init_motor_1) { motor[M1]->motor_id = dc_motor::MOTOR_1; }
    if (init_motor_2) { motor[M2]->motor_id = dc_motor::MOTOR_2; }
    set_pwm_frequency();
}

/**
 * set_speed()
 * @param id  Motor to adjust, 0 for motor_1 and 1 for motor_2
 * @param new_speed Value from 0 to 255
 * @return new speed setting for the given motor
 */
uint8_t dc_motor_controller::set_speed(uint8_t id, uint8_t new_speed)
{
    if ( new_speed >= 255)  { motor[id]->speed = 255; }
    else if (new_speed < 0) { motor[id]->speed = 0; }
    else                    { motor[id]->speed = new_speed; }
    set_rotation_directions();
    send_motor_command(SPEED_SET, motor[M1]->speed, motor[M2]->speed);
    us_delay(200);
    return motor[id]->speed;
}

/**
 * set_speed_and_direction()
 * @param id  Motor to adjust, 0 for motor_1 and 1 for motor_2
 * @param new_speed Value from 0 to 255
 * @param new_direction clockwise = 1, counterclockwise = -1
 * @return new speed setting for the given motor
 */
uint8_t dc_motor_controller::set_speed_and_direction(uint8_t id, uint8_t new_speed, int8_t new_direction)
{
    motor[id]->direction = new_direction;
    set_rotation_directions();
    set_speed(id, new_speed);
    us_delay(200);
    return motor[id]->speed;
}

/**
 * nudge_speed_up()
 * @param id Motor to adjust, 0 for motor_1 and 1 for motor_2
 * @param amount Value by which to increase the selected motor's speed
 * @return Call to set_speed() which carries out the increase and returns the newly set speed
 */
uint8_t dc_motor_controller::nudge_speed_up(uint8_t id, uint8_t amount)
{
    uint16_t new_speed = motor[id]->speed + amount;
    if (new_speed > 255)    { new_speed = 255; }
    else if (new_speed < 0) { new_speed = 0; }
    return set_speed(id, new_speed);
}

/**
 * nudge_speed_down()
 * @param id Motor to adjust, 0 for motor_1 and 1 for motor_2
 * @param amount Value by which to decrease the selected motor's speed
 * @return
 */
uint8_t dc_motor_controller::nudge_speed_down(uint8_t id, uint8_t amount)
{
    uint16_t new_speed = motor[id]->speed - amount;
    if (new_speed > 255)    { new_speed = 255; }
    else if (new_speed < 0) { new_speed = 0; }
    return set_speed(id, new_speed);
}

/**
 * stop()
 * @param id Motor to adjust, 0 for motor_1 and 1 for motor_2
 * @return Call to set_speed() which in turn returns the new speed
 */
uint8_t dc_motor_controller::stop(uint8_t id)
{
    return set_speed(id, 0);
}

/**
 * get_speed()
 * @param id Motor of which current speed is desired
 * @return Speed (-255, 255) of motor specified by id
 */
uint8_t dc_motor_controller::get_speed(uint8_t id)
{
    return motor[id]->speed;
}

/**
 * get_rotation_directions()
 * @param id Motor of which the rotation direction is desired
 * @return Id (0 for motor_1, 1 for motor_2)
 */
int8_t dc_motor_controller::get_rotation_direction(uint8_t id)
{
    return motor[id]->direction;
}
/****************************************************** private *******************************************************/
/**
 * send_motor_command()
 * Helper function to send bytes from master to motor controller via I2C
 * Sends address followed by flag and 2 data bytes
 * @param flag Tells controller what byte is coming next (frequency, speed, or
 * direction value), see I2CMotor.h for applicable constants
 * @param data_byte_1 Frequency, speed, or direction value, again see .h file
 * @param data_byte_2 Frequency, speed, or direction value, both data_byte 1 and
 * 2 are passed the EMPTY byte by default
 */
void dc_motor_controller::send_motor_command(uint8_t flag, uint8_t data_byte_1, uint8_t data_byte_2)
{
    uint8_t motor_command_data[3] = { flag, data_byte_1, data_byte_2 };
    hal::i2c_2.controller_send(i2c_address, motor_command_data, 3, HAL_MAX_DELAY);
}

/**
 * set_rotation_directions()
 * Helper function for set_speed() below Sets directions of both motors driven by the motor controller
 * Because of how the i2c motor controller operates, both direction must be set at once
 * @param motor_directions Selected from symbolic constants in I2CMotor.h,
 * this argument is determined by logic in set_speed()
 * motor_directions can be any combination of clockwise and counterclockwise for the two motors
 */
void dc_motor_controller::set_rotation_directions()
{
    uint8_t directions = 0;
    if (motor[M1]->direction == CLOCKWISE && motor[M2]->direction == CLOCKWISE)               { directions = BOTH_CW;  }
    if (motor[M1]->direction == CLOCKWISE && motor[M2]->direction == COUNTERCLOCKWISE)        { directions = CW_CCW;   }
    if (motor[M1]->direction == COUNTERCLOCKWISE && motor[M2]->direction == CLOCKWISE)        { directions = CCW_CW;   }
    if (motor[M1]->direction == COUNTERCLOCKWISE && motor[M2]->direction == COUNTERCLOCKWISE) { directions = BOTH_CCW; }
    send_motor_command(DIRECTION_SET, directions, EMPTY);
    us_delay(200);
}

/**
 * set_pwm_frequency()
 * This function sets operating frequency of the  motor controller
 * Value can be selected from constants in I2CMotor.h
 * Set value on initialization, see I2CMotor() above
 */
void dc_motor_controller::set_pwm_frequency()
{
    send_motor_command(FREQ_SET, pwm_frequency, EMPTY);
    us_delay(200);
}
/**********************************************************************************************************************/
