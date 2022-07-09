/***********************************************************************************************************************
 * Main_Controller
 * i2c_motor.c
 *
 * wilson
 * 6/29/22
 * 8:39 PM
 *
 * Description:
 *
 **********************************************************************************************************************/

#include <stdint.h>

#include "stm32f4xx.h"

#include "mcu_clock_timers.h"
#include "i2c.h"
#include "i2c_motor.h"

const char CLOCKWISE = 1;
const char COUNTERCLOCKWISE = -1;

typedef enum
{
    MOTOR_1 = 0,
    MOTOR_2

} MOTOR_ID;

typedef struct
{
    uint8_t i2c_controller_address;
    MOTOR_ID motor_id;
    unsigned short current_speed;
    short current_direction;
} Motor_Object;

// motor_1 and motor_2 init speed = 0, direction = clockwise
Motor_Object motor_1 = { 0x0C, MOTOR_1, 0, 1 };
Motor_Object motor_2 = { 0x0C, MOTOR_2, 0, 1 };

Motor_Object* motor_objects[2] = { &motor_1, &motor_2 };


// bytes for communicating with I2C motor controller
uint8_t DIRECTION_SET   = 0xaa;     // indicate next byte is direction
uint8_t SPEED_SET       = 0x82;     // indicate next byte is speed
uint8_t FREQ_SET        = 0x84;     // indicate next byte is frequency

// motor directions
uint8_t BOTH_CW         = 0x0a;     // both clockwise
uint8_t BOTH_CCW        = 0x05;     // both counter
uint8_t CW_CCW          = 0x06;     // M1 clockwise, M2 counter
uint8_t CCW_CW          = 0x09;     // M1 counter, M2 clockwise

uint8_t EMPTY           = 0x01;     // send empty byte
uint8_t F_3921Hz        = 0x02;     // use this one

// alternative PWM frequencies compatible with I2C motor controller
uint8_t F_31372Hz       = 0x01;
uint8_t F_490Hz         = 0x03;
uint8_t F_122Hz         = 0x04;
uint8_t F_30Hz          = 0x05;

// speed -255 to 255 ( 0 = stop )
unsigned char SPEED_MOTOR_1 = 0;
unsigned char SPEED_MOTOR_2 = 0;

// clockwise = 1 | counterclockwise = -1, internal to class methods
// interface allows negative speed values to indicate direction
// functions interpolate behind the scenes
int DIRECTION_MOTOR_1 = 1;
int DIRECTION_MOTOR_2 = 1;




void i2c_motor_init( void )
{
    i2c_motor_set_pwm_frequency();
}


/**
 * send_I2C()
 *
 * Helper function to send bytes from master to motor controller via I2C
 * Sends address followed by flag and 2 data bytes
 *
 * @param flag Tells controller what byte is coming next (frequency, speed, or
 * direction value), see I2CMotor.h for applicable constants
 *
 * @param data_byte_1 Frequency, speed, or direction value, again see .h file
 *
 * @param data_byte_2 Frequency, speed, or direction value, both data_byte 1 and
 * 2 are passed the EMPTY byte by default
 */
void i2c_motor_send_command( uint8_t flag, uint8_t data_byte_1, uint8_t data_byte_2)
{
    uint8_t addr = 0x0C << 1;
    uint8_t motor_command_data[3] = { flag, data_byte_1, data_byte_2 };
    I2C_HandleTypeDef* i2c_module = i2c_get_module();
//    HAL_I2C_Master_Transmit(i2c_instance, (0x0C << 1), &flag, 1, HAL_MAX_DELAY);
//    us_delay(10);
//    HAL_I2C_Master_Transmit(i2c_instance, (0x0C << 1), &data_byte_1, 1, HAL_MAX_DELAY);
//    us_delay(10);
//    HAL_I2C_Master_Transmit(i2c_instance, (0x0C << 1), &data_byte_2, 1, HAL_MAX_DELAY);
    i2c_controller_write(i2c_module, (0x0C << 1), motor_command_data, 3, HAL_MAX_DELAY);
    //HAL_I2C_Master_Transmit(i2c_instance, (0x0C << 1), motor_command_data, 3, HAL_MAX_DELAY);
}

/**
 * set_PWM_frequency()
 *
 * This function sets operating frequency of the  motor controller
 *
 * Value can be selected from constants in I2CMotor.h
 *
 * Set value on initialization, see I2CMotor() above
 */
void i2c_motor_set_pwm_frequency()
{
    i2c_motor_send_command(FREQ_SET, F_3921Hz, EMPTY);
    us_delay(200);
}


/**
 * get_motor_speed()
 *
 * @param motor_id Motor of which current speed is desired
 *
 * @return Speed (-255, 255) of motor specified by motor_id
 */
float i2c_motor_get_speed( unsigned char motor_id )
{
    return ((float)motor_objects[motor_id]->current_speed);
}

short i2c_motor_get_direction(unsigned char motor_id)
{
    return motor_objects[motor_id]->current_direction;
}


/**
 * set_motor_directions()
 *
 * Helper function for set_speed() below
 *
 * Sets directions of both motors driven by the motor controller
 *
 * @param motor_directions Selected from symbolic constants in I2CMotor.h,
 * this argument is determined by logic in set_speed()
 *
 * motor_directions can be any combination of clockwise and counterclockwise
 * for the two motors
 */
void i2c_motor_set_direction( uint8_t motor_directions )
{
    i2c_motor_send_command( DIRECTION_SET, motor_directions, EMPTY );
    us_delay(200);
}

/**
 * set_speed()
 *
 * @param motor_id  Motor to adjust, 0 for motor_1 and 1 for motor_2
 *
 * @param new_speed Value from -255 to 255 (-255 and 255 being 100% duty cycle
 * in clockwise and counterclockwise respectively)
 *
 * @return new speed setting for the given motor
 */
float i2c_motor_set_speed( unsigned char motor_id, short new_speed )
{
    char new_direction = 1;

    // handle counterclockwise direction setting
    if ( new_speed < 0 )
    {
        new_direction = -1;
        new_speed = new_speed * -1;
    }

    // set new direction for selected motor based on +/- new speed
    motor_objects[motor_id]->current_direction = new_direction;

    new_speed = (new_speed * 256) / 100;
    // if new_speed is greater than max, set new speed to max
    if ( new_speed >= 255 || new_speed <= -255)
    {
        motor_objects[motor_id]->current_speed = 255;
    }
    else
    {
        motor_objects[motor_id]->current_speed = new_speed;
    }

    // I2C motor controller requires both motor directions be set at once, so
    // we need to set the new direction for the current motor while maintaining
    // the other motor's direction
    if ( motor_objects[0]->current_direction == 1 && motor_objects[1]->current_direction == 1 )
    {
        i2c_motor_set_direction( BOTH_CW );
    }
    if ( motor_objects[0]->current_direction == 1 && motor_objects[1]->current_direction == -1 )
    {
        i2c_motor_set_direction( CW_CCW );
    }
    if ( motor_objects[0]->current_direction == -1 && motor_objects[1]->current_direction == 1 )
    {
        i2c_motor_set_direction( CCW_CW );
    }
    if ( motor_objects[0]->current_direction == -1 && motor_objects[1]->current_direction == -1 )
    {
        i2c_motor_set_direction( BOTH_CCW );
    }

    // send command
    i2c_motor_send_command(SPEED_SET, motor_objects[0]->current_speed, motor_objects[1]->current_speed);
    us_delay(200);

    // return the new speed
    return ((float)motor_objects[motor_id]->current_speed / 255) * 100 * motor_objects[motor_id]->current_direction;
}

/**
 * nudge_up()
 *
 * @param motor_id Motor to adjust, 0 for motor_1 and 1 for motor_2
 *
 * @param amount Value by which to increase the selected motor's speed
 *
 * @return Call to set_speed() which carries out the increase and returns the
 * newly set speed
 */
float i2c_motor_nudge_speed_up( unsigned char motor_id, short amount )
{
    short new_speed = 0;
    // current speed negative -> subtract to "nudge up"
    if ( motor_objects[motor_id]->current_speed < 0 )
        amount *= -1;

    new_speed = motor_objects[motor_id]->current_speed + amount;

    return i2c_motor_set_speed( motor_id, new_speed );
}

/**
 * nudge_down()
 *
 * @param motor_id Motor to adjust, 0 for motor_1 and 1 for motor_2
 * @param amount Value by which to decrease the selected motor's speed
 * @return
 */
float i2c_motor_nudge_speed_down( unsigned char motor_id, short amount )
{
    short new_speed = 0;

    // current speed negative -> add to "nudge down"
    if ( motor_objects[motor_id]->current_speed > 0 )
        amount *= -1;

    new_speed = motor_objects[motor_id]->current_speed + amount;

    // if signs are different, nudge will result in motor direction change
    // don't allow this to happen. Instead, stop motor
    if (new_speed * motor_objects[motor_id]->current_speed < 0)
        new_speed = 0;

    return i2c_motor_set_speed( motor_id, new_speed );
}

/**
 * stop()
 *
 * @param motor_id Motor to adjust, 0 for motor_1 and 1 for motor_2
 *
 * @return Call to set_speed() which in turn returns the new speed
 */
float i2c_motor_stop( unsigned char motor_id )
{
    return i2c_motor_set_speed( motor_id, 0 );
}
