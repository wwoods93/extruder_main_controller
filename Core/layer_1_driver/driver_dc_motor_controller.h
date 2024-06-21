/***********************************************************************************************************************
 * Main_Controller
 * driver_i2c_motor.h
 *
 * wilson
 * 10/8/22
 * 3:20 AM
 *
 * Description:
 *
 **********************************************************************************************************************/
#ifndef MAIN_CONTROLLER_DRIVER_DC_MOTOR_CONTROLLER_H
#define MAIN_CONTROLLER_DRIVER_DC_MOTOR_CONTROLLER_H

#include <cstdint>
#include "stm32f4xx.h"
#include "../layer_0_hal//hal_i2c.h"
#include "../meta_structure/meta_structure_user.h"
#include "driver_dc_motor.h"

class dc_motor_controller : public user
{
    public:
        /****************************************** compile time constants ********************************************/
        // bytes for communicating with I2C motor controller
        static constexpr uint8_t DIRECTION_SET   = 0xAA;     // indicate next byte is direction
        static constexpr uint8_t SPEED_SET       = 0x82;     // indicate next byte is speed
        static constexpr uint8_t FREQ_SET        = 0x84;     // indicate next byte is frequency
        // motor directions
        static constexpr uint8_t BOTH_CW         = 0x0A;     // both clockwise
        static constexpr uint8_t BOTH_CCW        = 0x05;     // both counter
        static constexpr uint8_t CW_CCW          = 0x06;     // M1 clockwise, M2 counter
        static constexpr uint8_t CCW_CW          = 0x09;     // M1 counter, M2 clockwise

        static constexpr uint8_t EMPTY           = 0x01;     // send empty byte
        static constexpr uint8_t F_3921Hz        = 0x02;     // use this one
        // alternative PWM frequencies compatible with I2C motor controller
        static constexpr uint8_t F_31372Hz       = 0x01;
        static constexpr uint8_t F_490Hz         = 0x03;
        static constexpr uint8_t F_122Hz         = 0x04;
        static constexpr uint8_t F_30Hz          = 0x05;

        static constexpr int8_t CLOCKWISE = 1;
        static constexpr int8_t COUNTERCLOCKWISE = -1;
        /********************************************* type definitions ***********************************************/
        typedef enum
        {
            M1 = dc_motor::MOTOR_1,
            M2 = dc_motor::MOTOR_2,
        }MOTOR_ID;
        /****************************************** public member functions *******************************************/
        dc_motor_controller();
        void initialize_controller(uint8_t controller_address, uint8_t controller_frequency, bool init_motor_1, bool init_motor_2);
        uint8_t set_speed(uint8_t id, uint8_t new_speed);
        uint8_t set_speed_and_direction(uint8_t id, uint8_t new_speed, int8_t new_direction);
        uint8_t nudge_speed_up(uint8_t id, uint8_t amount);
        uint8_t nudge_speed_down(uint8_t id, uint8_t amount);
        uint8_t stop(uint8_t id);
        uint8_t get_speed(uint8_t id);
        int8_t get_rotation_direction(uint8_t id);

    private:
        /********************************************** private objects ***********************************************/
        uint8_t i2c_address;
        uint8_t pwm_frequency;
        dc_motor dc_motor_1;
        dc_motor dc_motor_2;
        dc_motor* motor[2] = {&dc_motor_1, &dc_motor_2 };
        /****************************************** private member functions ******************************************/
        void send_motor_command( uint8_t flag, uint8_t data_byte_1, uint8_t data_byte_2 );
        void set_rotation_directions();
        void set_pwm_frequency();
};

#endif //MAIN_CONTROLLER_DRIVER_DC_MOTOR_CONTROLLER_H
