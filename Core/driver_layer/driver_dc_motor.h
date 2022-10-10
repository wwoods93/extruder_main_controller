/***********************************************************************************************************************
 * Main_Controller
 * driver_dc_motor.h
 *
 * wilson
 * 10/8/22
 * 4:00 AM
 *
 * Description:
 *
 **********************************************************************************************************************/

#ifndef MAIN_CONTROLLER_DRIVER_DC_MOTOR_H
#define MAIN_CONTROLLER_DRIVER_DC_MOTOR_H

#include <cstdint>

class dc_motor
{
    public:

        typedef enum
        {
            MOTOR_NULL = -1,
            MOTOR_1 = 0,
            MOTOR_2 = 1,
        } MOTOR_ID;

        MOTOR_ID motor_id = MOTOR_NULL;
        uint8_t speed = 0;
        int8_t  direction = 1;

};


#endif //MAIN_CONTROLLER_DRIVER_DC_MOTOR_H
