/***********************************************************************************************************************
 * Main_Controller
 * touch_screen.h
 *
 * wilson
 * 10/13/24
 * 9:09 PM
 *
 * Description:
 *
 **********************************************************************************************************************/

#ifndef MAIN_CONTROLLER_TOUCH_SCREEN_H
#define MAIN_CONTROLLER_TOUCH_SCREEN_H

/* c/c++ includes */
#include <cstdint>
/* stm32 includes */
#include "stm32f4xx.h"
/* third-party includes */

/* layer_0_hal includes */

/* layer_1_rtosal includes */

/* layer_2_device includes */

/* layer_3_control includes */

/* layer_4_sys_op includes */

/* layer_n_meta_structure includes */




class touch_screen
{
    public:
        uint32_t initialize(I2C_HandleTypeDef* arg_i2c_module);
    private:
        I2C_HandleTypeDef* i2c_module;
};


#endif //MAIN_CONTROLLER_TOUCH_SCREEN_H
