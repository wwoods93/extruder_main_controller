/***********************************************************************************************************************
 * Main_Controller
 * utility.h
 *
 * wilson
 * 10/20/24
 * 10:52 PM
 *
 * Description:
 *
 **********************************************************************************************************************/

#ifndef MAIN_CONTROLLER_UTILITY_H
#define MAIN_CONTROLLER_UTILITY_H

/* c/c++ includes */
#include <cstdint>
/* stm32 includes */

/* third-party includes */

/* layer_0_hal includes */

/* layer_1_rtosal includes */

/* layer_2_device includes */

/* layer_3_control includes */

/* layer_4_sys_op includes */

/* layer_n_meta_structure includes */


namespace utility
{
    typedef union
    {
        uint8_t byte[4];
        float value;
    } converter_float_to_bytes_t;

    typedef union
    {
        uint8_t byte[4];
        uint32_t value;
    } converter_uint32_to_bytes_t;

    void convert_float_to_uint8_array(float arg_value, uint8_t* arg_array);
    void convert_uint32_to_uint8_array(uint32_t arg_value, uint8_t* arg_array);
}






#endif //MAIN_CONTROLLER_UTILITY_H
