/***********************************************************************************************************************
 * Main_Controller
 * utility.cpp
 *
 * wilson
 * 10/20/24
 * 10:52 PM
 *
 * Description:
 *
 **********************************************************************************************************************/

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

/* utility header */
#include "utility.h"


namespace utility
{

    converter_float_to_bytes_t float_converter;
    converter_uint32_to_bytes_t uint32_converter;

    void convert_float_to_uint8_array(float arg_value, uint8_t* arg_array)
    {
        float_converter.value = arg_value;
        arg_array[0] = float_converter.byte[0];
        arg_array[1] = float_converter.byte[1];
        arg_array[2] = float_converter.byte[2];
        arg_array[3] = float_converter.byte[3];
    }

    void convert_uint32_to_uint8_array(uint32_t arg_value, uint8_t* arg_array)
    {
        uint32_converter.value = arg_value;
        arg_array[0] = uint32_converter.byte[0];
        arg_array[1] = uint32_converter.byte[1];
        arg_array[2] = uint32_converter.byte[2];
        arg_array[3] = uint32_converter.byte[3];
    }

}
