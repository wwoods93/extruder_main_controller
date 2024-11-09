/***********************************************************************************************************************
 * Main_Controller
 * device_band_heater.h
 *
 * wilson
 * 9/17/24
 * 10:39 PM
 *
 * Description:
 *
 **********************************************************************************************************************/

#ifndef MAIN_CONTROLLER_BAND_HEATER_H
#define MAIN_CONTROLLER_BAND_HEATER_H

/* c/c++ includes */
#include <cstdint>
/* stm32 includes */
#include "stm32f4xx.h"
/* third-party includes */

/* layer_0 includes */
#include "../layer_0/hal.h"
#include "../layer_0/hal_wrapper.h"
/* layer_1_rtosal includes */
#include "../layer_0/rtosal.h"
#include "../layer_0/rtosal_wrapper.h"
/* layer_1 includes */

/* layer_3_control includes */

/* layer_4_sys_op includes */

/* layer_n_meta_structure includes */


namespace device
{
    class band_heater;
}

class device::band_heater
{
    public:


        static constexpr uint16_t OUTPUT_PULSE_WIDTH = 250U;

        static hal::timer_handle_t* zero_crossing_pulse_timer_module;
        hal::timer_handle_t* output_pulse_timer_module;
        uint32_t demand_percentage = 0U;
        uint32_t period = 8300U;

        uint32_t initialize(uint8_t arg_temperature_zone_id, uint8_t arg_output_pulse_timer_id);
        uint32_t initialize_output_pulse_timer(uint8_t arg_output_pulse_timer_id);
        uint32_t set_demand(uint32_t arg_percentage);
        hal::timer_handle_t* get_output_pulse_timer_module() const;
        static void zero_crossing_pulse_restart();
        friend void output_pulse_restart(band_heater* arg_band_heater);


    private:

        static constexpr uint8_t DEMAND_LOOKUP_TABLE_LENGTH = 101U;
        uint8_t output_pulse_timer_id;
        uint32_t demand_lookup_table[DEMAND_LOOKUP_TABLE_LENGTH] =
        {
            8123, 	8061, 	7998, 	7936, 	7873, 	7811, 	7749, 	7686, 	7624, 	7561,
            7499, 	7437, 	7374, 	7312, 	7249, 	7187, 	7125, 	7062, 	7000, 	6937,
            6875, 	6813, 	6750, 	6688, 	6625, 	6563, 	6501, 	6438, 	6376, 	6313,
            6251, 	6189, 	6126, 	6064, 	6001, 	5939, 	5877, 	5814, 	5752, 	5689,
            5627, 	5565, 	5502, 	5440, 	5377, 	5315, 	5253, 	5190, 	5128, 	5065,
            5003, 	4941, 	4878, 	4816, 	4753, 	4691, 	4629, 	4566, 	4504, 	4441,
            4379, 	4317, 	4254, 	4192, 	4129, 	4067, 	4005, 	3942, 	3880, 	3817,
            3755, 	3693, 	3630, 	3568, 	3505, 	3443, 	3381, 	3318, 	3256, 	3193,
            3131, 	3069, 	3006, 	2944, 	2881, 	2819, 	2757, 	2694, 	2632, 	2569,
            2507, 	2445, 	2382, 	2320, 	2257, 	2195, 	2133, 	2070, 	2008, 	1945,
            1883
        };




};


#endif //MAIN_CONTROLLER_BAND_HEATER_H
