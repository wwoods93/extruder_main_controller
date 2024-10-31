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
        uint16_t new_period;
        uint16_t period = 8300U;
        osMutexId_t mutex_handle;

        uint32_t initialize(uint8_t arg_temperature_zone_id, uint8_t arg_output_pulse_timer_id, osMutexId_t arg_mutex_handle);
        uint32_t initialize_output_pulse_timer(uint8_t arg_output_pulse_timer_id);
        uint32_t set_period(uint16_t arg_period);
        uint32_t update_period();
        hal::timer_handle_t* get_output_pulse_timer_module();
        static void zero_crossing_pulse_restart();
        friend void output_pulse_restart(band_heater* arg_band_heater);


    private:

        uint8_t output_pulse_timer_id;




};


#endif //MAIN_CONTROLLER_BAND_HEATER_H
