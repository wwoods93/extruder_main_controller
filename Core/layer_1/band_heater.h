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

        static TIM_HandleTypeDef* zero_crossing_pulse_timer_module;


        uint32_t initialize(uint8_t arg_temperature_zone_id, uint8_t arg_output_pulse_timer_id);
        uint32_t initialize_output_pulse_timer(uint8_t arg_output_pulse_timer_id);
        TIM_HandleTypeDef* get_output_pulse_timer_module();
        static void zero_crossing_pulse_restart();
        friend void output_pulse_restart(band_heater* arg_band_heater, uint16_t arg_period);


    private:

        uint8_t output_pulse_timer_id;
        TIM_HandleTypeDef* output_pulse_timer_module;

};


#endif //MAIN_CONTROLLER_BAND_HEATER_H
