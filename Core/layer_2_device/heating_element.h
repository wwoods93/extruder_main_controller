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

#ifndef MAIN_CONTROLLER_HEATING_ELEMENT_H
#define MAIN_CONTROLLER_HEATING_ELEMENT_H

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




class heating_element
{
    public:


        static constexpr uint16_t OUTPUT_PULSE_WIDTH = 250U;



        uint32_t initialize(uint8_t arg_zero_crossing_pulse_timer_id, uint8_t arg_zone_1_output_pulse_timer_id, uint8_t arg_zone_2_output_pulse_timer_id, uint8_t arg_zone_3_output_pulse_timer_id);
        uint32_t initialize_output_pulse_timer(uint8_t arg_temperature_zone_id, uint8_t arg_output_pulse_timer_id);
        friend void output_pulse_restart(heating_element* arg_heating_element, uint8_t arg_temperature_zone_id, uint16_t arg_period);
        TIM_HandleTypeDef* get_zero_crossing_pulse_timer_module();
        TIM_HandleTypeDef* get_zone_1_output_pulse_timer_module();
        TIM_HandleTypeDef* get_zone_2_output_pulse_timer_module();
        TIM_HandleTypeDef* get_zone_3_output_pulse_timer_module();

    private:
        TIM_HandleTypeDef* zero_crossing_pulse_timer_module;
        TIM_HandleTypeDef* zone_1_output_pulse_timer_module;
        TIM_HandleTypeDef* zone_2_output_pulse_timer_module;
        TIM_HandleTypeDef* zone_3_output_pulse_timer_module;

};


#endif //MAIN_CONTROLLER_HEATING_ELEMENT_H
