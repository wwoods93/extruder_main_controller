/***********************************************************************************************************************
 * Main_Controller
 * device_band_heater.cpp
 *
 * wilson
 * 9/17/24
 * 10:39 PM
 *
 * Description:
 *
 **********************************************************************************************************************/

/* c/c++ includes */

/* stm32 includes */
#include "stm32f4xx.h"
/* third-party includes */
#include "../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/cmsis_os2.h"
/* layer_0 includes */
#include "../layer_0/hal.h"
/* layer_1_rtosal includes */

/* layer_1 includes */

/* layer_3_control includes */

/* layer_3 includes */

/* layer_n_meta_structure includes */


#include "../application/extruder.h"

/* device_band_heater header */
#include "band_heater.h"

namespace device
{
    uint32_t band_heater::initialize(uint8_t arg_temperature_zone_id, uint8_t arg_output_pulse_timer_id)
    {
        output_pulse_timer_id = arg_output_pulse_timer_id;
        initialize_output_pulse_timer(output_pulse_timer_id);

        return 0;
    }

    uint32_t band_heater::initialize_output_pulse_timer(uint8_t arg_output_pulse_timer_id)
    {
        switch (arg_output_pulse_timer_id)
        {
            case TIMER_10_ID:
            {
                hal::tim_10_initialize();
                output_pulse_timer_module = hal::tim_10_get_handle();
                break;
            }
            case TIMER_13_ID:
            {
                hal::tim_13_initialize();
                output_pulse_timer_module = hal::tim_13_get_handle();
                break;
            }
            case TIMER_14_ID:
            {
                hal::tim_14_initialize();
                output_pulse_timer_module = hal::tim_14_get_handle();
                break;
            }
            default:
            {
                break;
            }
        }

        return 0;
    }

    uint32_t band_heater::set_demand(uint32_t arg_percentage)
    {
        demand_percentage = arg_percentage;
        period = demand_lookup_table[arg_percentage];

        return 0;
    }

    TIM_HandleTypeDef *band_heater::get_output_pulse_timer_module() const
    {
        return output_pulse_timer_module;
    }

    void band_heater::zero_crossing_pulse_restart()
    {
        if (zero_crossing_pulse_timer_module != nullptr)
        {
            HAL_TIM_IC_Start_IT(zero_crossing_pulse_timer_module, TIM_CHANNEL_2);
        }
    }

    void output_pulse_restart(band_heater *arg_band_heater)
    {
        TIM_OC_InitTypeDef output_compare_init = { 0 };

        arg_band_heater->output_pulse_timer_module->Init.Period = arg_band_heater->period;

        if (HAL_TIM_Base_Init(arg_band_heater->output_pulse_timer_module) != HAL_OK)
        {
            error_handler();
        }

        output_compare_init.OCMode = TIM_OCMODE_PWM2;
        output_compare_init.Pulse = arg_band_heater->period - band_heater::OUTPUT_PULSE_WIDTH;
        output_compare_init.OCPolarity = TIM_OCPOLARITY_HIGH;
        output_compare_init.OCIdleState = TIM_OCIDLESTATE_RESET;
        output_compare_init.OCFastMode = TIM_OCFAST_DISABLE;

        if (HAL_TIM_OC_ConfigChannel(arg_band_heater->output_pulse_timer_module, &output_compare_init, TIM_CHANNEL_1) != HAL_OK)
        {
            error_handler();
        }

        HAL_TIM_OC_Start_IT(arg_band_heater->output_pulse_timer_module, TIM_CHANNEL_1);
    }

}
