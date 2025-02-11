/***********************************************************************************************************************
 * Main_Controller
 * system_operation_extrusion_process.cpp
 *
 * wilson
 * 11/6/22
 * 3:51 PM
 *
 * Description:
 *
 **********************************************************************************************************************/

/* c/c++ includes */
#include <cstdint>
#include <cstring>
/* stm32 includes */
#include "stm32f4xx_it.h"
/* third-party includes */

/* layer_0 includes */
#include "../layer_0/hal.h"
#include "../layer_0/rtosal.h"
/* layer_1 includes */
#include "../layer_1/device.h"
#include "../layer_1/device_callback.h"
/* layer_2 includes */

/* layer_3 includes */

/* application includes */
#include "../application/extruder.h"
/* sys_op_temp_control header  */
#include "sys_op_temp_control.h"


static constexpr uint8_t TEMP_CONTROL_INIT     = 0U;
static constexpr uint8_t TEMP_CONTROL_RUN      = 1U;

spi::module_t spi_2_handle;

int16_t z0_channel_id = ID_INVALID;
int16_t z1_channel_id = ID_INVALID;
int16_t z2_channel_id = ID_INVALID;

uint8_t z0_heater_demand = 50U;
uint8_t z1_heater_demand = 10U;
uint8_t z2_heater_demand = 10U;

namespace sys_op::temp_control
{
    rtosal::message_queue_handle_t comms_handler_output_data_queue_handle = nullptr;
    rtosal::event_flag_handle_t initialization_event_flags_handle = nullptr;

    char debug_msg[] = "period is 8000";

    void task_intitialize()
    {

    }

    void task_state_machine()
    {
        static uint8_t state = TEMP_CONTROL_INIT;

        switch (state)
        {
            case TEMP_CONTROL_INIT:
            {
                initialization_event_flags_handle = rtosal::get_initialization_event_flags_handle();

                rtosal::event_flag_wait(initialization_event_flags_handle, rtosal::READY_FOR_TEMP_CONTROL_INIT_FLAG, rtosal::OS_FLAGS_ANY, rtosal::OS_WAIT_FOREVER);

                comms_handler_output_data_queue_handle = rtosal::get_comms_handler_output_data_queue_handle();

                hal::timer_register_callback(get_timer_1_handle(), hal::TIMER_INPUT_CAPTURE_CALLBACK_ID, device_callback_tim_1_input_capture_pulse_detected_callback);
                hal::timer_2_initialize();
                hal::timer_6_initialize();
                hal::timer_time_base_start(get_timer_2_handle());
                hal::timer_time_base_start(get_timer_6_handle());
                hal::spi_2.initialize(&spi_2_handle, SPI_2_ID, get_timer_2_handle());
                hal::spi_2.create_channel(z0_channel_id, PORT_B, GPIO_PIN_14);
                hal::spi_2.create_channel(z1_channel_id, PORT_B, GPIO_PIN_15);
                hal::spi_2.create_channel(z2_channel_id, PORT_B, GPIO_PIN_1);

                device::z0_rtd.initialize(hal::spi_2, PORT_B, PIN_14, 0U, comms_handler_output_data_queue_handle, get_timer_6_handle(), 21.34F, 20.45F);
                device::z1_rtd.initialize(hal::spi_2, PORT_B, PIN_15, 1U, comms_handler_output_data_queue_handle, get_timer_6_handle(), 23.15F, 20.45F);
                device::z2_rtd.initialize(hal::spi_2, PORT_B, PIN_1,  2U, comms_handler_output_data_queue_handle, get_timer_6_handle(), 22.67F, 20.45F);
                device::z0_heater.initialize(TEMPERATURE_ZONE_1, TIMER_10_ID);
                device::z1_heater.initialize(TEMPERATURE_ZONE_2, TIMER_13_ID);
                device::z2_heater.initialize(TEMPERATURE_ZONE_3, TIMER_14_ID);

                hal::timer_input_capture_start_interrupt(get_timer_1_handle(), hal::TIMER_CHANNEL_2);

                state = TEMP_CONTROL_RUN;

                break;
            }
            case TEMP_CONTROL_RUN:
            {
                device::z0_rtd.read();
                device::z1_rtd.read();
                device::z2_rtd.read();
                device::z0_heater.set_demand(z0_heater_demand);
                device::z1_heater.set_demand(z1_heater_demand);
                device::z2_heater.set_demand(z2_heater_demand);

                rtosal::thread_yield();

                break;
            }
            default:
                break;
        }
    }
}
