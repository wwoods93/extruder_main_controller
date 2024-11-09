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

#include <cstdint>
#include "stm32f4xx_it.h"
#include "../layer_0/hal.h"
#include "../layer_0/hal_general.h"
#include "../layer_0/rtosal_globals.h"
#include "../layer_0/rtosal.h"
#include "../layer_1/device.h"
#include "../layer_1/device_callback.h"

#include "../application/extruder.h"
#include "sys_op_extrusion_process.h"


static constexpr uint8_t EXTRUSION_PROCESS_STATE_INITIALIZE     = 0U;
static constexpr uint8_t EXTRUSION_PROCESS_STATE_RUN            = 1U;

namespace sys_op::extrusion
{
    rtosal::message_queue_handle_t to_comms_handler_queue_1_handle = nullptr;
    rtosal::message_queue_handle_t to_comms_handler_queue_2_handle = nullptr;
    rtosal::message_queue_handle_t to_comms_handler_queue_3_handle = nullptr;
    rtosal::message_queue_handle_t from_comms_handler_queue_1_handle = nullptr;
    rtosal::message_queue_handle_t from_comms_handler_queue_2_handle = nullptr;
    rtosal::message_queue_handle_t from_comms_handler_queue_3_handle = nullptr;
    rtosal::message_queue_handle_t comms_handler_output_data_queue_handle = nullptr;

    rtosal::event_flag_handle_t initialization_event_flags_handle = nullptr;

    char debug_msg[] = "period is 8000";

    void task_intitialize()
    {

    }

    void task_state_machine()
    {
        static uint8_t extrusion_process_state = EXTRUSION_PROCESS_STATE_INITIALIZE;

        switch (extrusion_process_state)
        {
            case EXTRUSION_PROCESS_STATE_INITIALIZE:
            {
                initialization_event_flags_handle = get_initialization_event_flags_handle();
                rtosal::event_flag_wait(initialization_event_flags_handle, READY_FOR_USER_INIT_FLAG, rtosal::OS_FLAGS_ANY, rtosal::OS_WAIT_FOREVER);

                to_comms_handler_queue_1_handle = get_extrusion_task_to_comms_handler_queue_1_handle();
                to_comms_handler_queue_2_handle = get_extrusion_task_to_comms_handler_queue_2_handle();
                to_comms_handler_queue_3_handle = get_extrusion_task_to_comms_handler_queue_3_handle();

                from_comms_handler_queue_1_handle = get_comms_handler_to_extrusion_task_queue_1_handle();
                from_comms_handler_queue_2_handle = get_comms_handler_to_extrusion_task_queue_2_handle();
                from_comms_handler_queue_3_handle = get_comms_handler_to_extrusion_task_queue_3_handle();

                comms_handler_output_data_queue_handle = get_comms_handler_output_data_queue_handle();

                timer_6_initialize();
                hal::timer_time_base_start(get_timer_6_handle());

                device::rtd_zone_1.initialize(0U, to_comms_handler_queue_1_handle, from_comms_handler_queue_1_handle, comms_handler_output_data_queue_handle, get_timer_6_handle());
                device::rtd_zone_2.initialize(1U, to_comms_handler_queue_2_handle, from_comms_handler_queue_2_handle, comms_handler_output_data_queue_handle, get_timer_6_handle());
                device::rtd_zone_3.initialize(2U, to_comms_handler_queue_3_handle, from_comms_handler_queue_3_handle, comms_handler_output_data_queue_handle, get_timer_6_handle());

                device::zone_1_band_heater.initialize(TEMPERATURE_ZONE_1, TIMER_10_ID);
                device::zone_2_band_heater.initialize(TEMPERATURE_ZONE_2, TIMER_13_ID);
                device::zone_3_band_heater.initialize(TEMPERATURE_ZONE_3, TIMER_14_ID);

                hal::timer_register_callback(get_timer_1_handle(), hal::TIMER_INPUT_CAPTURE_CALLBACK_ID, device_callback_tim_1_input_capture_pulse_detected_callback);
                hal::timer_input_capture_start_interrupt(get_timer_1_handle(), hal::TIMER_CHANNEL_2);

                extrusion_process_state = EXTRUSION_PROCESS_STATE_RUN;

                break;
            }
            case EXTRUSION_PROCESS_STATE_RUN:
            {
                device::zone_1_band_heater.set_demand(0U);
                device::zone_2_band_heater.set_demand(0U);
                device::zone_3_band_heater.set_demand(0U);
                device::rtd_zone_1.read();
                device::rtd_zone_2.read();
                device::rtd_zone_3.read();
                rtosal::thread_yield();

                break;
            }
            default:
                break;
        }
    }
}
