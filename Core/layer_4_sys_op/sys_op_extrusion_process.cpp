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

#include "stm32f4xx.h"
#include "stm32f4xx_it.h"
#include "cmsis_os2.h"

#include "../layer_0/hal_timer.h"
#include "system_clock.h"
#include "gpio.h"

#include "../layer_0/hal.h"
#include "../layer_0/hal_general.h"
#include "../layer_0/hal_wrapper.h"
#include "../layer_0/hal_callback.h"
#include "../layer_0/hal.h"
#include "../layer_0/rtosal_globals.h"
#include "../layer_0/rtosal.h"
#include "../layer_1/device.h"
#include "../layer_1/device_callback.h"
#include "../layer_1/rtd.h"
#include "../layer_1/band_heater.h"

#include "../application/extruder.h"
#include "sys_op_extrusion_process.h"

#define EXTRUSION_PROCESS_STATE_INITIALIZE                          0
#define EXTRUSION_PROCESS_STATE_WAIT_FOR_SYSTEM_INITIALIZATION      1
#define EXTRUSION_PROCESS_STATE_CONFIGURE_USERS                     2
#define EXTRUSION_PROCESS_STATE_RUN                                 3




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





    uint8_t tx[8] = { 0x01, 0x03, 0x05, 0x07, 0x09, 0x0B, 0x0D, 0x0F };

    void task_intitialize()
    {

    }

    void task_state_machine()
    {
        static uint8_t extrusion_process_state = EXTRUSION_PROCESS_STATE_INITIALIZE;
        static uint32_t counter = 0;
        static uint8_t flag = 0;
        switch (extrusion_process_state)
        {
            case EXTRUSION_PROCESS_STATE_INITIALIZE:
            {
                initialization_event_flags_handle = get_initialization_event_flags_handle();

                extrusion_process_state = EXTRUSION_PROCESS_STATE_WAIT_FOR_SYSTEM_INITIALIZATION;
                break;
            }
            case EXTRUSION_PROCESS_STATE_WAIT_FOR_SYSTEM_INITIALIZATION:
            {
                rtosal::event_flag_wait(initialization_event_flags_handle, READY_FOR_USER_INIT_FLAG, rtosal::OS_FLAGS_ANY, rtosal::OS_WAIT_FOREVER);
                extrusion_process_state = EXTRUSION_PROCESS_STATE_CONFIGURE_USERS;
                break;
            }
            case EXTRUSION_PROCESS_STATE_CONFIGURE_USERS:
            {
                to_comms_handler_queue_1_handle = get_extrusion_task_to_comms_handler_queue_1_handle();
                to_comms_handler_queue_2_handle = get_extrusion_task_to_comms_handler_queue_2_handle();
                to_comms_handler_queue_3_handle = get_extrusion_task_to_comms_handler_queue_3_handle();

                from_comms_handler_queue_1_handle = get_comms_handler_to_extrusion_task_queue_1_handle();
                from_comms_handler_queue_2_handle = get_comms_handler_to_extrusion_task_queue_2_handle();
                from_comms_handler_queue_3_handle = get_comms_handler_to_extrusion_task_queue_3_handle();

                comms_handler_output_data_queue_handle = get_comms_handler_output_data_queue_handle();

                timer_6_initialize();
                hal::timer_time_base_start(get_timer_6_handle());

                device::rtd_zone_1.initialize(rtd::READ_RATE_10_HZ, 0, to_comms_handler_queue_1_handle, from_comms_handler_queue_1_handle, comms_handler_output_data_queue_handle, get_timer_6_handle());
                device::rtd_zone_2.initialize(rtd::READ_RATE_10_HZ, 1, to_comms_handler_queue_2_handle, from_comms_handler_queue_2_handle, comms_handler_output_data_queue_handle, get_timer_6_handle());
                device::rtd_zone_3.initialize(rtd::READ_RATE_10_HZ, 2, to_comms_handler_queue_3_handle, from_comms_handler_queue_3_handle, comms_handler_output_data_queue_handle, get_timer_6_handle());

                device::zone_1_band_heater.initialize(TEMPERATURE_ZONE_1, TIMER_10_ID, get_zone_1_band_heater_mutex_handle());
                device::zone_2_band_heater.initialize(TEMPERATURE_ZONE_2, TIMER_13_ID, get_zone_2_band_heater_mutex_handle());
                device::zone_3_band_heater.initialize(TEMPERATURE_ZONE_3, TIMER_14_ID, get_zone_3_band_heater_mutex_handle());

                hal::timer_register_callback(get_timer_1_handle(), hal::TIMER_INPUT_CAPTURE_CALLBACK_ID, device_callback_tim_1_input_capture_pulse_detected_callback);
                hal::timer_input_capture_start_interrupt(get_timer_1_handle(), hal::TIMER_CHANNEL_2);

                extrusion_process_state = EXTRUSION_PROCESS_STATE_RUN;
                break;
            }
            case EXTRUSION_PROCESS_STATE_RUN:
            {
                device::zone_1_band_heater.set_period(8000);
                device::zone_2_band_heater.set_period(8000);
                device::zone_3_band_heater.set_period(8000);

                device::rtd_zone_1.read();
                device::rtd_zone_2.read();
                device::rtd_zone_3.read();

                break;
            }
            default:
                break;
        }
    }
}
