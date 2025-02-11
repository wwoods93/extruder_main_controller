/***********************************************************************************************************************
 * Main_Controller
 * sys_op_comms_handler.cpp
 *
 * wilson
 * 11/6/22
 * 3:46 PM
 *
 * Description:
 *
 **********************************************************************************************************************/


/* c/c++ includes */
#include <cstdint>
#include <cstring>
#include <string>
/* stm32 includes */
#include "stm32f4xx_it.h"
/* 3rd-party includes */

/* hal includes */
#include "../layer_0/hal_callback.h"
#include "../layer_0/hal.h"
/* driver includes */
#include "../layer_1/device.h"
/* system includes */
/* rtos includes */
#include "../layer_0/rtosal.h"
/* system_operation_comms_handler header */
#include "sys_op_user_comms.h"


static constexpr uint8_t USER_COMMS_INIT = 0U;
static constexpr uint8_t USER_COMMS_RUN = 1U;

hal::timer_handle_t* device::band_heater::zero_crossing_pulse_timer_module = get_timer_1_handle();

namespace sys_op::user_comms
{
    rtosal::event_flag_handle_t  initialization_event_flags_handle = nullptr;
    rtosal::message_queue_handle_t comms_handler_output_data_queue_handle = nullptr;
    rtosal::message_queue_handle_t serial_monitor_usart_queue_handle = nullptr;

    void task_intitialize()
    {
        rtosal::event_flag_set(initialization_event_flags_handle, rtosal::READY_FOR_USER_COMMS_INIT_FLAG);
    }

    void task_state_machine()
    {
        static uint8_t comms_handler_state = USER_COMMS_INIT;

        switch (comms_handler_state)
        {
            case USER_COMMS_INIT:
            {
                initialization_event_flags_handle       = rtosal::get_initialization_event_flags_handle();
                comms_handler_output_data_queue_handle  = rtosal::get_comms_handler_output_data_queue_handle();
                serial_monitor_usart_queue_handle       = rtosal::get_serial_monitor_usart_queue_handle();

                rtosal::event_flag_wait(initialization_event_flags_handle, rtosal::READY_FOR_USER_COMMS_INIT_FLAG, osFlagsWaitAny, osWaitForever);

                hal::i2c_register_callback(get_i2c_2_handle(), hal::I2C_CONTROLLER_TX_COMPLETE_CALLBACK_ID, hal_callback_i2c_controller_tx_complete);
                hal::i2c_register_callback(get_i2c_2_handle(), hal::I2C_ERROR_CALLBACK_ID, hal_callback_i2c_controller_error);

                device::debug_serial_monitor.initialize(get_usart_2_handle(), serial_monitor_usart_queue_handle);
                device::built_in_display.initialize(get_i2c_2_handle(), comms_handler_output_data_queue_handle);

                rtosal::event_flag_set(initialization_event_flags_handle, rtosal::READY_FOR_TEMP_CONTROL_INIT_FLAG);

                comms_handler_state = USER_COMMS_RUN;

                break;
            }
            case USER_COMMS_RUN:
            {
                device::debug_serial_monitor.process_send_buffer();
                device::built_in_display.get_intertask_output_data();
                device::built_in_display.update_output();
                rtosal::thread_yield();

                break;
            }
            default:
            {
                break;
            }
        }
    }
}
