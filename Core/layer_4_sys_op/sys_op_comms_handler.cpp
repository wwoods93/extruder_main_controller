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
#include "cmsis_os2.h"
/* hal includes */
#include "../layer_0/hal_callback.h"
#include "../layer_0/hal.h"
/* driver includes */
#include "../layer_1/device.h"
/* system includes */
/* rtos includes */
#include "../layer_0/rtosal_globals.h"
#include "../layer_0/rtosal.h"
/* system_operation_comms_handler header */
#include "sys_op_comms_handler.h"


#define COMMS_HANDLER_STATE_INITIALIZE      0
#define COMMS_HANDLER_STATE_RUN             1

spi::module_t spi_2_handle;

hal::timer_handle_t* device::band_heater::zero_crossing_pulse_timer_module = get_timer_1_handle();

namespace sys_op::comms_handler
{
    rtosal::event_flag_handle_t  initialization_event_flags_handle = nullptr;

    rtosal::message_queue_handle_t to_extrusion_task_queue_1_handle = nullptr;
    rtosal::message_queue_handle_t to_extrusion_task_queue_2_handle = nullptr;
    rtosal::message_queue_handle_t to_extrusion_task_queue_3_handle = nullptr;

    rtosal::message_queue_handle_t from_extrusion_task_queue_1_handle = nullptr;
    rtosal::message_queue_handle_t from_extrusion_task_queue_2_handle = nullptr;
    rtosal::message_queue_handle_t from_extrusion_task_queue_3_handle = nullptr;

    rtosal::message_queue_handle_t comms_handler_output_data_queue_handle = nullptr;
    rtosal::message_queue_handle_t serial_monitor_usart_queue_handle = nullptr;

    int16_t rtd_0_channel_id = ID_INVALID;
    int16_t rtd_1_channel_id = ID_INVALID;
    int16_t rtd_2_channel_id = ID_INVALID;

    volatile uint32_t comms_handler_timer_tick;
    volatile uint32_t comms_handler_execution_time_us;

    uint8_t rx_d[TX_SIZE_MAX] = {0, 0, 0, 0, 0, 0, 0, 0 };

    void task_intitialize()
    {

    }

    void task_state_machine()
    {
        static uint8_t comms_handler_state = COMMS_HANDLER_STATE_INITIALIZE;

        char time_stamp[9];
        spi::packet_t spi_rx_packet;

        switch (comms_handler_state)
        {
            case COMMS_HANDLER_STATE_INITIALIZE:
            {
                // TODO: fix initialization procedure
                initialization_event_flags_handle       = get_initialization_event_flags_handle();

                to_extrusion_task_queue_1_handle = get_comms_handler_to_extrusion_task_queue_1_handle();
                to_extrusion_task_queue_2_handle = get_comms_handler_to_extrusion_task_queue_2_handle();
                to_extrusion_task_queue_3_handle = get_comms_handler_to_extrusion_task_queue_3_handle();
                from_extrusion_task_queue_1_handle = get_extrusion_task_to_comms_handler_queue_1_handle();
                from_extrusion_task_queue_2_handle = get_extrusion_task_to_comms_handler_queue_2_handle();
                from_extrusion_task_queue_3_handle = get_extrusion_task_to_comms_handler_queue_3_handle();
                comms_handler_output_data_queue_handle  = get_comms_handler_output_data_queue_handle();
                serial_monitor_usart_queue_handle       = get_serial_monitor_usart_queue_handle();

                rtosal::event_flag_wait(initialization_event_flags_handle, READY_FOR_RESOURCE_INIT_FLAG, osFlagsWaitAny, osWaitForever);

                device::debug_serial_monitor.initialize(get_usart_2_handle(), serial_monitor_usart_queue_handle);
                device::built_in_display.initialize(get_i2c_2_handle(), comms_handler_output_data_queue_handle);


                hal::spi_2.initialize(&spi_2_handle, SPI_2_ID, get_timer_2_handle());
                hal::spi_2.register_callback(spi::TX_RX_COMPLETE_CALLBACK_ID, hal_callback_spi_2_tx_rx_complete);
                hal::spi_2.register_callback(spi::ERROR_CALLBACK_ID, hal_callback_spi_2_error);
                hal::spi_2.create_channel(rtd_0_channel_id, PORT_B, GPIO_PIN_14, from_extrusion_task_queue_1_handle, to_extrusion_task_queue_1_handle);
                hal::spi_2.create_channel(rtd_1_channel_id, PORT_B, GPIO_PIN_15, from_extrusion_task_queue_2_handle, to_extrusion_task_queue_2_handle);
                hal::spi_2.create_channel(rtd_2_channel_id, PORT_B, GPIO_PIN_1, from_extrusion_task_queue_3_handle, to_extrusion_task_queue_3_handle);

                hal::i2c_register_callback(get_i2c_2_handle(), hal::I2C_CONTROLLER_TX_COMPLETE_CALLBACK_ID, hal_callback_i2c_controller_tx_complete);
                hal::i2c_register_callback(get_i2c_2_handle(), hal::I2C_ERROR_CALLBACK_ID, hal_callback_i2c_controller_error);

                hal::timer_2_initialize();
                hal::timer_time_base_start(get_timer_2_handle());

                rtosal::event_flag_set(initialization_event_flags_handle, READY_FOR_USER_INIT_FLAG);

                comms_handler_state = COMMS_HANDLER_STATE_RUN;

                break;
            }
            case COMMS_HANDLER_STATE_RUN:
            {
                comms_handler_timer_tick = get_timer_2_count();
                device::debug_serial_monitor.process_send_buffer();
                device::built_in_display.get_intertask_output_data();
                device::built_in_display.update_output();

                hal::spi_2.receive_inter_task_transaction_requests();
                hal::spi_2.process_send_buffer();
                hal::spi_2.process_return_buffers(spi_rx_packet, 0, rx_d);

                hal::rtc_get_time_stamp(time_stamp);
                comms_handler_execution_time_us = get_timer_2_count() - comms_handler_timer_tick;
                osThreadYield();
                break;
            }
            default:
            {
                break;
            }
        }
    }
}
