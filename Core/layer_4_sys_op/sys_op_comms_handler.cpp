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
#include <queue>
/* stm32 includes */
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"
/* 3rd-party includes */
#include "cmsis_os2.h"
/* hal includes */
#include "../layer_0/hal_general.h"
#include "../layer_0/hal_callback.h"
#include "../layer_0/hal_wrapper.h"
#include "../layer_0/hal_spi.h"
#include "../layer_0/hal_i2c.h"
#include "../layer_0/hal.h"
#include "../layer_0/hal_irq.h"
#include "../layer_0/hal_timer.h"
#include "system_clock.h"
#include "gpio.h"
#include "spi.h"
/* driver includes */
#include "../layer_1/device.h"
#include "../layer_1/rtd.h"
#include "../layer_1/band_heater.h"
#include "../layer_1/serial_monitor.h"
#include "../layer_1/touch_screen.h"
/* system includes */
/* rtos includes */
#include "../layer_0/rtosal_globals.h"
#include "../layer_0/rtosal_wrapper.h"
#include "../layer_0/rtosal.h"
/* system_operation_comms_handler header */
#include "sys_op_comms_handler.h"


#include "../utility/utility.h"
#include "../application/extruder.h"
#include "../meta_structure/meta_structure_system_manager.h"

#define COMMS_HANDLER_STATE_INITIALIZE      0
#define COMMS_HANDLER_STATE_RUN             1


spi::module_t spi_2_handle;


hal::timer_handle_t* device::band_heater::zero_crossing_pulse_timer_module = get_timer_1_handle();

namespace sys_op::comms_handler
{
    rtosal::event_flag_handle_t  initialization_event_flags_handle = nullptr;
    rtosal::message_queue_handle_t spi_tx_queue_handle = nullptr;
    rtosal::message_queue_handle_t spi_rx_queue_handle = nullptr;
    rtosal::message_queue_handle_t comms_handler_output_data_queue_handle = nullptr;
    rtosal::message_queue_handle_t serial_monitor_usart_queue_handle = nullptr;

    static uint32_t i2c_iteration_tick;

    int16_t rtd_0_channel_id = ID_INVALID;
    int16_t rtd_1_channel_id = ID_INVALID;
    int16_t rtd_2_channel_id = ID_INVALID;

    common_packet_t tx_common_packet;

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

                spi_tx_queue_handle                     = get_spi_2_extrusion_task_tx_queue_handle();
                spi_rx_queue_handle                     = get_spi_2_extrusion_task_rx_queue_handle();
                comms_handler_output_data_queue_handle  = get_comms_handler_output_data_queue_handle();
                serial_monitor_usart_queue_handle       = get_serial_monitor_usart_queue_handle();

                rtosal::event_flag_wait(initialization_event_flags_handle, READY_FOR_RESOURCE_INIT_FLAG, osFlagsWaitAny, osWaitForever);

                device::debug_serial_monitor.initialize(get_usart_2_handle(), serial_monitor_usart_queue_handle);
                device::built_in_display.initialize(get_i2c_2_handle(), comms_handler_output_data_queue_handle);

                hal::spi_2.initialize(&spi_2_handle, SPI_2_ID, get_timer_2_handle(), FREQUENCY_1_MHZ);
                hal::spi_2.register_callback(spi::TX_RX_COMPLETE_CALLBACK_ID, hal_callback_spi_2_tx_rx_complete);
                hal::spi_2.register_callback(spi::ERROR_CALLBACK_ID, hal_callback_spi_2_error);
                hal::spi_2.create_channel(rtd_0_channel_id, PORT_B, GPIO_PIN_14, spi_tx_queue_handle, spi_rx_queue_handle);
                hal::spi_2.create_channel(rtd_1_channel_id, PORT_B, GPIO_PIN_15, spi_tx_queue_handle, spi_rx_queue_handle);
                hal::spi_2.create_channel(rtd_2_channel_id, PORT_B, GPIO_PIN_1, spi_tx_queue_handle, spi_rx_queue_handle);

                hal::i2c_register_callback(get_i2c_2_handle(), hal::I2C_CONTROLLER_TX_COMPLETE_CALLBACK_ID, hal_callback_i2c_controller_tx_complete);
                hal::i2c_register_callback(get_i2c_2_handle(), hal::I2C_ERROR_CALLBACK_ID, hal_callback_i2c_controller_error);

//                device::zone_1_band_heater.initialize(TEMPERATURE_ZONE_1, TIMER_10_ID, get_zone_1_band_heater_mutex_handle());
//                device::zone_2_band_heater.initialize(TEMPERATURE_ZONE_2, TIMER_13_ID, get_zone_2_band_heater_mutex_handle());
//                device::zone_3_band_heater.initialize(TEMPERATURE_ZONE_3, TIMER_14_ID, get_zone_3_band_heater_mutex_handle());
//
//                hal::timer_register_callback(get_timer_1_handle(), hal::TIMER_INPUT_CAPTURE_CALLBACK_ID, hal_callback_tim_1_input_capture_pulse_detected_callback);
//                hal::timer_input_capture_start_interrupt(get_timer_1_handle(), hal::TIMER_CHANNEL_2);
                hal::timer_2_initialize();
                hal::timer_time_base_start(get_timer_2_handle());

                rtosal::event_flag_set(initialization_event_flags_handle, READY_FOR_USER_INIT_FLAG);

                comms_handler_state = COMMS_HANDLER_STATE_RUN;

                break;
            }
            case COMMS_HANDLER_STATE_RUN:
            {
                device::debug_serial_monitor.process_send_buffer();
                device::built_in_display.get_intertask_output_data();
                device::built_in_display.update_output();

//                device::zone_1_band_heater.update_period();
//                device::zone_2_band_heater.update_period();
//                device::zone_3_band_heater.update_period();

                hal::spi_2.receive_inter_task_transaction_requests(spi_tx_queue_handle, tx_common_packet);
                hal::spi_2.process_send_buffer();
                hal::spi_2.process_return_buffers(spi_rx_packet, 0, rx_d);

                // TODO: move heartbeat led
                hal::gpio_toggle_pin(PORT_A, PIN_5);

                hal::rtc_get_time_stamp(time_stamp);

                break;
            }
            default:
            {
                break;
            }
        }

    }
}
