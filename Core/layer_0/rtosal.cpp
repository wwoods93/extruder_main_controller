/***********************************************************************************************************************
 * Main_Controller
 * layer_1_rtosal.cpp
 *
 * wilson
 * 11/6/22
 * 2:46 AM
 *
 * Description:
 *
 **********************************************************************************************************************/

/* c/c++ includes */
#include <cstdint>
#include <cstring>
/* stm32 includes */

/* third-party includes */
#include "cmsis_os2.h"
/* layer_0 includes */

/* layer_1 includes */
#include "../layer_1/device.h"
/* layer_2 includes */

/* layer_3 includes */

/* application includes */

/* rtosal header */
#include "rtosal.h"


namespace rtosal
{
    event_flag_handle_t initialization_event_flags_handle;
    message_queue_handle_t user_comms_output_queue_handle;
    message_queue_handle_t serial_monitor_usart_queue_handle;

    const event_flag_attr_t     initialization_event_flags_attributes   = { .name = "initialization_event_flags" };
    const message_queue_attr_t  user_comms_output_queue_attributes      = { .name = "touchscreen_i2c_tx_queue" };
    const message_queue_attr_t  serial_monitor_usart_queue_attr         = { .name = "serial_monitor_usart_queue" };

    void rtosal_resource_init()
    {
        user_comms_output_queue_handle      = message_queue_create((uint32_t)QUEUE_LENGTH_MAX, (uint32_t)sizeof(common_float_data_t),       &user_comms_output_queue_attributes);
        serial_monitor_usart_queue_handle   = message_queue_create((uint32_t)QUEUE_LENGTH_MAX, (uint32_t)sizeof(serial_monitor::packet_t),  &serial_monitor_usart_queue_attr);

        initialization_event_flags_handle   = event_flag_create(&initialization_event_flags_attributes);

        event_flag_clear(initialization_event_flags_handle, 0xFFFFFFFF);
    }

    #if (USE_CMSIS_OS2 == 1U)

        event_flag_handle_t get_initialization_event_flags_handle()
        {
            return initialization_event_flags_handle;
        }

        message_queue_handle_t get_comms_handler_output_data_queue_handle()
        {
            return user_comms_output_queue_handle;
        }

        message_queue_handle_t get_serial_monitor_usart_queue_handle()
        {
            return serial_monitor_usart_queue_handle;
        }

        uint32_t get_rtos_kernel_tick_frequency()
        {
            return osKernelGetTickFreq();
        }

        uint32_t get_rtos_kernel_tick_count()
        {
            return osKernelGetTickCount();
        }

    #endif

    void build_common_packet(common_packet_t& arg_packet, int16_t arg_channel_id, uint8_t (&arg_bytes)[8], uint8_t (&arg_bytes_per_tx)[8])
    {
        memset(&arg_packet, '\0', sizeof(common_packet_t));
        arg_packet.status = 0xFF; // packet active
        arg_packet.channel_id = arg_channel_id;
        memcpy(&arg_packet.bytes_per_transaction, arg_bytes_per_tx, sizeof(arg_packet.bytes_per_transaction));
        memcpy(&arg_packet.bytes, arg_bytes, sizeof(arg_packet.bytes));
    }
}
