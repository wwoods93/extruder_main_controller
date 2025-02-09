/***********************************************************************************************************************
 * Main_Controller
 * device_serial_monitor.cpp
 *
 * wilson
 * 10/8/24
 * 12:02 AM
 *
 * Description:
 *
 **********************************************************************************************************************/

/* c/c++ includes */
#include <cstdint>
#include <memory>
#include <cstring>
#include <cstdio>
/* stm32 includes */

/* third-party includes */
#include "../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/cmsis_os2.h"
/* layer_0 includes */
#include "../layer_0/hal_wrapper.h"
/* layer_1_rtosal includes */
#include "../layer_0/rtosal.h"
/* layer_1 includes */

/* layer_3_control includes */

/* layer_3 includes */

/* layer_n_meta_structure includes */

/* device_serial_monitor header */
#include "serial_monitor.h"


uint32_t serial_monitor::initialize(UART_HandleTypeDef* arg_usart_module, rtosal::message_queue_handle_t arg_message_queue_handle)
{

    usart_module = arg_usart_module;
    usart_message_queue_send_handle = arg_message_queue_handle;
    usart_message_queue_receive_handle = arg_message_queue_handle;
    return 0;
}



uint32_t serial_monitor::print(const char* arg_output_string)
{
    packet_t packet;
    hal::rtc_get_time_stamp(time_stamp);
    memset(&packet, '\0', sizeof(packet_t));
    sprintf(packet.message, "%s->%s:: %s", time_stamp, name, arg_output_string);

    if (rtosal::message_queue_send(usart_message_queue_send_handle, &packet, 0U) == rtosal::OS_OK)
    {
        // success
    }
    else
    {
        // handle error
    }

    return 0;
}

uint32_t serial_monitor::process_send_buffer()
{
    packet_t packet;
    memset(&packet, '\0', sizeof(packet_t));
    if (rtosal::message_queue_receive(usart_message_queue_receive_handle, &packet, 0U) == rtosal::OS_OK)
    {
        memset(&message, '\0', sizeof(message));
        memcpy(&message, &packet.message, sizeof(packet.message));
        HAL_UART_Transmit_IT(usart_module, (uint8_t *) message,strlen(message));
    }
    else
    {
        // handle error
    }
    return 0;
}
