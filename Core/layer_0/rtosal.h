/***********************************************************************************************************************
 * Main_Controller
 * layer_1_rtosal.h
 *
 * wilson
 * 11/6/22
 * 2:46 AM
 *
 * Description:
 *
 **********************************************************************************************************************/

#ifndef MAIN_CONTROLLER_RTOSAL_H
#define MAIN_CONTROLLER_RTOSAL_H

/* c/c++ includes */
#include <cstdint>
#include <cstring>
/* stm32 includes */

/* third-party includes */

/* layer_0 includes */
#include "rtosal_wrapper.h"
/* layer_1 includes */

/* layer_2 includes */

/* layer_3 includes */

/* application includes */


static constexpr uint8_t USE_FREERTROS = 1U;
static constexpr uint8_t TRANSACTION_BYTE_COUNT_MAX = 8U;
static constexpr uint8_t QUEUE_LENGTH_MAX = 16U;

namespace rtosal
{
    static constexpr uint32_t READY_FOR_USER_COMMS_INIT_FLAG    = (uint32_t)0x10000000U;
    static constexpr uint32_t READY_FOR_TEMP_CONTROL_INIT_FLAG  = (uint32_t)0x01000000U;
    static constexpr uint32_t READY_FOR_SPEED_CONTROL_INIT      = (uint32_t)0X00100000U;

    typedef struct
    {
        uint8_t status;
        int16_t channel_id;
        uint8_t bytes_per_transaction[TRANSACTION_BYTE_COUNT_MAX];
        uint8_t bytes[TRANSACTION_BYTE_COUNT_MAX];
        uint8_t tx_byte_count;
    } common_packet_t;

    typedef struct
    {
        int16_t id;
        float value;
    } common_float_data_t;

    void rtosal_resource_init();
    event_flag_handle_t get_initialization_event_flags_handle();
    message_queue_handle_t user_comms_queue_get_handle();
    message_queue_handle_t get_serial_monitor_usart_queue_handle();

    void build_common_packet(common_packet_t& arg_packet, int16_t arg_channel_id, uint8_t (&arg_bytes)[8], uint8_t (&arg_bytes_per_tx)[8]);
}

#endif //MAIN_CONTROLLER_RTOSAL_H
