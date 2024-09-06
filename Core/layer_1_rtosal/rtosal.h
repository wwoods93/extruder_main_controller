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

#include "../meta_structure/meta_structure_system_manager.h"

#define USE_FREERTROS                       1U
#define USE_CMSIS_OS2                       1U

#define BINARY_SEMPAHORE_MAX_COUNT          1U
#define BINARY_SEMAPHORE_INITIAL_COUNT      0U
#define TRANSACTION_BYTE_COUNT_MAX          8U
#define COMMON_PACKET_ARRAY_LENGTH_MAX      32U


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



namespace rtosal
{
    uint32_t get_rtos_kernel_tick_count();
    uint32_t get_rtos_kernel_tick_frequency();



    void initializae_spi_common_packet_array();
    void build_common_packet(common_packet_t& arg_packet, int16_t arg_channel_id, uint8_t (&arg_bytes)[8], uint8_t (&arg_bytes_per_tx)[8]);
    uint8_t add_packet_to_common_packet_array(common_packet_t& _packet);
    uint8_t remove_packet_from_common_packet_array(common_packet_t& _packet);
    void increment_packet_add_index();
    void increment_packet_remove_index();
    uint8_t common_packet_is_valid();
}

#endif //MAIN_CONTROLLER_RTOSAL_H
