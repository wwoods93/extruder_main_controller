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

#include <cstdint>
#include "../meta_structure/meta_structure_system_manager.h"

#include "rtosal_wrapper.h"

#define USE_FREERTROS                       1U


#define BINARY_SEMPAHORE_MAX_COUNT          1U
#define BINARY_SEMAPHORE_INITIAL_COUNT      0U
#define TRANSACTION_BYTE_COUNT_MAX          8U
#define COMMON_PACKET_ARRAY_LENGTH_MAX      32U


osMessageQueueId_t get_extrusion_task_to_comms_handler_queue_1_handle();
osMessageQueueId_t get_extrusion_task_to_comms_handler_queue_2_handle();
osMessageQueueId_t get_extrusion_task_to_comms_handler_queue_3_handle();

osMessageQueueId_t get_comms_handler_to_extrusion_task_queue_1_handle();
osMessageQueueId_t get_comms_handler_to_extrusion_task_queue_2_handle();
osMessageQueueId_t get_comms_handler_to_extrusion_task_queue_3_handle();


osMessageQueueId_t get_spi_2_extrusion_task_tx_queue_handle();
osMessageQueueId_t get_spi_2_extrusion_task_rx_queue_handle();
osMessageQueueId_t get_comms_handler_output_data_queue_handle();
osMessageQueueId_t get_serial_monitor_usart_queue_handle();
osMutexId_t get_zone_1_band_heater_mutex_handle();
osMutexId_t get_zone_2_band_heater_mutex_handle();
osMutexId_t get_zone_3_band_heater_mutex_handle();
osMutexId_t get_serial_monitor_usart_mutex_handle();


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
    typedef struct
    {
        message_queue_handle_t message_queue_id;
        int16_t id;

    } message_queue_t;

    void rtosal_initialize();
    uint32_t get_rtos_kernel_tick_count();
    uint32_t get_rtos_kernel_tick_frequency();


    void build_common_packet(common_packet_t& arg_packet, int16_t arg_channel_id, uint8_t (&arg_bytes)[8], uint8_t (&arg_bytes_per_tx)[8]);

}

#endif //MAIN_CONTROLLER_RTOSAL_H
