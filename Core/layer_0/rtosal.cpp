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
/* hal includes */

/* driver includes */
#include "../layer_1/serial_monitor.h"
/* rtos abstraction includes */
#include "rtosal_globals.h"
/* sys op includes */

/* meta structure includes */
#include "../meta_structure/meta_structure_system_manager.h"
/* layer_1_rtosal header */
#include "rtosal.h"


static uint8_t spi_tx_buffer_mutex_initialized_flag = false;


osMutexId_t zone_1_band_heater_mutex_handle;
osMutexId_t zone_2_band_heater_mutex_handle;
osMutexId_t zone_3_band_heater_mutex_handle;
osMutexId_t serial_monitor_usart_mutex_handle;


osMessageQueueId_t spi_2_extrusion_task_tx_queue_handle;
osMessageQueueId_t spi_2_extrusion_task_rx_queue_handle;
osMessageQueueId_t comms_handler_output_data_queue_handle;
osMessageQueueId_t serial_monitor_usart_queue_handle;



const osMessageQueueAttr_t spi_2_extrusion_task_tx_queue_attributes = { .name = "spi_2_extrusion_task_tx_queue" };
const osMessageQueueAttr_t spi_2_extrusion_task_rx_queue_attributes = { .name = "spi_2_extrusion_task_rx_queue" };
const osMessageQueueAttr_t i2c_tx_queue_attributes = { .name = "touchscreen_i2c_tx_queue" };
const osMessageQueueAttr_t serial_monitor_usart_queue_attr = { .name = "serial_monitor_usart_queue" };

const osMutexAttr_t zone_1_band_heater_mutex_attr = { .name = "zone_1_band_heater_mutex", .attr_bits = osMutexPrioInherit, .cb_mem = nullptr, .cb_size = 0U };
const osMutexAttr_t zone_2_band_heater_mutex_attr = { .name = "zone_2_band_heater_mutex", .attr_bits = osMutexPrioInherit, .cb_mem = nullptr, .cb_size = 0U };
const osMutexAttr_t zone_3_band_heater_mutex_attr = { .name = "zone_3_band_heater_mutex", .attr_bits = osMutexPrioInherit, .cb_mem = nullptr, .cb_size = 0U };
const osMutexAttr_t serial_monitor_usart_mutex_attr = { .name = "serial_monitor_usart_mutex", .attr_bits = osMutexPrioInherit, .cb_mem = nullptr, .cb_size = 0U };


osMessageQueueId_t get_spi_2_extrusion_task_tx_queue_handle()
{
    return spi_2_extrusion_task_tx_queue_handle;
}

osMessageQueueId_t get_spi_2_extrusion_task_rx_queue_handle()
{
    return spi_2_extrusion_task_rx_queue_handle;
}

osMessageQueueId_t get_comms_handler_output_data_queue_handle()
{
    return comms_handler_output_data_queue_handle;
}

osMessageQueueId_t get_serial_monitor_usart_queue_handle()
{
    return serial_monitor_usart_queue_handle;
}

osMutexId_t get_zone_1_band_heater_mutex_handle()
{
    return zone_1_band_heater_mutex_handle;
}

osMutexId_t get_zone_2_band_heater_mutex_handle()
{
    return zone_2_band_heater_mutex_handle;
}

osMutexId_t get_zone_3_band_heater_mutex_handle()
{
    return zone_3_band_heater_mutex_handle;
}

osMutexId_t get_serial_monitor_usart_mutex_handle()
{
    return zone_3_band_heater_mutex_handle;
}

namespace rtosal
{
    void rtosal_initialize()
    {
        spi_2_extrusion_task_tx_queue_handle = osMessageQueueNew((uint32_t)QUEUE_LENGTH_MAX, (uint32_t)sizeof(common_packet_t), &spi_2_extrusion_task_tx_queue_attributes);
        spi_2_extrusion_task_rx_queue_handle = osMessageQueueNew((uint32_t)QUEUE_LENGTH_MAX, (uint32_t)sizeof(common_packet_t), &spi_2_extrusion_task_rx_queue_attributes);
        comms_handler_output_data_queue_handle = osMessageQueueNew((uint32_t)QUEUE_LENGTH_MAX, (uint32_t)sizeof(common_packet_t), &i2c_tx_queue_attributes);

        serial_monitor_usart_queue_handle = osMessageQueueNew((uint32_t)QUEUE_LENGTH_MAX, (uint32_t)sizeof(serial_monitor::packet_t), &serial_monitor_usart_queue_attr);
        zone_1_band_heater_mutex_handle = osMutexNew(&zone_1_band_heater_mutex_attr);
        zone_2_band_heater_mutex_handle = osMutexNew(&zone_2_band_heater_mutex_attr);
        zone_3_band_heater_mutex_handle = osMutexNew(&zone_3_band_heater_mutex_attr);
        serial_monitor_usart_mutex_handle = osMutexNew(&serial_monitor_usart_mutex_attr);
        osMutexRelease(zone_1_band_heater_mutex_handle);
        osMutexRelease(zone_2_band_heater_mutex_handle);
        osMutexRelease(zone_3_band_heater_mutex_handle);
        osMutexRelease(serial_monitor_usart_mutex_handle);

    }

    #if (USE_CMSIS_OS2 == 1U)
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
