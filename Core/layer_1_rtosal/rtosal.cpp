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
#include "../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/cmsis_os2.h"
/* hal includes */

/* driver includes */

/* rtos abstraction includes */

/* sys op includes */

/* meta structure includes */
#include "../meta_structure/meta_structure_system_manager.h"
/* layer_1_rtosal header */
#include "rtosal.h"


static uint8_t spi_tx_buffer_mutex_initialized_flag = false;
static uint8_t packet_add_index = 0;
static uint8_t packet_remove_index = 0;
static uint8_t packet_added = false;
static uint8_t packet_removed = false;
static uint8_t array_packet_counter = 0;
static uint8_t packet_valid = false;
static common_packet_t spi_common_packet_array[COMMON_PACKET_ARRAY_LENGTH_MAX];


namespace rtosal
{

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
    void set_spi_tx_buffer_mutex_initialized_flag(uint8_t _flag)
    {
        spi_tx_buffer_mutex_initialized_flag = _flag;
    }

    uint8_t get_spi_tx_buffer_mutex_initialized_flag()
    {
        return spi_tx_buffer_mutex_initialized_flag;
    }



    void initializae_spi_common_packet_array()
    {
        for (uint8_t index = 0; index < COMMON_PACKET_ARRAY_LENGTH_MAX; ++index)
        {
            memset(&spi_common_packet_array[index], '\0', sizeof(common_packet_t));
            spi_common_packet_array[index].status = 0xFF;
        }
    }

    void build_common_packet(common_packet_t& arg_packet, id_number_t arg_channel_id, uint8_t (&arg_bytes)[8], uint8_t (&arg_bytes_per_tx)[8], uint8_t arg_total_byte_count, uint8_t arg_tx_byte_count)
    {
        memset(&arg_packet, '\0', sizeof(common_packet_t));
        arg_packet.status = 0xFF; // packet active
        arg_packet.channel_id = arg_channel_id;
        memcpy(&arg_packet.bytes_per_tx, arg_bytes_per_tx, sizeof(arg_packet.bytes_per_tx));
        memcpy(&arg_packet.bytes, arg_bytes, sizeof(arg_packet.bytes));
        arg_packet.total_byte_count = arg_total_byte_count;
        arg_packet.tx_byte_count = arg_tx_byte_count;
    }

    uint8_t add_packet_to_common_packet_array(common_packet_t& _packet)
    {
        memset(&spi_common_packet_array[packet_add_index], '\0', sizeof(common_packet_t));
        memcpy(&spi_common_packet_array[packet_add_index], &_packet, sizeof(common_packet_t));
        packet_added = true;
        return packet_added;
    }

    void increment_packet_add_index()
    {
        if (packet_added == true)
        {
            ++array_packet_counter;
            ++packet_add_index;
            if (packet_add_index >= COMMON_PACKET_ARRAY_LENGTH_MAX) { packet_add_index = 0; }
            packet_added = false;
        }
    }


    uint8_t remove_packet_from_common_packet_array(common_packet_t& _packet)
    {
        if (array_packet_counter > 0)
        {
            if (spi_common_packet_array[packet_remove_index].status != 0xFF)
            {
                memset(&_packet, '\0', sizeof(common_packet_t));
                memcpy(&_packet, &spi_common_packet_array[packet_remove_index], sizeof(common_packet_t));
                packet_valid = true;
            }
            else
            {
                packet_valid = false;
            }

            memset(&spi_common_packet_array[packet_remove_index], '\0', sizeof(common_packet_t));
            spi_common_packet_array[packet_remove_index].status = 0xFF;
            packet_removed = true;
        }
        return packet_removed;
    }

    void increment_packet_remove_index()
    {
        if (packet_removed == true)
        {
            if (array_packet_counter > 0)
            {
                --array_packet_counter;
            }
            else
            {
                array_packet_counter = 0;
            }
            ++packet_remove_index;
            if (packet_remove_index >= COMMON_PACKET_ARRAY_LENGTH_MAX) { packet_remove_index = 0; }
            packet_removed = false;
        }
    }

    uint8_t common_packet_is_valid()
    {
        return packet_valid;
    }


}
