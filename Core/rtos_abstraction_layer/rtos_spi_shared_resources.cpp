/***********************************************************************************************************************
 * Main_Controller
 * osal_spi_shared_resources.cpp
 *
 * wilson
 * 11/6/22
 * 2:31 AM
 *
 * Description:
 *
 **********************************************************************************************************************/

#include <queue>

#include "../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/cmsis_os2.h"
#include "rtos_spi_shared_resources.h"


//std::queue<packet_t*> send_buffer;
//osSemaphoreId_t send_buffer_semaphore_id;
//
//void rtos_spi_transmit(id_number_t _channel_id, uint8_t* _tx_bytes, uint8_t _tx_size)
//{
//    packet_t spi_packet;
//    spi_packet.tx_size = _tx_size;
//    uint8_t* tx_bytes;
//    memcpy(tx_bytes, _tx_bytes, spi_packet.tx_size);
//
//    osStatus_t semaphore_status = osSemaphoreAcquire(send_buffer_semaphore_id, SPI_SEMAPHORE_TIMEOUT_MS);
//    if (semaphore_status == osOK)
//    {
//        send_buffer.push(&spi_packet);
//        osSemaphoreRelease(send_buffer_semaphore_id);
//        // return success
//    }
//    // log failure
//
//    // return failure
//}
