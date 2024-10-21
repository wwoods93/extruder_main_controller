/***********************************************************************************************************************
 * Main_Controller
 * rtos_globals.h
 *
 * wilson
 * 2/27/24
 * 9:37 PM
 *
 * Description:
 *
 **********************************************************************************************************************/

#ifndef MAIN_CONTROLLER_RTOSAL_GLOBALS_H
#define MAIN_CONTROLLER_RTOSAL_GLOBALS_H

/* c/c++ includes */
#include <cstdint>
/* stm32 includes */

/* third-party includes */
#include "cmsis_os2.h"
/* hal includes */

/* driver includes */

/* rtos abstraction includes */

/* sys op includes */

/* meta structure includes */

#define QUEUE_LENGTH_MAX    16

static constexpr uint32_t READY_FOR_RESOURCE_INIT_FLAG  = 0x10000000;
//static constexpr uint32_t READY_FOR_DEVICE_INIT_FLAG    = 0x001000000;
static constexpr uint32_t READY_FOR_USER_INIT_FLAG      = 0x01000000;
static constexpr uint32_t READY_FOR_COMMS_RUN           = 0X00100000;


osMessageQueueId_t get_initialization_task_queue_handle();
//osMessageQueueId_t get_spi_2_extrusion_task_tx_queue_handle();
//osMessageQueueId_t get_spi_2_extrusion_task_rx_queue_handle();

osMessageQueueId_t get_extrusion_task_i2c_tx_queue_handle();
osMessageQueueId_t get_extrusion_task_i2c_rx_queue_handle();
osMessageQueueId_t get_spooling_task_i2c_tx_queue_handle();
osMessageQueueId_t get_spooling_task_i2c_rx_queue_handle();

osEventFlagsId_t get_initialization_event_flags_handle();

osMutexId_t get_spi_tx_buffer_mutex();


#endif //MAIN_CONTROLLER_RTOSAL_GLOBALS_H
