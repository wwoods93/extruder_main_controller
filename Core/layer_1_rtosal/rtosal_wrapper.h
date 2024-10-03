/***********************************************************************************************************************
 * Main_Controller
 * rtosal_wrapper.h
 *
 * wilson
 * 8/27/24
 * 12:21 AM
 *
 * Description:
 *
 **********************************************************************************************************************/

#ifndef MAIN_CONTROLLER_RTOSAL_WRAPPER_H
#define MAIN_CONTROLLER_RTOSAL_WRAPPER_H

/* c/c++ includes */
#include <type_traits>
/* stm32 includes */

/* third-party includes */
#include "../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/cmsis_os2.h"
/* hal includes */

/* driver includes */

/* rtos abstraction includes */

/* sys op includes */

/* meta structure includes */


#define USE_CMSIS_OS2       1U

typedef osMessageQueueId_t cmsis_message_queue_id_t;
typedef osEventFlagsId_t cmsis_os2_event_flag_id;






namespace rtosal
{
    typedef std::conditional <USE_CMSIS_OS2 == 1U, osMessageQueueId_t, void*>::type message_queue_id_t;

    static constexpr int32_t OS_OK                 =  0;
    static constexpr int32_t OS_ERROR              = -1;
    static constexpr int32_t OS_ERROR_TIMEOUT      = -2;
    static constexpr int32_t OS_ERROR_RESOURCE     = -3;
    static constexpr int32_t OS_ERROR_PARAMETER    = -4;
    static constexpr int32_t OS_ERROR_NO_MEMORY    = -5;
    static constexpr int32_t OS_ERROR_ISR          = -6;
    static constexpr int32_t OS_STATUS_RESERVED    = 0x7FFFFFFF;

    int32_t message_queue_send(message_queue_id_t arg_message_queue_id, void* arg_message_ptr, uint32_t arg_timeout);
    int32_t message_queue_receive(message_queue_id_t arg_message_queue_id, void* arg_message_ptr, uint32_t arg_timeout);
    uint32_t event_flag_set(message_queue_id_t arg_event_flag_id, uint32_t arg_flags);
    uint32_t event_flag_wait(message_queue_id_t arg_event_flag_id, uint32_t arg_flags, uint32_t arg_options, uint32_t arg_timeout);
}

#endif //MAIN_CONTROLLER_RTOSAL_WRAPPER_H
