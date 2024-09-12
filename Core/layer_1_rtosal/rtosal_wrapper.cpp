/***********************************************************************************************************************
 * Main_Controller
 * rtosal_wrapper.cpp
 *
 * wilson
 * 8/27/24
 * 12:21 AM
 *
 * Description:
 *
 **********************************************************************************************************************/

/* c/c++ includes */

/* stm32 includes */

/* third-party includes */

/* hal includes */

/* driver includes */

/* rtos abstraction includes */

/* sys op includes */

/* meta structure includes */

/* rtosal_wrapper header */
#include "rtosal_wrapper.h"




namespace rtosal
{
    uint32_t cmsis_os2_message_queue_send(cmsis_message_queue_id_t arg_message_queue_id, void* arg_message_ptr, uint8_t arg_message_priority, uint32_t arg_timeout)
    {
        return (int32_t) osMessageQueuePut((osMessageQueueId_t) arg_message_queue_id, arg_message_ptr, arg_message_priority, arg_timeout);
    }

    uint32_t cmsis_os2_message_queue_receive(cmsis_message_queue_id_t arg_message_queue_id, void* arg_message_ptr, uint8_t* arg_message_priority, uint32_t arg_timeout)
    {
        return (int32_t) osMessageQueueGet((osMessageQueueId_t) arg_message_queue_id, arg_message_ptr, arg_message_priority, arg_timeout);
    }

    uint32_t cmsis_os2_event_flag_set(cmsis_os2_event_flag_id arg_event_flag_id, uint32_t arg_flags)
    {
        return osEventFlagsSet(arg_event_flag_id, arg_flags);
    }

    uint32_t cmsis_os2_event_flag_wait(cmsis_os2_event_flag_id arg_event_flag_id, uint32_t arg_flags, uint32_t arg_options, uint32_t arg_timeout)
    {
        return osEventFlagsWait(arg_event_flag_id, arg_flags, arg_options, arg_timeout);
    }

}
