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

#if (USE_CMSIS_OS2 == 1U)

    int32_t kernel_init()
    {
        return (int32_t)osKernelInitialize();
    }

    int32_t kernel_start()
    {
        return (int32_t)osKernelStart();
    }

    message_queue_handle_t message_queue_create(uint32_t arg_message_count, uint32_t arg_message_size, const message_queue_attr_t* arg_message_queue_attr)
    {
        return (message_queue_handle_t)osMessageQueueNew(arg_message_count, arg_message_size, (const osMessageQueueAttr_t*)arg_message_queue_attr);
    }

    int32_t message_queue_send(message_queue_handle_t arg_message_queue_id, void* arg_message_ptr, uint32_t arg_timeout)
    {
        uint8_t tmp = 0U;
        return (int32_t) osMessageQueuePut((osMessageQueueId_t) arg_message_queue_id, arg_message_ptr, tmp, arg_timeout);
    }

    int32_t message_queue_receive(message_queue_handle_t arg_message_queue_id, void* arg_message_ptr, uint32_t arg_timeout)
    {
        return (int32_t) osMessageQueueGet((osMessageQueueId_t) arg_message_queue_id, arg_message_ptr, nullptr, arg_timeout);
    }

    uint32_t event_flag_set(message_queue_handle_t arg_event_flag_id, uint32_t arg_flags)
    {
        return osEventFlagsSet(arg_event_flag_id, arg_flags);
    }
    uint32_t event_flag_clear(event_flag_handle_t arg_event_flag_handle, uint32_t arg_flags)
    {
        return osEventFlagsClear((osEventFlagsId_t)arg_event_flag_handle, arg_flags);
    }

    uint32_t event_flag_wait(message_queue_handle_t arg_event_flag_id, uint32_t arg_flags, uint32_t arg_options, uint32_t arg_timeout)
    {
        return osEventFlagsWait(arg_event_flag_id, arg_flags, arg_options, arg_timeout);
    }

    event_flag_handle_t event_flag_create(const event_flag_attr_t* arg_event_flag_attr)
    {
        return (event_flag_handle_t) osEventFlagsNew(arg_event_flag_attr);
    }

    thread_handle_t thread_create( thread_function_t arg_function, const thread_attr_t* arg_thread_attr)
    {
        return (thread_handle_t)osThreadNew((osThreadFunc_t)arg_function, nullptr, (const osThreadAttr_t*)arg_thread_attr);
    }

    int32_t thread_yield()
    {
        return osThreadYield();
    }

#endif
}
