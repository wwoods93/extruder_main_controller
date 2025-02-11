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
#include "cmsis_os2.h"
/* hal includes */

/* driver includes */

/* rtos abstraction includes */

/* sys op includes */

/* meta structure includes */


#define USE_CMSIS_OS2       1U


namespace rtosal
{
    static constexpr int32_t OS_OK                 = (int32_t)0U;
    static constexpr int32_t OS_ERROR              = (int32_t)-1;
    static constexpr int32_t OS_ERROR_TIMEOUT      = (int32_t)-2;
    static constexpr int32_t OS_ERROR_RESOURCE     = (int32_t)-3;
    static constexpr int32_t OS_ERROR_PARAMETER    = (int32_t)-4;
    static constexpr int32_t OS_ERROR_NO_MEMORY    = (int32_t)-5;
    static constexpr int32_t OS_ERROR_ISR          = (int32_t)-6;
    static constexpr int32_t OS_STATUS_RESERVED    = (int32_t)0x7FFFFFFF;

    static constexpr uint32_t OS_FLAGS_ANY = osFlagsWaitAny;
    static constexpr uint32_t OS_FLAGS_ALL = osFlagsWaitAll;
    static constexpr uint32_t OS_FLAGS_NO_CLEAR = osFlagsNoClear;
    static constexpr uint32_t OS_WAIT_FOREVER = osWaitForever;

    typedef std::conditional <USE_CMSIS_OS2 == 1U, osThreadId_t, void*>::type thread_handle_t;
    typedef std::conditional <USE_CMSIS_OS2 == 1U, osMessageQueueId_t, void*>::type message_queue_handle_t;
    typedef std::conditional <USE_CMSIS_OS2 == 1U, osEventFlagsId_t, void*>::type event_flag_handle_t;

    typedef std::conditional <USE_CMSIS_OS2 == 1U, osThreadAttr_t, void*>::type thread_attr_t;
    typedef std::conditional <USE_CMSIS_OS2 == 1U, osMessageQueueAttr_t, void*>::type message_queue_attr_t;
    typedef std::conditional <USE_CMSIS_OS2 == 1U, osEventFlagsAttr_t, void*>::type event_flag_attr_t;

    typedef std::conditional <USE_CMSIS_OS2 == 1U, osThreadFunc_t, void*>::type thread_function_t;

    int32_t kernel_init();
    int32_t kernel_start();
    message_queue_handle_t message_queue_create(uint32_t arg_message_count, uint32_t arg_message_size, const message_queue_attr_t* arg_message_queue_attr);
    int32_t message_queue_send(message_queue_handle_t arg_message_queue_id, void* arg_message_ptr, uint32_t arg_timeout);
    int32_t message_queue_receive(message_queue_handle_t arg_message_queue_id, void* arg_message_ptr, uint32_t arg_timeout);
    uint32_t event_flag_set(message_queue_handle_t arg_event_flag_id, uint32_t arg_flags);
    uint32_t event_flag_clear(event_flag_handle_t arg_event_flag_handle, uint32_t arg_flags);
    uint32_t event_flag_wait(message_queue_handle_t arg_event_flag_id, uint32_t arg_flags, uint32_t arg_options, uint32_t arg_timeout);
    event_flag_handle_t event_flag_create(const event_flag_attr_t* arg_event_flag_attr);
    thread_handle_t thread_create( thread_function_t arg_function, const thread_attr_t* arg_thread_attr);
    int32_t thread_yield();
}

#endif //MAIN_CONTROLLER_RTOSAL_WRAPPER_H
