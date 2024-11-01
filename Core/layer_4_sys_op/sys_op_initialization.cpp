/***********************************************************************************************************************
 * Main_Controller
 * system_operation_initialization.cpp
 *
 * wilson
 * 11/6/22
 * 3:49 PM
 *
 * Description:
 *
 **********************************************************************************************************************/
#include <cstdint>
#include "stm32f4xx.h"
#include "cmsis_os2.h"
#include "../layer_0/rtosal_globals.h"
#include "sys_op_initialization.h"

#define INITIALIZATION_TASK_STATE_INITIALIZE      0
#define INITIALIZATION_TASK_STATE_RUN             1

namespace sys_op::initialization
{
    osEventFlagsId_t initialization_event_flags_handle = nullptr;

    void task_state_machine()
    {
        static uint8_t initialization_state = INITIALIZATION_TASK_STATE_INITIALIZE;

        switch (initialization_state)
        {
            case INITIALIZATION_TASK_STATE_INITIALIZE:
            {
                initialization_event_flags_handle = get_initialization_task_queue_handle();
                osEventFlagsSet(initialization_event_flags_handle, READY_FOR_RESOURCE_INIT_FLAG);
                initialization_state = INITIALIZATION_TASK_STATE_RUN;
                break;
            }
            case INITIALIZATION_TASK_STATE_RUN:
            {

                break;
            }
            default:
                break;
        }
    }
}
