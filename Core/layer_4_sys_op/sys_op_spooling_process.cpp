/***********************************************************************************************************************
 * Main_Controller
 * system_operation_spooling_process.cpp
 *
 * wilson
 * 11/6/22
 * 3:52 PM
 *
 * Description:
 *
 **********************************************************************************************************************/

#include "../layer_0/rtosal.h"
#include "sys_op_spooling_process.h"

namespace sys_op::spooling
{
    void task_intitialize()
    {

    }

    void task_state_machine()
    {
        rtosal::thread_yield();
    }
}
