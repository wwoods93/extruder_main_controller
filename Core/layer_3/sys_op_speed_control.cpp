/***********************************************************************************************************************
 * Main_Controller
 * system_operation_preparation_process.cpp
 *
 * wilson
 * 11/6/22
 * 3:50 PM
 *
 * Description:
 *
 **********************************************************************************************************************/

#include "../layer_0/rtosal.h"
#include "sys_op_speed_control.h"

namespace sys_op::speed_control
{
    void task_intitialize()
    {

    }

    void task_state_machine()
    {
        rtosal::thread_yield();

    }
}
