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

#include "cmsis_os2.h"
#include "sys_op_spooling_process.h"

namespace sys_op::spooling
{
    void task_intitialize()
    {

    }

    void task_state_machine()
    {
        osThreadYield();
    }
}
