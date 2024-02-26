/***********************************************************************************************************************
 * Main_Controller
 * system_operation_extrusion_process.cpp
 *
 * wilson
 * 11/6/22
 * 3:51 PM
 *
 * Description:
 *
 **********************************************************************************************************************/

#include <cstdint>
#include "cmsis_os2.h"
#include "sys_op_extrusion_process.h"

#define EXTRUSION_PROCESS_STATE_INITIALIZE      0
#define EXTRUSION_PROCESS_STATE_RUN             1

namespace sys_op
{
    static uint32_t extrusion_process_iteration_tick;
    uint32_t kernel_tick_frequency_hz;

    void extrusion_process_intitialize()
    {

    }

    void extrusion_process_state_machine()
    {
        static uint8_t extrusion_process_state = EXTRUSION_PROCESS_STATE_INITIALIZE;

        switch (extrusion_process_state)
        {
            case EXTRUSION_PROCESS_STATE_INITIALIZE:
            {
                extrusion_process_iteration_tick = 0;
                kernel_tick_frequency_hz = osKernelGetTickFreq() * 4;

                extrusion_process_state = EXTRUSION_PROCESS_STATE_RUN;
                break;
            }
            case EXTRUSION_PROCESS_STATE_RUN:
            {
                if (osKernelGetTickCount() - extrusion_process_iteration_tick > kernel_tick_frequency_hz)
                {

                    extrusion_process_iteration_tick = osKernelGetTickCount();
                }
                break;
            }
            default:
                break;
        }
    }
}
