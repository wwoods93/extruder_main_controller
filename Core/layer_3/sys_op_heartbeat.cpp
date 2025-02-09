/***********************************************************************************************************************
 * Main_Controller
 * sys_op_heartbeat_task.cpp
 *
 * wilson
 * 11/1/24
 * 2:35 PM
 *
 * Description:
 *
 **********************************************************************************************************************/

/* c/c++ includes */
#include <cstdint>
/* stm32 includes */

/* third-party includes */

/* layer_0_hal includes */
#include "../layer_0/hal.h"
#include "../layer_0/hal_wrapper.h"
/* layer_1_rtosal includes */

/* layer_2_device includes */

/* layer_3_control includes */

/* layer_3 includes */

/* layer_n_meta_structure includes */

/* sys_op_heartbeat_task header */
#include "sys_op_heartbeat.h"


namespace sys_op::heartbeat
{
    static constexpr uint8_t TASK_STATE_INITIALIZE = 0U;
    static constexpr uint8_t TASK_STATE_RUN = 1U;



    void task_intitialize()
    {

    }

    void task_state_machine()
    {
        static uint8_t task_state = TASK_STATE_INITIALIZE;

        switch (task_state)
        {
            case TASK_STATE_INITIALIZE:
            {
                task_state = TASK_STATE_RUN;
                break;
            }
            case TASK_STATE_RUN:
            {
                hal::gpio_toggle_pin(PORT_A, PIN_5);
                rtosal::thread_yield();
                break;
            }
            default:
            {
                break;
            }
        }

    }
}
