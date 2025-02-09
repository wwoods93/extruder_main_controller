/***********************************************************************************************************************
 * Main_Controller
 * system_operation_comms_handler.h
 *
 * wilson
 * 11/6/22
 * 3:46 PM
 *
 * Description:
 *
 **********************************************************************************************************************/

#ifndef MAIN_CONTROLLER_SYS_OP_USER_COMMS_H
#define MAIN_CONTROLLER_SYS_OP_USER_COMMS_H

#include "../layer_0/hal_spi.h"
#include "../layer_0/hal_i2c.h"
#include "../layer_1/rtd.h"
#include "../layer_1/band_heater.h"


namespace sys_op::user_comms
{
    void task_intitialize();
    void task_state_machine();
}


#endif //MAIN_CONTROLLER_SYS_OP_USER_COMMS_H
