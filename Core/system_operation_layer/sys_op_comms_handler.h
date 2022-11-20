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

#ifndef MAIN_CONTROLLER_SYS_OP_COMMS_HANDLER_H
#define MAIN_CONTROLLER_SYS_OP_COMMS_HANDLER_H

#include "../hardware_abstraction_layer/hal_spi.h"
#include "../hardware_abstraction_layer/hal_i2c.h"


spi* get_spi_object();
spi::handle_t* get_spi_handle();

namespace sys_op
{
    void comms_handler_intitialize();
    void comms_handler_state_machine();
}


#endif //MAIN_CONTROLLER_SYS_OP_COMMS_HANDLER_H
