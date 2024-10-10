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

#include "../layer_0_hal/hal_spi.h"
#include "../layer_0_hal/hal_i2c.h"
#include "../layer_2_device/device_rtd.h"
#include "../layer_2_device/heating_element.h"


namespace hal
{
    spi* get_spi_1_object();
    spi* get_spi_2_object();
}

namespace device
{
    heating_element* get_heating_element_object();
}

spi::module_t* get_spi_handle();
//rtd* get_rtd_object();

namespace sys_op::comms_handler
{
    void task_intitialize();
    void task_state_machine();
}


#endif //MAIN_CONTROLLER_SYS_OP_COMMS_HANDLER_H
