/***********************************************************************************************************************
 * Main_Controller
 * device_serial_monitor.h
 *
 * wilson
 * 10/8/24
 * 12:02 AM
 *
 * Description:
 *
 **********************************************************************************************************************/

#ifndef MAIN_CONTROLLER_DEVICE_SERIAL_MONITOR_H
#define MAIN_CONTROLLER_DEVICE_SERIAL_MONITOR_H

/* c/c++ includes */
#include <cstdint>
/* stm32 includes */
#include "stm32f4xx.h"
/* third-party includes */

/* layer_0_hal includes */

/* layer_1_rtosal includes */

/* layer_2_device includes */

/* layer_3_control includes */

/* layer_4_sys_op includes */

/* layer_n_meta_structure includes */




class serial_monitor
{
    public:
        uint32_t initialize(UART_HandleTypeDef arg_uart_module);
        uint32_t print(char arg_output_string);



    private:
};


#endif //MAIN_CONTROLLER_DEVICE_SERIAL_MONITOR_H
