/***********************************************************************************************************************
 * Main_Controller
 * device.h
 *
 * wilson
 * 10/12/24
 * 10:49 PM
 *
 * Description:
 *
 **********************************************************************************************************************/

#ifndef MAIN_CONTROLLER_DEVICE_H
#define MAIN_CONTROLLER_DEVICE_H

/* c/c++ includes */

/* stm32 includes */

/* third-party includes */

/* layer_0_hal includes */

/* layer_1_rtosal includes */

/* layer_2_device includes */
#include "rtd.h"
#include "band_heater.h"
#include "serial_monitor.h"
#include "touch_screen.h"
/* layer_3_control includes */

/* layer_3 includes */

/* layer_n_meta_structure includes */

namespace device
{
    extern rtd z0_rtd;
    extern rtd z1_rtd;
    extern rtd z2_rtd;
    extern band_heater z0_heater;
    extern band_heater z1_heater;
    extern band_heater z2_heater;

    extern serial_monitor debug_serial_monitor;
    extern touch_screen built_in_display;
}

#endif //MAIN_CONTROLLER_DEVICE_H
