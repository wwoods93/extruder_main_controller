/***********************************************************************************************************************
 * Main_Controller
 * device.cpp
 *
 * wilson
 * 10/12/24
 * 10:49 PM
 *
 * Description:
 *
 **********************************************************************************************************************/

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

/* device header */
#include "device.h"

namespace device
{
    rtd z0_rtd;
    rtd z1_rtd;
    rtd z2_rtd;

    band_heater z0_heater;
    band_heater z1_heater;
    band_heater z2_heater;

    serial_monitor debug_serial_monitor;

    touch_screen built_in_display;

}
