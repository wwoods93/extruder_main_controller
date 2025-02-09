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
    rtd rtd_zone_1;
    rtd rtd_zone_2;
    rtd rtd_zone_3;

    band_heater zone_1_band_heater;
    band_heater zone_2_band_heater;
    band_heater zone_3_band_heater;

    serial_monitor debug_serial_monitor;

    touch_screen built_in_display;

}
