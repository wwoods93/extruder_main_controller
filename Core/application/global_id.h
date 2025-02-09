/***********************************************************************************************************************
 * Main_Controller
 * global_id.h
 *
 * wilson
 * 10/13/24
 * 9:45 PM
 *
 * Description:
 *
 **********************************************************************************************************************/

#ifndef MAIN_CONTROLLER_GLOBAL_ID_H
#define MAIN_CONTROLLER_GLOBAL_ID_H

/* c/c++ includes */
#include <cstdint>
/* stm32 includes */

/* third-party includes */

/* layer_0_hal includes */

/* layer_1_rtosal includes */

/* layer_2_device includes */

/* layer_3_control includes */

/* layer_3 includes */

/* layer_n_meta_structure includes */


static constexpr uint8_t ZONE_1_TEMP_GLOBAL_ID                         = 0x00;
static constexpr uint8_t ZONE_2_TEMP_GLOBAL_ID                         = 0x01;
static constexpr uint8_t ZONE_3_TEMP_GLOBAL_ID                         = 0x02;
static constexpr uint8_t ZONE_1_DUTY_CYCLE_GLOBAL_ID                   = 0x03;
static constexpr uint8_t ZONE_2_DUTY_CYCLE_GLOBAL_ID                   = 0x04;
static constexpr uint8_t ZONE_3_DUTY_CYCLE_GLOBAL_ID                   = 0x05;
static constexpr uint8_t RTD_SPI_PACKETS_RECEIVED_GLOBAL_ID            = 0x06;
static constexpr uint8_t RTD_SPI_PACKETS_REQUESTED_GLOBAL_ID           = 0x07;
static constexpr uint8_t MAIN_CONTROLLER_TIME_STAMP                    = 0x08;


#endif //MAIN_CONTROLLER_GLOBAL_ID_H
