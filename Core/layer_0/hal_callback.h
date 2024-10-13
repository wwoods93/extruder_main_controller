/***********************************************************************************************************************
 * Main_Controller
 * hal_callbacks.h
 *
 * wilson
 * 11/4/22
 * 12:38 AM
 *
 * Description:
 *
 **********************************************************************************************************************/

#ifndef MAIN_CONTROLLER_HAL_CALLBACK_H
#define MAIN_CONTROLLER_HAL_CALLBACK_H

/* c/c++ includes */

/* stm32 includes */

/* third-party includes */

/* layer_0 includes */
#include "hal_spi.h"
/* layer_1_rtosal includes */

/* layer_1 includes */

/* layer_3_control includes */

/* layer_4_sys_op includes */

/* layer_n_meta_structure includes */

void hal_callback_spi_1_tx_rx_complete(spi *arg_object);
void hal_callback_spi_2_tx_rx_complete(spi *arg_object);
void hal_callback_spi_1_error(spi *arg_object);
void hal_callback_spi_2_error(spi *arg_object);

void hal_callback_i2c_controller_tx_complete(I2C_HandleTypeDef *hi2c);
void hal_callback_i2c_controller_error(I2C_HandleTypeDef *hi2c);

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);


#endif //MAIN_CONTROLLER_HAL_CALLBACK_H
