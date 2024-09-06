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

/* hal includes */
#include "hal_spi.h"
/* driver includes */

/* rtos abstraction includes */

/* sys op includes */

/* meta structure includes */


void hal_callback_spi_rx_tx_complete(spi *arg_object);
void hal_callback_spi_error(spi *arg_object);

void hal_callback_i2c_controller_tx_complete(I2C_HandleTypeDef *hi2c);
void hal_callback_i2c_controller_error(I2C_HandleTypeDef *hi2c);

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);


#endif //MAIN_CONTROLLER_HAL_CALLBACK_H
