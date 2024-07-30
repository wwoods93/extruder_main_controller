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

#ifndef MAIN_CONTROLLER_HAL_CALLBACKS_H
#define MAIN_CONTROLLER_HAL_CALLBACKS_H

/* c/c++ includes */

/* stm32 includes */

/* third-party includes */

/* hal includes */
#include "hal_spi.h"
/* driver includes */

/* rtos abstraction includes */

/* sys op includes */

/* meta structure includes */


uint8_t hal_callbacks_get_spi_rx_data_ready_flag();
void hal_callbacks_set_spi_rx_data_ready_flag(uint8_t status);

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

void hal_callback_spi_rx_tx_complete(spi::module_t *hspi);

void HAL_SPI_Error_Callback(spi::module_t *hspi);


//uint8_t hal_callbacks_get_spi_rx_data_ready_flag();
//void hal_callbacks_set_spi_rx_data_ready_flag(uint8_t status);
//
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
//
//
//void hal_callback_spi_tx_rx_complete(spi::module_t* _module);
//void hal_callback_spi_tx_complete(spi::module_t* _module);
//void hal_callback_spi_rx_complete(spi::module_t* _module);
//
//void hal_callback_spi_tx_rx_half_complete(spi::module_t* _module);
//void hal_callback_spi_tx_half_complete(spi::module_t* _module);
//void hal_callback_spi_rx_half_complete(spi::module_t* _module);
//
//void hal_callback_spi_error(spi::module_t* _module);
//void hal_callback_spi_abort(spi::module_t* _module);
//
//void hal_callback_spi_msp_init(spi::module_t* _module);
//void hal_callback_spi_msp_deinit(spi::module_t* _module);
//
//
//void HAL_SPI_TxRxCplt_Callback(spi::module_t *hspi);
//void HAL_SPI_Error_Callback(spi::module_t *hspi);
//void HAL_SPI_TxCpltCallback(spi::module_t *spi_handle);
//void HAL_SPI_RxCpltCallback(spi::module_t *spi_handle);
//
//void HAL_SPI_TxHalfCpltCallback(spi::module_t *spi_handle);
//
//void HAL_SPI_RxHalfCpltCallback(spi::module_t *spi_handle);
//
//void HAL_SPI_TxRxHalfCpltCallback(spi::module_t *spi_handle);
//void HAL_SPI_AbortCpltCallback(spi::module_t *spi_handle);


#endif //MAIN_CONTROLLER_HAL_CALLBACKS_H
