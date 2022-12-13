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

uint8_t hal_callbacks_get_spi_rx_data_ready_flag();
void hal_callbacks_set_spi_rx_data_ready_flag(uint8_t status);

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

void HAL_SPI_TxRxCplt_Callback(spi::handle_t *hspi);

void HAL_SPI_Error_Callback(spi::handle_t *hspi);



#endif //MAIN_CONTROLLER_HAL_CALLBACKS_H
