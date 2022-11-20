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

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

void HAL_SPI_TxRxCplt_Callback(spi::handle_t *hspi);

void HAL_SPI_Error_Callback(spi::handle_t *hspi);



#endif //MAIN_CONTROLLER_HAL_CALLBACKS_H
