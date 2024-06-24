/***********************************************************************************************************************
 * Main_Controller
 * hal_callbacks.cpp
 *
 * wilson
 * 11/4/22
 * 12:38 AM
 *
 * Description:
 *
 **********************************************************************************************************************/

/* c/c++ includes */

/* stm32 includes */
#include "stm32f4xx_hal.h"
/* third-party includes */

/* hal includes */

/* driver includes */

/* rtos abstraction includes */

/* sys op includes */

/* meta structure includes */

/* hal_callbacks header */
#include "hal_callbacks.h"


static uint8_t spi_rx_data_ready_flag = 0;

uint8_t hal_callbacks_get_spi_rx_data_ready_flag()
{
    return spi_rx_data_ready_flag;
}

void hal_callbacks_set_spi_rx_data_ready_flag(uint8_t status)
{
    spi_rx_data_ready_flag = status;
}

void hal_callbacks_assert_spi_chip_select(spi::handle_t* _module)
{
    if (HAL_GPIO_ReadPin(_module->chip_select.port, _module->chip_select.pin) == spi::CHIP_SELECT_RESET)
        HAL_GPIO_WritePin(_module->chip_select.port, _module->chip_select.pin, (GPIO_PinState) spi::CHIP_SELECT_SET);
}
void hal_callbacks_deassert_spi_chip_select(spi::handle_t* _module)
{
    if (HAL_GPIO_ReadPin(_module->chip_select.port, _module->chip_select.pin) == spi::CHIP_SELECT_SET)
        HAL_GPIO_WritePin(_module->chip_select.port, _module->chip_select.pin, (GPIO_PinState) spi::CHIP_SELECT_RESET);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3)
    {
        HAL_IncTick();
    }
}

void hal_callback_spi_tx_rx_complete(spi::handle_t* _module)
{
    hal_callbacks_deassert_spi_chip_select(_module);
    spi_rx_data_ready_flag = 1;
}

void hal_callback_spi_tx_complete(spi::handle_t* _module)
{
    STM_HAL_UNUSED(_module);
}

void hal_callback_spi_rx_complete(spi::handle_t* _module)
{
    STM_HAL_UNUSED(_module);
}

void hal_callback_spi_tx_rx_half_complete(spi::handle_t* _module)
{
    STM_HAL_UNUSED(_module);
}

void hal_callback_spi_tx_half_complete(spi::handle_t* _module)
{
    STM_HAL_UNUSED(_module);
}

void hal_callback_spi_rx_half_complete(spi::handle_t* _module)
{
    STM_HAL_UNUSED(_module);
}

void hal_callback_spi_error(spi::handle_t* _module)
{
    hal_callbacks_deassert_spi_chip_select(_module);
    spi_rx_data_ready_flag = 1;
}

void hal_callback_spi_abort(spi::handle_t* _module)
{
    STM_HAL_UNUSED(_module);
}

void hal_callback_spi_msp_init(spi::handle_t* _module)
{
    STM_HAL_UNUSED(_module);
}

void hal_callback_spi_msp_deinit(spi::handle_t* _module)
{
    STM_HAL_UNUSED(_module);
}





void HAL_SPI_TxRxCplt_Callback(spi::handle_t *hspi)
{
    hal_callbacks_deassert_spi_chip_select(hspi);
    spi_rx_data_ready_flag = 1;
}

void HAL_SPI_Error_Callback(spi::handle_t *hspi)
{
    hal_callbacks_deassert_spi_chip_select(hspi);
    spi_rx_data_ready_flag = 1;
}

void HAL_SPI_TxCpltCallback(spi::handle_t *spi_handle)
{
    STM_HAL_UNUSED(spi_handle);
}

void HAL_SPI_RxCpltCallback(spi::handle_t *spi_handle)
{
    STM_HAL_UNUSED(spi_handle);
}

void HAL_SPI_TxHalfCpltCallback(spi::handle_t *spi_handle)
{
    STM_HAL_UNUSED(spi_handle);
}

void HAL_SPI_RxHalfCpltCallback(spi::handle_t *spi_handle)
{
    STM_HAL_UNUSED(spi_handle);
}

void HAL_SPI_TxRxHalfCpltCallback(spi::handle_t *spi_handle)
{
    STM_HAL_UNUSED(spi_handle);
}

void HAL_SPI_AbortCpltCallback(spi::handle_t *spi_handle)
{
    STM_HAL_UNUSED(spi_handle);
}
