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

#include "stm32f4xx_hal.h"
#include "../system_operation_layer/sys_op_comms_handler.h"
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

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3)
    {
        HAL_IncTick();
    }
}

void HAL_SPI_TxRxCplt_Callback(spi::handle_t *hspi)
{
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14) == GPIO_PIN_RESET)
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
    spi_rx_data_ready_flag = 1;

}

void HAL_SPI_Error_Callback(spi::handle_t *hspi)
{
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14) == GPIO_PIN_RESET)
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
    spi_rx_data_ready_flag = 1;
}
