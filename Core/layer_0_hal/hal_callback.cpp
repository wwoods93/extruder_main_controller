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
#include "hal_general.h"
/* driver includes */

/* rtos abstraction includes */

/* sys op includes */

/* meta structure includes */

/* hal_callbacks header */
#include "hal_callback.h"

static uint32_t consecutive_i2c_bus_error_count = 0;

void hal_callback_spi_rx_tx_complete(spi::module_t *arg_module)
{

    if (hal::gpio_read_pin(arg_module->chip_select_port, arg_module->chip_select_pin) == GPIO_PIN_RESET)
    {
        hal::gpio_write_pin(arg_module->chip_select_port, arg_module->chip_select_pin, GPIO_PIN_SET);
    }

    arg_module->rx_data_ready_flag = 1U;

}

void hal_callback_spi_error(spi::module_t *arg_module)
{
    if (hal::gpio_read_pin(arg_module->chip_select_port, arg_module->chip_select_pin) == GPIO_PIN_RESET)
    {
        hal::gpio_write_pin(arg_module->chip_select_port, arg_module->chip_select_pin, GPIO_PIN_SET);
    }

    arg_module->rx_data_ready_flag = 1U;
}


void hal_callback_i2c_controller_tx_complete(I2C_HandleTypeDef *hi2c)
{
    consecutive_i2c_bus_error_count = 0;
}

void hal_callback_i2c_controller_error(I2C_HandleTypeDef *hi2c)
{
    if ((hi2c->Instance->SR2 & I2C_SR2_BUSY) == I2C_SR2_BUSY)
    {
        consecutive_i2c_bus_error_count++;
    }
//    if (consecutive_i2c_bus_error_count > 10)
//    {
////        hi2c->Instance->CR1 |= I2C_CR1_SWRST;
////        hi2c->Instance->CR1 &= ~I2C_CR1_SWRST;
//        HAL_I2C_Master_Abort_IT(hi2c, (0x14 << 1));
//    }

    HAL_I2C_Master_Abort_IT(hi2c, (0x14 << 1));

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3)
    {
        HAL_IncTick();
    }
}
